// Golden WebGL versus one-device native WebGPU. No robot or live connection.
const assert=require('node:assert/strict'),fs=require('node:fs');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
(async()=>{
 const server=createTestServer();await new Promise(r=>server.listen(0,'127.0.0.1',r));const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
 try{
  const page=await browser.newPage(),errors=[],consoleErrors=[];page.on('pageerror',e=>errors.push(e.stack));page.on('console',m=>{if(m.type()==='error')consoleErrors.push(m.text());});
  if(process.env.CAPTURE_PACKET)await page.route('**/quality.rgbd',r=>r.fulfill({body:fs.readFileSync(process.env.CAPTURE_PACKET)}));await page.goto(`http://127.0.0.1:${server.address().port}/`);
  const result=await page.evaluate(async({captured,exact,width,height,sequence})=>{
   await import('/hybrid_rgbd_renderer.js');const f=await import('/web_rendering_fixture.js'),{state,gl}=f.setup(),p=window.probe;const {ExperimentalWebGpuBlendPCR}=await import('/experimental_webgpu_blendpcr.js');const {compileGpuProgram,FULLSCREEN_VERTEX_SHADER}=await import('/hybrid_camera_quality.js?v=20261021_original_rgb');
   if(width&&height){state.canvas.style.width=width+'px';state.canvas.style.height=height+'px';}
   const decoded=[];
   if(captured){state.socketOptions={exactDepth:true};const packet=p.parsePacket(await(await fetch('/quality.rgbd')).arrayBuffer());state.baseViewer={...packet.metadata.viewer,transform:new Float32Array(packet.metadata.viewer.transform)};for(const payload of packet.payloads)decoded.push(await p.decodeCamera(payload));}
   else for(let i=0;i<3;i++){const width=79,height=53,depth=new Uint16Array(width*height),color=new Uint8Array(192*128*3);for(let y=0;y<height;y++)for(let x=0;x<width;x++)depth[y*width+x]=x%19===0&&y%11<3?0:1000-i*8;for(let j=0;j<color.length;j+=3)color.set(i===0?[240,40,20]:i===1?[0,255,0]:[10,30,230],j);const model=p.identityMatrix();model[12]=(i-1)*.03;decoded.push({metadata:{id:String(i),model:'D435',intrinsics:[80,80,39,26],color_space:'original_rgb',color_width:192,color_height:128,color_projection:{model:'pinhole',size:[192,128],intrinsics:[180,180,95.5,63.5],extrinsics:[1,0,0,0,1,0,0,0,1,.015,0,0]}},width,height,depth,colorWidth:192,colorHeight:128,colorPixels:color,modelMatrix:model});}
   const info=decoded.map(c=>({...c,intrinsicsArray:new Float32Array(c.metadata.intrinsics),lastUpdateMs:performance.now()}));let native;
   try{
    native=await ExperimentalWebGpuBlendPCR.create(info,p.inverseRigidTransform,{exact});for(const c of decoded){native.acceptDecoded(c);state.cameras.push(p.uploadCamera(c));}
    state.cameraFusion.visibility.useRegions=false;for(const c of state.cameras)c.lastUpdateMs=performance.now();window.setGodotHybridViewSettings({display_mode:'blendpcr'});p.render();
    let view,capsules,overrideMask=null;const original=state.blendPCR.render.bind(state.blendPCR);state.blendPCR.render=(...args)=>{view=args[2];capsules=overrideMask||args[3];args[3]=capsules;return original(...args);};for(const c of state.cameras)c.lastUpdateMs=performance.now();p.render();gl.finish();
    const framebuffer=gl.createFramebuffer(),copyTexture=gl.createTexture(),vao=gl.createVertexArray();gl.bindTexture(gl.TEXTURE_2D,copyTexture);gl.texStorage2D(gl.TEXTURE_2D,1,gl.R32UI,Math.max(native.gpu.width,view.width),Math.max(native.gpu.height,view.height));
    const depthPrograms={};for(const array of [false,true])depthPrograms[array]=compileGpuProgram(gl,FULLSCREEN_VERTEX_SHADER,`#version 300 es\nprecision highp float;precision highp int;uniform highp ${array?'sampler2DArray':'sampler2D'} u_depth;uniform int u_layer;layout(location=0)out highp uint bits;void main(){bits=floatBitsToUint(texelFetch(u_depth,${array?'ivec3(ivec2(gl_FragCoord.xy),u_layer)':'ivec2(gl_FragCoord.xy)'},0).r);}`);
    function readGl(texture,width,height,layer=null,depth=false,integer=false){const out=integer||depth?new Uint32Array(width*height):new Uint8Array(width*height*4);gl.bindFramebuffer(gl.FRAMEBUFFER,framebuffer);gl.viewport(0,0,width,height);gl.disable(gl.DEPTH_TEST);gl.disable(gl.BLEND);gl.drawBuffers([gl.COLOR_ATTACHMENT0]);gl.readBuffer(gl.COLOR_ATTACHMENT0);
     if(depth){gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,copyTexture,0);const program=depthPrograms[layer!==null];gl.useProgram(program);gl.bindVertexArray(vao);gl.activeTexture(gl.TEXTURE0);gl.bindTexture(layer===null?gl.TEXTURE_2D:gl.TEXTURE_2D_ARRAY,texture);gl.uniform1i(gl.getUniformLocation(program,'u_depth'),0);gl.uniform1i(gl.getUniformLocation(program,'u_layer'),layer||0);gl.drawArrays(gl.TRIANGLES,0,3);}
     else if(layer===null)gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,texture,0);else gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,texture,0,layer);
     gl.readPixels(0,0,width,height,integer||depth?gl.RED_INTEGER:gl.RGBA,integer||depth?gl.UNSIGNED_INT:gl.UNSIGNED_BYTE,out);gl.bindFramebuffer(gl.FRAMEBUFFER,null);return out;
    }
    const different=(a,b)=>{let n=0,max=0,examples=[];for(let i=0;i<a.length;i++)if(a[i]!==b[i]){n++;max=Math.max(max,Math.abs(a[i]-b[i]));if(examples.length<8)examples.push({at:i,a:a[i],b:b[i]});}return {different:n,total:a.length,max,examples};};
    const glQuality=state.cameras.map((c,i)=>readGl(state.cameraFusion.quality,native.depthWidth,native.depthHeight,i));const glMetadata=state.cameras.map((c,i)=>readGl(state.blendPCR.metadata,view.width,view.height,i,false,true));const glColor=readGl(state.blendPCR.compositeColor,view.width,view.height),glDepth=readGl(state.blendPCR.compositeDepth,view.width,view.height,null,true);
    native.render(state.cameras,view,capsules,state.settings);await native.device.queue.onSubmittedWorkDone();
    const q=await native.readTexture(native.quality,'rgba',native.count),m=new Uint32Array((await native.readTexture(native.metadata,'uint',native.count)).buffer),color=await native.readTexture(native.color,'rgba'),depth=await native.readDepth(native.compositeDepth);
    const quality=glQuality.map((v,i)=>different(v,q.subarray(i*native.depthWidth*native.depthHeight*4,(i+1)*native.depthWidth*native.depthHeight*4))),metadata=glMetadata.map((v,i)=>different(v,m.subarray(i*view.width*view.height,(i+1)*view.width*view.height)));
    const vbits=await native.gpu.readDepth(),visibility=[];
    for(let camera=0;camera<native.count;camera++)for(let kind=0;kind<4;kind++){const slot=camera*4+kind,a=readGl(state.cameraFusion.visibility.texture,native.gpu.width,native.gpu.height,state.cameraFusion.visibility.layers[slot],true),layer=native.layerIndices[slot];visibility.push(different(a,vbits.subarray(layer*native.gpu.width*native.gpu.height,(layer+1)*native.gpu.width*native.gpu.height)));}
    const result={captured,exact,adapter:native.gpu.adapterInfo,quality,metadata,visibility,color:different(glColor,color),depth:different(glDepth,depth),glError:gl.getError(),gpuError:native.failed,view:[view.width,view.height],sequence:[]};
    const {WebGpuBlendPcrPresenter}=await import('/webgpu_blendpcr_present.js'),presenter=await WebGpuBlendPcrPresenter.create(native,gl,compileGpuProgram),presentColor=gl.createTexture(),presentDepth=gl.createTexture(),presentTarget=gl.createFramebuffer();
    try{
     gl.bindTexture(gl.TEXTURE_2D,presentColor);gl.texStorage2D(gl.TEXTURE_2D,1,gl.RGBA8,view.width,view.height);gl.bindTexture(gl.TEXTURE_2D,presentDepth);gl.texStorage2D(gl.TEXTURE_2D,1,gl.DEPTH24_STENCIL8,view.width,view.height);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);
     gl.bindFramebuffer(gl.FRAMEBUFFER,presentTarget);gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,presentColor,0);gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.DEPTH_STENCIL_ATTACHMENT,gl.TEXTURE_2D,presentDepth,0);
     presenter.present(null,presentTarget);gl.finish();result.presentation={color:different(glColor,readGl(presentColor,view.width,view.height)),depth:different(glDepth,readGl(presentDepth,view.width,view.height,null,true)),glError:gl.getError()};
    }finally{presenter.destroy();gl.deleteTexture(presentColor);gl.deleteTexture(presentDepth);gl.deleteFramebuffer(presentTarget);}
    if(sequence){
     const originals=state.cameras.map(c=>c.depth.slice()),base=state.baseViewer.transform.slice();
     function updateDepth(i,mutate){const c=state.cameras[i];c.depth=originals[i].slice();mutate(c.depth);c.uploadRevision=(c.uploadRevision||0)+1;gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,c.depthTexture);gl.texSubImage2D(gl.TEXTURE_2D,0,0,0,c.width,c.height,gl.RED_INTEGER,gl.UNSIGNED_SHORT,c.depth);}
     async function compare(label,stale=[]){
      gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.bindVertexArray(null);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);gl.depthMask(true);
      for(let i=0;i<state.cameras.length;i++)state.cameras[i].lastUpdateMs=performance.now()-(stale.includes(i)?2000:0);p.render();gl.finish();
      const aColor=readGl(state.blendPCR.compositeColor,view.width,view.height),aDepth=readGl(state.blendPCR.compositeDepth,view.width,view.height,null,true),aMeta=state.cameras.map((c,i)=>readGl(state.blendPCR.metadata,view.width,view.height,i,false,true));
      native.render(state.cameras,view,capsules,state.settings);const bColor=await native.readTexture(native.color,'rgba'),bDepth=await native.readDepth(native.compositeDepth),bMeta=new Uint32Array((await native.readTexture(native.metadata,'uint',native.count)).buffer);
      const quality=await native.readTexture(native.quality,'rgba',native.count),visibilityBits=await native.gpu.readDepth(),visibility=[];
      for(let camera=0;camera<native.count;camera++)for(let kind=0;kind<4;kind++){const slot=camera*4+kind,a=readGl(state.cameraFusion.visibility.texture,native.gpu.width,native.gpu.height,state.cameraFusion.visibility.layers[slot],true),layer=native.layerIndices[slot];visibility.push(different(a,visibilityBits.subarray(layer*native.gpu.width*native.gpu.height,(layer+1)*native.gpu.width*native.gpu.height)));}
      const item={label,color:different(aColor,bColor),depth:different(aDepth,bDepth),metadata:aMeta.map((a,i)=>different(a,bMeta.subarray(i*view.width*view.height,(i+1)*view.width*view.height))),quality:state.cameras.map((c,i)=>different(readGl(state.cameraFusion.quality,native.depthWidth,native.depthHeight,i),quality.subarray(i*native.depthWidth*native.depthHeight*4,(i+1)*native.depthWidth*native.depthHeight*4))),visibility,glError:gl.getError(),gpuError:native.failed};result.sequence.push(item);
     }
     state.baseViewer.transform[12]+=.013;await compare('viewpoint');
     for(let axis=0;axis<3;axis++)state.baseViewer.transform[12+axis]-=base[8+axis]*.995;await compare('near-plane');
     state.baseViewer.transform=base.slice();state.baseViewer.transform[12]+=.013;
     updateDepth(1,depth=>{for(let n=0;n<depth.length;n++)if(depth[n]&&n%5===0)depth[n]+=2;});await compare('independent-camera-update');
     Object.assign(state.settings,{workspace_crop_enabled:true,workspace_crop_min:[-.4,-.5,-1.5],workspace_crop_max:[.4,.4,-.3],workspace_crop_rotation:[8,5,13]});await compare('rotated-crop');
     Object.assign(state.settings,{workspace_crop_enabled:false,modeled_workspace_enabled:true,modeled_workspace_center:[0,-.25,-1],modeled_workspace_width:.75,modeled_workspace_depth:.65,modeled_workspace_height:.7,modeled_workspace_heading:5.6});await compare('modeled-table');
     state.settings.modeled_workspace_enabled=false;state.useCachedRobotMask=false;state.cameraFusion.maskTexture=null;
     overrideMask={starts:new Float32Array(24),ends:new Float32Array(24)};overrideMask.starts.set([.0183,.0207,-1,.133]);overrideMask.ends.set([.1,.04,-1,.133]);await compare('robot-mask');overrideMask=null;
     state.settings.depth_only_enabled=true;updateDepth(0,depth=>{for(let n=0;n<depth.length;n++)if(depth[n]&&n%7===0)depth[n]+=1;});await compare('gray-capture');state.settings.depth_only_enabled=false;await compare('restore-rgb');
     await compare('one-stale',[1]);await compare('all-stale',[0,1,2]);await compare('restore-fresh');
     state.canvas.style.width='321px';state.canvas.style.height='257px';await compare('resize');
     updateDepth(2,depth=>depth.fill(0));await compare('empty-source');updateDepth(2,()=>{});await compare('restored-source');state.baseViewer.transform=base;
    }
    for(const program of Object.values(depthPrograms))gl.deleteProgram(program);gl.deleteTexture(copyTexture);gl.deleteFramebuffer(framebuffer);gl.deleteVertexArray(vao);return result;
   }finally{native?.destroy();}
  },{captured:Boolean(process.env.CAPTURE_PACKET),exact:process.env.GPU_NATIVE_MATH!=='1',width:Number(process.env.VIEW_WIDTH)||0,height:Number(process.env.VIEW_HEIGHT)||0,sequence:process.env.TEST_SEQUENCE==='1'});
  const data={...result,errors,consoleErrors};if(process.env.RESULT_PATH)fs.writeFileSync(process.env.RESULT_PATH,JSON.stringify(data,null,2));console.log(JSON.stringify(data));assert.deepEqual(errors,[]);assert.equal(result.glError,0);assert.equal(result.gpuError,null);assert.equal(result.presentation.glError,0);for(const group of [...result.quality,...result.metadata,...result.visibility,result.color,result.depth,result.presentation.color,result.presentation.depth])assert.equal(group.different,0);for(const frame of result.sequence){assert.equal(frame.glError,0,frame.label);assert.equal(frame.gpuError,null,frame.label);for(const group of [frame.color,frame.depth,...frame.metadata,...frame.quality,...frame.visibility])assert.equal(group.different,0,frame.label);}
 }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
