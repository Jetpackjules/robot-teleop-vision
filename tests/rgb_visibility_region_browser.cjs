// Exact output/history parity for RGB sample regions, including outside-crop
// occluders. The optional capture is local test input, never a robot connection.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
(async()=>{
 const server=createTestServer();await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
 try{
  const page=await browser.newPage(),errors=[],consoleErrors=[];page.on('pageerror',e=>errors.push(e.message));page.on('console',m=>{if(m.type()==='warning'||m.type()==='error')consoleErrors.push(m.text());});
  await page.route('**/hybrid_rgb_visibility.js*',route=>route.fulfill({contentType:'text/javascript',body:fs.readFileSync(path.join(__dirname,'../web/hybrid_rgb_visibility.js'),'utf8')+'\nexport {projection as visibilityProjection,multiply as visibilityMultiply};'}));
  if(process.env.CAPTURE_PACKET)await page.route('**/quality.rgbd',route=>route.fulfill({body:fs.readFileSync(process.env.CAPTURE_PACKET)}));
  await page.goto(`http://127.0.0.1:${server.address().port}/`);
  const result=await page.evaluate(async({captured,benchmark})=>{
   await import('/hybrid_rgbd_renderer.js');const f=await import('/web_rendering_fixture.js'),{state,gl}=f.setup(),p=window.probe;
   const {GpuCameraRig,compileGpuProgram,FULLSCREEN_VERTEX_SHADER}=await import('/hybrid_camera_quality.js?v=20261021_original_rgb');
   const {workspaceCropBounds,WORKSPACE_CROP_CONTEXT}=await import('/workspace_crop.js?v=20261021_original_rgb');
   const {modeledWorkspaceBounds}=await import('/modeled_workspace.js?v=20261021_original_rgb');
   const {rgbVisibilityRegion}=await import('/rgb_visibility_region.js');
   const {visibilityProjection,visibilityMultiply}=await import('/hybrid_rgb_visibility.js?v=20261021_original_rgb');
   if(captured){state.socketOptions={exactDepth:true};const packet=p.parsePacket(await(await fetch('/quality.rgbd')).arrayBuffer());state.baseViewer={...packet.metadata.viewer,transform:new Float32Array(packet.metadata.viewer.transform)};for(const payload of packet.payloads)state.cameras.push(p.uploadCamera(await p.decodeCamera(payload)));}
   else for(let i=0;i<3;i++){
    const width=79,height=53,depth=new Uint16Array(width*height),color=new Uint8Array(192*128*3);
    for(let y=0;y<height;y++)for(let x=0;x<width;x++){depth[y*width+x]=x%19===0&&y%11<3?0:1000-i*8;}
    for(let j=0;j<color.length;j+=3)color.set(i===0?[240,40,20]:i===1?[0,255,0]:[10,30,230],j);
    const matrix=p.identityMatrix();matrix[12]=(i-1)*.03;
    state.cameras.push(p.uploadCamera({metadata:{id:String(i),model:'D435',intrinsics:[80,80,39,26],color_space:'original_rgb',color_width:192,color_height:128,color_projection:{model:'pinhole',size:[192,128],intrinsics:[180,180,95.5,63.5],extrinsics:[1,0,0,0,1,0,0,0,1,.015,0,0]}},width,height,depth,colorPixels:color,modelMatrix:matrix}));
   }
   state.cameraFusion.clear();const rigs=[new GpuCameraRig(gl),new GpuCameraRig(gl)];rigs[0].visibility.useRegions=false;rigs[1].visibility.useRegions=true;
   const width=Math.max(...state.cameras.map(c=>c.colorWidth)),height=Math.max(...state.cameras.map(c=>c.colorHeight));
   const fb=gl.createFramebuffer(),texture=gl.createTexture(),vao=gl.createVertexArray();gl.bindTexture(gl.TEXTURE_2D,texture);gl.texStorage2D(gl.TEXTURE_2D,1,gl.R32UI,width,height);gl.bindFramebuffer(gl.FRAMEBUFFER,fb);gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,texture,0);
   const copy=compileGpuProgram(gl,FULLSCREEN_VERTEX_SHADER,`#version 300 es
    precision highp float;precision highp int;uniform highp sampler2DArray u_depth;uniform int u_layer;layout(location=0) out highp uint out_bits;void main(){out_bits=floatBitsToUint(texelFetch(u_depth,ivec3(ivec2(gl_FragCoord.xy),u_layer),0).r);}`);
   gl.bindFramebuffer(gl.FRAMEBUFFER,null);const preflightError=gl.getError();
   const read=(rig,region,slot)=>{
    const values=new Uint32Array(region.width*region.height);if(!values.length)return values;
    gl.bindFramebuffer(gl.FRAMEBUFFER,fb);gl.useProgram(copy);gl.bindVertexArray(vao);gl.disable(gl.DEPTH_TEST);gl.disable(gl.BLEND);gl.disable(gl.SCISSOR_TEST);gl.viewport(0,0,width,height);gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D_ARRAY,rig.visibility.texture);gl.uniform1i(gl.getUniformLocation(copy,'u_depth'),0);gl.uniform1i(gl.getUniformLocation(copy,'u_layer'),rig.visibility.layers[slot]);gl.drawArrays(gl.TRIANGLES,0,3);gl.readPixels(region.x,region.y,region.width,region.height,gl.RED_INTEGER,gl.UNSIGNED_INT,values);gl.bindFramebuffer(gl.FRAMEBUFFER,null);return values;
   };
   const changed=(a,b)=>{let n=0;for(let i=0;i<a.length;i++)n+=a[i]!==b[i];return n;};
   const mutate=(index,offset)=>{
    const c=state.cameras[index],depth=new Uint16Array(c.depth);for(let y=Math.floor(c.height*.35);y<Math.floor(c.height*.55);y++)for(let x=Math.floor(c.width*.35);x<Math.floor(c.width*.55);x++){const at=y*c.width+x;if(depth[at])depth[at]+=offset;}
    c.depth=depth;c.depthTiles=null;c.uploadRevision++;gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,c.depthTexture);gl.pixelStorei(gl.UNPACK_ALIGNMENT,1);gl.texSubImage2D(gl.TEXTURE_2D,0,0,0,c.width,c.height,gl.RED_INTEGER,gl.UNSIGNED_SHORT,depth);
   };
   const crop=(scale=1,shift=0)=>({workspace_crop_enabled:true,workspace_crop_min:[-.25*scale+shift,-.05,-1-.36*scale],workspace_crop_max:[.25*scale+shift,.53*scale,-1+.36*scale],workspace_crop_rotation:[0,0,0]});
   window.setGodotHybridViewSettings({display_mode:'blendpcr',...crop()});
   let checks=0,narrowFrames=0;const failures=[],steps=[];
   const check=(condition,label)=>{checks++;if(!condition)failures.push(label);};
   const frame=async(label,settings={})=>{
    Object.assign(state.settings,settings);const images=[],warnings=[];
    for(const rig of rigs){for(const c of state.cameras)c.lastUpdateMs=performance.now();state.cameraFusion=rig;if(state.blendPCR)state.blendPCR.cacheKey='';p.render();gl.finish();images.push(f.image());warnings.push(state.stats.renderWarning);}
    const imageDifferent=changed(images[0],images[1]);check(imageDifferent===0,label+': final pixels differ '+imageDifferent);
    let depthDifferent=0;
    if(!state.settings.depth_only_enabled){
     const bounds=modeledWorkspaceBounds(state.settings,workspaceCropBounds(state.settings,WORKSPACE_CROP_CONTEXT),WORKSPACE_CROP_CONTEXT);
     for(let target=0;target<state.cameras.length;target++){
      const c=state.cameras[target],matrix=visibilityMultiply(visibilityProjection(c.metadata.color_projection),p.inverseRigidTransform(c.modelMatrix)),region=rgbVisibilityRegion(bounds,matrix,c.colorWidth,c.colorHeight);
      for(let kind=0;kind<4;kind++){const a=read(rigs[0],region,target*4+kind),b=read(rigs[1],region,target*4+kind);const n=changed(a,b);depthDifferent+=n;check(n===0,label+': visibility/history '+target+':'+kind+' differs '+n);}
     }
    }
    const narrow=rigs[1].visibility.layerStates.some(Boolean);narrowFrames+=narrow?1:0;
    check(gl.getError()===gl.NO_ERROR&&warnings.every(w=>!w)&&state.stats.renderMethod==='blendpcr',label+': active BlendPCR path and valid WebGL state');steps.push({label,imageDifferent,depthDifferent,warnings,narrow,replays:rigs[1].visibility.regionReplays||0});
   };
   await frame('initial narrow crop');mutate(0,10);await frame('one changed camera');await frame('expand with unchanged captures',crop(1.8,.05));
   mutate(1,8);await frame('shift and shrink',crop(.7,-.08));await frame('disable crop with unchanged captures',{workspace_crop_enabled:false});
   await frame('rotated crop',{...crop(),workspace_crop_rotation:[0,19,0]});
   for(let i=0;i<4;i++){mutate(i%3,5);await frame('depth-only capture '+i,{depth_only_enabled:true});}
   await frame('restore RGB and expand after depth-only',{...crop(1.8),depth_only_enabled:false,mesh_depth_delta:.025});
   await frame('modeled table',{workspace_crop_enabled:false,modeled_workspace_enabled:true,modeled_workspace_center:[0,0,-1],modeled_workspace_width:.5,modeled_workspace_depth:.73,modeled_workspace_height:.58,modeled_workspace_heading:5.6});
   state.workspaceEditor={active:false,drag:{}};
   await frame('begin table drag');await frame('resize during table drag',{modeled_workspace_width:.8,modeled_workspace_depth:1.1});
   state.workspaceEditor=null;await frame('end table drag');
   rigs[1].visibility.useRegions=false;await frame('disable region optimization');rigs[1].visibility.useRegions=true;await frame('enable region optimization');
   const timings=[],ext=gl.getExtension('EXT_disjoint_timer_query_webgl2');
   if(benchmark&&ext)for(const region of [false,true,false,true]){
    const rig=rigs[region?1:0],values=[];
    for(let i=0;i<9;i++){
     for(const c of state.cameras){c.lastUpdateMs=performance.now();c.uploadRevision++;}
     const q=gl.createQuery();gl.beginQuery(ext.TIME_ELAPSED_EXT,q);rig.update(state.cameras,p.inverseRigidTransform,performance.now(),state.settings);gl.endQuery(ext.TIME_ELAPSED_EXT);gl.finish();while(!gl.getQueryParameter(q,gl.QUERY_RESULT_AVAILABLE))await new Promise(r=>setTimeout(r,5));if(i>2&&!gl.getParameter(ext.GPU_DISJOINT_EXT))values.push(gl.getQueryParameter(q,gl.QUERY_RESULT)/1e6);gl.deleteQuery(q);
    }timings.push({region,gpuMedianMs:values.sort((a,b)=>a-b)[Math.floor(values.length/2)]});
   }
   for(const rig of rigs)rig.clear();gl.deleteProgram(copy);gl.deleteTexture(texture);gl.deleteFramebuffer(fb);gl.deleteVertexArray(vao);
   return {captured,preflightError,checks,failures,steps,narrowFrames,timings};
  },{captured:Boolean(process.env.CAPTURE_PACKET),benchmark:process.env.PROFILE_REGIONS==='1'});
  if(process.env.RESULT_PATH)fs.writeFileSync(process.env.RESULT_PATH,JSON.stringify({...result,errors,consoleErrors},null,2));console.log(JSON.stringify(result));assert.deepEqual(errors,[]);assert.deepEqual(result.failures,[]);assert.equal(result.preflightError,0);assert(result.narrowFrames>0,'the optimization really narrowed visibility maps');assert(result.steps.some(s=>s.replays>0),'crop expansion exercised historical capture replay');
 }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
