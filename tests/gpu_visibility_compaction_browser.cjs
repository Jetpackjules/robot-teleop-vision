// Opt-in WebGPU experiment. No robot connection, control input or deployment.
const assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
const root=path.resolve(__dirname,'..');
(async()=>{
  const server=createTestServer();await new Promise(r=>server.listen(0,'127.0.0.1',r));
  const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
  try{
    const page=await browser.newPage(),errors=[];page.on('pageerror',e=>errors.push(e.message));
    await page.route('**/hybrid_rgb_visibility.js*',route=>route.fulfill({contentType:'text/javascript',body:fs.readFileSync(path.join(root,'web/hybrid_rgb_visibility.js'),'utf8')+'\nexport {projection as visibilityProjection,multiply as visibilityMultiply};'}));
    if(process.env.CAPTURE_PACKET)await page.route('**/quality.rgbd',route=>route.fulfill({contentType:'application/octet-stream',body:fs.readFileSync(process.env.CAPTURE_PACKET)}));
    await page.goto(`http://127.0.0.1:${server.address().port}/`);
    const result=await page.evaluate(async ({captured,skipBenchmark,matrixMath,geometry})=>{
      await import('/hybrid_rgbd_renderer.js');
      const f=await import('/web_rendering_fixture.js'),{state,gl}=f.setup(),p=window.probe;
      const {visibilityProjection,visibilityMultiply}=await import('/hybrid_rgb_visibility.js?v=20261021_original_rgb');
      const {compileGpuProgram,FULLSCREEN_VERTEX_SHADER}=await import('/hybrid_camera_quality.js?v=20261021_original_rgb');
      const {ExperimentalGpuVisibility}=await import('/experimental_gpu_visibility.js');
      if(captured){
        state.socketOptions={exactDepth:true};const packet=p.parsePacket(await(await fetch('/quality.rgbd')).arrayBuffer());
        state.baseViewer={...packet.metadata.viewer,transform:new Float32Array(packet.metadata.viewer.transform)};
        for(const payload of packet.payloads)state.cameras.push(p.uploadCamera(await p.decodeCamera(payload)));
      }else{
        const width=79,height=53;
        for(let i=0;i<3;i++){
          const depth=new Uint16Array(width*height),color=new Uint8Array(192*128*3);
          for(let y=0;y<height;y++)for(let x=0;x<width;x++)depth[y*width+x]=geometry==='empty'?0:geometry==='dense'?1000+i*4:x<4||x>74||y>47||x>28&&x<34&&y<30?0:920+x*2+y+i*4;
          const matrix=p.identityMatrix();matrix[12]=(i-1)*.12;
          state.cameras.push(p.uploadCamera({metadata:{id:String(i),model:'D435',intrinsics:[80,80,39,26],color_space:'original_rgb',color_width:192,color_height:128,color_projection:{model:'pinhole',size:[192,128],intrinsics:[180,180,95.5,63.5],extrinsics:[1,0,0,0,1,0,0,0,1,.015,0,0]}},width,height,depth,colorPixels:color,modelMatrix:matrix}));
        }
      }
      for(const c of state.cameras)c.lastUpdateMs=performance.now();window.setGodotHybridViewSettings({display_mode:'blendpcr'});p.render();gl.finish();
      const rig=state.cameraFusion,fb=gl.createFramebuffer(),cameras=[];
      // Shader compilation on the first render can exceed camera freshness.
      // Rebuild the oracle after warm-up with all three captures current.
      for(const c of state.cameras)c.lastUpdateMs=performance.now();
      rig.visibility.update(state.cameras,rig.quality,p.inverseRigidTransform,true,performance.now(),true,state.cameras.map((_,i)=>`oracle-${i}`));gl.finish();
      for(let i=0;i<state.cameras.length;i++){
        const c=state.cameras[i],quality=new Uint8Array(c.width*c.height*4);gl.bindFramebuffer(gl.FRAMEBUFFER,fb);gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,rig.quality,0,i);gl.readPixels(0,0,c.width,c.height,gl.RGBA,gl.UNSIGNED_BYTE,quality);
        cameras.push({width:c.width,height:c.height,colorWidth:c.colorWidth,colorHeight:c.colorHeight,depth:c.depth,quality,intrinsics:c.intrinsicsArray});
      }
      const matrices=state.cameras.map(target=>{const projection=visibilityMultiply(visibilityProjection(target.metadata.color_projection),p.inverseRigidTransform(target.modelMatrix));return state.cameras.map(source=>visibilityMultiply(projection,source.modelMatrix));});
      const width=Math.max(...cameras.map(c=>c.colorWidth)),height=Math.max(...cameras.map(c=>c.colorHeight)),layers=cameras.length*2;
      const oracle=new Uint32Array(width*height*layers),texture=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,texture);gl.texStorage2D(gl.TEXTURE_2D,1,gl.R32UI,width,height);gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,texture,0);
      const copy=compileGpuProgram(gl,FULLSCREEN_VERTEX_SHADER,`#version 300 es
        precision highp float;precision highp int;uniform highp sampler2DArray u_depth;uniform int u_layer;
        layout(location=0) out highp uint out_bits;void main(){out_bits=floatBitsToUint(texelFetch(u_depth,ivec3(ivec2(gl_FragCoord.xy),u_layer),0).r);}`);
      gl.useProgram(copy);gl.disable(gl.DEPTH_TEST);gl.disable(gl.BLEND);gl.disable(gl.SCISSOR_TEST);gl.bindVertexArray(gl.createVertexArray());gl.viewport(0,0,width,height);gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D_ARRAY,rig.visibility.texture);gl.uniform1i(gl.getUniformLocation(copy,'u_depth'),0);
      for(let i=0;i<cameras.length;i++)for(let kind=0;kind<2;kind++){gl.uniform1i(gl.getUniformLocation(copy,'u_layer'),rig.visibility.layers[i*4+(kind===0?1:0)]);gl.drawArrays(gl.TRIANGLES,0,3);gl.readPixels(0,0,width,height,gl.RED_INTEGER,gl.UNSIGNED_INT,oracle.subarray((i*2+kind)*width*height,(i*2+kind+1)*width*height));}
      const glError=gl.getError();gl.deleteFramebuffer(fb);gl.deleteTexture(texture);gl.deleteProgram(copy);gl.bindFramebuffer(gl.FRAMEBUFFER,null);
      const gpu=await ExperimentalGpuVisibility.create(cameras,matrices,{matrixMath}),gpuErrors=[];gpu.device.addEventListener('uncapturederror',e=>gpuErrors.push(e.error.message));
      try{
        if(!skipBenchmark&&!gpu.timestamps)throw Error('GPU timing requires timestamp-query; set SKIP_BENCHMARK=1 for parity checks');
        await gpu.render(false);const reference=await gpu.readDepth();await gpu.render(true);const compacted=await gpu.readDepth();
        let different=0,crossApiDifferent=0,maximumDepthDifference=0;const a=new Float32Array(reference.buffer),b=new Float32Array(oracle.buffer);
        for(let i=0;i<reference.length;i++){if(reference[i]!==compacted[i])different++;if(reference[i]!==oracle[i])crossApiDifferent++;maximumDepthDifference=Math.max(maximumDepthDifference,Math.abs(a[i]-b[i]));}
        const layerStats=[];
        for(let layer=0;layer<layers;layer++){let glCovered=0,gpuCovered=0,onlyGl=0,onlyGpu=0,bothDifferent=0,maxBoth=0;for(let i=layer*width*height;i<(layer+1)*width*height;i++){const g=oracle[i]!==0x3f800000,w=reference[i]!==0x3f800000;glCovered+=g;gpuCovered+=w;onlyGl+=g&&!w;onlyGpu+=w&&!g;if(g&&w&&oracle[i]!==reference[i]){bothDifferent++;maxBoth=Math.max(maxBoth,Math.abs(a[i]-b[i]));}}layerStats.push({glCovered,gpuCovered,onlyGl,onlyGpu,bothDifferent,maxBoth});}
        const counts=[];
        for(let source=0;source<cameras.length;source++){
          const c=cameras[source],indices=await gpu.readIndices(source),seen=new Uint8Array((c.width-1)*(c.height-1)*2);let expected=0,invalid=0,duplicates=0;
          for(let y=0;y<c.height-1;y++)for(let x=0;x<c.width-1;x++){const q=(y*c.width+x)*4;expected+=(c.quality[q]>127)+(c.quality[q+1]>127);}
          for(let i=0;i<indices.length;i+=3){const [a,b,cIndex]=indices.subarray(i,i+3);let top,half;
            if(b===a+1&&cIndex===a+c.width){top=a;half=0;}else if(b===a+c.width&&cIndex===a+c.width-1){top=a-1;half=1;}else{invalid++;continue;}
            const x=top%c.width,y=Math.floor(top/c.width),id=(y*(c.width-1)+x)*2+half;
            if(x>=c.width-1||y>=c.height-1||c.quality[top*4+half]<=127){invalid++;continue;}if(seen[id])duplicates++;seen[id]=1;
          }
          counts.push({expected,actual:indices.length/3,invalid,duplicates,total:(c.width-1)*(c.height-1)*2});
        }
        const timings=[];
        for(const compact of skipBenchmark?[]:[false,true,false,true]){const samples=[];for(let i=0;i<9;i++){const t=await gpu.render(compact);if(i>2)samples.push(t);}const median=key=>samples.map(t=>t[key]).sort((a,b)=>a-b)[Math.floor(samples.length/2)];timings.push({compacted:compact,computeMs:median('computeMs'),renderMs:median('renderMs'),totalGpuMs:samples.map(t=>t.computeMs+t.renderMs).sort((a,b)=>a-b)[Math.floor(samples.length/2)],wallMs:median('wallMs')});}
        const productionComparison=[],timer=gl.getExtension('EXT_disjoint_timer_query_webgl2');
        if(!skipBenchmark&&timer){
          const runGl=async()=>{
            for(const c of state.cameras)c.lastUpdateMs=performance.now();
            const q=gl.createQuery();gl.beginQuery(timer.TIME_ELAPSED_EXT,q);
            rig.visibility.update(state.cameras,rig.quality,p.inverseRigidTransform,true,performance.now(),true,state.cameras.map(()=>Math.random()));
            gl.endQuery(timer.TIME_ELAPSED_EXT);gl.finish();while(!gl.getQueryParameter(q,gl.QUERY_RESULT_AVAILABLE))await new Promise(r=>setTimeout(r,5));
            const value=gl.getParameter(timer.GPU_DISJOINT_EXT)?null:gl.getQueryParameter(q,gl.QUERY_RESULT)/1e6;gl.deleteQuery(q);return value;
          };
          for(let i=0;i<10;i++){
            let webgl,webgpu;if(i%2){webgl=await runGl();webgpu=await gpu.render(true);}else{webgpu=await gpu.render(true);webgl=await runGl();}
            if(i>1)productionComparison.push({webglMs:webgl,webgpuComputeMs:webgpu.computeMs,webgpuRenderMs:webgpu.renderMs,webgpuTotalMs:webgpu.computeMs+webgpu.renderMs});
          }
        }
        return {captured,geometry,matrixMath,adapter:gpu.adapterInfo,productionParityPassed:crossApiDifferent===0,counts,depthParity:{different,crossApiDifferent,maximumDepthDifference,total:reference.length,layerStats},timings,productionComparison,glError,gpuErrors};
      }finally{gpu.destroy();}
    },{captured:Boolean(process.env.CAPTURE_PACKET),skipBenchmark:process.env.SKIP_BENCHMARK==='1',matrixMath:process.env.MATRIX_MATH||'webgl_exact',geometry:process.env.TEST_GEOMETRY||'holes'});
    if(process.env.RESULT_PATH)fs.writeFileSync(process.env.RESULT_PATH,JSON.stringify({...result,pageErrors:errors},null,2));
    console.log(JSON.stringify(result));assert.deepEqual(errors,[]);assert.deepEqual(result.gpuErrors,[]);assert.equal(result.glError,0);assert.equal(result.depthParity.different,0,'compaction preserves every WebGPU depth texel');for(const c of result.counts){assert.equal(c.expected,c.actual);assert.equal(c.invalid,0);assert.equal(c.duplicates,0);}
    if(process.env.REQUIRE_PRODUCTION_PARITY==='1')assert.equal(result.productionParityPassed,true,'production WebGL parity is required before enabling this experiment');
  }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
