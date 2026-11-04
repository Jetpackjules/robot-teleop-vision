const assert=require('node:assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
  const page=await browser.newPage();await page.goto(`http://127.0.0.1:${server.address().port}/`);
  const result=await page.evaluate(async()=>{
   await import('/hybrid_rgbd_renderer.js');const f=await import('/web_rendering_fixture.js'),{state,gl,canvas}=f.setup(),p=window.probe;
   const {SampledPointPreview}=await import('/sampled_point_preview.js');
   const {SQUARES_FUSION_SHADER}=await import('/squares_camera_fusion.js');
   const {compileGpuProgram,FULLSCREEN_VERTEX_SHADER}=await import('/hybrid_camera_quality.js');
   let checks=0;const check=(v,label)=>{checks++;if(!v)throw Error(label);};
   const n=17,now=Date.now();
   const make=(id,mm,color,missing=false)=>{
    const count=n*n,block=new ArrayBuffer(count*9),indices=new Uint32Array(block,0,count),depths=new Uint16Array(block,count*4,count),colors=new Uint8Array(block,count*6,count*3);
    for(let i=0;i<count;i++){indices[i]=i;depths[i]=mm;colors.set(color,i*3);}
    return {width:n,height:n,depth:null,previewSamples:{indices,depths,colors,valid:count,budget:0},intrinsicsArray:new Float32Array([17,17,8,8]),
     modelMatrix:p.identityMatrix(),lastUpdateMs:performance.now(),uploadRevision:1,
     metadata:{id,intrinsics:[17,17,8,8],color_space:'aligned_depth',geometry_capture_unix_ms:now,
      color_projection:{model:'pinhole',size:[17,17],intrinsics:[17,17,missing?100:8,8],extrinsics:[1,0,0,0,1,0,0,0,1,0,0,0]}}};
   };
   const b=make('B',1005,[255,0,0]),c=make('C',1000,[140,140,140],true),cameras=[b,c];
   const poses=JSON.stringify(cameras.map(c=>Array.from(c.modelMatrix))),preview=new SampledPointPreview(gl);
   const settings={...state.settings,point_preview_budget:0,point_cleanup_enabled:false,robot_overlay_enabled:false};
   const view={width:256,height:256,now:performance.now(),viewProjection:p.perspectiveMatrix(55,1,.01,10)};
   const render=enabled=>{
    for(const camera of cameras)camera.lastUpdateMs=performance.now();view.now=performance.now();
    gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.clearColor(.1,.1,.1,1);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
    const status=preview.render(cameras,view,null,{...settings,squares_fusion_enabled:enabled});
    if(enabled&&!status.fusion.active)throw Error(status.fusion.message);
    return status;
   };
   render(false);const original=f.image();render(true);
   check(gl.getError()===gl.NO_ERROR,'production square shader and integer atlases have valid GL state');
   const fused=f.image(),red=bytes=>{let count=0;for(let i=0;i<bytes.length;i+=4)if(bytes[i]>200&&bytes[i+1]<30)count++;return count;};
   check(red(fused)>red(original),'actual compact draw fills missing grey geometry with peer RGB');
   // Query the exact production GLSL against the built atlases. This avoids
   // conflating visibility from the virtual viewer with donor-camera visibility.
   const fragment=`#version 300 es\nprecision highp float;precision highp int;\nuniform vec4 u_probe_color;out vec4 result;${SQUARES_FUSION_SHADER}\nvoid main(){vec4 color=u_probe_color;bool keep=fuse_square_surface(vec3(0,0,-1),vec3(0,0,-1),color);result=keep?(color.a>0.?vec4(color.rgb,1):vec4(.55,.55,.55,1)):vec4(0,1,0,1);}`;
   const probe=compileGpuProgram(gl,FULLSCREEN_VERTEX_SHADER,fragment),vao=gl.createVertexArray();
   const query=(color=[.55,.55,.55,0])=>{
    render(true);gl.useProgram(probe);preview.fusion.bind(probe,c,performance.now());
    gl.bindVertexArray(vao);gl.disable(gl.DEPTH_TEST);gl.viewport(0,0,1,1);
    gl.uniform4fv(gl.getUniformLocation(probe,'u_probe_color'),color);gl.drawArrays(gl.TRIANGLES,0,3);
    const pixel=new Uint8Array(4);gl.readPixels(0,0,1,1,gl.RGBA,gl.UNSIGNED_BYTE,pixel);gl.bindVertexArray(null);return Array.from(pixel);
   };
   const pixel=query();check(pixel[0]>250&&pixel[1]<2,'missing colour borrows red from matching depth');
   let builds=preview.fusion.rebuilds;query();check(preview.fusion.rebuilds===builds,'unchanged captures reuse atlases');
   c.metadata.color_projection.intrinsics[2]=8;
   check(query([140/255,140/255,140/255,1]).slice(0,3).every(v=>v===140),'valid real grey is not mistaken for missing colour');
   check(query([0,0,0,1]).slice(0,3).every(v=>v===0),'valid black remains black');
   c.metadata.color_projection.intrinsics[2]=100;
   b.metadata.geometry_capture_unix_ms=now-100;
   check(query()[0]===140,'stale peer cannot recolour current geometry');b.metadata.geometry_capture_unix_ms=now;
   b.previewSamples.depths.fill(500);b.uploadRevision++;
   check(query()[0]===140,'foreground donor cannot paint a red background surface');
   b.previewSamples.depths.fill(1100);b.uploadRevision++;
   check(query()[1]===255,'dense smooth peer free space rejects an inconsistent foreground point');
   b.previewSamples.depths.fill(0);b.previewSamples.depths[8*n+8]=1100;b.uploadRevision++;
   check(query()[0]===140,'unknown sparse depth never proves free space');
   b.previewSamples.depths.fill(1005);b.previewSamples.depths[8*n+7]=500;b.uploadRevision++;
   b.metadata.color_projection.extrinsics[9]=.1;
   check(query()[0]===140,'RGB-camera foreground occlusion rejects an otherwise matching-depth donor');
   b.metadata.color_projection.extrinsics[9]=0;b.previewSamples.depths.fill(1005);b.uploadRevision++;
   builds=preview.fusion.rebuilds;b.uploadRevision++;query();check(preview.fusion.rebuilds===builds+1,'one fresh capture rebuilds only its own atlases');
   const originalB=b.modelMatrix.slice(),angle=.15,cs=Math.cos(angle),sn=Math.sin(angle);
   b.modelMatrix=new Float32Array([cs,0,-sn,0,0,1,0,0,sn,0,cs,0,.08,0,.01,1]);
   for(let y=0;y<n;y++)for(let x=0;x<n;x++)b.previewSamples.depths[y*n+x]=Math.round(1.01/(cs+sn*(x-8)/17)*1000);
   b.uploadRevision++;
   check(query()[0]>250,'calibrated rotated/translated cameras transfer colour on a sloped depth plane');
   b.modelMatrix=originalB;b.previewSamples.depths.fill(1005);b.uploadRevision++;
   const a=make('A',1100,[0,0,255]);cameras.push(a);
   check(query()[0]>250,'one disagreeing camera cannot erase a surface corroborated by B/C');cameras.pop();
   const afterFusion=preview.fusion.rebuilds;render(false);check(preview.fusion.rebuilds===afterFusion,'off bypasses all fusion work');
   const after=f.image();check(original.every((v,i)=>v===after[i]),'off restores exact original pixels');
   check(JSON.stringify(cameras.map(c=>Array.from(c.modelMatrix)))===poses,'B/C transforms remain bit-for-bit unchanged');
   check(c.previewSamples.colors.every(v=>v===140),'fusion never overwrites received colours');
   check(gl.getError()===gl.NO_ERROR,'valid GL state after toggles and occlusion tests');
   preview.dispose();return {checks};
  });
  assert.ok(result.checks>=16);console.log(JSON.stringify(result));
 }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
