const assert=require('node:assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));const browser=await chromium.launch({channel:'msedge',headless:true});
 try{
  const page=await browser.newPage();await page.goto(`http://127.0.0.1:${server.address().port}/`);
  const result=await page.evaluate(async()=>{
   await import('/hybrid_rgbd_renderer.js');const f=await import('/web_rendering_fixture.js'),{state,gl}=f.setup(),p=window.probe;
   const {SampledPointPreview}=await import('/sampled_point_preview.js');
   const {SQUARES_FUSION_SHADER}=await import('/squares_camera_fusion.js');
   const {compileGpuProgram,FULLSCREEN_VERTEX_SHADER}=await import('/hybrid_camera_quality.js');
   let checks=0;const check=(v,label)=>{checks++;if(!v)throw Error(label);};
   const n=65,now=Date.now();
   const make=(id,mm)=>{
    const count=n*n,block=new ArrayBuffer(count*9),indices=new Uint32Array(block,0,count),depths=new Uint16Array(block,count*4,count),colors=new Uint8Array(block,count*6,count*3);
    for(let i=0;i<count;i++){indices[i]=i;depths[i]=mm;colors.set([255,0,0],i*3);}
    return {width:n,height:n,depth:null,previewSamples:{indices,depths,colors,valid:count,budget:0},intrinsicsArray:new Float32Array([500,500,32,32]),
     modelMatrix:p.identityMatrix(),lastUpdateMs:performance.now(),uploadRevision:1,
     metadata:{id,geometry_capture_unix_ms:now,color_space:'aligned_depth',color_projection:{model:'pinhole',size:[n,n],intrinsics:[500,500,32,32],extrinsics:[1,0,0,0,1,0,0,0,1,0,0,0]}}};
   };
   const a=make('262522074294',988),b=make('262522075787',1000),c=make('realsense:262522076048',1000),cameras=[c,a,b];
   const poses=JSON.stringify(cameras.map(c=>Array.from(c.modelMatrix))),depthBefore=Array.from(b.previewSamples.depths);
   const preview=new SampledPointPreview(gl),settings={...state.settings,point_preview_budget:0,point_cleanup_enabled:false,robot_overlay_enabled:false,squares_fusion_enabled:true};
   const view={width:256,height:256,now:performance.now(),viewProjection:p.perspectiveMatrix(55,1,.01,10)};
   const fragment=`#version 300 es\nprecision highp float;precision highp int;\nuniform vec3 u_world,u_local;out vec4 result;${SQUARES_FUSION_SHADER}\nvoid main(){result=square_keep_supplement(u_world,u_local)?vec4(1,0,0,1):vec4(0,1,0,1);}`;
   const program=compileGpuProgram(gl,FULLSCREEN_VERTEX_SHADER,fragment),vao=gl.createVertexArray();
   const query=(source=a,world=[0,0,-.988],local=world)=>{
    view.now=performance.now();for(const camera of cameras)camera.lastUpdateMs=view.now;
    const status=preview.render(cameras,view,null,settings);if(!status.fusion.active)throw Error(status.fusion.message);
    for(const camera of cameras)camera.lastUpdateMs=performance.now();
    gl.useProgram(program);preview.fusion.bind(program,source,performance.now());gl.bindVertexArray(vao);gl.disable(gl.DEPTH_TEST);gl.viewport(0,0,1,1);
    gl.uniform3fv(gl.getUniformLocation(program,'u_world'),world);gl.uniform3fv(gl.getUniformLocation(program,'u_local'),local);gl.drawArrays(gl.TRIANGLES,0,3);
    const pixel=new Uint8Array(4);gl.readPixels(0,0,1,1,gl.RGBA,gl.UNSIGNED_BYTE,pixel);gl.bindVertexArray(null);return pixel[0]===255;
   };
   check(!query(),'A duplicate layer in front of B/C surface rejected');
   a.previewSamples.depths.fill(1012);a.uploadRevision++;
   check(!query(a,[0,0,-1.012]),'A duplicate layer behind B/C surface rejected too');
   check(query(b,[0,0,-1]),'B surface is never demoted');check(query(c,[0,0,-1]),'C surface is never demoted');
   for(let y=0;y<n;y++)for(let x=0;x<n;x++)if(x<32){b.previewSamples.depths[y*n+x]=0;c.previewSamples.depths[y*n+x]=0;}
   b.uploadRevision++;c.uploadRevision++;
   check(!query(a,[0,0,-1.012]),'incomplete central-difference stencils still reject duplicate edge');
   b.previewSamples.depths.fill(1000);c.previewSamples.depths.fill(1000);b.uploadRevision++;c.uploadRevision++;
   a.previewSamples.depths.fill(1002);a.uploadRevision++;
   check(!query(a,[0,0,-1.002]),'already covered aligned A geometry omitted to avoid duplicate edges');
   check(query(a,[.2,0,-1.012]),'A-only coverage outside B/C retained');
   c.metadata.geometry_capture_unix_ms=now-100;
   check(query(a,[0,0,-1.012]),'stale C cannot establish B/C consensus');c.metadata.geometry_capture_unix_ms=now;
   c.previewSamples.depths.fill(1020);c.uploadRevision++;
   check(query(a,[0,0,-1.012]),'B/C disagreement does not erase A');c.previewSamples.depths.fill(1000);c.uploadRevision++;
   c.previewSamples.depths.fill(0);c.uploadRevision++;
   check(query(a,[0,0,-1.012]),'missing reference depths retain A');c.previewSamples.depths.fill(1000);c.uploadRevision++;
   cameras.splice(cameras.indexOf(c),1);
   check(query(a,[0,0,-1.012]),'disabled reference camera bypasses supplementary filter');cameras.unshift(c);
   // Genuine perpendicular face x=.02: B/C observe z=1, A observes its side.
   for(let y=0;y<n;y++)for(let x=0;x<n;x++)a.previewSamples.depths[y*n+x]=x>32?Math.round(.02*500/(x-32)*1000):0;
   a.uploadRevision++;
   check(query(a,[.02,0,-1.012]),'genuine perpendicular A face retained');
   // Distinct camera pose: same world plane with local z=1.02 and translation.
   b.modelMatrix[12]=.01;b.modelMatrix[14]=.02;b.previewSamples.depths.fill(1020);b.uploadRevision++;
   a.previewSamples.depths.fill(1012);a.uploadRevision++;
   check(!query(a,[0,0,-1.012]),'translated reference uses world-plane distances');
   const angle=.15,cs=Math.cos(angle),sn=Math.sin(angle);
   b.modelMatrix=new Float32Array([cs,0,-sn,0,0,1,0,0,sn,0,cs,0,1.01*Math.tan(angle),0,.01,1]);
   for(let y=0;y<n;y++)for(let x=0;x<n;x++)b.previewSamples.depths[y*n+x]=Math.round(1.01/(cs+sn*(x-32)/500)*1000);
   b.uploadRevision++;
   check(!query(a,[0,0,-1.012]),'rotated reference normals agree in world coordinates');
   b.modelMatrix=p.identityMatrix();b.previewSamples.depths.fill(1000);b.uploadRevision++;
   c.modelMatrix=new Float32Array([0,0,-1,0,0,1,0,0,1,0,0,0,.5,0,-1,1]);c.previewSamples.depths.fill(480);c.uploadRevision++;
   check(!query(a,[.012,0,-1.012]),'different B/C cube faces suppress the duplicate A corner lip');
   c.modelMatrix=p.identityMatrix();c.previewSamples.depths.fill(1000);c.uploadRevision++;
   check(JSON.stringify(cameras.map(c=>Array.from(c.modelMatrix)))===poses,'camera poses unchanged by filtering');
   check(b.previewSamples.depths.every((v,i)=>v===depthBefore[i]),'raw B depth samples unchanged');
   check(a.previewSamples.colors.every((v,i)=>v===[255,0,0][i%3]),'raw A RGB remains usable as donor');
   check(gl.getError()===gl.NO_ERROR,'no GL errors');preview.dispose();return {checks};
  });assert.ok(result.checks>=16);console.log(JSON.stringify(result));
 }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
