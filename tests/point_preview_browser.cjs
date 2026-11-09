const assert=require('node:assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
 try{
  const page=await browser.newPage();await page.goto(`http://127.0.0.1:${server.address().port}/`);
  const result=await page.evaluate(async()=>{
   await import('/hybrid_rgbd_renderer.js');const f=await import('/web_rendering_fixture.js'),{state,gl}=f.setup(),p=window.probe;
   const w=256,h=192;let checks=0;const check=(v,s)=>{checks++;if(!v)throw Error(s);};
   for(let c=0;c<3;c++){
    const depth=new Uint16Array(w*h),rgb=new Uint8Array(w*h*3),m=p.identityMatrix();m[12]=(c-1)*.03;
    for(let i=0;i<depth.length;i++){depth[i]=i%17?1000+i%31:0;rgb.set([60+c*60,i%250,120],i*3);}
    state.cameras.push(p.uploadCamera({metadata:{id:String(c),intrinsics:[250,250,127.5,95.5],color_space:'original_rgb',color_width:w,color_height:h,
     color_projection:{model:'pinhole',size:[w,h],intrinsics:[250,250,127.5,95.5],extrinsics:[1,0,0,0,1,0,0,0,1,0,0,0]}},width:w,height:h,depth,colorPixels:rgb,modelMatrix:m}));
   }
   const depths=state.cameras.map(c=>c.depth.slice()),poses=JSON.stringify(state.cameras.map(c=>Array.from(c.modelMatrix)));
   const image=()=>{for(const c of state.cameras)c.lastUpdateMs=performance.now();p.render();return f.image();};
   window.setGodotHybridViewSettings({display_mode:'blendpcr',point_preview_enabled:false});const before=image();
   const update=state.cameraFusion.update;state.cameraFusion.update=()=>{throw Error('dense fusion ran during sparse preview');};
   window.setGodotHybridViewSettings({point_preview_enabled:true});const sparse=image();
   check(state.stats.renderMethod==='sampled_points','sparse renderer selected');
   check(state.stats.pointPreview.total===60000,'three cameras each submit 20000 points');
   check(Object.values(state.stats.pointPreview.counts).every(n=>n===20000),'per-camera cap');
   check(!state.stats.renderWarning,'preview compiled without fallback');
   window.setGodotHybridViewSettings({point_preview_budget:40000});image();check(state.stats.pointPreview.total===120000,'40k per camera cap');
   window.setGodotHybridViewSettings({point_preview_budget:0});image();
   check(state.stats.pointPreview.total===Object.values(state.stats.pointPreview.sourceCounts).reduce((a,b)=>a+b,0),'All draws every valid point using the square sprite renderer');
   check(state.stats.pointPreview.total>120000,'All exceeds the previous 40k per-camera cap');
   window.setGodotHybridViewSettings({point_preview_budget:20000});image();
   window.setGodotHybridViewSettings({point_preview_budget:20000});image();check(state.stats.pointPreview.total===60000,'20k restored');
   check(sparse.some((v,i)=>i%4!==3&&v>50),'preview paints coloured samples');
   state.rawRgbViewer={status:()=>({displaySource:'original_rgb'})};
   check(window.getGodotHybridRendererStatus().pointPreview===null,'RGB view cannot show stale sparse-preview status');
   state.rawRgbViewer=null;
   state.cameraFusion.update=update;window.setGodotHybridViewSettings({point_preview_enabled:false});const after=image();
   check(before.every((v,i)=>v===after[i]),'turning off restores exact full-detail pixels');
   check(state.cameras.every((c,i)=>c.depth.every((v,j)=>v===depths[i][j])),'source depths untouched');
   check(JSON.stringify(state.cameras.map(c=>Array.from(c.modelMatrix)))===poses,'camera calibration untouched');
   const {NativeRgbdDecoderPool}=await import('/rgbd_decode_pool.js');const pool=new NativeRgbdDecoderPool();
   try{
    const depth=depths[0],packed=await new Response(new Blob([depth]).stream().pipeThrough(new CompressionStream('deflate'))).arrayBuffer();
    const canvas=document.createElement('canvas');canvas.width=w;canvas.height=h;canvas.getContext('2d').fillRect(0,0,w,h);
    const jpg=await new Promise(r=>canvas.toBlob(r,'image/jpeg'));
    const decoded=await pool.decode({metadata:{width:w,height:h,depth_encoding:'u16_mm_deflate'},color:new Uint8Array(await jpg.arrayBuffer()),depth:new Uint8Array(packed),pointPreview:true});
    check(decoded.previewSamples.indices.length===20000,'worker transfers capped preview indices');
    check(decoded.depth.every((v,i)=>v===depth[i]),'worker preserves complete decoded depth');decoded.bitmap?.close();
    const {sampleDepthPoints}=await import('/point_preview_sampling.js');
    const chosen=sampleDepthPoints(depth,20000),count=chosen.indices.length,raw=new ArrayBuffer(count*9),view=new DataView(raw);
    chosen.indices.forEach((index,i)=>{view.setUint32(i*4,index,true);view.setUint16(count*4+i*2,depth[index],true);new Uint8Array(raw).set([index%256,index*7%256,index*13%256],count*6+i*3);});
    const compact=await new Response(new Blob([raw]).stream().pipeThrough(new CompressionStream('deflate'))).arrayBuffer();
    const metadata={id:'sparse-wire',width:w,height:h,intrinsics:[250,250,127.5,95.5],transform:Array.from(p.identityMatrix()),depth_encoding:'u16_mm_sparse_points_deflate',color_space:'aligned_depth',preview_budget:20000,preview_count:count,preview_source_points:chosen.valid};
    const points=await pool.decode({metadata,color:new Uint8Array(0),depth:new Uint8Array(compact)});
    check(points.previewSamples.indices.every((v,i)=>v===chosen.indices[i]),'compact worker preserves host-selected indices');
    check(points.depth===null&&points.rgbaPixels===null,'compact worker never rebuilds full images');
    check(points.previewSamples.depths.every((z,i)=>z===depth[chosen.indices[i]]),'compact worker preserves selected depths');
    check(points.previewSamples.indices.buffer.byteLength===count*9,'worker transfers only nine bytes per selected point');
    check(points.validPoints===chosen.valid,'compact worker retains full source count');
    const checkPredictor=async(block,meta)=>{
      const predicted=block.slice(0),n=meta.preview_count,ix=new Uint32Array(predicted,0,n),dz=new Uint16Array(predicted,n*4,n),rgb=new Uint8Array(predicted,n*6,n*3);
      for(let i=n-1;i>0;i--){ix[i]-=ix[i-1];dz[i]-=dz[i-1];}
      for(let i=rgb.length-1;i>=3;i--)rgb[i]-=rgb[i-3];
      const wire=await new Response(new Blob([predicted]).stream().pipeThrough(new CompressionStream('deflate'))).arrayBuffer();
      const restored=await pool.decode({metadata:{...meta,preview_predictor:'delta_v1'},color:new Uint8Array(0),depth:new Uint8Array(wire)});
      check(new Uint8Array(restored.previewSamples.indices.buffer).every((v,i)=>v===new Uint8Array(block)[i]),`${n} lossless predictor restores every GPU byte`);
      return restored;
    };
    await checkPredictor(raw,metadata);
    const camerasBefore=state.cameras,flat=new Uint8Array(w*h*3);for(let i=0;i<w*h;i++)flat.set([i%256,i*7%256,i*13%256],i*3);
    state.cameras=[p.uploadCamera({metadata:{...metadata,depth_encoding:'u16_mm_deflate'},width:w,height:h,depth,colorPixels:flat,modelMatrix:p.identityMatrix()})];
    window.setGodotHybridViewSettings({point_preview_enabled:true,point_preview_budget:20000});const localPoints=image();
    const textureUpload=gl.texSubImage2D,textureAllocation=gl.texStorage2D;
    let hostPoints;
    try{
      gl.texSubImage2D=gl.texStorage2D=()=>{throw Error('compact preview attempted a full image texture operation');};
      state.cameras=[p.uploadCamera({metadata,width:w,height:h,depth:points.depth,previewSamples:points.previewSamples,validPoints:points.validPoints,colorRgbaPixels:points.rgbaPixels,modelMatrix:p.identityMatrix()})];
      hostPoints=image();
    }finally{gl.texSubImage2D=textureUpload;gl.texStorage2D=textureAllocation;}
    check(localPoints.every((v,i)=>v===hostPoints[i]),'compact GPU samples match dense-texture rendering exactly');
    check(state.stats.pointPreview.gpuUploadBytes===count*9,'GPU receives only the compact point block');
    const {cameraDepthSamples}=await import('/point_preview_sampling.js');
    const chosenSet=new Set(chosen.indices),reference=[];for(let y=0;y<h;y+=3)for(let x=0;x<w;x+=3){const index=y*w+x;if(chosenSet.has(index))reference.push([x,y,depth[index]]);}
    check(JSON.stringify([...cameraDepthSamples(state.cameras[0],3)])===JSON.stringify(reference),'compact picking preserves original grid samples and order');
    window.setGodotHybridViewSettings({point_preview_budget:40000});image();check(state.stats.pointPreview.total===count,'budget increase tolerates an older compact packet');
    window.setGodotHybridViewSettings({point_preview_budget:20000});image();
    const chosen40=sampleDepthPoints(depth,40000),n40=chosen40.indices.length,block40=new ArrayBuffer(n40*9),v40=new DataView(block40);
    chosen40.indices.forEach((index,i)=>{v40.setUint32(i*4,index,true);v40.setUint16(n40*4+i*2,depth[index],true);new Uint8Array(block40).set([index%256,index*7%256,index*13%256],n40*6+i*3);});
    const compressed40=await new Response(new Blob([block40]).stream().pipeThrough(new CompressionStream('deflate'))).arrayBuffer();
    const meta40={...metadata,id:'sparse-40',preview_budget:40000,preview_count:n40};
    const points40=await pool.decode({metadata:meta40,color:new Uint8Array(0),depth:new Uint8Array(compressed40)});
    await checkPredictor(block40,meta40);
    window.setGodotHybridViewSettings({point_preview_budget:40000});
    state.cameras=[p.uploadCamera({metadata:{...meta40,depth_encoding:'u16_mm_deflate'},width:w,height:h,depth,colorPixels:flat,modelMatrix:p.identityMatrix()})];const dense40=image();
    state.cameras=[p.uploadCamera({metadata:meta40,width:w,height:h,depth:null,previewSamples:points40.previewSamples,validPoints:points40.validPoints,modelMatrix:p.identityMatrix()})];const compact40=image();
    check(dense40.every((v,i)=>v===compact40[i]),'40k compact pixels match the original texture renderer');
    check(state.stats.pointPreview.gpuUploadBytes===40000*9,'40k GPU upload is 360000 bytes');
    window.setGodotHybridViewSettings({point_preview_budget:20000});image();check(state.stats.pointPreview.total===20000,'lower budget safely subsets an older 40k packet');
    window.setGodotHybridViewSettings({point_preview_enabled:false});check(state.cameras.length===0,'full-detail switch discards compact-only camera textures');
    state.cameras=[p.uploadCamera({metadata:{...meta40,depth_encoding:'u16_mm_deflate'},width:w,height:h,depth,colorPixels:flat,modelMatrix:p.identityMatrix()})];image();check(!state.stats.renderWarning,'full-detail GPU uploads and rendering recover after compact mode');
    state.cameras=camerasBefore;window.setGodotHybridViewSettings({point_preview_enabled:false});
   }finally{pool.close();}
   const savedCameras=state.cameras;
   const d=new Uint16Array(16);d[0]=d[1]=d[4]=1000;
   state.cameras=[p.uploadCamera({metadata:{id:'filter-probe',intrinsics:[100,100,1,1]},width:4,height:4,depth:d,colorPixels:new Uint8Array(48),modelMatrix:p.identityMatrix()})];
   const quality=()=>{image();const fb=gl.createFramebuffer(),v=new Uint8Array(4);gl.bindFramebuffer(gl.FRAMEBUFFER,fb);gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,state.cameraFusion.quality,0,0);gl.readPixels(0,0,1,1,gl.RGBA,gl.UNSIGNED_BYTE,v);gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.deleteFramebuffer(fb);return v[0];};
   window.setGodotHybridViewSettings({blendpcr_relaxed:false});check(quality()===0,'standard rejects unsupported triangle');
   window.setGodotHybridViewSettings({blendpcr_relaxed:true});check(quality()>0,'relaxed retains measured triangle with one supporting neighbour');
   window.setGodotHybridViewSettings({blendpcr_relaxed:false});check(quality()===0,'standard filtering restored');
   window.setGodotHybridViewSettings({blendpcr_support:1});check(quality()>0,'support slider retains triangle');
   window.setGodotHybridViewSettings({blendpcr_support:2});check(quality()===0,'support slider restores rejection');
   const discontinuity=d.slice();discontinuity[1]=1040;
   state.cameras=[p.uploadCamera({metadata:{id:'tolerance-probe',intrinsics:[100,100,1,1]},width:4,height:4,depth:discontinuity,colorPixels:new Uint8Array(48),modelMatrix:p.identityMatrix()})];
   window.setGodotHybridViewSettings({blendpcr_support:0,blendpcr_depth_tolerance:1,blendpcr_max_edge:.08});check(quality()===0,'default tolerance rejects depth discontinuity');
   window.setGodotHybridViewSettings({blendpcr_depth_tolerance:4});check(quality()>0,'tolerance slider changes adaptive depth limit');
   window.setGodotHybridViewSettings({blendpcr_max_edge:.02});check(quality()===0,'edge slider limits long triangle');
   window.setGodotHybridViewSettings({blendpcr_max_edge:.08});check(quality()>0,'edge slider restores accepted triangle');
   const wide=discontinuity.slice();wide[1]=1300;
   state.cameras=[p.uploadCamera({metadata:{id:'wide-tolerance',intrinsics:[100,100,1,1]},width:4,height:4,depth:wide,colorPixels:new Uint8Array(48),modelMatrix:p.identityMatrix()})];
   window.setGodotHybridViewSettings({blendpcr_depth_tolerance:4,blendpcr_max_edge:1});check(quality()===0,'4x still rejects large discontinuity');
   window.setGodotHybridViewSettings({blendpcr_depth_tolerance:100});check(quality()>0,'100x and one-metre edge settings reach the shader');
   state.cameras=savedCameras;
   check(gl.getError()===gl.NO_ERROR,'valid GL state after preview round trip');
   return {checks,pointsPerCamera:20000,cameras:3,totalPoints:60000};
  });
  assert(result.checks>=11);console.log(JSON.stringify(result));
 }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
