const assert=require('node:assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
 try{
  const page=await browser.newPage({viewport:{width:1200,height:900}});await page.goto(`http://127.0.0.1:${server.address().port}/`);
  const result=await page.evaluate(async()=>{
   await import('/hybrid_rgbd_renderer.js');const f=await import('/web_rendering_fixture.js');const {state,gl}=f.setup(),p=window.probe;
   let checks=0;const check=(v,s)=>{checks++;if(!v)throw Error(s);};
   const names=['a','b','c','d','e','f','g'];
   state.robotRenderer={...state.robotRenderer,maskChain:{names,radii:[.11,.09,.08,.08,.07,.06]},modelUrls:{}};
   Object.assign(state.settings,{robot_overlay_enabled:true,robot_overlay_mask_scanned_robot:true});
   for(let c=0;c<3;c++){
    const w=79,h=53,depth=new Uint16Array(w*h),color=new Uint8Array(128*96*3),matrix=p.identityMatrix();matrix[12]=(c-1)*.05;
    for(let y=0;y<h;y++)for(let x=0;x<w;x++)depth[y*w+x]=(x>30&&x<34&&y<24)?0:940+x+y*2+c*3;
    for(let y=0;y<96;y++)for(let x=0;x<128;x++)color.set([(x*9)%256,(y*7)%256,80+c*40],(y*128+x)*3);
    state.cameras.push(p.uploadCamera({metadata:{id:String(c),model:'D435',intrinsics:[80,80,39,26],color_space:'original_rgb',color_width:128,color_height:96,
     color_projection:{model:'pinhole',size:[128,96],intrinsics:[128,128,63.5,47.5],extrinsics:[1,0,0,0,1,0,0,0,1,.015,0,0]}},width:w,height:h,depth,colorPixels:color,modelMatrix:matrix}));
   }
   window.setGodotHybridViewSettings({display_mode:'blendpcr'});
   const image=()=>{for(const c of state.cameras)c.lastUpdateMs=performance.now();p.render();return f.image();};
   for(let step=0;step<8;step++){
    for(let j=0;j<names.length;j++)state.robotTransforms[names[j]]={basis_x:[1,0,0],basis_y:[0,1,0],basis_z:[0,0,1],origin:[-.2+step*.045,j*.02-.07,-1.02]};
    state.useCachedRobotMask=false;const direct=image();state.useCachedRobotMask=true;const cached=image();
    check(Boolean(state.cameraFusion.maskTexture),'native mask cache active');
    let max=0;for(let i=0;i<direct.length;i++)max=Math.max(max,Math.abs(direct[i]-cached[i]));check(max<=1,`mask pose ${step} preserves pixels (${max})`);
    const builds=state.robotMaskCache.rebuilds;state.baseViewer.transform[12]+=.01;image();
    check(state.robotMaskCache.rebuilds===builds,'viewpoint motion reuses native mask flags');state.baseViewer.transform[12]-=.01;
   }
   const builds=state.robotMaskCache.rebuilds;state.cameras[1].uploadRevision++;image();
   check(state.robotMaskCache.rebuilds===builds+1,'partial camera updates rebuild only that mask layer');
   const mask=state.cameraFusion.maskTexture;state.settings.robot_overlay_mask_scanned_robot=false;image();
   check(!state.cameraFusion.maskTexture,'mask off uses the original unmasked material');
   state.settings.robot_overlay_mask_scanned_robot=true;image();check(state.cameraFusion.maskTexture===mask,'mask on restores the measured cache');
   for(const shift of [0,.2,-.4]){
    state.baseViewer.transform[12]=shift;
    state.cameraFusion.visibility.tiled=false;state.cameraFusion.clear();state.blendPCR.useViewCulling=false;state.blendPCR.cacheKey='';const original=image();
    state.cameraFusion.visibility.tiled=true;state.cameraFusion.clear();state.blendPCR.useViewCulling=true;state.blendPCR.cacheKey='';const culled=image();
    let max=0;for(let i=0;i<original.length;i++)max=Math.max(max,Math.abs(original[i]-culled[i]));
    check(max<=1,`native visibility and view culling retain masked pixels (${max})`);
   }
   check(gl.getError()===gl.NO_ERROR,'cached masks retain valid GL state');return {checks};
  });
  assert.ok(result.checks>=28);console.log(JSON.stringify(result));
 }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
