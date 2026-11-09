// Real GPU parity for native point colors and conservative view culling.
const assert=require('node:assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
  await new Promise(r=>server.listen(0,'127.0.0.1',r));
  const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
  try{
    const page=await browser.newPage({viewport:{width:1200,height:900}});
    await page.goto(`http://127.0.0.1:${server.address().port}/`);
    const result=await page.evaluate(async()=>{
      await import('/hybrid_rgbd_renderer.js');const fixture=await import('/web_rendering_fixture.js');
      const {state,gl}=fixture.setup(),probe=window.probe;
      const {visibleNativeTiles}=await import('/native_geometry.js');
      let checks=0;const check=(ok,label)=>{checks++;if(!ok)throw new Error(label);};
      const width=79,height=53;
      for(let i=0;i<3;i++){
        const depth=new Uint16Array(width*height),color=new Uint8Array(192*128*3);
        for(let y=0;y<height;y++)for(let x=0;x<width;x++)depth[y*width+x]=x<4||x>74||y>47||x>28&&x<34&&y<30?0:920+x*2+y+i*4;
        for(let y=0;y<128;y++)for(let x=0;x<192;x++)color.set([(x*7+i*53)%256,(y*11)%256,(x+y)%13?180:0],(y*192+x)*3);
        const matrix=probe.identityMatrix();matrix[12]=(i-1)*.12;
        state.cameras.push(probe.uploadCamera({metadata:{id:String(i),model:'D435',intrinsics:[80,80,39,26],color_space:'original_rgb',color_width:192,color_height:128,
          color_projection:{model:'pinhole',size:[192,128],intrinsics:[180,180,95.5,63.5],extrinsics:[1,0,0,0,1,0,0,0,1,.015,0,0]}},
          width,height,depth,colorPixels:color,modelMatrix:matrix}));
      }
      const image=()=>{for(const c of state.cameras)c.lastUpdateMs=performance.now();probe.render();return fixture.image();};
      const compare=(a,b,label)=>{
        let maximum=0;for(let i=0;i<a.length;i++)maximum=Math.max(maximum,Math.abs(a[i]-b[i]));
        check(maximum<=1,`${label}: maximum channel difference ${maximum}`);
      };
      window.setGodotHybridViewSettings({display_mode:'point_cloud'});
      for(const x of [0,.2,-.35,.7]){
        state.baseViewer.transform[12]=x;
        state.usePointColorCache=false;state.useNativeTileCulling=false;const direct=image();
        state.usePointColorCache=true;const cached=image();check(Boolean(state.pointColors),state.stats.renderWarning||'point color cache created');compare(cached,direct,'cached native RGB at viewpoint '+x);
        const builds=state.pointColors.rebuilds;state.baseViewer.transform[13]+=.02;image();
        check(state.pointColors.rebuilds===builds,'viewpoint movement reuses exact point colors');
        state.baseViewer.transform[13]-=.02;state.useNativeTileCulling=true;
        compare(image(),cached,'conservative culling at viewpoint '+x);
      }
      const before=state.pointColors.rebuilds;state.cameras[1].uploadRevision++;image();
      check(state.pointColors.rebuilds===before+3,'one changed peer invalidates colors throughout the rig');
      state.cameras[0].metadata.color_projection.extrinsics[9]+=.01;
      const calibrationBuilds=state.pointColors.rebuilds;image();
      check(state.pointColors.rebuilds===calibrationBuilds+3,'optical calibration invalidates cached colors');
      state.cameraFusion.clear();const clearBuilds=state.pointColors.rebuilds;image();
      check(state.pointColors.rebuilds===clearBuilds+3,'rig cache reallocation invalidates cached colors');
      state.settings.depth_only_enabled=true;const grayBuilds=state.pointColors.rebuilds;image();
      check(state.pointColors.rebuilds===grayBuilds,'depth-only viewing skips RGB point shading');
      state.settings.depth_only_enabled=false;image();
      check(state.pointColors.rebuilds===grayBuilds+3,'returning to RGB refreshes the point cache');
      for(const mode of ['independent_mesh','unified_mesh','blendpcr']){
        window.setGodotHybridViewSettings({display_mode:mode});state.useNativeTileCulling=false;const full=image();
        state.useNativeTileCulling=true;compare(image(),full,'native mesh culling '+mode);
      }
      const p=probe.perspectiveMatrix(60,1,.01,10),camera=state.cameras[0];
      for(const shift of [0,.1,-.8,1.5,-3]){
        const view=new Float32Array(p);view[12]=shift;
        const tiles=visibleNativeTiles(camera,view,null,[.125,.125]);
        check(visibleNativeTiles(camera,view,null,[.125,.125])===tiles,'unchanged viewpoint reuses culling result');
        for(let y=0;y<height;y+=3)for(let x=0;x<width;x+=3){
          const z=camera.depth[y*width+x]*.001;if(!z)continue;
          const local=[(x-39)*z/80,-(y-26)*z/80,-z],m=camera.modelMatrix;
          const w=[m[0]*local[0]+m[4]*local[1]+m[8]*local[2]+m[12],m[1]*local[0]+m[5]*local[1]+m[9]*local[2]+m[13],m[2]*local[0]+m[6]*local[1]+m[10]*local[2]+m[14]];
          const clip=[0,1,2,3].map(r=>view[r]*w[0]+view[4+r]*w[1]+view[8+r]*w[2]+view[12+r]);
          if(clip[3]<=0||Math.abs(clip[0])>clip[3]*1.125||Math.abs(clip[1])>clip[3]*1.125||Math.abs(clip[2])>clip[3])continue;
          let retained=tiles===null;
          for(let i=0;tiles&&i<tiles.length;i+=2)if(x>=tiles[i]&&x<tiles[i]+32&&y>=tiles[i+1]&&y<tiles[i+1]+8)retained=true;
          check(retained,'visible native point including sprite margin survives tile rejection');
        }
      }
      const empty={...camera,depth:new Uint16Array(width*height),depthTiles:null,uploadRevision:1};
      check(visibleNativeTiles(empty,p).length===0,'empty depth tiles submit no geometry');
      check(gl.getError()===gl.NO_ERROR,'valid WebGL state after cache and culling transitions');
      return {checks};
    });
    assert.ok(result.checks>100);console.log(JSON.stringify(result));
  }finally{await browser.close();server.close();}
})().catch(error=>{console.error(error);server.close();process.exitCode=1;});
