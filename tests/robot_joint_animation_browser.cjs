// Replay recorded live RGB-D packets through the production decoder and renderer.
// This does not connect to robot control or issue any robot commands.
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
const directory=path.resolve(process.argv[2]||'.teleop/joint-animation-sync-20261027/live-packets');
const server=createTestServer(),normal=server.listeners('request')[0];server.removeAllListeners('request');
const names=JSON.parse(fs.readFileSync(path.join(directory,'index.json'),'utf8')).slice(0,24);
server.on('request',(req,res)=>{
  const name=new URL(req.url,'http://localhost').pathname.slice(6);
  if(req.url==='/live/index.json'){res.setHeader('Content-Type','application/json');res.end(JSON.stringify(names));}
  else if(req.url.startsWith('/live/')&&/^\d+\.rgbd$/.test(name)){res.end(fs.readFileSync(path.join(directory,name)));}
  else normal(req,res);
});
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
  try{
    const page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[];
    page.on('pageerror',error=>errors.push(error.message));
    await page.goto(`http://127.0.0.1:${server.address().port}/`);
    const result=await page.evaluate(async()=>{
      await import('/hybrid_rgbd_renderer.js');const fixture=await import('/web_rendering_fixture.js');
      const {state,gl}=fixture.setup(),p=window.probe;
      state.robotRenderer=await(await fetch('/so101-renderer.json')).json();
      state.robotProgram=p.createProgram(gl,p.ROBOT_VERTEX_SHADER,p.ROBOT_FRAGMENT_SHADER);
      state.robotUniforms=p.findRobotUniforms(gl,state.robotProgram);state.robotVao=gl.createVertexArray();
      await p.loadRobotMeshes(gl,state.robotLoadToken);
      Object.assign(state.settings,{robot_overlay_enabled:true,robot_overlay_scope:'claw',robot_overlay_style:'realistic',display_mode:'points'});
      state.deferCameraUploads=false;state.socketOptions={exactDepth:true};
      const names=await(await fetch('/live/index.json')).json(),results=[];
      for(const name of names){
        const buffer=await(await fetch('/live/'+name)).arrayBuffer(),parsed=p.parsePacket(buffer);
        parsed.receivedAtMs=performance.now();state.pendingFrameQueue.push(parsed);
        await p.decodeNewestPackets();p.render();
        const status=window.getGodotHybridRendererStatus(),sync=status.robotJointAnimation;
        if(status.decodeErrors)throw Error(status.status);
        if(!sync||!sync.samples)throw Error('Measured joints were not used by the renderer');
        const displayed=Math.max(...state.cameras.map(c=>c.metadata.geometry_capture_unix_ms));
        if(sync.captureUnixMs!==displayed)throw Error('Animation clock differs from displayed geometry');
        if(sync.captureUnixMs===Date.now())throw Error('Arrival time was used instead of capture time');
        const expected=parsed.metadata.robot.kinematics.base_transform,actual=state.robotTransforms.base_link;
        if(JSON.stringify(expected)!==JSON.stringify(actual))throw Error('Base alignment changed');
        results.push({mode:sync.mode,errorMs:sync.errorMs,captureUnixMs:sync.captureUnixMs});
      }
      const keys=Object.keys(state.robotTransforms),synchronized=results.filter(r=>r.mode==='interpolated'||r.mode==='measured').length;
      const base=JSON.stringify(state.robotTransforms.base_link);
      window.setGodotRobotPreviewActive(true);p.render();
      if(JSON.stringify(state.robotTransforms.base_link)!==base)throw Error('Opening manual preview changed the base');
      window.setGodotRobotPreviewActive(false);
      const error=gl.getError();if(error)throw Error('WebGL error '+error);
      return {frames:results.length,synchronized,held:results.length-synchronized,models:state.stats.robotModelsLoaded,links:keys.length,
        cameraCount:state.cameras.length,baseUnchanged:true,captureClockMatched:true,maxHoldErrorMs:Math.max(...results.map(r=>r.errorMs))};
    });
    assert.deepEqual(errors,[]);assert(result.frames>0&&result.models===7&&result.links===7);
    console.log(JSON.stringify(result));
  }finally{await browser.close();server.close();}
})().catch(error=>{console.error(error);server.close();process.exit(1);});
