const assert=require('node:assert/strict');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {createTestServer}=require('./serve_web_rendering.cjs');
const server=createTestServer();
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
 try {
  const page=await browser.newPage();await page.goto(`http://127.0.0.1:${server.address().port}/`);
  await page.waitForFunction(()=>window.probe);
  const result=await page.evaluate(async()=>{
   const sockets=[],checks=[];window.robotTeleopPageId='mode-isolation';
   const check=(v,t)=>{if(!v)throw Error(t);checks.push(t);};
   class Socket{static OPEN=1;static CONNECTING=0;constructor(url){this.url=url;this.readyState=1;sockets.push(this);}close(){this.readyState=3;}send(){}}
   window.WebSocket=Socket;
   const rgb=document.createElement('canvas');document.body.append(rgb);
   window.setGodotHybridViewSettings({display_mode:'rgb_camera'});
   await window.startGodotHybridRenderer({canvas:'view',rgbDisplayCanvas:rgb,camera:'fixture',transport:'websocket'});
   check(sockets.length===1&&sockets[0].url.includes('/camera-preview-stream?'),'RGB mode opens only one compressed camera stream');
   const camera=sockets[0];window.setGodotHybridViewSettings({display_mode:'point_cloud'});
   check(camera.readyState===3,'3D mode closes separate RGB stream');
   check(sockets.at(-1).url.includes('/rgbd-stream?'),'3D mode resumes RGB-D transport');
   const depth=sockets.at(-1);window.setGodotHybridViewSettings({display_mode:'rgb_camera'});
   check(depth.readyState===3,'RGB mode stops hidden RGB-D transport');
   check(sockets.at(-1).url.includes('/camera-preview-stream?'),'return to RGB opens the selected camera');
   window.stopGodotHybridRenderer();
   check(sockets.every(s=>s.readyState===3),'stopping releases every preview connection');
   return {checks};
  });
  assert.equal(result.checks.length,6);console.log(JSON.stringify({modeIsolationChecks:result.checks.length}));
 }finally{await browser.close();await new Promise(r=>server.close(r));}
})().catch(e=>{console.error(e);process.exitCode=1;server.close();});
