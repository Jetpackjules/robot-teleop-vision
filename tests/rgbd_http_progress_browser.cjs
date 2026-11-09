// Exercise browser fetch streams with real, delayed HTTP chunks; no robot connection.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),http=require('node:http');
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const root=path.resolve(__dirname,'..');
const strip=s=>s.replace(/^import .*;\r?\n/gm,'').replace(/^export /gm,'');
const code=`const FUSION_FRAGMENT_SHADER='',COLOR_PROJECTION_SHADER='',DEPTH_SHADING_SHADER='',SCENE_LIGHTING_SHADER='',MODELED_WORKSPACE_SHADER='',MODELED_WORKSPACE_DEFAULTS={};
const RobotPosePreview=(()=>{${strip(fs.readFileSync(path.join(root,'web/robot_pose_preview.js'),'utf8'))};return RobotPosePreview;})();
const MeasuredJointAnimation=(()=>{${strip(fs.readFileSync(path.join(root,'web/robot_joint_animation.js'),'utf8'))};return MeasuredJointAnimation;})();
${strip(fs.readFileSync(path.join(root,'web/hybrid_rgbd_renderer.js'),'utf8'))}
window.readPacketTest=readRgbdHttpPacket;`;
const server=http.createServer((req,res)=>{
  if(req.url==='/'){res.end('<!doctype html><title>Isolated transport test</title>');return;}
  res.writeHead(200,{'Content-Type':'application/octet-stream'});res.write(Buffer.from([1,2]));
  const timers=[];
  if(req.url==='/progress'){
    timers.push(setTimeout(()=>res.write(Buffer.from([3])),3000));
    timers.push(setTimeout(()=>res.end(Buffer.from([4,5])),6000));
  }
  res.on('close',()=>timers.forEach(clearTimeout));
});
(async()=>{
 await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
 const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
 try{
  const page=await browser.newPage();await page.goto(`http://127.0.0.1:${server.address().port}`);await page.addScriptTag({content:code});
  const result=await page.evaluate(async()=>{
    const read=async url=>{
      const controller=new AbortController();let timer;
      const progress=()=>{clearTimeout(timer);timer=setTimeout(()=>controller.abort(),5000);};
      progress();const start=performance.now();
      try{
        const response=await fetch(url,{signal:controller.signal});
        const data=await window.readPacketTest(response,progress);
        return {bytes:[...new Uint8Array(data)],elapsed:performance.now()-start};
      }catch(error){return {error:error.name,message:error.message,elapsed:performance.now()-start};}
      finally{clearTimeout(timer);}
    };
    return Promise.all([read('/progress'),read('/stall')]);
  });
  assert.deepEqual(result[0].bytes,[1,2,3,4,5],JSON.stringify(result));
  assert.ok(result[0].elapsed>=5800,'a progressing frame may take over five seconds without being discarded');
  assert.equal(result[1].error,'AbortError');
  assert.ok(result[1].elapsed>=4800&&result[1].elapsed<8000,'an actually stalled response is still bounded');
  console.log(JSON.stringify({progressingFrameMs:Math.round(result[0].elapsed),stalledAbortMs:Math.round(result[1].elapsed),byteExact:true}));
 }finally{await browser.close();server.closeAllConnections();server.close();}
})().catch(error=>{console.error(error);server.closeAllConnections();server.close();process.exitCode=1;});
