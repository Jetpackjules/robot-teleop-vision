// Exercise the production module, viewer editor bridge, and SVG handles. Only
// robot telemetry/transport are mocked; this test never connects to hardware.
const {chromium}=require(process.env.PLAYWRIGHT_MODULE||'playwright');
const {readFileSync,mkdirSync}=require('node:fs');
const {resolve}=require('node:path');
const assert=require('node:assert/strict');
const {createTestServer}=require('./serve_web_rendering.cjs');
const root=resolve(__dirname,'..'),output=resolve(root,'.teleop/base-placement-browser');
mkdirSync(output,{recursive:true});
const css=readFileSync(resolve(root,'web/controller.html'),'utf8').split('<style>')[1].split('</style>')[0];
const html=`<!doctype html><meta charset="utf-8"><title>Base placement test — synthetic telemetry</title>
<style>${css} ${readFileSync(resolve(root,'web/controller_ui.css'),'utf8')}
body{margin:0}#scene{position:fixed;inset:0;width:100vw;height:100vh;background:#33383e}#advanced-setup{position:fixed;left:20px;top:20px;width:430px;background:var(--panel);padding:18px}</style>
<canvas id="scene"></canvas><button id="open-advanced" style="position:fixed;top:15px;left:15px">Settings</button>
<div id="advanced-setup"><div id="settings" hidden></div><div id="setup" hidden></div><div id="view"></div></div><div id="status"></div><div id="floating"></div>
<script type="module">
import '/hybrid_rgbd_renderer.js';
import {rotation3} from '/workspace_transform.js';
import {createRobotModule} from '/module.js';
const module=createRobotModule({});
await module.mount(Object.fromEntries(['settings','setup','view','status','floating'].map(id=>[id+'Root',document.getElementById(id)])));
window.calibration={state:'complete',manual_calibration:{supported:true,registered:true,enabled:false,reply:{},base_adjustment:{supported:true,enabled:false}}};
window.getSo101ArmStatus=()=>({state:'hold',serverConnected:true,followerConnected:true,status:{measured_feedback_fresh:true,robot_calibration:calibration}});
window.sent=[];window.failBaseSave=false;
window.so101ArmController={clearKeyboardMotion(){},requestViewSettings(payload){
 sent.push(payload);const a=payload.robot_visual_calibration_action;
 if(!a)return Promise.resolve(true);
 if(a.operation==='base_preview')return Promise.resolve(true);
 setTimeout(()=>{const ok=!(a.operation==='base_save'&&failBaseSave),base=calibration.manual_calibration.base_adjustment;
  if(a.operation==='base_begin'){base.enabled=true;base.session_id=a.request_id;}
  if(ok&&['base_save','base_cancel'].includes(a.operation))base.enabled=false;
  calibration.manual_calibration.reply={request_id:a.request_id,ok,session_id:base.session_id,origin:[0,0,0],message:ok?'Base placement accepted.':'Could not save base placement.'};
  module.renderStatus();
 },20);return Promise.resolve(true);
}};
module.bind({sendViewSettings:()=>true,updateStatus:()=>module.renderStatus()});
const s=probe.state;s.canvas=document.getElementById('scene');s.baseViewer=probe.identityMatrix();s.settings.display_mode='independent_mesh';
s.robotRenderer=module.renderer;
const link=y=>({basis_x:[1,0,0],basis_y:[0,1,0],basis_z:[0,0,1],origin:[0,y,0]});
window.updateGodotRobotOverlay({pose_unix_ms:1,links:{base_link:link(0),shoulder_link:link(.1)},parents:{shoulder_link:'base_link'}});
const matrix=rotation3([-45,0,0]),viewer=new Float32Array([matrix[0],matrix[1],matrix[2],0,matrix[3],matrix[4],matrix[5],0,matrix[6],matrix[7],matrix[8],0,0,.8,.8,1]);
setInterval(()=>{module.renderStatus();if(!s.workspaceEditor?.active)return;const r=s.canvas.getBoundingClientRect();
 const projection=probe.perspectiveMatrix(55,r.width/r.height,.01,100);
 s.workspaceEditor.update({viewer,viewProjection:probe.multiplyMatrix(projection,probe.inverseRigidTransform(viewer)),fov:55,width:r.width,height:r.height,left:r.left,top:r.top});
},40);
window.testModule=module;
</script>`;
const server=createTestServer(),serve=server.listeners('request')[0];server.removeAllListeners('request');
server.on('request',(req,res)=>{
 const p=new URL(req.url,'http://localhost').pathname;
 if(p==='/'){res.setHeader('Content-Type','text/html');res.end(html);}
 else if(p==='/module.js'){res.setHeader('Content-Type','text/javascript');res.end(readFileSync(resolve(root,'robot_modules/so101/web/module.js')));}
 else if(['/so101_web_serial.js','/wrist_camera_view.js'].includes(p)){res.setHeader('Content-Type','text/javascript');res.end('export {};');}
 else serve(req,res);
});
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const browser=await chromium.launch({headless:true,...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
 try{
  const page=await browser.newPage({viewport:{width:1280,height:800}}),errors=[];page.on('pageerror',e=>errors.push(e.message));
  await page.goto('http://127.0.0.1:'+server.address().port);
  await page.waitForFunction(()=>window.testModule);
  await page.getByRole('button',{name:'Adjust Base in 3D',exact:true}).click();
  await page.locator('#workspace-editor').waitFor({state:'visible'});
  const center=page.getByRole('slider',{name:'Move across table',exact:true});
  await center.waitFor({state:'visible'});
  assert.equal(await page.getByRole('button',{name:'Resize R',exact:true}).count(),0);
  assert.equal(await page.getByRole('slider',{name:'Move Y',exact:true}).count(),1);
  const b=await center.boundingBox();await page.mouse.move(b.x+b.width/2,b.y+b.height/2);await page.mouse.down();await page.mouse.move(b.x+b.width/2+40,b.y+b.height/2+20,{steps:6});await page.mouse.up();
  await page.waitForFunction(()=>sent.some(p=>p.robot_visual_calibration_action?.operation==='base_preview'&&Math.abs(p.robot_visual_calibration_action.offset_x)>.001));
  const pose=await page.evaluate(()=>testModule.baseEditor.geometry.getBox());assert.equal(pose.center[1],0,'drag cannot change height');
  const height=page.getByRole('slider',{name:'Move Y',exact:true});await height.focus();await height.press('ArrowUp');
  await page.waitForFunction(()=>Math.abs(testModule.baseOffsets.offset_y-.01)<1e-6);
  assert(Math.abs(await page.evaluate(()=>probe.state.robotTransforms.base_link.origin[1])-.01)<1e-6,'height preview changes the displayed overlay before a camera packet arrives');
  await page.getByRole('button',{name:'Yaw E',exact:true}).click();
  const yaw=page.getByRole('slider',{name:'Rotate Y',exact:true});await yaw.focus();await yaw.press('ArrowRight');
  await page.waitForFunction(()=>testModule.baseOffsets.yaw_degrees===1);
  assert.equal(await page.getByRole('slider',{name:'Rotate X',exact:true}).count(),0);
  assert.equal(await page.getByRole('slider',{name:'Rotate Z',exact:true}).count(),0);
  await page.screenshot({path:resolve(output,'laptop.png')});
  await page.evaluate(()=>window.failBaseSave=true);await page.getByRole('button',{name:'Save Base',exact:true}).click();
  await page.waitForFunction(()=>document.getElementById('workspace-edit-help').textContent.includes('Could not save'));
  assert(await page.locator('#workspace-editor').isVisible(),'failed save retains editable preview');
  await page.evaluate(()=>window.failBaseSave=false);await page.getByRole('button',{name:'Save Base',exact:true}).click();
  await page.locator('#workspace-editor').waitFor({state:'hidden'});
  const saved=await page.evaluate(()=>sent.filter(p=>p.robot_visual_calibration_action?.operation==='base_save').at(-1).robot_visual_calibration_action);
  assert.equal(saved.yaw_degrees,1);assert(Math.abs(saved.offset_y-.01)<1e-6);assert(!('pitch' in saved));
  await page.evaluate(()=>document.getElementById('advanced-setup').hidden=false);
  await page.getByRole('button',{name:'Adjust Base in 3D',exact:true}).click();
  await page.locator('#workspace-editor').waitFor({state:'visible'});
  await page.getByRole('button',{name:'Reset',exact:true}).click();
  await page.waitForFunction(()=>sent.some(p=>p.robot_visual_calibration_action?.operation==='base_reset'));
  await page.waitForFunction(()=>!testModule.manualPending);
  await page.keyboard.press('Escape');await page.locator('#workspace-editor').waitFor({state:'hidden'});
  assert(await page.evaluate(()=>sent.some(p=>p.robot_visual_calibration_action?.operation==='base_cancel')));
  assert.equal(await page.evaluate(()=>document.body.classList.contains('workspace-editing')),false);
  assert.equal(await page.locator('#so101-overlay-occlusion-enabled').isChecked(),true,'restore masking after edit');
  assert.deepEqual(errors,[]);
  console.log('Base placement browser checks passed: planar drag, immediate vertical preview, yaw only, failed/successful save, reset, Escape cancellation, restored view settings.');
 }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});
