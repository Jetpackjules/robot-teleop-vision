// Real DOM controls; mock only telemetry/transport, never connect to a robot.
// PLAYWRIGHT_MODULE can point at an existing bundled Playwright installation.
const { chromium } = require(process.env.PLAYWRIGHT_MODULE || 'playwright');
const { readFileSync, mkdirSync } = require('node:fs');
const { resolve } = require('node:path');
const http = require('node:http');
const assert = require('node:assert/strict');
const root = resolve(__dirname, '..');
const output = resolve(root, '.teleop/manual-calibration-browser');
mkdirSync(output, { recursive: true });
const css = readFileSync(resolve(root, 'web/controller.html'), 'utf8').split('<style>')[1].split('</style>')[0];
const html = `<!doctype html><meta name="viewport" content="width=device-width,initial-scale=1">
<style>${css}</style><body><div class="control-stack" style="margin:16px">
<div id="settings" hidden></div><div id="setup" hidden></div><div id="view"></div></div>
<div id="status"></div><div id="floating"></div><script type="module">
import {createRobotModule} from '/module.js';
const module = createRobotModule({});
await module.mount(Object.fromEntries(['settings','setup','view','status','floating'].map(id=>[id+'Root',document.getElementById(id)])));
window.calibration = {state:'failed',manual_calibration:{supported:true,registered:true,enabled:false,reply:{}}};
window.getSo101ArmStatus = () => ({state:'hold',serverConnected:true,followerConnected:true,status:{robot_calibration:window.calibration}});
window.sent=[]; window.failSave=false; window.failImport=false; window.connected=true;
window.fixture={type:'so101_visual_calibration',schema_version:1,robot:'so101',registration:{joint_angle_offsets_degrees:[40,80,0,-70,0,0]}};
module.bind({sendViewSettings(payload){
  if(!window.connected)return false;
  window.sent.push(payload);
  const a=payload.robot_visual_calibration_action;
  if(!a)return true;
  if(a.operation==='preview')return true;
  setTimeout(()=>{
    const ok=!(a.operation==='save'&&window.failSave)&&!(a.operation==='import'&&window.failImport);
    if(ok&&['begin','reset'].includes(a.operation))calibration.manual_calibration.enabled=true;
    if(ok&&['save','cancel'].includes(a.operation))calibration.manual_calibration.enabled=false;
    calibration.manual_calibration.reply={request_id:a.request_id,ok,message:ok?'Saved/accepted':'Cannot write calibration file',...(a.operation==='export'?{file:window.fixture}:{})};
    module.renderStatus();
  },50);
  return true;
},updateStatus(){module.renderStatus();}});
setInterval(()=>module.renderStatus(),100);
module.renderStatus(); window.testModule=module;
</script>`;
const server = http.createServer((req, res) => {
  if(req.url==='/module.js') {
    res.setHeader('Content-Type','text/javascript');
    res.end(readFileSync(resolve(root,'robot_modules/so101/web/module.js')));
  } else if(req.url.endsWith('.js')) {
    // Disable WebSerial and auxiliary camera startup in this test-only page.
    res.setHeader('Content-Type','text/javascript'); res.end('export {};');
  } else {res.setHeader('Content-Type','text/html');res.end(html);}
});
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  const browser=await chromium.launch({headless:true, ...(process.env.TEST_BROWSER_CHANNEL?{channel:process.env.TEST_BROWSER_CHANNEL}:{})});
  try {
    const page=await browser.newPage({viewport:{width:1280,height:900}});
    const errors=[];page.on('pageerror',error=>errors.push(error.message));
    await page.goto(`http://127.0.0.1:${server.address().port}`);
    await page.waitForFunction(()=>window.testModule);
    await page.getByText('Recovery tools',{exact:true}).click();
    await page.getByRole('button',{name:'Manual Joint Tuning',exact:true}).click();
    const panel=page.locator('#so101-manual-workspace');
    await panel.waitFor({state:'visible'});
    const shoulder=page.getByRole('spinbutton',{name:'Shoulder lift (J2) offset in degrees',exact:true});
    await shoulder.fill('12.5');
    await page.waitForFunction(()=>window.sent.some(p=>p.manual_shoulder_lift_trim_degrees===12.5));
    const elbow=page.getByRole('slider',{name:'Elbow bend (J3) offset slider',exact:true});
    await elbow.fill('-90');
    assert.equal(await page.getByRole('spinbutton',{name:'Elbow bend (J3) offset in degrees',exact:true}).inputValue(),'-90');
    await shoulder.fill('999');
    await page.getByRole('button',{name:'Save Calibration',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('so101-manual-status').textContent.includes('valid angle'));
    assert(await panel.isVisible(),'invalid input must not save a different hidden value');
    await shoulder.fill('12.5');
    await page.evaluate(()=>window.failSave=true);
    await page.getByRole('button',{name:'Save Calibration',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('so101-manual-status').textContent.includes('Cannot write'));
    assert(await panel.isVisible(),'failed save must leave editable preview open');
    await page.evaluate(()=>window.failSave=false);
    await page.screenshot({path:resolve(output,'desktop.png')});
    await page.getByRole('button',{name:'Save Calibration',exact:true}).click();
    await panel.waitFor({state:'hidden'});
    const downloadPromise=page.waitForEvent('download');
    await page.getByRole('button',{name:'Export JSON',exact:true}).click();
    const download=await downloadPromise;
    assert(download.suggestedFilename().startsWith('so101-visual-calibration-'));
    const exportedPath=resolve(output,'export.json');await download.saveAs(exportedPath);
    assert.equal(JSON.parse(readFileSync(exportedPath,'utf8')).type,'so101_visual_calibration');
    await page.locator('#so101-calibration-file').setInputFiles(exportedPath);
    await page.locator('#so101-import-dialog').waitFor({state:'visible'});
    assert.equal(await page.locator('#so101-import-base').isChecked(),false);
    await page.evaluate(()=>window.failImport=true);
    await page.getByRole('button',{name:'Import and Save',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('so101-import-status').textContent.includes('Cannot write'));
    assert(await page.locator('#so101-import-dialog').isVisible());
    await page.evaluate(()=>window.failImport=false);
    await page.getByRole('button',{name:'Import and Save',exact:true}).click();
    await page.locator('#so101-import-dialog').waitFor({state:'hidden'});
    assert.equal(await page.evaluate(()=>window.sent.filter(p=>p.robot_visual_calibration_action?.operation==='import').at(-1).robot_visual_calibration_action.restore_base),false);
    await page.locator('#so101-calibration-file').setInputFiles(exportedPath);
    await page.locator('#so101-import-base').check();
    await page.getByRole('button',{name:'Import and Save',exact:true}).click();
    await page.locator('#so101-import-dialog').waitFor({state:'hidden'});
    assert.equal(await page.evaluate(()=>window.sent.filter(p=>p.robot_visual_calibration_action?.operation==='import').at(-1).robot_visual_calibration_action.restore_base),true);
    await page.getByRole('button',{name:'Manual Joint Tuning',exact:true}).click();
    await panel.waitFor({state:'visible'});
    await shoulder.fill('30');
    await page.getByRole('button',{name:'Reset Preview',exact:true}).click();
    await page.waitForFunction(()=>document.querySelector('[data-manual-number="manual_shoulder_lift_trim_degrees"]').value==='0');
    await page.setViewportSize({width:390,height:844});
    const bounds=await panel.boundingBox();
    assert(bounds.x>=0 && bounds.x+bounds.width<=390,'panel fits phone width');
    await page.screenshot({path:resolve(output,'mobile.png')});
    await page.locator('#so101-manual-cancel').click();
    await panel.waitFor({state:'hidden'});
    await page.evaluate(()=>window.connected=false);
    await page.getByRole('button',{name:'Manual Joint Tuning',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('so101-calibration-file-status').textContent.includes('Connect the controller'));
    assert.equal(await panel.isVisible(),false,'offline begin cannot show working preview');
    assert.deepEqual(errors,[]);
    console.log('Manual calibration browser checks passed: preview, numeric/slider sync, save failure/success, JSON download/upload, base opt-in, reset/cancel, disconnected state, desktop/mobile layout.');
  } finally {await browser.close();server.close();}
})().catch(error=>{console.error(error);server.close();process.exitCode=1;});
