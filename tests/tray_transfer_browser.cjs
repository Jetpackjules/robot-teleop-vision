// Actual browser and recorder, with synthetic cameras/robot only.
const {chromium}=require('C:/Users/jetpa/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const {spawn}=require('node:child_process');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..'),out=path.join(root,'.teleop/tray-transfer-20261104');
const recordingRoot=path.join(out,'synthetic-'+Date.now());fs.mkdirSync(out,{recursive:true});
const server=spawn(process.env.TEST_PYTHON,[path.join(root,'tests/serve_recording_fixture.py'),'--port','0','--output',recordingRoot],{cwd:root,windowsHide:true,stdio:['ignore','pipe','pipe']});
let logs='';server.stderr.on('data',d=>logs+=d);
(async()=>{let browser;try{
 const port=await new Promise((resolve,reject)=>{let text='';const timer=setTimeout(()=>reject(Error(logs.slice(-2000))),25000);
  server.stdout.on('data',d=>{text+=d;for(const line of text.split('\n'))try{const v=JSON.parse(line);if(v.port){clearTimeout(timer);resolve(v.port);}}catch{}});
  server.on('exit',code=>{clearTimeout(timer);reject(Error('Synthetic server exited '+code+': '+logs.slice(-1000)));});});
 browser=await chromium.launch({channel:'msedge',headless:true});const page=await browser.newPage({viewport:{width:1366,height:900}}),errors=[];
 page.on('pageerror',e=>errors.push(e.message));page.setDefaultTimeout(25000);
 await page.route('**/controller_hybrid.js*',async route=>{const r=await route.fetch();await route.fulfill({response:r,body:(await r.text()).replace('const trialController = new TrialController','const trialController = window.testTrials = new TrialController')});});
 await page.route('**/api/v1/recording',async route=>{
  const request=route.request();
  if(request.method()==='POST'){
   const data=request.postDataJSON();
   if(data.command==='start'&&data.trial?.index===2){data.trial.time_limit_seconds=1;await route.continue({postData:JSON.stringify(data)});return;}
  }
  await route.continue();
 }); // Accelerate one synthetic host deadline; the configured plan stays at 60 seconds.
 await page.goto(`http://127.0.0.1:${port}/controller.html?rgbd_transport=websocket`,{waitUntil:'domcontentloaded'});
 await page.waitForFunction(()=>window.testTrials?.protocolMessage?.includes('loaded')&&window.testTrials.recording.status?.state==='idle');
 assert.equal(await page.evaluate(()=>window.testTrials.state.plan.length),21);
 await page.evaluate(()=>{const c=window.testTrials;c.prepareSession=c.prepareTrial=async()=>{};c.sessionConnected=c.viewReady=()=>true;c.state.randomize=false;c.state.order=['fixed_rgb','mouse_3d','head_3d'];});
 await page.locator('#open-advanced').click();await page.locator('#tab-session').click();
 await page.locator('#trial-task-kind').selectOption('tray_transfer_v1');
 await page.waitForFunction(()=>window.testTrials.state.plan.length===6&&!window.testTrials.pending);
 const plan=await page.evaluate(()=>window.testTrials.state.plan);
 assert.ok(plan.every(p=>p.task_id==='tray_transfer_v1'&&p.phase==='measured'&&p.time_limit_seconds===60&&p.success_metric==='block_count'));
 assert.deepEqual(plan.map(p=>p.condition),['fixed_rgb','mouse_3d','head_3d','fixed_rgb','mouse_3d','head_3d']);
 await page.locator('#trial-save-protocol').click();await page.waitForFunction(()=>window.testTrials.protocolMessage.includes('Saved on host'));
 const protocol=await page.evaluate(async()=>await(await fetch('/api/v1/trial-protocol')).json());
 assert.equal(protocol.tasks.tray_transfer_v1.time_limit_seconds,60);assert.equal(protocol.time_limit_seconds,75);assert.equal(protocol.practice_time_limit_seconds,60);
 await page.locator('#close-advanced').click();await page.waitForFunction(()=>!document.getElementById('trial-action').disabled);
 await page.locator('#trial-action').click();await page.waitForFunction(()=>window.testTrials.state.prepared&&!window.testTrials.setupPending);
 for(let i=0;i<6;i++){
  await page.waitForFunction(index=>window.testTrials.state.index===index&&!document.getElementById('trial-action').disabled,i);
  assert.match(await page.locator('#trial-task').textContent(),/^Task 1: Move as many objects/);
  await page.locator('#trial-action').click();
  if(i!==1){await page.waitForFunction(()=>window.testTrials.recording.status?.state==='recording'&&window.testTrials.recording.status.frames>=3);await page.locator('#trial-action').click();}
  await page.waitForFunction(()=>document.getElementById('trial-result-dialog').open);
  assert.equal(await page.locator('#trial-count-label').textContent(),'Successful placements (full release)');
  assert.equal(await page.locator('#trial-outcome-control').isVisible(),false);
  assert.equal(await page.evaluate(()=>window.testTrials.state.index),i);
  if(i===0)await page.screenshot({path:path.join(out,'placement-count.png'),fullPage:true});
  await page.locator('#trial-block-count').fill(String(i===0?0:i));await page.locator('#trial-save-block-count').click();
  await page.waitForFunction(index=>index===5?window.testTrials.state.finished:window.testTrials.state.index===index+1,i);
 }
 const manifests=fs.readdirSync(recordingRoot).filter(n=>fs.existsSync(path.join(recordingRoot,n,'episode.json'))).map(n=>JSON.parse(fs.readFileSync(path.join(recordingRoot,n,'episode.json'),'utf8'))).sort((a,b)=>a.trial.index-b.trial.index);
 assert.equal(manifests.length,6);
 manifests.forEach((m,i)=>{assert.equal(m.trial.task_id,'tray_transfer_v1');assert.equal(m.status,'complete');assert.equal(m.block_count,i);assert.equal(m.result_reviewed,true);assert.equal(m.trial.phase,'measured');assert.equal(m.timed_out,i===1);assert.ok(m.frames>0);});
 const snapshot=await page.evaluate(()=>window.testTrials.state.plan);
 await page.reload({waitUntil:'domcontentloaded'});await page.waitForFunction(()=>window.testTrials?.protocolMessage?.includes('loaded'));
 assert.deepEqual(await page.evaluate(()=>window.testTrials.state.plan),snapshot,'refresh preserves saved plan and its original durations');
 assert.equal(await page.evaluate(()=>window.testTrials.state.finished),true);
 assert.deepEqual(errors,[]);
 fs.writeFileSync(path.join(out,'browser-results.json'),JSON.stringify({passed:true,synthetic:true,hardwareConnected:false,plan,protocol,episodes:manifests.map(m=>({id:m.episode_id,trial:m.trial,count:m.block_count,timedOut:m.timed_out})),errors},null,2));
 console.log(JSON.stringify({passed:true,completedRecordings:6,zeroCountSaved:true,timeoutAndDone:true,restoredPlanPreserved:true,errors}));
 }finally{await browser?.close();server.kill();}
})().catch(e=>{console.error(e);console.error(logs.slice(-1600));process.exitCode=1;});
