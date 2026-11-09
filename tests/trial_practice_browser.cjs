// Real browser + host recorder with synthetic cameras and robot telemetry only.
const {chromium}=require('C:/Users/jetpa/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const {spawn}=require('node:child_process');
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const root=path.resolve(__dirname,'..'),out=path.join(root,'.teleop/task1-practice-20261102');
fs.mkdirSync(out,{recursive:true});
const server=spawn(process.env.TEST_PYTHON||path.join(root,'.venv/Scripts/python.exe'),
  [path.join(root,'tests/serve_recording_fixture.py'),'--port','0','--output',path.join(out,'synthetic-'+Date.now())],
  {cwd:root,windowsHide:true,stdio:['ignore','pipe','pipe']});
let logs='';server.stderr.on('data',d=>logs+=d);
(async()=>{let browser;try{
 const port=await new Promise((accept,reject)=>{let text='';const timer=setTimeout(()=>reject(Error(logs.slice(-2000))),20000);
  server.stdout.on('data',d=>{text+=d;for(const line of text.split('\n'))try{const v=JSON.parse(line);if(v.port){clearTimeout(timer);accept(v.port);}}catch{}});
  server.on('exit',code=>{clearTimeout(timer);reject(Error('Synthetic server exited '+code+': '+logs.slice(-1000)));});});
 browser=await chromium.launch({headless:true,channel:'msedge'});
 const page=await browser.newPage({viewport:{width:1463,height:820}}),errors=[],starts=[];
 page.setDefaultTimeout(20000);page.on('pageerror',e=>{errors.push(e.message);console.error('Browser error: '+e.message);});
 page.on('request',r=>{if(r.url().endsWith('/api/v1/recording')&&r.method()==='POST'){const data=r.postDataJSON();if(data.command==='start')starts.push(data.trial);}});
 await page.route('**/controller_hybrid.js*',async route=>{const r=await route.fetch();const s=(await r.text()).replace('const trialController = new TrialController','const trialController = window.testTrials = new TrialController');await route.fulfill({response:r,body:s});});
 await page.goto(`http://127.0.0.1:${port}/controller.html?rgbd_transport=websocket`,{waitUntil:'domcontentloaded'});
 await page.waitForFunction(()=>window.testTrials?.protocolMessage?.includes('loaded')&&window.testTrials.recording.status?.state==='idle');
 const plan=await page.evaluate(async()=>{const c=window.testTrials;c.state.randomize=false;c.state.randomActivities=false;
  c.prepareSession=async()=>{};c.sessionConnected=c.viewReady=()=>true;
  window.trialPreparationOptions=[];
  window.so101ArmController.prepareTrialControl=async options=>window.trialPreparationOptions.push(options);
  await c.newSession();return c.state.plan;});
 assert.equal(plan.length,21);assert.ok(plan.slice(0,3).every(p=>p.phase==='practice'&&p.time_limit_seconds===60));
 assert.ok(plan.slice(3).every(p=>p.phase==='measured'&&p.time_limit_seconds===(p.task_id==='cup_stack_v1'?60:75)));
 assert.deepEqual(plan.slice(0,3).map(p=>p.condition),['fixed_rgb','mouse_3d','head_3d']);
 await page.waitForFunction(()=>!document.getElementById('trial-action').disabled);await page.locator('#trial-action').click();
 assert.equal(starts.length,0,'setup must not record');
 assert.equal(await page.locator('#trial-timer').textContent(),'01:00');
 // Shorten only the synthetic run. The real recorder still enforces the deadline.
 await page.evaluate(()=>{const c=window.testTrials;for(const entry of c.state.plan)entry.time_limit_seconds=8;c.syncTask();c.render();});
 const layouts=[];
 for(let index=0;index<3;index++){
  if(index===0) {await page.waitForFunction(()=>document.getElementById('trial-action').textContent==='Start practice'&&!document.getElementById('trial-action').disabled);await page.locator('#trial-action').click();}
  await page.waitForFunction(i=>window.testTrials.state.index===i&&window.testTrials.recording.status?.state==='recording'&&window.testTrials.recording.status.frames>=3,index);
  assert.equal(await page.locator('#trial-action').isVisible(),false);
  assert.equal(await page.locator('#trial-action').isDisabled(),true);
  assert.match(await page.locator('#trial-message').textContent(),[/switching cameras/,/orbiting/,/Move your head/][index]);
  for(const [width,height]of [[1024,768],[1920,1080]]){
   await page.setViewportSize({width,height});
   const box=await page.locator('#trial-hud').boundingBox();assert.ok(box.x>=0&&box.x+box.width<=width&&box.height<170);
   await page.screenshot({path:path.join(out,`practice-${index+1}-${width}.png`)});layouts.push({mode:plan[index].condition,width,...box});
  }
  await page.waitForFunction(i=>window.testTrials.state.index===i+1,index);
  assert.equal(await page.locator('#trial-result-dialog').evaluate(el=>el.open),false);
  assert.equal(await page.evaluate(()=>window.testTrials.recording.status.last_saved.timed_out),true);
 }
 assert.equal(starts.length,3,'practice cycles all views and stops before measured recording');
 await page.waitForFunction(()=>window.testTrials.state.taskId==='cup_stack_v1'&&!document.getElementById('trial-action').disabled);
 assert.equal(await page.locator('#trial-position').textContent(),'1/18');
 await page.evaluate(()=>{
   for(const angle of [0,2])window.dispatchEvent(new CustomEvent('so101-arm-status',{
     detail:{followerConnected:true,status:{measured_feedback_fresh:true,follower_normalized:[angle,0,0,0,0,0]}}
   }));
 });
 const movementReminder=page.locator('#trial-movement-reminder');
 assert.equal(await movementReminder.textContent(),'START TRIAL TO MOVE');
 assert.equal(await movementReminder.isVisible(),true);
 assert.equal(await movementReminder.evaluate(el=>getComputedStyle(el).pointerEvents),'none');
 await page.locator('#trial-action').click();
 assert.equal(await movementReminder.isVisible(),false);
 await page.waitForFunction(()=>window.testTrials.recording.status?.state==='recording'&&window.testTrials.recording.status.frames>=3);
 const urgentBorder=page.locator('#trial-urgent-border');
 await urgentBorder.waitFor({state:'visible'});
 assert.match(await page.locator('#trial-timer').textContent(),/^00:0[1-5]$/);
 for(const locator of [urgentBorder,page.locator('#trial-timer')]){
   assert.equal(await locator.evaluate(el=>getComputedStyle(el).animationName),'trial-urgent-pulse');
 }
 await page.emulateMedia({reducedMotion:'reduce'});
 assert.equal(await urgentBorder.evaluate(el=>getComputedStyle(el).animationName),'none');
 await page.waitForFunction(()=>document.getElementById('trial-result-dialog').open);
 assert.equal(await urgentBorder.isVisible(),false);
 assert.equal(await page.locator('#trial-timer').textContent(),'00:00');
 assert.equal(await page.evaluate(()=>window.testTrials.recording.status.last_saved.timed_out),true);
 assert.equal(await page.locator('#trial-block-result').isVisible(),true);
 assert.equal(await page.locator('#trial-experience-ratings').isVisible(),true);
 assert.equal(await page.locator('#trial-experience-preference').isVisible(),false);
 await page.locator('#trial-block-count').fill('0');await page.locator('#trial-save-block-count').click();
 assert.equal(await page.locator('#trial-result-dialog').evaluate(el=>el.open),true,'unanswered ratings block save');
 await page.locator('#trial-experience-stress').selectOption('2');
 await page.locator('#trial-experience-view_ease').selectOption('6');
 await page.locator('#trial-save-block-count').click();
 await page.waitForFunction(()=>window.testTrials.state.index===4);
 assert.equal(starts.length,4,'saving a measured score does not start the next recording');
 const preparation=await page.evaluate(()=>window.trialPreparationOptions);
 assert.equal(preparation.length,4);
 assert.ok(preparation.every(options=>options.continuation===true&&!options.connect),'practice and measured starts verify the same session instead of enabling it');
 assert.equal(await page.evaluate(()=>window.testTrials.recording.status.last_saved.block_count),0);
 assert.deepEqual(await page.evaluate(()=>window.testTrials.recording.status.last_saved.experience),{stress:2,view_ease:6});
 assert.equal(starts.at(-1).experience_protocol_version,1);
 // Settings persistence uses the real authenticated protocol API.
 await page.locator('#open-advanced').click();await page.locator('#tab-session').click();
 await page.locator('#trial-practice-duration').fill('90');await page.locator('#trial-duration').focus();
 await page.waitForFunction(()=>!document.getElementById('trial-save-protocol').disabled);
 await page.locator('#trial-save-protocol').click();
 await page.waitForFunction(()=>document.getElementById('trial-protocol-status').textContent.includes('Saved on host'));
 const stored=await page.evaluate(async()=>await(await fetch('/api/v1/trial-protocol')).json());
 assert.equal(stored.version,4);assert.equal(stored.practice_time_limit_seconds,90);assert.equal(stored.time_limit_seconds,75);
 const snapshot=await page.evaluate(()=>window.testTrials.state.plan);
 await page.reload({waitUntil:'domcontentloaded'});await page.waitForFunction(()=>window.testTrials?.protocolMessage?.includes('loaded'));
 assert.deepEqual(await page.evaluate(()=>window.testTrials.state.plan),snapshot,'reload preserves an existing plan');
 assert.equal(await page.locator('#trial-practice-duration').inputValue(),'90');
 assert.deepEqual(errors,[]);
 const result={synthetic:true,hardwareConnected:false,plan,starts,layouts,errors,practiceUnscored:true,measuredCountSaved:true,hostSettingsPersisted:true,restoredPlanPreserved:true};
 fs.writeFileSync(path.join(out,'ui-results.json'),JSON.stringify(result,null,2));
 console.log(JSON.stringify({passed:true,practiceRecordings:3,measuredRecordings:1,settingsPersisted:true,layouts,errors}));
 }finally{await browser?.close();server.kill();}
})().catch(e=>{console.error(e);console.error(logs.slice(-1400));process.exitCode=1;});
