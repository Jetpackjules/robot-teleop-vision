const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
(async()=>{
 class Node extends EventTarget{
  constructor(){super();this.value='';this.checked=false;this.hidden=false;this.disabled=false;this.dataset={};this.attributes={};this.options=[];}
  get valueAsNumber(){return Number(this.value);} reportValidity(){return this.value!=='' && Number.isInteger(this.valueAsNumber) && this.valueAsNumber>=0;}
  setAttribute(k,v){this.attributes[k]=v;}getAttribute(k){return this.attributes[k];}
 }
 const nodes=new Map();const node=id=>{if(!nodes.has(id))nodes.set(id,new Node());return nodes.get(id);};
 node('trial-session-input').reportValidity=()=>node('trial-session-input').value.trim().length>0;
 global.document={getElementById:node,addEventListener(){}};
 let uid=0;global.window={crypto:{randomUUID:()=>`session-${++uid}`},addEventListener(){},godotWebcamTrackerLatest:{active:true}};
 const stored=new Map();global.sessionStorage={getItem:k=>stored.get(k)||null,setItem:(k,v)=>stored.set(k,v)};
 node('display-mode').value='independent_mesh';node('advanced-setup').hidden=true;
 const src=fs.readFileSync(path.join(__dirname,'../web/trial_controller.js'),'utf8');
 const {TrialController,randomizedConditions,makeTrialPlan,namedSession}=await import('data:text/javascript;base64,'+Buffer.from(src).toString('base64'));
 assert.equal(namedSession('  Participant 01  ',new Date('2026-10-28T14:05:06Z')),'Participant 01 — 2026-10-28 15:05:06 GMT+1');
 assert.match(namedSession('Summer',new Date('2026-07-28T14:05:06Z')),/16:05:06 GMT\+2$/,'date suffix handles Berlin daylight saving');
 for(const r of [0,.2,.5,.99])assert.deepEqual([...randomizedConditions(()=>r)].sort(),['fixed_rgb','head_3d','mouse_3d']);
 TrialController.prototype.bindSettings=function(){};
 const calls=[];const recording={task:new Node(),outcome:new Node(),root:new Node(),startButton:new Node(),busy:false,offline:false,
  status:{state:'idle',supported:true,ready:true,controller_active:true},command:async(command,extra)=>{calls.push([command,extra]);return true;}};
 let setups=0,trialPreparations=0;
 const options={recording,prepareSession:async()=>{setups++;},prepareTrial:async()=>{trialPreparations++;},applyCondition:async condition=>{node('display-mode').value=condition==='fixed_rgb'?'rgb_camera':'independent_mesh';node('head-tracking-enabled').checked=condition==='head_3d';},resetView(){},resetTracking(){}};
 const c=new TrialController(options);
 try{
  const views=['fixed_rgb','head_3d','mouse_3d'];
  const checkPlan=(plan,ids)=>{
   assert.equal(plan.length,ids.reduce((n,id)=>n+(id==='single_block'?3:6),0));
   const seen=[];
   for(let offset=0;offset<plan.length;){
    const id=plan[offset].task_id,practice=id==='single_block',size=practice?3:6;
    const block=plan.slice(offset,offset+size);seen.push(id);offset+=size;
    assert.ok(block.every(p=>p.task_id===id),'both rounds finish before changing activity');
    for(let round=0;round<(practice?1:2);round++)assert.deepEqual(block.slice(round*3,round*3+3).map(p=>p.condition).sort(),views,'every round contains all three views exactly once');
    for(const condition of views)assert.equal(block.filter(p=>p.condition===condition).length,practice?1:2);
    assert.ok(block.every(p=>p.time_limit_seconds===(practice||id==='cup_stack_v1'?60:75)));
    assert.ok(block.every(p=>p.success_metric===(practice?'practice':'block_count')));
    assert.ok(block.every(p=>p.phase===(practice?'practice':'measured')));
    assert.ok(block.every(p=>p.outcome_rule===('manual')));
   }
   if(ids.includes('single_block'))assert.equal(seen[0],'single_block','practice always comes first');
   assert.deepEqual(seen.sort(),[...ids].sort());
  };
  assert.equal(c.state.randomize,true);checkPlan(c.state.plan,['single_block','cup_stack_v1','cube_stack_v1','stack_to_mug_v1']);assert.equal(c.state.randomActivities,true);assert.equal(node('trial-order').value,'random');
  assert.ok(c.state.plan.slice(0,3).every(entry=>entry.experience_protocol_version===undefined));
  assert.ok(c.state.plan.slice(3).every(entry=>entry.experience_protocol_version===1));
  assert.deepEqual(c.state.plan.filter(entry=>entry.preference_prompt),[c.state.plan.at(-1)]);
  assert.ok(makeTrialPlan({...c.state,activitySelection:'cup_stack_v1'}).every(entry=>!entry.preference_prompt));
  assert.match(c.state.tasks.stack_to_mug_v1.instruction,/into the mug/);
  const withThird=structuredClone(c.state);withThird.tasks.stack_to_mug_v1.instruction='Named third task';checkPlan(makeTrialPlan(withThird),['single_block','cup_stack_v1','cube_stack_v1','stack_to_mug_v1']);
  checkPlan(makeTrialPlan({...withThird,activitySelection:'cup_stack_v1'}),['cup_stack_v1']);
  checkPlan(makeTrialPlan({...withThird,activitySelection:'practice_cups'}),['single_block','cup_stack_v1']);
  const one={...withThird,activitySelection:'cube_stack_v1',randomActivities:false};
  const randomValues=[0,0,.99,.99];const shuffled=makeTrialPlan(one,()=>{assert.ok(randomValues.length);return randomValues.shift();});
  checkPlan(shuffled,['cube_stack_v1']);assert.equal(randomValues.length,0);assert.notDeepEqual(shuffled.slice(0,3),shuffled.slice(3),'each round gets its own shuffle');
  assert.deepEqual(makeTrialPlan({...one,randomize:false}).map(p=>p.condition),[...one.order,...one.order],'fixed order repeats for round two');
  assert.notEqual(makeTrialPlan(withThird,()=>0)[3].task_id,makeTrialPlan(withThird,()=>.99)[3].task_id,'measured activity order remains randomized after practice');
  await c.initialize();assert.equal(node('trial-action').textContent,'Set up session');assert.throws(()=>recording.context(),/Set up/);
  node('trial-session-input').value='Participant 01';node('trial-session-input').dispatchEvent(new Event('input'));
  await c.act();assert.equal(setups,1);assert.equal(calls.length,0,'setup never starts recording');assert.equal(node('trial-action').textContent,'Start practice');
  assert.match(c.state.name,/^Participant 01 — \d{4}-\d\d-\d\d \d\d:\d\d:\d\d GMT/);assert.equal(recording.context().session_name,c.state.name);
  await c.act();assert.equal(trialPreparations,1);assert.equal(calls.at(-1)[0],'start');assert.equal(recording.context().time_limit_seconds,60);
  assert.equal(node('trial-timer').textContent,'01:00');
  const beforePress=calls.length;
  for (const state of ['recording','paused','saving']) {
   recording.status={...recording.status,state,elapsed_seconds:state==='saving'?0:12.2};
   c.render();assert.equal(node('trial-timer').textContent,'00:48');
   assert.equal(node('trial-action').hidden,true);assert.equal(node('trial-action').disabled,true);
   await c.act();assert.equal(calls.length,beforePress,'a second press cannot stop a guided trial');
  }
  for (const [state,elapsed,urgent] of [['recording',54.9,false],['recording',55,true],['paused',56,false],['recording',60,false]]) {
   recording.status={...recording.status,state,elapsed_seconds:elapsed};
   c.render();assert.equal(node('trial-urgent-border').hidden,!urgent);
   assert.equal(node('trial-hud').dataset.timeUrgent,String(urgent));
  }
  recording.status={...recording.status,state:'idle',elapsed_seconds:0};
  assert.equal(recording.context().phase,'practice');assert.equal(recording.context().success_metric,'practice');assert.equal(recording.context().result_prompt,false);
  assert.equal(node('trial-position').textContent,'Practice 1/3');
  const practiceStarts=calls.filter(p=>p[0]==='start').length;
  recording.status.last_saved={episode:'practice-one',status:'complete',outcome:'unrated',trial:{...recording.context()}};
  c.render();assert.equal(node('trial-action').hidden,false);assert.equal(c.resultNeedsReview(),false);await c.act();assert.equal(c.state.index,1);
  assert.equal(calls.filter(p=>p[0]==='start').length,practiceStarts,'Next practice never starts another recording');
  node('trial-task-kind').value='cube_stack_v1';node('trial-task-kind').dispatchEvent(new Event('change'));await Promise.resolve();await Promise.resolve();
  await c.act();assert.equal(c.state.taskId,'cube_stack_v1');assert.equal(c.state.timeTargetSeconds,75);assert.equal(recording.context().time_limit_seconds,75);
  assert.equal(recording.context().success_metric,'block_count');assert.equal(recording.context().outcome_rule,'manual');
  let fade;
  window.setTimeout=(callback,delay)=>{assert.equal(delay,3200);fade=callback;return 1;};
  window.clearTimeout=()=>{fade=undefined;};
  const arm=(base,gripper=0,fresh=true)=>({followerConnected:true,status:{measured_feedback_fresh:fresh,follower_normalized:[base,0,0,0,0,gripper]}});
  c.observePretrialArm(arm(0));c.observePretrialArm(arm(0.5));
  assert.equal(node('trial-movement-reminder').hidden,true,'small feedback changes do not trigger');
  c.observePretrialArm(arm(10,0,false));c.observePretrialArm(arm(10));
  assert.equal(node('trial-movement-reminder').hidden,true,'stale feedback breaks the comparison');
  c.observePretrialArm(arm(10,2));
  assert.equal(node('trial-movement-reminder').hidden,false);
  fade();assert.equal(node('trial-movement-reminder').hidden,true);
  c.observePretrialArm(arm(12));assert.equal(node('trial-movement-reminder').hidden,true,'reminder appears once per trial');
  recording.status.last_saved={episode:'one',status:'complete',trial:{session_id:c.state.id,index:1,condition:c.condition,task_id:'cube_stack_v1'},timed_out:true};c.render();
  assert.equal(node('trial-block-result').hidden,false);assert.equal(node('trial-action').disabled,false);
  assert.equal(node('trial-action').textContent,'Review result');assert.equal(node('trial-timer').textContent,'00:00');
  await c.act();assert.equal(c.state.index,0,'missing count cannot advance');
  node('trial-block-count').value='0';node('trial-save-block-count').dispatchEvent(new Event('click'));await Promise.resolve();
  assert.deepEqual(calls.at(-1),['score',{block_count:0,session_id:c.state.id}]);
  recording.status.last_saved.block_count=0;c.render();assert.equal(node('trial-action').disabled,false);assert.match(node('trial-block-score').textContent,/Cubes in the tallest stack 0/);
  await c.act();assert.equal(c.state.index,1,'facilitator explicitly advances after saving count');
  const id=c.state.id;await c.newSession();assert.notEqual(c.state.id,id);assert.equal(c.state.order.length,3);assert.equal(c.state.plan.length,6);
  for(let index=0;index<6;index++){
   recording.status.last_saved={episode:`round-${index}`,status:'complete',block_count:0,trial:{session_id:c.state.id,index:index+1,condition:c.condition,task_id:'cube_stack_v1'}};c.render();
   assert.equal(c.state.index,index,'saving alone never advances a trial');if(index<5)await c.act();
  }
  assert.equal(c.state.finished,true,'the last saved trial completes the session without another End session click');assert.equal(c.state.index,5);
  c.render();assert.equal(node('trial-action').textContent,'Restart session');assert.equal(node('trial-action').disabled,false);
  const finishedId=c.state.id,finishedEpisode=recording.status.last_saved;node('trial-session-input').value='Participant 02';node('trial-session-input').dispatchEvent(new Event('input'));
  const starts=calls.filter(p=>p[0]==='start').length;await c.act();assert.notEqual(c.state.id,finishedId);assert.match(c.state.name,/^Participant 02 — /);assert.equal(c.state.index,0);assert.deepEqual(c.state.completed,[]);
  assert.equal(recording.status.last_saved,finishedEpisode,'restart preserves host recordings');assert.equal(calls.filter(p=>p[0]==='start').length,starts,'restart prepares without recording');assert.equal(node('trial-action').textContent,'Start first trial');
  const legacyPlan=['single_block','count_blocks','custom'].flatMap(id=>Array.from({length:2},()=>views.map(condition=>({task_id:id,task:withThird.tasks[id].instruction,time_limit_seconds:75,condition,success_metric:id==='count_blocks'?'block_count':'pass_fail',outcome_rule:id==='count_blocks'?'manual':'finish_before_limit'}))).flat());
  const snapshot={...withThird,id:'session-eighteen',name:c.state.name,nameBase:c.state.nameBase,prepared:true,plan:legacyPlan,index:17,completed:['saved-before-refresh'],finished:false};
  stored.set('robotTeleopTrialSession.v1',JSON.stringify(snapshot));
  const reloaded=new TrialController(options);
  try{
   assert.equal(reloaded.state.id,snapshot.id);assert.equal(reloaded.state.index,17);assert.deepEqual(reloaded.state.plan,snapshot.plan,'refresh must not reshuffle the eighteen-trial plan');
   assert.deepEqual(reloaded.state.completed,snapshot.completed);assert.equal(reloaded.state.name,snapshot.name);await reloaded.initialize();assert.equal(recording.context().total,18);assert.equal(recording.context().index,18);assert.equal(node('trial-position').textContent,'18/18');
  }finally{clearInterval(reloaded.timer);}
  const legacy={...snapshot,id:'prior-protocol',index:2,plan:snapshot.plan.slice(0,3).concat(snapshot.plan.slice(6,9))};
  stored.set('robotTeleopTrialSession.v1',JSON.stringify(legacy));assert.deepEqual(c.restore().plan,legacy.plan,'existing shorter sessions are preserved until New session');
  c.state.activitySelection='custom';await c.newSession();
  const dialog=node('trial-result-dialog');dialog.showModal=()=>{dialog.open=true;};dialog.close=()=>{dialog.open=false;};
  recording.status.last_saved={episode:'review-binary',status:'complete',outcome:'failure',timed_out:true,
    trial:{session_id:c.state.id,index:1,condition:c.condition,task_id:'custom',success_metric:'pass_fail',result_prompt:true}};
  c.render();assert.equal(dialog.open,true);assert.equal(node('trial-outcome').value,'failure');assert.equal(node('trial-result-title').textContent,'Time’s up');
  await c.act();assert.equal(c.state.index,0,'showing a result never advances without explicit review');
  const originalCommand=recording.command;
  recording.command=async(command,extra)=>{calls.push([command,extra]);recording.status.last_saved.result_reviewed=true;recording.status.last_saved.outcome=extra.outcome;return true;};
  await c.saveResult();assert.equal(c.state.index,1);assert.equal(dialog.open,false);assert.equal(calls.at(-1)[0],'rate');
  assert.equal(recording.status.last_saved.outcome,'failure');
  recording.status.last_saved={episode:'review-error',status:'complete',outcome:'success',trial:{session_id:c.state.id,index:2,condition:c.condition,task_id:'custom',result_prompt:true}};
  c.render();recording.command=async()=>false;await c.saveResult();assert.equal(c.state.index,1,'a failed result save cannot advance');assert.equal(dialog.open,true);assert.equal(node('trial-result-error').hidden,false);
  recording.command=originalCommand;
  c.state.activitySelection='stack_to_mug_v1';await c.newSession();c.state.prepared=true;
  recording.status.last_saved={episode:'two-counts',status:'complete',trial:{session_id:c.state.id,index:1,condition:c.condition,task_id:'stack_to_mug_v1',success_metric:'block_count',result_prompt:true}};
  c.render();assert.equal(node('trial-knocked-over-control').hidden,false);
  node('trial-block-count').value='3';
  const beforeScore=calls.length;await c.saveResult();assert.equal(calls.length,beforeScore,'both counts are required');
  node('trial-knocked-over-count').value='0';
  recording.command=async(command,extra)=>{calls.push([command,extra]);Object.assign(recording.status.last_saved,extra,{result_reviewed:true});return true;};
  await c.saveResult();assert.deepEqual(calls.at(-1),['score',{block_count:3,knocked_over_count:0,session_id:c.state.id}]);
  assert.equal(c.state.index,1);recording.command=originalCommand;
  c.state.activitySelection='all';await c.newSession();c.state.prepared=true;
  c.state.index=c.state.plan.length-1;
  const lastEntry=c.state.plan.at(-1);
  recording.status.last_saved={episode:'survey-last',status:'complete',trial:{...lastEntry,session_id:c.state.id,index:c.state.index+1,result_prompt:true}};
  c.render();assert.equal(node('trial-experience-preference').hidden,false);
  node('trial-block-count').value='2';node('trial-knocked-over-count').value='0';
  node('trial-experience-stress').value='1';node('trial-experience-view_ease').value='7';
  node('trial-experience-preferred_view').reportValidity=()=>!!node('trial-experience-preferred_view').value;
  node('trial-experience-preference_reason').reportValidity=()=>true;
  const beforePreference=calls.length;await c.saveResult();assert.equal(calls.length,beforePreference);
  node('trial-experience-preferred_view').value='head_3d';
  recording.command=async(command,extra)=>{calls.push([command,extra]);Object.assign(recording.status.last_saved,extra,{result_reviewed:true});return true;};
  await c.saveResult();assert.equal(c.state.finished,true);
  assert.deepEqual(calls.at(-1)[1].experience,{stress:1,view_ease:7,preferred_view:'head_3d',preference_reason:''});
  recording.command=originalCommand;
  await c.newSession();c.state.prepared=true;
  c.viewReady=c.sessionConnected=()=>true;
  const flush=()=>new Promise(resolve=>setImmediate(resolve));
  const practiceSaved=(index,timed_out=false)=>({episode:'auto-'+index,status:'complete',outcome:'unrated',timed_out,
    trial:{session_id:c.state.id,index:index+1,condition:c.condition,phase:'practice',success_metric:'practice'}});
  await c.act();assert.equal(c.autoPractice,true,'an explicit Start practice arms automatic cycling');
  let before=calls.filter(p=>p[0]==='start').length;
  recording.status.last_saved=practiceSaved(0,true);c.render();await flush();await flush();
  assert.equal(c.state.index,1,'a practice timeout advances automatically');
  assert.equal(calls.filter(p=>p[0]==='start').length,before+1,'the next practice starts automatically');
  window.godotWebcamTrackerLatest.active=false;
  recording.status.last_saved=practiceSaved(1);c.render();await flush();await flush();
  assert.equal(c.state.index,2);assert.equal(c.practiceStartPending,true,'Head practice waits for valid tracking');
  before=calls.filter(p=>p[0]==='start').length;
  c.render();await flush();assert.equal(calls.filter(p=>p[0]==='start').length,before);
  window.godotWebcamTrackerLatest.active=true;c.render();await flush();
  assert.equal(calls.filter(p=>p[0]==='start').length,before+1);
  recording.status.last_saved=practiceSaved(2,true);c.render();await flush();await flush();
  assert.equal(c.state.index,3);assert.equal(c.practice,false);assert.equal(c.autoPractice,false);
  assert.equal(calls.filter(p=>p[0]==='start').length,before+1,'measured trials never auto-start');
  c.state.activitySelection='single_block';await c.newSession();c.state.index=2;c.state.prepared=true;
  await c.apply();const practiceSession=c.state.id;c.autoPractice=true;
  recording.status.last_saved=practiceSaved(2,true);c.render();await flush();await flush();
  assert.equal(c.state.id,practiceSession,'practice-only completion never restarts a session');
  assert.equal(c.state.finished,true);assert.equal(c.autoPractice,false);
  console.log('PASS named setup/start/restart, date suffix, no setup recording, six trials per activity, independent rounds, refresh preservation, zero score and manual Next');
 }finally{clearInterval(c.timer);}
})().catch(e=>{console.error(e.message);process.exitCode=1;});
