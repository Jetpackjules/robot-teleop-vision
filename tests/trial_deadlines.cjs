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
   assert.equal(plan.length,ids.length*6);
   const seen=[];
   for(let offset=0;offset<plan.length;offset+=6){
    const block=plan.slice(offset,offset+6),id=block[0].task_id;seen.push(id);
    assert.ok(block.every(p=>p.task_id===id),'both rounds finish before changing activity');
    for(let round=0;round<2;round++)assert.deepEqual(block.slice(round*3,round*3+3).map(p=>p.condition).sort(),views,'every round contains all three views exactly once');
    for(const condition of views)assert.equal(block.filter(p=>p.condition===condition).length,2);
    assert.ok(block.every(p=>p.time_limit_seconds===75));
    assert.ok(block.every(p=>p.success_metric===(id==='count_blocks'?'block_count':'pass_fail')));
    assert.ok(block.every(p=>p.outcome_rule===(id==='count_blocks'?'manual':'finish_before_limit')));
   }
   assert.deepEqual(seen.sort(),[...ids].sort());
  };
  assert.equal(c.state.randomize,true);checkPlan(c.state.plan,['single_block','count_blocks','custom']);assert.equal(c.state.randomActivities,true);assert.equal(node('trial-order').value,'random');
  assert.match(c.state.tasks.custom.instruction,/Balance test/);
  const withThird=structuredClone(c.state);withThird.tasks.custom.instruction='Named third task';checkPlan(makeTrialPlan(withThird),['single_block','count_blocks','custom']);
  const one={...withThird,activitySelection:'single_block',randomActivities:false};
  const randomValues=[0,0,.99,.99];const shuffled=makeTrialPlan(one,()=>{assert.ok(randomValues.length);return randomValues.shift();});
  checkPlan(shuffled,['single_block']);assert.equal(randomValues.length,0);assert.notDeepEqual(shuffled.slice(0,3),shuffled.slice(3),'each round gets its own shuffle');
  assert.deepEqual(makeTrialPlan({...one,randomize:false}).map(p=>p.condition),[...one.order,...one.order],'fixed order repeats for round two');
  assert.notEqual(makeTrialPlan(withThird,()=>0)[0].task_id,makeTrialPlan(withThird,()=>.99)[0].task_id,'activity order remains randomized');
  await c.initialize();assert.equal(node('trial-action').textContent,'Set up session');assert.throws(()=>recording.context(),/Set up/);
  node('trial-session-input').value='Participant 01';node('trial-session-input').dispatchEvent(new Event('input'));
  await c.act();assert.equal(setups,1);assert.equal(calls.length,0,'setup never starts recording');assert.equal(node('trial-action').textContent,'Start first trial');
  assert.match(c.state.name,/^Participant 01 — \d{4}-\d\d-\d\d \d\d:\d\d:\d\d GMT/);assert.equal(recording.context().session_name,c.state.name);
  await c.act();assert.equal(trialPreparations,1);assert.equal(calls.at(-1)[0],'start');assert.equal(recording.context().time_limit_seconds,c.state.timeTargetSeconds);
  node('trial-task-kind').value='count_blocks';node('trial-task-kind').dispatchEvent(new Event('change'));await Promise.resolve();await Promise.resolve();
  await c.act();assert.equal(c.state.taskId,'count_blocks');assert.equal(c.state.timeTargetSeconds,75);assert.equal(recording.context().time_limit_seconds,75);
  assert.equal(recording.context().success_metric,'block_count');assert.equal(recording.context().outcome_rule,'manual');
  recording.status.last_saved={episode:'one',status:'complete',trial:{session_id:c.state.id,index:1,condition:c.condition,task_id:'count_blocks'},timed_out:true};c.render();
  assert.equal(node('trial-block-result').hidden,false);assert.equal(node('trial-action').disabled,false);
  assert.equal(node('trial-action').textContent,'Review result');
  await c.act();assert.equal(c.state.index,0,'missing count cannot advance');
  node('trial-block-count').value='0';node('trial-save-block-count').dispatchEvent(new Event('click'));await Promise.resolve();
  assert.deepEqual(calls.at(-1),['score',{block_count:0,session_id:c.state.id}]);
  recording.status.last_saved.block_count=0;c.render();assert.equal(node('trial-action').disabled,false);assert.match(node('trial-block-score').textContent,/0 blocks/);
  await c.act();assert.equal(c.state.index,1,'facilitator explicitly advances after saving count');
  const id=c.state.id;await c.newSession();assert.notEqual(c.state.id,id);assert.equal(c.state.order.length,3);assert.equal(c.state.plan.length,6);
  for(let index=0;index<6;index++){
   recording.status.last_saved={episode:`round-${index}`,status:'complete',block_count:0,trial:{session_id:c.state.id,index:index+1,condition:c.condition,task_id:'count_blocks'}};c.render();
   assert.equal(c.state.index,index,'saving alone never advances a trial');if(index<5)await c.act();
  }
  assert.equal(c.state.finished,true,'the last saved trial completes the session without another End session click');assert.equal(c.state.index,5);
  c.render();assert.equal(node('trial-action').textContent,'Restart session');assert.equal(node('trial-action').disabled,false);
  const finishedId=c.state.id,finishedEpisode=recording.status.last_saved;node('trial-session-input').value='Participant 02';node('trial-session-input').dispatchEvent(new Event('input'));
  const starts=calls.filter(p=>p[0]==='start').length;await c.act();assert.notEqual(c.state.id,finishedId);assert.match(c.state.name,/^Participant 02 — /);assert.equal(c.state.index,0);assert.deepEqual(c.state.completed,[]);
  assert.equal(recording.status.last_saved,finishedEpisode,'restart preserves host recordings');assert.equal(calls.filter(p=>p[0]==='start').length,starts,'restart prepares without recording');assert.equal(node('trial-action').textContent,'Start first trial');
  const snapshot={...withThird,id:'session-eighteen',name:c.state.name,nameBase:c.state.nameBase,prepared:true,plan:makeTrialPlan(withThird),index:17,completed:['saved-before-refresh'],finished:false};
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
  console.log('PASS named setup/start/restart, date suffix, no setup recording, six trials per activity, independent rounds, refresh preservation, zero score and manual Next');
 }finally{clearInterval(c.timer);}
})().catch(e=>{console.error(e.message);process.exitCode=1;});
