// Selected-input setup with simulated acknowledgements; never touches hardware.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
(async()=>{
 global.window={setTimeout:(fn,ms)=>setTimeout(fn,Math.min(ms,40)),clearTimeout,so101ArmController:{},addEventListener(){}};
 global.WebSocket={OPEN:1};
 const text=fs.readFileSync(path.join(__dirname,'../robot_modules/so101/web/so101_web_serial.js'),'utf8');
 const {So101ArmController}=await import('data:text/javascript;base64,'+Buffer.from(text).toString('base64'));
 const make=()=>{
  const arm=Object.setPrototypeOf(new EventTarget(),So101ArmController.prototype);
  Object.assign(arm,{latest:{serverConnected:true,followerConnected:true,leaderConnected:false,keyboardConnected:true,armed:false,state:'hold',controlMode:'keyboard',status:{control_source:'none',last_seq:-1,measured_feedback_fresh:true}},
    seq:0,statusRevision:0,keyboardKeys:new Set(['KeyW']),keyboardKeyOrder:new Map([['KeyW',1]]),keyboardInputActive:true,sent:[],cleared:0,
    wire:{armed:false,state:'hold',control_source:'none',last_seq:-1,measured_feedback_fresh:true},
    emitStatus(){this.dispatchEvent(new Event('status'));}});
  const clear=So101ArmController.prototype.clearKeyboardMotion;
  arm.clearKeyboardMotion=function(){this.cleared++;return clear.call(this);};
  arm.socket={readyState:1,bufferedAmount:0,send(raw){const m=JSON.parse(raw);arm.sent.push(m);
    if(m.type==='arm_cartesian_velocity'){
      assert(m.linear.every(x=>x===0)&&m.wrist.every(x=>x===0)&&m.gripper===0,'handoff must send zero motion');
      arm.wire.last_seq=m.seq;
      if(arm.holdOnNextIdle){arm.holdOnNextIdle=false;Object.assign(arm.wire,{armed:false,state:'hold',control_source:'none'});}
    }
    if(m.type==='arm_enable')Object.assign(arm.wire,{armed:true,state:'armed',control_source:m.source});
    if(arm.replyOnStatusOnly&&m.type!=='arm_status_request')return;
    queueMicrotask(()=>{arm.latest.status={...arm.wire};arm.latest.armed=arm.wire.armed;arm.latest.state=arm.wire.state;arm.statusRevision++;arm.emitStatus();});
  }};
  arm.connectLeader=async()=>{throw Error('Keyboard setup must not open the USB chooser');};return arm;
 };
 const enables=arm=>arm.sent.filter(m=>m.type==='arm_enable');
 const arm=make();await arm.prepareTrialControl({connect:true});
 assert.deepEqual(enables(arm),[{type:'arm_enable',source:'keyboard'}]);assert.equal(arm.cleared,1);
 await arm.prepareTrialControl();assert.equal(enables(arm).length,1,'already armed keyboard is reused after fresh idle acknowledgement');
 arm.latest.armed=false;Object.assign(arm.wire,{armed:false,state:'hold',control_source:'none'});await arm.prepareTrialControl();assert.equal(enables(arm).length,2,'next trial can re-enable the selected keyboard after Hold');
 const paused=make();paused.trialSuspended=true;await assert.rejects(paused.prepareTrialControl(),/paused trial/);assert.equal(paused.sent.length,0);
 const offline=make();offline.latest.followerConnected=false;await assert.rejects(offline.prepareTrialControl(),/did not confirm/);assert.ok(offline.sent.every(m=>m.type==='arm_status_request'),'offline wait sends only status probes');
 const noAck=make();noAck.socket.send=raw=>noAck.sent.push(JSON.parse(raw));await assert.rejects(noAck.prepareTrialControl(),/did not confirm/);
 const staleArmed=make();Object.assign(staleArmed.latest,{armed:true,state:'armed'});staleArmed.latest.status.control_source='keyboard';staleArmed.holdOnNextIdle=true;
 await staleArmed.prepareTrialControl();assert.equal(enables(staleArmed).length,1,'host timeout Hold overrides cached pre-timeout armed status');
 const polled=make();polled.replyOnStatusOnly=true;await polled.prepareTrialControl();
 assert.equal(enables(polled).length,1,'status recovery never repeats Enable');
 assert.ok(polled.sent.some(m=>m.type==='arm_status_request'),'lost spontaneous status is recovered with read-only polling');
 const busy=make();busy.socket.bufferedAmount=65536;await assert.rejects(busy.prepareTrialControl(),/idle keyboard packet/);assert.equal(busy.sent.length,0);
 assert.equal(enables(noAck).length,0,'no Enable is sent before the idle packet is acknowledged');
 const leader=make();leader.latest.keyboardConnected=false;let received;leader.prepareLeaderTrial=async options=>{received=options;};
 await leader.prepareTrialControl({connect:true});assert.deepEqual(received,{connect:true});assert.equal(leader.sent.length,0,'default setup stays on the physical leader');
 const fallback=make();fallback.latest.keyboardConnected=false;let keyboardConnections=0;
 fallback.connectKeyboard=async()=>{keyboardConnections++;fallback.latest.keyboardConnected=true;};
 await fallback.prepareTrialControl({connect:true,keyboardFallback:true});
 assert.equal(keyboardConnections,1);assert.equal(enables(fallback)[0].source,'keyboard');
 const connected=make();connected.latest.keyboardConnected=false;connected.latest.leaderConnected=true;
 connected.prepareLeaderTrial=async()=>{};connected.connectKeyboard=async()=>{throw Error('Do not replace a connected leader');};
 await connected.prepareTrialControl({connect:true,keyboardFallback:true});
 console.log('PASS opt-in keyboard setup, no USB chooser, acknowledged Enable, Hold re-enable, Pause/offline gates and default leader setup');
})().catch(e=>{console.error(e.message);process.exitCode=1;});
