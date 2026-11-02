// Selected-input setup with simulated acknowledgements; never touches hardware.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
(async()=>{
 global.window={setTimeout:(fn,ms)=>setTimeout(fn,Math.min(ms,40)),clearTimeout,so101ArmController:{},addEventListener(){}};
 global.WebSocket={OPEN:1};
 const text=fs.readFileSync(path.join(__dirname,'../robot_modules/so101/web/so101_web_serial.js'),'utf8');
 const {So101ArmController}=await import('data:text/javascript;base64,'+Buffer.from(text).toString('base64'));
 const make=()=>{
  const arm=Object.setPrototypeOf(new EventTarget(),So101ArmController.prototype);
  Object.assign(arm,{latest:{serverConnected:true,followerConnected:true,leaderConnected:false,keyboardConnected:true,armed:false,controlMode:'keyboard',status:{control_source:'none'}},sent:[],cleared:0,
   emitStatus(){this.dispatchEvent(new Event('status'));},clearKeyboardMotion(){this.cleared++;},sendKeyboardIntent(){return true;}});
  arm.socket={readyState:1,bufferedAmount:0,send(raw){const m=JSON.parse(raw);arm.sent.push(m);if(m.type==='arm_enable')queueMicrotask(()=>{arm.latest.armed=true;arm.latest.status.control_source=m.source;arm.emitStatus();});}};
  arm.connectLeader=async()=>{throw Error('Keyboard setup must not open the USB chooser');};return arm;
 };
 const arm=make();await arm.prepareTrialControl({connect:true});
 assert.deepEqual(arm.sent,[{type:'arm_enable',source:'keyboard'}]);assert.equal(arm.cleared,1);
 await arm.prepareTrialControl();assert.equal(arm.sent.length,1,'already armed keyboard is reused');
 arm.latest.armed=false;await arm.prepareTrialControl();assert.equal(arm.sent.length,2,'next trial can re-enable the selected keyboard after Hold');
 const paused=make();paused.trialSuspended=true;await assert.rejects(paused.prepareTrialControl(),/paused trial/);assert.equal(paused.sent.length,0);
 const offline=make();offline.latest.followerConnected=false;await assert.rejects(offline.prepareTrialControl(),/did not confirm/);assert.equal(offline.sent.length,0);
 const noAck=make();noAck.socket.send=raw=>noAck.sent.push(JSON.parse(raw));await assert.rejects(noAck.prepareTrialControl(),/did not confirm/);
 const leader=make();leader.latest.keyboardConnected=false;let received;leader.prepareLeaderTrial=async options=>{received=options;};
 await leader.prepareTrialControl({connect:true});assert.deepEqual(received,{connect:true});assert.equal(leader.sent.length,0,'default setup stays on the physical leader');
 console.log('PASS opt-in keyboard setup, no USB chooser, acknowledged Enable, Hold re-enable, Pause/offline gates and default leader setup');
})().catch(e=>{console.error(e.message);process.exitCode=1;});
