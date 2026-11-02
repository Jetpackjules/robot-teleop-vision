const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
(async()=>{
 global.window={so101ArmController:{},addEventListener(){}};
 global.HTMLElement=class{};
 global.document={hidden:false,hasFocus:()=>true};
 const armSource=fs.readFileSync(path.join(__dirname,'../robot_modules/so101/web/so101_web_serial.js'),'utf8');
 const {So101ArmController}=await import('data:text/javascript;base64,'+Buffer.from(armSource).toString('base64'));
 const source=fs.readFileSync(path.join(__dirname,'../web/controller_hybrid.js'),'utf8');
 for(const control of ['streamPresetSelect','displayModeSelect']){
  const events=[],arm=Object.create(So101ArmController.prototype);
  Object.assign(arm,{latest:{keyboardConnected:true,armed:true},keyboardKeys:new Set(['KeyW']),keyboardKeyOrder:new Map([['KeyW',1]]),keyboardKeySequence:1,seq:1,emitStatus(){},send(packet){events.push(packet);return true;}});
  window.so101ArmController=arm;
  const marker=control+'.addEventListener("change", () => {';
  const start=source.indexOf(marker)+marker.length,end=source.indexOf('\n    });',start);
  assert.ok(start>=marker.length && end>start);
  const body=source.slice(start,end);
  new Function('window','sendViewSettings','userStarted','startSelectedViewport','applyDisplayMode','streamPresetSelect','displayModeSelect','updateStatus',body)(window,()=>events.push('settings'),true,()=>{events.push('restart graphics');return Promise.resolve();},()=>events.push('apply mode'),{blur(){}},{blur(){}},()=>{});
  assert.equal(events[0].type,'arm_cartesian_velocity');assert.deepEqual(events[0].linear,[0,0,0]);assert.equal(events[0].gripper,0);
  assert.equal(arm.latest.armed,true);assert.equal(arm.latest.keyboardConnected,true);assert.equal(arm.keyboardKeys.size,0);
  arm.handleKeyboardEvent({code:'KeyD',preventDefault(){},stopPropagation(){},stopImmediatePropagation(){}},true);
  assert.equal(events.at(-1).linear[0],1);assert.equal(events.at(-1).type,'arm_cartesian_velocity');
 }
 console.log('PASS both view changes stop stale motion first, retain keyboard/arm enable, and accept the next key');
})().catch(e=>{console.error(e);process.exitCode=1});
