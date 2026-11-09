const fs=require('node:fs'),assert=require('node:assert/strict');
const source=fs.readFileSync(require('node:path').join(__dirname,'../robot_modules/so101/web/module.js'),'utf8');
const start=source.indexOf('    element("so101-return-rest").addEventListener'),end=source.indexOf('    element("so101-restart")',start);
let handler,returned=0;new Function('element','window',source.slice(start,end)).call({confirmAction(){throw Error('Confirmation must not be shown')},reportError(e){throw e;}},()=>({addEventListener(_,fn){handler=fn}}),{so101ArmController:{returnToRest(){returned++}}});
handler();assert.equal(returned,1);console.log('PASS Return to Rest executes on the first click without confirmation');

// Exercise the Enable click sequence with a fake clock and an inert arm.
const enableStart=source.indexOf('    const enableButton = element("so101-enable");');
const enableEnd=source.indexOf('    element("so101-save-rest")',enableStart);
const handlers={},pending=new Map(),actions=[];
let timer=0;
const button={disabled:false,addEventListener(type,fn){handlers[type]=fn;}};
const hold={addEventListener(type,fn){handlers.hold=fn;}};
const arm={latest:{state:'hold',status:{control_source:'leader'}},socket:{},
 enableArmAndConfirm({matchLeader}){actions.push(matchLeader?'slow match':'enable');},hold(){actions.push('hold');}};
const fakeWindow={so101ArmController:arm,setTimeout(fn){pending.set(++timer,fn);return timer;},clearTimeout(id){pending.delete(id);}};
new Function('element','window','updateStatus',source.slice(enableStart,enableEnd))
 .call({},id=>id==='so101-enable'?button:hold,fakeWindow,()=>{});
const drain=()=>{for(const [id,fn] of pending){pending.delete(id);fn();}};
(async()=>{
 handlers.click({detail:1});handlers.click({detail:2});handlers.dblclick();drain();
 assert.deepEqual(actions,['slow match'],'double-click sends only slow match');
 await Promise.resolve();
 handlers.click({detail:1});drain();
 assert.deepEqual(actions,['slow match','enable'],'single click sends normal Enable');
 await Promise.resolve();
 handlers.click({detail:1});handlers.hold();drain();
 assert.deepEqual(actions,['slow match','enable','hold'],'Hold cancels a pending click');
 console.log('PASS Enable double-click, single-click and Hold cancellation');
})().catch(error=>{console.error(error);process.exitCode=1;});
