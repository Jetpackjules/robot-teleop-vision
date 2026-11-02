const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
(async()=>{
 class Element extends EventTarget {
  constructor(){super();this.style={};this.returnValue='';}
  append(){}setAttribute(){}focus(){}remove(){this.removed=true;}
  showModal(){this.open=true;}
  close(value=''){this.returnValue=value;this.open=false;this.dispatchEvent(new Event('close'));}
 }
 let clears=0;
 global.document={createElement:()=>new Element(),body:new Element()};
 global.window={clearInterval,clearTimeout,confirm(){throw Error('Blocking confirmation used');},
  so101ArmController:{clearKeyboardMotion(){clears++;},shutdown(){}}};
 const source=fs.readFileSync(path.join(__dirname,'../robot_modules/so101/web/module.js'),'utf8');
 assert.equal(source.includes('window.confirm('),false);
 const {createRobotModule}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
 const module=createRobotModule({}),pending=module.confirmAction('Return arm?','Return to Rest');
 assert.equal(window.so101ArmController.keyboardModalOpen,true);
 assert.equal(await module.confirmAction('Duplicate'),false);
 let ticks=0;const timer=setInterval(()=>ticks++,1);
 await new Promise(r=>setTimeout(r,20));clearInterval(timer);
 assert.ok(ticks>1,'heartbeat must run while confirmation is pending');
 const dialog=module.confirmationDialog;dialog.close('confirm');assert.equal(await pending,true);
 assert.equal(window.so101ArmController.keyboardModalOpen,false);assert.ok(dialog.removed);
 const cancelled=module.confirmAction('Cancel?');module.confirmationDialog.close();assert.equal(await cancelled,false);
 const stopped=module.confirmAction('Stop?');module.stop();assert.equal(await stopped,false);
 assert.equal(clears,3);
 console.log('PASS confirmation leaves timers running, blocks motion, preserves consent and cancels on shutdown');
})().catch(e=>{console.error(e);process.exitCode=1;});
