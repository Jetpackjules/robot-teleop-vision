const fs=require('node:fs'),assert=require('node:assert/strict');
const source=fs.readFileSync(require('node:path').join(__dirname,'../robot_modules/so101/web/module.js'),'utf8');
const start=source.indexOf('    element("so101-return-rest").addEventListener'),end=source.indexOf('    element("so101-restart")',start);
let handler,returned=0;new Function('element','window',source.slice(start,end)).call({confirmAction(){throw Error('Confirmation must not be shown')},reportError(e){throw e;}},()=>({addEventListener(_,fn){handler=fn}}),{so101ArmController:{returnToRest(){returned++}}});
handler();assert.equal(returned,1);console.log('PASS Return to Rest executes on the first click without confirmation');
