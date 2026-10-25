// Editing shortcuts must never reach the robot's keyboard listener.
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
(async()=>{
 const transform='data:text/javascript;base64,'+Buffer.from(fs.readFileSync(path.join(__dirname,'../web/workspace_transform.js'),'utf8')).toString('base64');
 const source=fs.readFileSync(path.join(__dirname,'../web/workspace_editor.js'),'utf8').replace(/'\.\/workspace_transform.js[^']*'/,JSON.stringify(transform));
 const {WorkspaceEditor}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
 let mode=null,finished=null;
 const editor={active:true,setMode:v=>mode=v,finish:v=>finished=v};
 const key=(value,extra={})=>{let stopped=false,prevented=false;WorkspaceEditor.prototype.keyDown.call(editor,{key:value,target:{tagName:'BUTTON'},preventDefault:()=>prevented=true,stopImmediatePropagation:()=>stopped=true,...extra});return {stopped,prevented};};
 for(const [value,expected]of [['w','move'],['E','rotate'],['r','resize']]){assert.deepEqual(key(value),{stopped:true,prevented:true});assert.equal(mode,expected);}
 for(const value of ['a','s','d','f','i','k','j','l','q'])assert.equal(key(value).stopped,true,'arm keyboard shortcuts stay inside the editor');
 assert.deepEqual(key('w',{target:{tagName:'INPUT'}}),{stopped:false,prevented:false},'normal form typing stays intact');
 assert.equal(key('w',{ctrlKey:true}).stopped,false,'browser shortcuts stay available');
 assert.equal(key('Escape').stopped,true);assert.equal(finished,false,'Escape cancels placement');
 editor.active=false;assert.equal(key('w').stopped,false,'closing the editor restores normal key routing');
 console.log('Workspace editor shortcut isolation and cancellation checks passed');
})();
