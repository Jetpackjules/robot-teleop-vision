const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
(async()=>{
 const source=fs.readFileSync(path.join(__dirname,'../web/workspace_transform.js'),'utf8');
 const g=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
 let checks=0;const check=(v,label)=>{assert.ok(v,label);checks++;},near=(a,b)=>Math.abs(a-b)<1e-8;
 for(let i=0;i<100;i++){
  const angles=[i*7.3-270,i*3.7-180,i*5.2-200],matrix=g.rotation3(angles),roundTrip=g.rotation3(g.rotationDegrees(matrix));
  check(matrix.every((v,i)=>near(v,roundTrip[i])),'Euler persistence retains the exact orientation including angles beyond 90 degrees');
  const box=g.orientedBox([.1,-.2,-1],[.8,.4,.6],angles);
  for(const p of box.corners){const local=g.transform3(box.inverse,g.vsub(p,box.center));check(local.every((v,a)=>near(Math.abs(v),box.half[a])),'rotated corner remains in the same local volume');}
  for(let axis=0;axis<3;axis++)for(const sign of [-1,1]){
   const signs=[0,0,0],delta=[0,0,0];signs[axis]=sign;delta[axis]=.13;
   const resized=g.resizeWorkspaceBox(box,signs,delta,'crop');
   const oldAnchor=g.vadd(box.center,g.transform3(box.matrix,box.half.map((v,a)=>a===axis?-sign*v:0)));
   const newAnchor=g.vadd(resized.center,g.transform3(resized.matrix,resized.half.map((v,a)=>a===axis?-sign*v:0)));
   check(oldAnchor.every((v,a)=>near(v,newAnchor[a])),'face resizing preserves its opposite face under arbitrary rotation');
  }
  const saved=g.workspaceEditSettings(box,'crop'),restored=g.workspaceEditBox(saved,'crop');
  check(box.corners.every((p,i)=>p.every((v,a)=>near(v,restored.corners[i][a]))),'save/reload preserves rotated crop corners');
 }
 const table=g.workspaceEditBox({modeled_workspace_center:[.1,.07,-.5],modeled_workspace_width:.8,modeled_workspace_depth:.6,modeled_workspace_table_height:.72,modeled_workspace_heading:130},'table');
 const settings=g.workspaceEditSettings(table,'table');check(near(settings.modeled_workspace_center[1],.07)&&near(settings.modeled_workspace_heading,130),'table pivot uses the actual tabletop and keeps headings beyond 90 degrees');
 check(g.rayPlane({origin:[0,0,0],direction:[0,0,-1]},[0,0,-1],[0,0,1])[2]===-1,'view-plane drag intersects the actual world plane');
 check(g.rayPlane({origin:[0,0,0],direction:[1,0,0]},[0,0,-1],[0,0,1])===null,'parallel drag never creates a jump');
 check(near(g.rayAxis({origin:[0,0,0],direction:g.vunit([2,0,-1])},[0,0,-1],[1,0,0]),2),'axis dragging follows the picked ray at the handle depth');
 console.log(`${checks} workspace transform checks passed`);
})();
