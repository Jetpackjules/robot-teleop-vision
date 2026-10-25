const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
(async()=>{
  const source="const SCENE_LIGHTING_SHADER='';\n"+fs.readFileSync(path.join(__dirname,'../web/modeled_workspace.js'),'utf8').replace(/^import .*;\r?\n/gm,'');
  const {MODELED_WORKSPACE_DEFAULTS,modeledWorkspace,modeledWorkspaceOutside,modeledWorkspaceBounds,modeledWorkspaceGeometry}
    =await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
  let checks=0;const check=(v,label)=>{assert.ok(v,label);checks++;};
  const settings={...MODELED_WORKSPACE_DEFAULTS,modeled_workspace_enabled:true};
  const stage=modeledWorkspace(settings);
  check(!modeledWorkspace(MODELED_WORKSPACE_DEFAULTS),'new mode starts disabled');
  check(modeledWorkspaceOutside([0,0,0],stage),'measured tabletop is excluded');
  check(!modeledWorkspaceOutside([0,.02,0],stage),'thin object above the exclusion survives');
  check(modeledWorkspaceOutside([.7,.2,0],stage),'out-of-bounds background is excluded');
  check(modeledWorkspaceOutside([0,1,0],stage),'height limit bounds the live object region');
  const existing={min:[-.1,-.1,-.1],max:[.1,.3,.1],key:'existing'};
  check(modeledWorkspaceBounds(MODELED_WORKSPACE_DEFAULTS,existing)===existing,'disabled mode preserves existing crop');
  const combined=modeledWorkspaceBounds(settings,existing);
  check(combined.min[0]===-.1&&combined.max[1]===.3,'modeled bounds intersect the independently configured crop');
  for(const heading of [-180,-75,-15,0,45,90,135,180]) {
    const value={...settings,modeled_workspace_center:[.3,-.2,-1],modeled_workspace_heading:heading};
    const s=modeledWorkspace(value),b=modeledWorkspaceBounds(value);
    for(let i=0;i<40;i++) {
      const x=(i/39-.5)*s.width*.99,z=((i*13%40)/39-.5)*s.depth*.99;
      const point=[s.center[0]+s.cos*x+s.sin*z,s.center[1]+.02,s.center[2]-s.sin*x+s.cos*z];
      check(!modeledWorkspaceOutside(point,s),'rotated table retains native interior samples');
      check(point.every((v,a)=>v>=b.min[a]&&v<=b.max[a]),'submission box conservatively contains every retained sample');
    }
  }
  for(const invalid of [{modeled_workspace_center:[NaN,0,0]},{modeled_workspace_center:[0,0]},
    {modeled_workspace_width:-1},{modeled_workspace_clearance:.1},{modeled_workspace_heading:Infinity}])
    check(!modeledWorkspace({...settings,...invalid}),'malformed settings never remove geometry');
  const vertices=modeledWorkspaceGeometry(stage);
  check(vertices.length===186*11&&vertices.every(Number.isFinite),'table, four legs and floor use 62 finite material triangles');
  check(vertices.some((v,i)=>i%11===1&&v===0),'artificial tabletop matches the configured plane');
  console.log(`${checks} modeled workspace checks passed`);
})().catch(error=>{console.error(error);process.exitCode=1;});
