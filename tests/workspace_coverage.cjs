const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict');
const urls=new Map();
function moduleUrl(name){
  name=name.split('?')[0];if(urls.has(name))return urls.get(name);
  let source=fs.readFileSync(path.resolve(__dirname,'../web',name),'utf8');
  source=source.replace(/from\s+(['"])(\.\/[^'"]+)\1/g,(_,quote,relative)=>'from '+JSON.stringify(moduleUrl(relative.slice(2))));
  const url='data:text/javascript;base64,'+Buffer.from(source).toString('base64');urls.set(name,url);return url;
}
(async()=>{
  const {countWorkspacePoints,WorkspaceCoverage,coverageText}=await import(moduleUrl('workspace_coverage.js'));
  const {modeledWorkspace,modeledWorkspaceOutside}=await import(moduleUrl('modeled_workspace.js'));
  const {rotation3}=await import(moduleUrl('workspace_transform.js'));
  const identity=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];let checks=0;
  const check=(condition,label)=>{assert.ok(condition,label);checks++;};
  const camera={metadata:{id:'camera'},width:5,height:1,depth:new Uint16Array([1000,0,1000,1000,1000]),modelMatrix:identity,intrinsics:[20,20,2,0]};
  const settings={modeled_workspace_enabled:true,modeled_workspace_center:[0,-.006,-1],modeled_workspace_width:.1};
  const table=modeledWorkspace(settings),half=countWorkspacePoints([camera],{table});
  check(half.total===4&&half.kept===2&&half.percent===50,'counts exact nonzero depth samples and excludes unmeasured zero samples');
  check(countWorkspacePoints([camera],{table:{...table,width:.3}}).percent===100,'widening the table keeps additional measured points');
  check(countWorkspacePoints([camera],{table:modeledWorkspace({...settings,modeled_workspace_center:[0,.004,-1]})}).kept===0,'raising the tabletop removes points immediately');
  check(countWorkspacePoints([camera]).percent===100,'a disabled region retains all measured points');
  check(countWorkspacePoints([{...camera,depth:new Uint16Array(5)}]).percent===null,'an empty capture does not claim a percentage');
  check(countWorkspacePoints([camera,camera],{table}).kept===4,'camera overlaps are counted per measured camera sample');
  for(let scene=0;scene<80;scene++){
    const width=19,height=13,depth=Uint16Array.from({length:width*height},(_,i)=>i%11?500+(i*137+scene*73)%1800:0);
    const r=rotation3([scene*1.3,scene*7.1,scene*.4]);
    const model=[...r.slice(0,3),0,...r.slice(3,6),0,...r.slice(6,9),0,.1,.3,-.4,1];
    const c={width,height,depth,modelMatrix:model,intrinsics:[25,24,9,6]};
    const table=modeledWorkspace({...settings,modeled_workspace_center:[0,-.2,-1],modeled_workspace_heading:scene*2-80,modeled_workspace_width:1.2,modeled_workspace_height:1.1});
    let total=0,kept=0;
    for(let y=0;y<height;y++)for(let x=0;x<width;x++){
      const z=depth[y*width+x]*.001;if(!z)continue;total++;
      const a=(x-9)*z/25,b=-(y-6)*z/24;
      const world=[0,1,2].map(i=>model[i]*a+model[i+4]*b-model[i+8]*z+model[i+12]);
      if(modeledWorkspaceOutside(world,table))continue;
      kept++;
    }
    const actual=countWorkspacePoints([c],{table});
    check(actual.total===total&&actual.kept===kept,'matches the existing table predicate across rigid camera poses');
  }
  const jobs=[],responses=[],cache=new Map();let worker,terminated=false;
  const frame={...camera,intrinsicsArray:camera.intrinsics,lastUpdateMs:performance.now(),uploadRevision:1};
  const counter=new WorkspaceCoverage({interval:0,onResult:r=>responses.push(r),createWorker:()=>worker={
    postMessage(data){jobs.push(data);const snapshot=structuredClone(data);setTimeout(()=>{if(terminated)return;
      for(const c of snapshot.updates)cache.set(c.key,c);
      worker.onmessage({data:{id:snapshot.id,...countWorkspacePoints(snapshot.keys.map(k=>cache.get(k)),snapshot)}});
    },5);},terminate(){terminated=true;}}});
  const wait=async predicate=>{for(let i=0;i<200&&!predicate();i++)await new Promise(r=>setTimeout(r,5));assert.ok(predicate(),'counter response arrived');};
  frame.metadata.stream_depth_origin=[0,0];counter.update({cameras:[frame],table});
  check(responses.at(-1).state==='waiting'&&jobs.length===0,'waits for uncropped frames rather than treating a cropped denominator as the full cloud');
  delete frame.metadata.stream_depth_origin;counter.update({cameras:[frame],table});await wait(()=>responses.at(-1).state==='ready');
  check(responses.at(-1).percent===50&&frame.depth.byteLength===10,'worker results preserve the original rendering depth array');
  const moved=modeledWorkspace({...settings,modeled_workspace_center:[0,.004,-1]});
  counter.update({cameras:[frame],table:moved});await wait(()=>responses.at(-1).percent===0);
  check(jobs.at(-1).updates.length===0,'dragging a table reuses worker camera buffers');
  frame.uploadRevision++;counter.update({cameras:[frame],table});await wait(()=>responses.at(-1).percent===50);
  check(jobs.at(-1).updates.length===1,'new capture revisions refresh their own camera buffer');
  counter.update({cameras:[frame],table:moved});
  for(let i=0;i<30;i++)counter.update({cameras:[frame],table:i===29?table:moved});
  await new Promise(r=>setTimeout(r,40));check(jobs.length<=5,'rapid handle events do not create a work backlog');
  check(coverageText(responses.at(-1)).includes('50.0%'),'readout reports a numeric point-retention percentage');
  counter.close();check(terminated,'closing the editor releases its worker');
  console.log(JSON.stringify({checks,success:true}));
})().catch(error=>{console.error(error);process.exitCode=1;});
