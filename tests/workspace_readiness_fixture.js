import {setup,draw,image,snapshot} from './web_rendering_fixture.js';

const bytes=encoded=>Uint8Array.from(atob(encoded),c=>c.charCodeAt(0)).buffer;
const difference=(a,b,tolerance=3)=>{let count=0;for(let i=0;i<a.length;i+=4)if([0,1,2,3].some(c=>Math.abs(a[i+c]-b[i+c])>tolerance))count++;return count;};

export async function runWorkspaceReadinessChecks() {
  const {state,gl,canvas}=setup();const probe=window.probe;
  canvas.style.width='768px';canvas.style.height='576px';
  let checks=0;const passed=[];
  const check=(ok,label)=>{checks++;if(!ok)throw new Error(label);passed.push(label);};
  const data=await (await fetch('./workspace-readiness-fixture.json')).json();
  const renderer=await (await fetch('./so101-renderer.json')).json();
  const install=async encoded=>{
    const frame=probe.parsePacket(bytes(encoded));
    for(const camera of state.cameras)probe.deleteCameraTextures(camera);
    state.cameraFusion.clear();state.blendPCR?.clearTargets();state.cameras=[];
    const decoded=await Promise.all(frame.payloads.map(payload=>probe.decodeCamera(payload)));
    state.cameras=decoded.map(camera=>probe.uploadCamera(camera));
    state.baseViewer={transform:new Float32Array(frame.metadata.viewer.transform),fov:55,near:.01,far:10};
  };
  Object.assign(state.settings,data.settings,{robot_overlay_enabled:false,scene_lighting_enabled:true,scene_shadows_enabled:true});
  for(const mode of ['point_cloud','independent_mesh','unified_mesh','blendpcr']) {
    await install(data.source);draw(mode);const original=image();
    await install(data.cropped);draw(mode);const cropped=image();
    let mismatches=difference(original,cropped,5);
    // JPEG-to-PNG preserves server-decoded colors; browser JPEG conversion may
    // round a channel differently, especially at exact nearest-texel ties.
    check(mismatches<original.length/4*.002,mode+': cropped native packets retain the rendered objects ('+mismatches+' differing pixels)');
  }
  check(data.stream.output_bytes<data.stream.input_bytes*.8,'server actually sends fewer bytes before transport');
  await install(data.source);Object.assign(state.settings,{modeled_workspace_enabled:false});draw('blendpcr');const full=image();
  Object.assign(state.settings,data.settings);await install(data.cropped);draw('blendpcr');
  Object.assign(state.settings,{modeled_workspace_enabled:false});await install(data.source);draw('blendpcr');
  check(difference(full,image(),0)===0,'switching off restores the full source scene exactly');

  state.robotRenderer=renderer;state.robotMeshes={};state.stats.robotModelsLoaded=0;
  state.robotProgram=probe.createProgram(gl,probe.ROBOT_VERTEX_SHADER,probe.ROBOT_FRAGMENT_SHADER);
  state.robotUniforms=probe.findRobotUniforms(gl,state.robotProgram);state.robotVao=gl.createVertexArray();
  await probe.loadRobotMeshes(gl,state.robotLoadToken);
  check(state.stats.robotModelsLoaded===7,'all seven actual SO-101 GLBs load');
  state.robotTransforms=data.poses[0].links;state.robotRegistration={registered:true};
  state.focusWorld=[8,9,10];state.manualPan=[1,2,3];
  const placement=window.alignGodotModeledWorkspace({source:'robot'});
  check(placement&&Math.abs(placement.modeled_workspace_center[0])<1e-6
    &&Math.abs(placement.modeled_workspace_center[1]+.002)<1e-6
    &&Math.abs(placement.modeled_workspace_center[2]+1)<1e-6,'table anchors below registered base independently of orbit and pan');
  state.robotRegistration={registered:false};
  check(window.alignGodotModeledWorkspace({source:'robot'})===null,'unregistered preview arm cannot supply a table anchor');
  state.robotRegistration={registered:true};
  // A stable three-quarter view makes the real CAD links and table legs easy
  // to inspect. This affects only the synthetic rehearsal, never a saved view.
  const eye=[.8,.6,.25],target=[0,.12,-1];
  const unit=a=>{const length=Math.hypot(...a);return a.map(v=>v/length);};
  const cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const back=unit(eye.map((v,i)=>v-target[i])),right=unit(cross([0,1,0],back)),up=cross(back,right);
  state.baseViewer.transform=new Float32Array([...right,0,...up,0,...back,0,...eye,1]);
  state.focusWorld=null;state.manualPan=[0,0,0];
  Object.assign(state.settings,data.settings,placement,{robot_overlay_enabled:true,robot_overlay_style:'solid',robot_overlay_mask_scanned_robot:false,
    scene_lighting_enabled:false,scene_shadows_enabled:false});
  draw('blendpcr');const flat=image();snapshot('Actual SO-101 assets · flat material');
  window.demoCaptures={flat:canvas.toDataURL()};
  Object.assign(state.settings,{scene_lighting_enabled:true});draw('blendpcr');const lit=image();
  check(difference(flat,lit)>500,'shared lights shade the CAD links and table');
  Object.assign(state.settings,{scene_shadows_enabled:true});draw('blendpcr');const shadowed=image();
  check(state.sceneShadow?.ready&&!state.stats.shadowWarning,'CAD shadow map is available');
  check(difference(lit,shadowed)>100,'table and actual robot cast visible shadows');
  snapshot('Shared white key + blue fill · CAD shadows');window.demoCaptures.lit=canvas.toDataURL();
  Object.assign(state.settings,{modeled_workspace_light_angle:-70});draw('blendpcr');
  check(difference(shadowed,image())>500,'light angle updates shading and shadows together');
  Object.assign(state.settings,{modeled_workspace_light_angle:35});
  const cache=state.modeledWorkspace.key;
  Object.assign(state.settings,{modeled_workspace_table_height:.9});draw('blendpcr');
  check(state.modeledWorkspace.key!==cache,'table height rebuilds the legs and floor geometry');
  Object.assign(state.settings,{modeled_workspace_table_height:.72});draw('blendpcr');const firstPose=image();
  let motionDifference=0;
  for(let i=1;i<data.poses.length;i++) {
    state.robotTransforms=data.poses[i].links;draw('blendpcr');
    motionDifference=Math.max(motionDifference,difference(firstPose,image()));
    await new Promise(resolve=>requestAnimationFrame(resolve));
  }
  check(motionDifference>50,'synthetic joint trajectory articulates the actual SO-101 links');
  state.robotTransforms=data.poses[5].links;draw('blendpcr');
  snapshot('Actual kinematics · another joint pose');window.demoCaptures.pose=canvas.toDataURL();
  check(gl.getError()===gl.NO_ERROR,'lighting, shadows and ROI decoding leave valid WebGL state');
  state.robotTransforms=data.poses[0].links;draw('blendpcr');
  const toolbar=document.createElement('div');toolbar.innerHTML='<label>Test pose <input id="offline-pose" type="range" min="0" max="15" value="0"></label> <label>Light angle <input id="offline-light" type="range" min="-180" max="180" value="35"></label>';
  document.getElementById('view').before(toolbar);
  toolbar.querySelector('#offline-pose').oninput=event=>{state.robotTransforms=data.poses[Number(event.target.value)].links;draw('blendpcr');};
  toolbar.querySelector('#offline-light').oninput=event=>{state.settings.modeled_workspace_light_angle=Number(event.target.value);draw('blendpcr');};
  return {checks,passed,synthetic:true,actualRobotModels:state.stats.robotModelsLoaded,jointPoses:data.poses.length,
    streamBytes:{source:data.stream.input_bytes,cropped:data.stream.output_bytes},
    retainedSamples:{source:data.stream.source_samples,cropped:data.stream.retained_samples}};
}
