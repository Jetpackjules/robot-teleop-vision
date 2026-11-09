const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const root=path.join(__dirname,'../web');const read=name=>fs.readFileSync(path.join(root,name),'utf8');
const source=read('hybrid_rgbd_renderer.js').replace(/^import .*;\r?\n/gm,'').replace(/^export /gm,'');
const c=vm.createContext({window:{},performance,FUSION_FRAGMENT_SHADER:'',COLOR_PROJECTION_SHADER:'',DEPTH_SHADING_SHADER:'',SCENE_LIGHTING_SHADER:''});
for(const [name,klass] of [['robot_pose_preview.js','RobotPosePreview'],['robot_joint_animation.js','MeasuredJointAnimation']])vm.runInContext(`globalThis.${klass}=(()=>{${read(name).replace(/^export /gm,'')};return ${klass}})()`,c);
for(const name of ['point_preview_sampling.js','workspace_transform.js','modeled_workspace.js'])vm.runInContext(read(name).replace(/^import .*;\r?\n/gm,'').replace(/^export /gm,''),c);
vm.runInContext(source+'\nwindow.probe={state,orbitViewer,currentHeadDelta,frameGodotHeadTrackingView,identityMatrix,getGodotHybridNavigationState,setGodotHybridNavigationState,attachPointerNavigation};',c);
const {state:s,orbitViewer:orbit,identityMatrix:identity,currentHeadDelta:head,frameGodotHeadTrackingView:frame,getGodotHybridNavigationState:get,setGodotHybridNavigationState:set}=c.window.probe;
const transform=origin=>({origin,basis_x:[1,0,0],basis_y:[0,1,0],basis_z:[0,0,1]});
Object.assign(s.settings,{modeled_workspace_center:[0,0,0],orbit_distance:.65,focus_vertical_offset:.35,orbit_pitch_offset:12,min_distance:.2,max_distance:3,dolly_enabled:true,dolly_gain:.01,yaw_gain:2.5,pitch_gain:1,max_yaw:80,max_pitch:60});
s.baseViewer={transform:identity(),fov:60,near:.05,far:100};s.robotRenderer={baseLink:'base_link'};s.robotTransforms={base_link:transform([0,0,.6]),gripper_link:transform([0,.1,.3])};
c.window.godotWebcamTrackerLatest={active:true,x:-6,y:0,z:35,neutral_z_cm:35};assert.equal(frame(),true);
const base=s.baseViewer.transform;const left=orbit(base,head());assert.equal(s.headBaseline.x,0);
c.window.godotWebcamTrackerLatest={active:true,x:0,y:0,z:35};const centered=orbit(base,head());assert.notDeepEqual(Array.from(left),Array.from(centered));assert.deepEqual(Array.from(s.focusWorld),[0,0,0]);assert.ok(centered[14]<0,'neutral camera is across table from the arm base');
const tip=[-.0079,.1-.000218121,.3-.0981274];const ray=tip.map((v,i)=>v-centered[12+i]);const length=Math.hypot(...ray);assert.ok(ray.reduce((sum,v,i)=>sum-v/length*centered[8+i],0)>.99999,'neutral camera aims at the calibrated claw');
c.window.godotWebcamTrackerLatest={active:true,x:-6,y:0,z:35};const beforeLoss=orbit(base,head());c.window.godotWebcamTrackerLatest={active:false,x:0,y:0,z:0};assert.deepEqual(Array.from(orbit(base,head())),Array.from(beforeLoss),'detection loss holds the exact view');
c.window.godotWebcamTrackerLatest={active:true,x:6,y:0,z:35};const opposite=orbit(base,head());assert.notDeepEqual(Array.from(opposite),Array.from(beforeLoss));c.window.godotWebcamTrackerLatest={active:false};head();c.window.godotWebcamTrackerLatest={active:true,x:6,y:0,z:35};assert.deepEqual(Array.from(orbit(base,head())),Array.from(opposite),'same absolute head position gives same view after reacquisition');
for(const delta of [[0,0,0],[.06,-.02,.04]]){
 s.manualDolly=0;const original=orbit(base,delta);
 for(const dolly of [-.2,.2,.6]){
  s.manualDolly=dolly;const changed=orbit(base,delta);
  for(let i=0;i<12;i++)assert.ok(Math.abs(changed[i]-original[i])<1e-7,'zoom cannot rotate the camera');
  const displacement=[0,1,2].map(i=>changed[12+i]-original[12+i]);
  const parallel=displacement.reduce((sum,v,i)=>sum+v*original[8+i],0);
  assert.ok(Math.hypot(...displacement.map((v,i)=>v-parallel*original[8+i]))<1e-6,'zoom follows one straight viewing line');
 }
}
s.manualDolly=.1;const snapshot=get({includeView:true}),held=orbit(base,head());s.orbitReference=null;s.orbitAimOffset=null;s.lastHeadDelta=[0,0,0];assert.equal(set(snapshot),true);c.window.godotWebcamTrackerLatest={active:false};assert.deepEqual(Array.from(orbit(s.baseViewer.transform,head())),Array.from(held),'stream restart preserves the loss-held camera frame');
// The old nominal-radius limit stopped far short of the aimed-at claw.
const baseline=Array.from(orbit(base,[0,0,0]));
s.manualDolly=0;frame();s.settings.min_distance=.125;s.lastHeadDelta=[0,0,0];
const neutral=Array.from(orbit(base,[0,0,0]));const wheelListeners={};
s.canvas={clientHeight:600,addEventListener:(name,fn)=>{wheelListeners[name]=fn;}};
c.window.probe.attachPointerNavigation();
for(let i=0;i<200;i++)wheelListeners.wheel({deltaY:-120,preventDefault(){}});
const close=orbit(base,[0,0,0]);
assert.ok(Math.abs(Math.hypot(...tip.map((v,i)=>close[12+i]-v))-.125)<1e-6,'wheel reaches actual 0.125m from the aimed-at claw');
for(let i=0;i<12;i++)assert.ok(Math.abs(close[i]-neutral[i])<1e-7,'near zoom cannot rotate/reframe');
s.manualDolly=0;assert.deepEqual(Array.from(orbit(base,[0,0,0])),neutral,'zero zoom preserves the existing baseline');
console.log('PASS wheel reaches optical near limit with a raised camera and offset claw target');
console.log('PASS table-centered opposite-side framing, absolute head reference, loss/reacquisition, straight dolly with/without tracking, and restart retention');
const t=vm.createContext({window:{},performance,console});vm.runInContext(read('godot_webcam_tracker_bridge.js')+'\nwindow.probe={state,emit};',t);const {state:ts,emit}=t.window.probe;ts.webglInfo={};Object.assign(ts.options,{xScale:-1,zScale:1,neutralZCm:35,smoothing:0});
emit(true,{x:6,y:0,z:35},'tracking');assert.equal(t.window.godotWebcamTrackerLatest.x,-6,'first off-center observation is not made neutral');const stamp=ts.lastPoseMs;emit(false,null,'lost');assert.equal(t.window.godotWebcamTrackerLatest.x,-6);assert.equal(ts.lastPoseMs,stamp);ts.options.smoothing=.9;emit(true,{x:-6,y:0,z:35},'reacquired');assert.equal(t.window.godotWebcamTrackerLatest.x,6,'first reacquired observation snaps without blending from the old edge');
console.log('PASS tracker screen-centered startup, loss hold and immediate reacquisition');

const controller=read('controller_hybrid.js');const a=controller.indexOf('function frameWhenTrackingStarts()'),b=controller.indexOf('function startVideoFallback()',a);let framed=0;
new Function('window','performance','let startupRecenterGeneration=0;'+controller.slice(a,b)+';frameWhenTrackingStarts();')({godotWebcamTrackerLatest:{active:false},frameGodotHeadTrackingView(){framed++;return true;},recenterGodotWebcamTracker(){throw Error('Startup cannot redefine neutral')},setTimeout(){throw Error('Scene was already ready')}},performance);assert.equal(framed,1);
console.log('PASS automatic table/claw framing does not depend on a detected startup head position');
