// Exercise the production camera and pointer math without a browser or device.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const source = fs.readFileSync(path.join(__dirname, '../web/hybrid_rgbd_renderer.js'), 'utf8')
  .replace(/^import .*;\r?\n/gm, '').replace(/^export /gm, '');
const context = vm.createContext({window: {}, performance, FUSION_FRAGMENT_SHADER: '', COLOR_PROJECTION_SHADER: '', DEPTH_SHADING_SHADER: '',SCENE_LIGHTING_SHADER:''});
vm.runInContext(fs.readFileSync(path.join(__dirname, '../web/workspace_transform.js'), 'utf8').replace(/^export /gm, ''), context);
vm.runInContext(fs.readFileSync(path.join(__dirname, '../web/workspace_crop.js'), 'utf8').replace(/^import .*;\r?\n/gm,'').replace(/^export /gm, ''), context);
vm.runInContext(fs.readFileSync(path.join(__dirname, '../web/modeled_workspace.js'), 'utf8').replace(/^import .*;\r?\n/gm,'').replace(/^export /gm, ''), context);
vm.runInContext(source + '\nwindow.navigationProbe = {state, orbitViewer, attachPointerNavigation, identityMatrix, cameraMarkerGeometry, trimIndependentFrameQueue};', context);
const {state, orbitViewer, attachPointerNavigation, identityMatrix, cameraMarkerGeometry, trimIndependentFrameQueue} = context.window.navigationProbe;
Object.assign(state.settings, {orbit_distance: 0.65, focus_vertical_offset: 0.35,
  orbit_pitch_offset: 0, dolly_enabled: false});
state.focusWorld = [0.4, 0.2, -1];
state.focusPicked = true;
state.manualPan = [0.02, 0.03, 0.04];
const focus = [0.42, 0.23, -0.96];
const radius = Math.hypot(0.65, 0.35);
const base = identityMatrix();
let checks = 0;
function check(condition, label) {checks++; assert.ok(condition, label);}

const advanceRenderClock=vm.runInContext('advanceRenderClock',context);
for(const refreshHz of [30,59.94,60,90,120,144,165,240]) {
  state.lastRenderMs=0;let frames=0;
  for(let tick=0;tick<Math.floor(refreshHz*4);tick++) {
    if(advanceRenderClock(100+tick*1000/refreshHz+Math.sin(tick*.7)*.15))frames++;
  }
  check(Math.abs(frames-Math.min(refreshHz,60)*4)<=2,'60 fps render pacing retains fractional refresh time at '+refreshHz+' Hz');
}
check(advanceRenderClock(30000),'a long pause resumes rendering immediately');
state.lastRenderMs=0;
const uploadBatches=[],closedBitmaps=[];
context.uploadSpy=items=>uploadBatches.push(items.map(item=>item.tag));
vm.runInContext('var savedCameraUpload=applyCameraUploads;applyCameraUploads=uploadSpy;',context);
const queueCameraUploads=vm.runInContext('queueCameraUploads',context);
const flushPendingCameraUploads=vm.runInContext('flushPendingCameraUploads',context);
const clearPendingCameraUploads=vm.runInContext('clearPendingCameraUploads',context);
const capture=(id,tag)=>({metadata:{id},tag,bitmap:{close(){closedBitmaps.push(tag);}}});
queueCameraUploads([capture('A','A1'),capture('B','B1')]);queueCameraUploads([capture('A','A2')]);
check(uploadBatches.length===0,'native decode can finish without interrupting rendering with GPU uploads');
check(closedBitmaps.join(',')==='A1','replaced independent captures release their bitmap');
flushPendingCameraUploads();
check(uploadBatches[0].join(',')==='A2,B1','a newer A capture never erases a pending B capture');
queueCameraUploads([capture('C','C1')]);clearPendingCameraUploads();
check(closedBitmaps.includes('C1')&&state.pendingCameraUploads.size===0,'stopping or reconnecting releases pending captures');
queueCameraUploads(Array.from({length:33},(_,i)=>capture(String(i),'camera'+i)));
check(uploadBatches[1].length===33&&state.pendingCameraUploads.size===0,'a large rig flushes its bounded batch without truncating cameras');
vm.runInContext('applyCameraUploads=savedCameraUpload;',context);

for (const yaw of [-170, -80, 0, 80, 170]) {
  for (const pitch of [-80, -50, -10, 0, 10, 50, 80]) {
    state.manualOrbitYaw = yaw;
    state.manualOrbitPitch = pitch;
    const viewer = orbitViewer(base, [0, 0, 0]);
    const offset = focus.map((value, index) => viewer[12 + index] - value);
    const distance = Math.hypot(...offset);
    check(Math.abs(distance - radius) < 1e-6, `constant pivot radius at ${yaw},${pitch}`);
    const alignment = offset.reduce((sum, value, index) => sum + value / distance * viewer[8 + index], 0);
    check(alignment > 0.99999, `camera looks at its fixed pivot at ${yaw},${pitch}`);
    const elevation = Math.atan2(offset[1], Math.hypot(offset[0], offset[2])) * 180 / Math.PI;
    check(Math.abs(elevation) <= 80.00001, `orbit avoids pole crossing at ${yaw},${pitch}`);
    check(viewer[5] > 0, `camera stays upright at ${yaw},${pitch}`);
  }
}

state.manualOrbitYaw = 0;
state.manualOrbitPitch = 0;
const initial = orbitViewer(base, [0, 0, 0]);
state.baseViewer = {transform:base,fov:55,near:.05,far:100};
const savedRuntime = vm.runInContext('getGodotHybridNavigationState({includeView:true})', context);
state.baseViewer = null;state.focusWorld = null;state.focusPicked = false;state.manualPan = [9,9,9];
context.savedRuntime = savedRuntime;
vm.runInContext('setGodotHybridNavigationState(savedRuntime)', context);
check(orbitViewer(state.baseViewer.transform,[0,0,0]).every((v,i)=>Math.abs(v-initial[i])<1e-6),
  'a stream restart retains the exact world pivot, pan, and camera frame');
check(Math.abs(initial[13] - (focus[1] + 0.35)) < 1e-6, 'initial camera height is preserved');
check(Math.abs(initial[14] - (focus[2] + 0.65)) < 1e-6, 'initial camera distance is preserved');
const listeners = {};
state.canvas = {getBoundingClientRect:()=>({left:0,top:0,width:200,height:200})};
state.viewProjection = identityMatrix();
state.cameras = [{width:2,height:2,metadata:{intrinsics:[1,1,0,0]},depth:new Uint16Array([1000,0,0,0]),modelMatrix:identityMatrix()}];
check(vm.runInContext('focusGodotHybridRendererAt(100,100)',context), 'a real depth point can be picked');
check(state.manualPan.every(v=>v===0), 'picking a point clears the previous pan translation');
check(state.focusWorld.every((v,i)=>Math.abs(v-[0,0,-1][i])<1e-6), 'the picked world point is the actual pivot');
state.settings.workspace_crop_enabled=true;state.settings.workspace_crop_min=[1,1,-2];state.settings.workspace_crop_max=[2,2,0];
check(!vm.runInContext('focusGodotHybridRendererAt(100,100)',context),'hidden outside geometry cannot become an orbit pivot');
state.settings.workspace_crop_min=[-.1,-.1,-1.1];state.settings.workspace_crop_max=[.1,.1,-.9];
check(vm.runInContext('focusGodotHybridRendererAt(100,100)',context),'native geometry inside the crop remains pickable');
state.settings.workspace_crop_enabled=false;
vm.runInContext('setGodotHybridNavigationState(savedRuntime)',context);
state.canvas = {addEventListener: (name, listener) => {listeners[name] = listener;},
  setPointerCapture() {}, hasPointerCapture: () => false, clientHeight: 600};
const focusEvents = [];
context.focusSpy = (x, y) => {focusEvents.push([x, y]); return true;};
vm.runInContext('focusGodotHybridRendererAt = focusSpy;', context);
let focusNotifications = 0;
attachPointerNavigation({onFocus: () => {focusNotifications++;}});
const pointer = (x, y, button = 0, type = 'pointerup') => ({button, type, pointerId: 1, clientX: x, clientY: y, preventDefault() {}});
listeners.pointerdown(pointer(100, 100));
listeners.pointermove(pointer(100, 60));
const upDrag = orbitViewer(base, [0, 0, 0]);
check(upDrag[13] < initial[13], 'dragging upward moves the scene upward by lowering the viewpoint');
check(Math.abs(upDrag[12] - initial[12]) < 1e-6, 'vertical drag leaves horizontal orbit unchanged');
listeners.pointermove(pointer(140, 60));
const rightDrag = orbitViewer(base, [0, 0, 0]);
check(rightDrag[12] < upDrag[12], 'horizontal drag retains its existing direction');
listeners.pointerup(pointer(140, 60));
check(focusEvents.length === 0, 'releasing a left drag does not change the pivot');
listeners.pointerdown(pointer(100, 100));
listeners.pointerup(pointer(100, 100));
check(focusEvents.length === 0, 'an ordinary left click does not change the pivot');
const beforeDoubleClick = [state.manualOrbitYaw, state.manualOrbitPitch];
listeners.dblclick(pointer(110, 90));
check(focusEvents.length === 1, 'double-click changes the orbit target');
check(state.manualOrbitYaw === beforeDoubleClick[0] && state.manualOrbitPitch === beforeDoubleClick[1], 'double-click does not reset the viewing angles');
listeners.pointerdown(pointer(150, 120, 1));
listeners.pointerup(pointer(150, 120, 1));
check(focusEvents.length === 2, 'middle-click also changes the orbit target');
listeners.pointerdown(pointer(150, 120, 1));
listeners.pointermove(pointer(175, 145, 1));
listeners.pointerup(pointer(175, 145, 1));
check(focusEvents.length === 2, 'middle dragging pans without refocusing on release');
listeners.pointerdown(pointer(150, 120, 1));
listeners.pointercancel(pointer(150, 120, 1, 'pointercancel'));
check(focusEvents.length === 2, 'a cancelled middle click does not change the target');
check(focusNotifications === 2, 'only deliberate target changes notify the controller');
state.currentViewer = orbitViewer(base,[0,0,0]);
const oldPan=state.manualPan.slice(),oldFocus=state.focusWorld.slice(),oldPicked=state.focusPicked;
const v=state.currentViewer,meters=2*.65*Math.tan(state.settings.fov*Math.PI/360)/600;
const delta=[0,1,2].map(i=>-13*meters*v[i]-7*meters*v[4+i]);
state.manualPan=oldPan.map((p,i)=>p+delta[i]);
const legacyPanView=orbitViewer(base,[0,0,0]);
state.manualPan=oldPan;state.focusWorld=oldFocus;state.focusPicked=oldPicked;
listeners.pointerdown(pointer(100,100,2));listeners.pointermove(pointer(113,93,2));listeners.pointerup(pointer(113,93,2));
check(orbitViewer(base,[0,0,0]).every((p,i)=>Math.abs(p-legacyPanView[i])<1e-6),
  'moving the pivot during pan is visually identical to the previous pan');
check(state.manualPan.every(p=>p===0)&&state.focusPicked,'pan moves the world pivot without a hidden residual offset');
vm.runInContext('recenterGodotHybridRenderer()',context);
check(state.focusWorld.every((p,i)=>Math.abs(p-[0,0,-state.settings.focus_distance][i])<1e-6),
  'explicit recenter restores the original pivot after panning or picking');
for (const [width, height, fx, fy] of [[640, 480, 600, 600], [192, 109, 96, 97], [1280, 720, 900, 890], [848, 480, 500, 500]]) {
  const vertices = cameraMarkerGeometry({width, height, metadata: {intrinsics: [fx, fy, (width - 1) / 2, (height - 1) / 2]}});
  check(vertices.length === 84, 'camera marker contains a complete frustum and body');
  check(vertices.every(Number.isFinite), 'camera marker handles different camera intrinsics');
  check(vertices[5] < 0, 'camera frustum points along the Godot optical forward axis');
}
const packet = (id, reference, keyframe, sequence) => ({sequence, metadata: {cameras: [{id,
  depth_temporal_keyframe: keyframe, depth_keyframe_id: reference}]}});
const queued = [packet('A', 1, true, 1), packet('B', 1, true, 2),
  ...Array.from({length: 9}, (_, i) => packet('A', 1, false, i + 3))];
trimIndependentFrameQueue(queued);
check(queued[0].sequence === 1 && queued[1].sequence === 2,
  'backpressure retains the depth reference and an idle camera reference');
check(queued.length === 6 && queued.at(-1).sequence === 11,
  'backpressure skips stale deltas while retaining the latest acquisition');
const frequentKeyframes = Array.from({length: 50}, (_, i) => packet('A', i + 1, true, i + 1));
trimIndependentFrameQueue(frequentKeyframes);
check(frequentKeyframes.length === 4 && frequentKeyframes[0].sequence === 47,
  'frequent replacement keyframes cannot build an unbounded decode backlog');
const changedReference = [packet('A', 1, true, 1), packet('A', 1, false, 2),
  packet('A', 2, true, 3), packet('A', 2, false, 4), packet('A', 2, false, 5)];
trimIndependentFrameQueue(changedReference, 4);
check(changedReference[0].sequence === 1,
  'a retained delta before a reference change keeps its own older keyframe');
// Restore real legacy view settings and run real controller startup with a
// stub renderer: saved compression must not switch live streams off the fast
// path, while unrelated viewing preferences and Low latency remain intact.
(async () => {
  const controller = fs.readFileSync(path.join(__dirname, '../web/controller_hybrid.js'), 'utf8');
  const section = (start, end) => controller.slice(controller.indexOf(start), controller.indexOf(end));
  const depthControl={checked:false};let depthSettingsSent=0;
  const shortcut=vm.createContext({hybridMode:true,displayModeSelect:{value:'independent_mesh'},
    document:{getElementById:()=>depthControl},sendViewSettings:()=>{depthSettingsSent++;}});
  vm.runInContext(section('function handleDepthOnlyShortcut(', 'document.addEventListener("keydown", handleDepthOnlyShortcut)'),shortcut);
  const key=extra=>({key:'Backspace',target:{},preventDefault(){this.prevented=true;},...extra});
  const press=event=>{shortcut.event=event;vm.runInContext('handleDepthOnlyShortcut(event)',shortcut);return event;};
  check(press(key()).prevented&&depthControl.checked,'Backspace enables depth-only and suppresses browser navigation');
  check(press(key({repeat:true})).prevented&&depthSettingsSent===1,'holding Backspace does not repeatedly toggle depth-only');
  press(key());check(!depthControl.checked&&depthSettingsSent===2,'a second Backspace restores RGB texturing');
  for(const extra of [{key:'Delete'},{ctrlKey:true},{altKey:true},{metaKey:true},{shiftKey:true},
    {defaultPrevented:true},{isComposing:true},{target:{isContentEditable:true}},
    {target:{closest:()=>({tagName:'INPUT'})}},{target:{closest:()=>({tagName:'TEXTAREA'})}},
    {target:{closest:()=>({tagName:'SELECT'})}}]) {
    const event=press(key(extra));
    check(!event.prevented&&!depthControl.checked&&depthSettingsSent===2,'editing, composition and modified keys keep their normal behavior');
  }
  shortcut.displayModeSelect.value='rgb_camera';
  check(!press(key()).prevented&&depthSettingsSent===2,'original RGB mode is unaffected by the 3D shortcut');
  const route=vm.createContext({window:{location:{protocol:'http:',host:'localhost:8766'}},pageHost:()=> 'localhost'});
  vm.runInContext(section('function remoteUrl(', 'function streamBaseUrl('),route);
  check(vm.runInContext('remoteUrl()',route)==='ws://localhost:8766/head-tracking','local HTTP uses the authenticated control route');
  route.window.location={protocol:'https:',host:'example.test'};
  check(vm.runInContext('remoteUrl()',route)==='wss://example.test/head-tracking','public HTTPS uses the same-origin authenticated route');
  const controls = new Map();
  const control = id => {
    if (id === 'persistent-temporal-reference-enabled') return null;
    if (!controls.has(id)) controls.set(id,{value:'1',checked:false});
    return controls.get(id);
  };
  let options;
  let requestedMode;
  const presets = await import('data:text/javascript;base64,' + Buffer.from(
    fs.readFileSync(path.join(__dirname, '../web/capture_presets.js'), 'utf8')).toString('base64'));
  const policy = vm.createContext({URLSearchParams, document:{getElementById:control,body:{classList:{contains:()=>false}}}, robotModule:{},
    viewSettingsVersion:15,displayModes:['point_cloud','independent_mesh','blendpcr','rgb_camera'],
    cropViewSettings:()=>({workspace_crop_enabled:false,workspace_crop_show_box:false,
      workspace_crop_min:[-.6,-.45,-.6],workspace_crop_max:[.6,.45,.6]}),
    workspaceCropBounds:()=>true,lastValidCrop:{},restoreCropSettings(){},
    modeledViewSettings:()=>({modeled_workspace_enabled:false}),modeledWorkspace:()=>null,lastValidModeled:{},restoreModeledSettings(){},
    defaultDisplayMode:'independent_mesh',displayModeSelect:control('display-mode'),
    streamPresetSelect:control('stream-preset'),localSavedViewSettings:()=>null,
    updateHeadTrackingOptions(){},updateViewLabels(){},showHybrid(){},updateStatus(){},
    viewportGeneration:0,retainedNavigation:null,selectedRgbCamera:'',capturePresetControl:null,workspaceStreamControl:{update(){}},remoteUrl:()=> 'wss://example.test/head-tracking',
    nativeStreamPreset:presets.nativeStreamPreset,
    CapturePresetControl:class {async request(mode){requestedMode=mode;return true;} cancel(){}},
    currentPreset:()=>({label:'Quality'}),startHybridControlPeer:async()=>true,
    hybridCanvas:{},rgbCameraCanvas:{},rgbdDataChannel:{},startupNavigation:null,startupNavigationPending:false,
    applyDisplayMode(){},window:{location:{search:'?rgbd_transport=websocket'},
      startGodotHybridRenderer:async value=>{options=value;}}});
  vm.runInContext('function stopViewport(){viewportGeneration++;capturePresetControl?.cancel();}\n'
    +section('function viewSettingsPayload(', 'function updateViewLabels(')
    +section('function restoreViewSettings(', 'async function fetchSiteSettings(')
    +section('async function startHybridViewport(', 'function startSelectedViewport('),policy);
  for(const legacy of [undefined,false,true]) {
    policy.saved={settings_version:14,persistent_temporal_reference_enabled:legacy,
      full_rgb_frame_updates_enabled:false,fov:71,orbit_distance:.83,display_mode:'blendpcr'};
    vm.runInContext('restoreViewSettings(saved);payload=viewSettingsPayload();',policy);
    check(policy.payload.persistent_temporal_reference_enabled===false,'old or absent saved compression stays off');
    check(policy.payload.full_rgb_frame_updates_enabled===true,'both camera profiles always send full RGB frames');
    check(policy.payload.fov===71&&policy.payload.orbit_distance===.83,'unrelated saved camera preferences are preserved');
    policy.saved.depth_only_enabled=true;
    vm.runInContext('restoreViewSettings(saved);payload=viewSettingsPayload();',policy);
    check(policy.payload.depth_only_enabled===true,'depth-only preference survives the settings round trip');
    for(const mode of ['quality','latency']) {
      control('stream-preset').value=mode;
      await vm.runInContext('startHybridViewport()',policy);
      check(options.persistentReference===false,'live startup never requests persistent compression');
      check(!options.temporalDepth&&!options.temporalColor,'neither native profile incurs gateway temporal recompression');
      check(options.exactDepth&&options.quality===95&&options.width===3840,'both profiles preserve native resolution and precision');
      check(requestedMode===mode&&options.fps===(mode==='latency'?60:30),'capture request and stream cadence agree');
    }
  }
  // A rapid second selection must win even if the first connection resolves late.
  let runtimeAvailable=true,restoredRuntime;
  policy.window.getGodotHybridNavigationState=()=>runtimeAvailable?savedRuntime:null;
  policy.window.setGodotHybridNavigationState=value=>{restoredRuntime=value;};
  const pending=[];
  policy.capturePresetControl={request:()=>new Promise(resolve=>pending.push(resolve)),cancel(){}};
  const oldStart=vm.runInContext('startHybridViewport()',policy);
  runtimeAvailable=false;
  const newStart=vm.runInContext('startHybridViewport()',policy);
  const previousOptions=options;
  pending[0](true);await oldStart;
  check(options===previousOptions,'a late capture request cannot launch an obsolete renderer');
  pending[1](true);await newStart;
  check(options!==previousOptions,'latest preset selection launches normally');
  check(restoredRuntime===savedRuntime,'rapid preset changes restore the complete pre-reconnect camera view');
  console.log(JSON.stringify({navigation_and_queue_checks:checks}));
})().catch(error=>{console.error(error);process.exitCode=1;});
