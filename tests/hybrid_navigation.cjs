// Exercise the production camera and pointer math without a browser or device.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const source = fs.readFileSync(path.join(__dirname, '../web/hybrid_rgbd_renderer.js'), 'utf8')
  .replace(/^import .*;\r?\n/gm, '').replace(/^export /gm, '');
const context = vm.createContext({window: {}, performance, FUSION_FRAGMENT_SHADER: ''});
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
check(Math.abs(initial[13] - (focus[1] + 0.35)) < 1e-6, 'initial camera height is preserved');
check(Math.abs(initial[14] - (focus[2] + 0.65)) < 1e-6, 'initial camera distance is preserved');
const listeners = {};
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
  const controls = new Map();
  const control = id => {
    if (id === 'persistent-temporal-reference-enabled') return null;
    if (!controls.has(id)) controls.set(id,{value:'1',checked:false});
    return controls.get(id);
  };
  let options;
  const policy = vm.createContext({URLSearchParams, document:{getElementById:control}, robotModule:{},
    viewSettingsVersion:14,displayModes:['point_cloud','independent_mesh','blendpcr','rgb_camera'],
    defaultDisplayMode:'independent_mesh',displayModeSelect:control('display-mode'),
    streamPresetSelect:control('stream-preset'),localSavedViewSettings:()=>null,
    updateHeadTrackingOptions(){},updateViewLabels(){},stopViewport(){},showHybrid(){},updateStatus(){},
    currentPreset:()=>({label:'Quality'}),startHybridControlPeer:async()=>true,
    hybridCanvas:{},rgbCameraCanvas:{},rgbdDataChannel:{},startupNavigation:null,startupNavigationPending:false,
    applyDisplayMode(){},window:{location:{search:'?rgbd_transport=websocket'},
      startGodotHybridRenderer:async value=>{options=value;}}});
  vm.runInContext(section('function viewSettingsPayload(', 'function updateViewLabels(')
    +section('function restoreViewSettings(', 'async function fetchSiteSettings(')
    +section('async function startHybridViewport(', 'function startSelectedViewport(')
    +section('function applyStreamQualityPolicy(', 'streamPresetSelect.addEventListener('),policy);
  for(const legacy of [undefined,false,true]) {
    policy.saved={settings_version:14,persistent_temporal_reference_enabled:legacy,
      fov:71,orbit_distance:.83,display_mode:'blendpcr'};
    vm.runInContext('restoreViewSettings(saved);payload=viewSettingsPayload();',policy);
    check(policy.payload.persistent_temporal_reference_enabled===false,'old or absent saved compression stays off');
    check(policy.payload.fov===71&&policy.payload.orbit_distance===.83,'unrelated saved camera preferences are preserved');
    for(const mode of ['quality','latency']) {
      control('stream-preset').value=mode;
      await vm.runInContext('startHybridViewport()',policy);
      check(options.persistentReference===false,'live startup never requests persistent compression');
      check(options.temporalDepth===(mode==='latency'),'Quality uses native depth while Low latency keeps its existing codec');
      if(mode==='quality')check(options.exactDepth&&options.quality===95&&options.temporalColor===false,'native Quality remains exact with full RGB');
    }
  }
  console.log(JSON.stringify({navigation_and_queue_checks:checks}));
})().catch(error=>{console.error(error);process.exitCode=1;});
