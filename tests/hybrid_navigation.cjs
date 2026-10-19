// Exercise the production camera and pointer math without a browser or device.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const source = fs.readFileSync(path.join(__dirname, '../web/hybrid_rgbd_renderer.js'), 'utf8')
  .replace(/^import .*;\r?\n/gm, '').replace(/^export /gm, '');
const context = vm.createContext({window: {}, performance, FUSION_FRAGMENT_SHADER: ''});
vm.runInContext(source + '\nwindow.navigationProbe = {state, orbitViewer, attachPointerNavigation, identityMatrix, cameraMarkerGeometry};', context);
const {state, orbitViewer, attachPointerNavigation, identityMatrix, cameraMarkerGeometry} = context.window.navigationProbe;
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
console.log(JSON.stringify({navigation_checks: checks}));
