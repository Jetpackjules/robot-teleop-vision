"""Execute production browser setup, including old-service and timeout behavior."""

import json
import shutil
import subprocess

import pytest

from test_so101_leader_coordinates import ROOT, run_node


def test_browser_save_never_sends_motion_and_requires_acknowledgement(tmp_path):
    result = run_node(tmp_path, """
globalThis.window = {so101ArmController: {}, addEventListener() {},
  setTimeout(callback) { callbacks.push(callback); }};
const callbacks = [], sent = [], errors = [];
const {So101ArmController} = await import('./controller.mjs');
const c = Object.create(So101ArmController.prototype);
c.latest = {serverConnected: true, state: 'ready', status: {}};
c.emitStatus = () => {};
c.send = message => {sent.push(message); return true;};
try { c.saveCurrentRestPose(); } catch(e) { errors.push(e.message); }
c.latest.status.rest_pose_save_supported = true;
c.latest.state = 'armed';
try { c.saveCurrentRestPose(); } catch(e) { errors.push(e.message); }
c.latest.state = 'hold';
c.saveCurrentRestPose();
callbacks.pop()(); // Missing acknowledgement must not imply a successful save.
const timeout = c.restSaveError;
c.saveCurrentRestPose();
c.latest.status.rest_save_request_id = c.restSaveRequestId;
c.latest.status.rest_save_message = 'saved';
callbacks.pop()();
console.log(JSON.stringify({sent, errors, timeout, afterAck: c.restSaveError}));
""", {}, production=True)
    assert len(result["errors"]) == 2
    assert "Restart" in result["errors"][0] and "Hold" in result["errors"][1]
    assert len(result["sent"]) == 2
    assert all(m["type"] == "arm_save_rest_pose" and m["rest_save_request_id"] for m in result["sent"])
    assert result["sent"][0]["rest_save_request_id"] != result["sent"][1]["rest_save_request_id"]
    assert "not resent" in result["timeout"] and result["afterAck"] == ""


def test_missing_rest_setup_and_old_service_controls_render_without_crashing(tmp_path):
    node = shutil.which("node")
    if not node:
        pytest.skip("Node required for browser setup checks")
    shutil.copyfile(ROOT / "robot_modules/so101/web/module.js", tmp_path / "module.mjs")
    (tmp_path / "probe.mjs").write_text("""
const elements = new Map();
globalThis.document = {querySelectorAll() { return []; }, getElementById(id) {
  if (!elements.has(id)) elements.set(id, {classList: {toggle() {}}, textContent: ''});
  return elements.get(id);
}};
let arm = {serverConnected: true, followerConnected: true, state: 'ready', status: {}};
globalThis.window = {getSo101ArmStatus: () => arm, so101ArmController: {}};
const {createRobotModule} = await import('./module.mjs');
const module = createRobotModule({});
function render() {
  module.renderStatus();
  return Object.fromEntries(['so101-return-rest', 'so101-save-rest', 'so101-rest-status',
    'so101-setup-warnings', 'so101-health'].map(id => [id, {...elements.get(id)}]));
}
const old = render();
arm.status = {rest_pose_available: false, rest_pose_save_supported: true,
  rest_pose_fault: 'no rest pose is saved', setup_warnings: ['Different coordinate conventions'],
  leader_joint_directions: [1,-1,1,1,1,1], leader_coordinate_system: 'legacy', follower_coordinate_system: 'lerobot_urdf'};
const missing = render();
arm.status.rest_pose_available = true;
const saved = render();
window.so101ArmController = {restSaveRequestId: 'new-save', restSavePendingUntil: Date.now() + 3000};
const pending = render();
arm.status.rest_save_request_id = 'new-save';
arm.status.rest_save_message = 'Current physical pose saved as rest.';
const ack = render();
console.log(JSON.stringify({old, missing, saved, pending, ack}));
""", encoding="utf-8")
    process = subprocess.run([node, str(tmp_path / "probe.mjs")], capture_output=True,
                             text=True, check=True, timeout=10)
    views = json.loads(process.stdout)
    assert views["old"]["so101-save-rest"]["disabled"]
    assert "Restart" in views["old"]["so101-setup-warnings"]["textContent"]
    assert views["missing"]["so101-return-rest"]["disabled"]
    assert not views["missing"]["so101-save-rest"]["disabled"]
    assert "Save Current Pose as Rest" in views["missing"]["so101-rest-status"]["textContent"]
    assert "Different coordinate" in views["missing"]["so101-setup-warnings"]["textContent"]
    assert "legacy / follower lerobot_urdf" in views["missing"]["so101-health"]["textContent"]
    assert not views["saved"]["so101-return-rest"]["disabled"]
    assert views["pending"]["so101-save-rest"]["disabled"]
    assert "waiting" in views["pending"]["so101-rest-status"]["textContent"]
    assert not views["ack"]["so101-save-rest"]["disabled"]
    assert "saved as rest" in views["ack"]["so101-rest-status"]["textContent"]
