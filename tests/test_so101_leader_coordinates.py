"""Compare browser conversion and physical raw-packet routing to Python."""

import json
import shutil
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_modules/so101/tools"))
from so101_arm_common import ArmCalibration, ArmPairProfile
from so101_follower_service import FakeFollowerBus, FollowerController


def hardware_calibration():
    return {
        "homing_offset": [183, 482, 1346, -120, 1668, 780],
        "drive_mode": [0] * 6,
        "start_pos": [739, 854, 854, 812, 0, 1377],
        "end_pos": [3435, 3207, 3069, 3122, 4095, 2878],
        "calib_mode": ["DEGREE"] * 5 + ["LINEAR"],
        "coordinate_system": "lerobot_urdf",
    }


def run_node(tmp_path, source, payload, *, production=False):
    node = shutil.which("node")
    if not node:
        pytest.skip("Node is required for executable browser coordinate tests")
    module = (
        "robot_modules/so101/web/so101_web_serial.js" if production
        else "examples/alignment_demo/web/simulation_controller.js"
    )
    shutil.copyfile(ROOT / module, tmp_path / "controller.mjs")
    (tmp_path / "probe.mjs").write_text(source, encoding="utf-8")
    result = subprocess.run(
        [node, str(tmp_path / "probe.mjs")], input=json.dumps(payload),
        capture_output=True, text=True, check=True, timeout=10,
    )
    return json.loads(result.stdout)


def test_demo_matches_python_for_both_coordinate_systems(tmp_path):
    cases = []
    legacy = json.loads((ROOT / "tests/fixtures/so101_arm_pair.json").read_text())["leader"]["calibration"]
    for calibration in [legacy, hardware_calibration()]:
        for fraction in [0, 0.2, 0.5, 0.8, 1]:
            raw = [round(low + fraction * (high - low)) for low, high in zip(
                calibration["start_pos"], calibration["end_pos"], strict=True,
            )]
            cases.append({"calibration": calibration, "raw": raw})
    actual = run_node(tmp_path, """
import {readFileSync} from 'node:fs';
import {parseLeaderCalibration, normalizeLeaderPositions} from './controller.mjs';
const cases = JSON.parse(readFileSync(0, 'utf8'));
console.log(JSON.stringify(cases.map(item => normalizeLeaderPositions(item.raw, parseLeaderCalibration(item.calibration)))));
""", cases)
    for case, values in zip(cases, actual, strict=True):
        assert values == pytest.approx(ArmCalibration.from_dict(case["calibration"]).raw_to_normalized(case["raw"]))


def test_demo_rejects_unsupported_modern_coordinate_settings(tmp_path):
    cases = []
    for field, value in [("coordinate_system", "unknown"), ("drive_mode", [1] * 6),
                         ("calib_mode", ["DEGREE"] * 6), ("start_pos", [-1] * 6)]:
        cases.append({**hardware_calibration(), field: value})
    actual = run_node(tmp_path, """
import {readFileSync} from 'node:fs';
import {parseLeaderCalibration} from './controller.mjs';
console.log(JSON.stringify(JSON.parse(readFileSync(0, 'utf8')).map(item => {
  try { parseLeaderCalibration(item); return false; } catch { return true; }
})));
""", cases)
    assert all(actual)


def test_physical_browser_raw_packets_give_one_to_one_arm_joint_deltas(tmp_path):
    initial = [1800, 1900, 2200, 2000, 2000, 1800]
    delta = [12, 18, -9, 14, -11, 5]
    messages = run_node(tmp_path, """
import {readFileSync} from 'node:fs';
// Avoid constructing the live controller/socket. Invoke the real poller with
// serial replies and its output transport replaced by in-memory fixtures.
globalThis.window = {so101ArmController: {}, addEventListener() {}};
const {So101ArmController} = await import('./controller.mjs');
const inputs = JSON.parse(readFileSync(0, 'utf8'));
const controller = Object.create(So101ArmController.prototype);
Object.assign(controller, {responses: new Map(), seq: 0, sendTimes: [],
  latest: {leaderConnected: true}, lastLeaderPositions: null,
  lastLeaderReadTimes: new Map(), leaderGeneration: 0, leaderReadFault: false});
const sent = [];
controller.send = value => sent.push(value);
controller.emitStatus = () => {};
controller.scheduleLoop = () => {};
for (const positions of inputs) {
  controller.port = {writable: {getWriter: () => ({
    write: async () => positions.forEach((value, i) => controller.responses.set(i + 1, value)),
    releaseLock() {},
  })}};
  await controller.pollLeader();
}
console.log(JSON.stringify(sent));
""", [initial, [a + b for a, b in zip(initial, delta, strict=True)]], production=True)
    assert [message["positions"] for message in messages] == [initial, [a + b for a, b in zip(initial, delta, strict=True)]]
    leader = ArmCalibration.from_dict(hardware_calibration())
    follower = ArmCalibration.from_dict({**hardware_calibration(), "homing_offset": [0] * 6})
    pair = replace(ArmPairProfile.load(ROOT / "tests/fixtures/so101_arm_pair.json"),
                   leader_calibration=leader, follower_calibration=follower)
    raw_start = follower.normalized_to_raw([-40, 130, 38, 78, 0, 25])
    bus = FakeFollowerBus(raw_start)
    controller = FollowerController(pair, bus)
    controller.connect()
    controller.receive(messages[0])
    controller.enable("leader")
    assert controller.state == "armed"
    controller.receive(messages[1])
    controller.update()
    assert bus.positions == [a + b for a, b in zip(raw_start, delta, strict=True)]


LEADER_SERIAL_PROBE = """
globalThis.window = {so101ArmController: {}, addEventListener() {}, clearTimeout() {}};
let clock = 0;
globalThis.performance = {now: () => clock};
globalThis.setTimeout = (callback, delay) => {clock += delay; queueMicrotask(callback); return 0;};
const {So101ArmController} = await import('./controller.mjs');
const controller = Object.create(So101ArmController.prototype);
Object.assign(controller, {responses: new Map(), seq: 0, sendTimes: [],
  latest: {leaderConnected: true, controlMode: 'leader', serverConnected: true}, lastLeaderPositions: null,
  lastLeaderReadTimes: new Map(), leaderGeneration: 0, leaderReadFault: false,
  keyboardKeys: new Set(), keyboardKeyOrder: new Map(),
  rxBuffer: new Uint8Array(0), reader: null, reading: true});
const sent = [], scheduled = [];
controller.send = value => sent.push(value);
controller.emitStatus = () => {};
controller.scheduleLoop = delay => scheduled.push(delay);
let replies = [1800, 1900, 2200, 2000, 2000, 1800];
controller.port = {close: async () => {}, writable: {getWriter: () => ({
  write: async () => replies.forEach((value, i) => {
    if (value !== null) controller.responses.set(i + 1, value);
  }), releaseLock() {},
})}};
"""


@pytest.mark.parametrize("armed", [False, True])
def test_missing_leader_encoder_cannot_be_reused_indefinitely(tmp_path, armed):
    actual = run_node(tmp_path, LEADER_SERIAL_PROBE + f"controller.latest.armed = {str(armed).lower()};" + """
await controller.pollLeader();
replies[1] = null;
await controller.pollLeader();
const briefDropoutCommands = sent.filter(item => item.type === 'arm_command').length;
for (let i = 0; i < 4; i++) await controller.pollLeader();
const beforeRecovery = sent.filter(item => item.type === 'arm_command').length;
let enableError = '';
try {controller.enableArm();} catch (error) {enableError = error.message;}
replies[1] = 1901;
await controller.pollLeader();
console.log(JSON.stringify({briefDropoutCommands, beforeRecovery, enableError,
  afterRecovery: sent.filter(item => item.type === 'arm_command').length,
  holds: sent.filter(item => item.type === 'arm_hold').length,
  enables: sent.filter(item => item.type === 'arm_enable').length}));
""", {}, production=True)
    assert actual["briefDropoutCommands"] == 2
    assert actual["beforeRecovery"] == 3
    assert actual["holds"] == int(armed)
    assert "fresh readings" in actual["enableError"]
    assert actual["afterRecovery"] == 4
    assert actual["enables"] == 0


def test_disconnect_clears_cached_encoders_and_cancels_inflight_poll(tmp_path):
    actual = run_node(tmp_path, LEADER_SERIAL_PROBE + """
controller.lastLeaderPositions = [1, 2, 3, 4, 5, 6];
controller.rxBuffer = new Uint8Array([255, 255, 1]);
let completeWrite;
controller.port.writable.getWriter = () => ({
  write: () => new Promise(resolve => {completeWrite = resolve;}), releaseLock() {},
});
const pending = controller.pollLeader();
await controller.disconnectLeader(false);
replies.forEach((value, i) => controller.responses.set(i + 1, value));
completeWrite();
await pending;
console.log(JSON.stringify({cached: controller.lastLeaderPositions,
  buffered: controller.rxBuffer.length, sent, scheduled}));
""", {}, production=True)
    assert actual["cached"] is None
    assert actual["buffered"] == 0
    assert actual["sent"] == []
    assert actual["scheduled"] == []


def test_unexpected_serial_end_is_reported(tmp_path):
    actual = run_node(tmp_path, LEADER_SERIAL_PROBE + """
controller.port.readable = {getReader: () => ({read: async () => ({done: true}), releaseLock() {}})};
let error = '';
try {await controller.readPump();} catch (failure) {error = failure.message;}
console.log(JSON.stringify({error}));
""", {}, production=True)
    assert "ended" in actual["error"]


def test_cancelled_open_does_not_restart_leader_streaming(tmp_path):
    actual = run_node(tmp_path, LEADER_SERIAL_PROBE + """
controller.port = null;
controller.latest.leaderConnected = false;
let completeOpen, closes = 0, pumps = 0;
const candidate = {open: () => new Promise(resolve => {completeOpen = resolve;}),
  close: async () => {closes++;}};
controller.readPump = async () => {pumps++;};
const pending = controller.openLeader(candidate);
await Promise.resolve();
await controller.disconnectLeader(false);
completeOpen();
await pending;
console.log(JSON.stringify({closes, pumps, connected: controller.latest.leaderConnected, scheduled}));
""", {}, production=True)
    assert actual == {"closes": 1, "pumps": 0, "connected": False, "scheduled": []}


def test_late_bytes_from_disconnected_reader_are_discarded(tmp_path):
    actual = run_node(tmp_path, LEADER_SERIAL_PROBE + """
let completeRead, parses = 0;
controller.port.readable = {getReader: () => ({
  read: () => new Promise(resolve => {completeRead = resolve;}),
  cancel: async () => {}, releaseLock() {},
})};
controller.parsePackets = () => {parses++;};
const pending = controller.readPump();
await controller.disconnectLeader(false);
completeRead({done: false, value: new Uint8Array([255, 255, 1])});
await pending;
console.log(JSON.stringify({parses, buffered: controller.rxBuffer.length}));
""", {}, production=True)
    assert actual == {"parses": 0, "buffered": 0}


def test_disconnect_releases_pending_writer_before_closing_port(tmp_path):
    actual = run_node(tmp_path, LEADER_SERIAL_PROBE + """
let finishWrite, released = false, closed = false, aborted = false;
controller.port.writable.getWriter = () => ({
  write: () => new Promise(resolve => {finishWrite = resolve;}),
  abort: async () => {aborted = true; finishWrite();},
  releaseLock: () => {released = true;},
});
controller.port.close = async () => {if (!released) throw new Error('writer locked'); closed = true;};
const pending = controller.pollLeader();
await controller.disconnectLeader(false);
await pending;
console.log(JSON.stringify({aborted, released, closed, sent, scheduled}));
""", {}, production=True)
    assert actual == {"aborted": True, "released": True, "closed": True, "sent": [], "scheduled": []}
