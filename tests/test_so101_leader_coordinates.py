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
  latest: {leaderConnected: true}, lastLeaderPositions: null});
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
