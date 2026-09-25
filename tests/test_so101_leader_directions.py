"""Per-pair direction corrections must affect only relative leader motion."""

from dataclasses import asdict
import json
from pathlib import Path
import sys
import subprocess
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_modules/so101/tools"))

from so101_arm_common import ArmPairProfile
from so101_follower_service import FollowerController
from test_so101_leader_limits import LimitedBus, calibrated_pair
from scripts.set_so101_leader_directions import set_directions

DIRECTIONS = [1, -1, -1, 1, 1, 1]


def write_profile(tmp_path, directions=DIRECTIONS, coordinate_system="lerobot_urdf"):
    pair = calibrated_pair()
    payload = json.loads((ROOT / "tests/fixtures/so101_arm_pair.json").read_text())
    for arm in ("leader", "follower"):
        payload[arm]["calibration"] = {
            **asdict(getattr(pair, f"{arm}_calibration")), "coordinate_system": coordinate_system,
        }
    payload["leader_joint_directions"] = directions
    path = tmp_path / "arm_pair.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


@pytest.mark.parametrize("coordinate_system", ["legacy", "lerobot_urdf"])
@pytest.mark.parametrize("joint", range(6))
@pytest.mark.parametrize("delta", [-10, 10])
def test_reverse_only_selected_joints_without_changing_gain_or_enable_pose(
    tmp_path, coordinate_system, joint, delta,
):
    pair = ArmPairProfile.load(write_profile(tmp_path, coordinate_system=coordinate_system))
    limits = list(zip(pair.follower_calibration.start_pos, pair.follower_calibration.end_pos))
    start = [round((low + high) / 2) + 25 for low, high in limits]
    bus = LimitedBus(start, limits)
    clock = [10.0]
    controller = FollowerController(pair, bus, now=lambda: clock[0], rest_pose_path=tmp_path / "rest.json")
    controller.connect()
    leader = [round((low + high) / 2) - 50 for low, high in zip(
        pair.leader_calibration.start_pos, pair.leader_calibration.end_pos,
    )]

    def send(seq, raw):
        clock[0] += 0.04
        controller.receive({
            "type": "arm_command", "seq": seq, "positions": raw,
            "sent_unix_ms": int(time.time() * 1000),
        })

    send(0, leader)
    controller.enable("leader")
    controller.update()
    assert controller.state == "armed"
    assert bus.positions == start  # Inverting a direction must never introduce an enable jump.

    moved = list(leader)
    moved[joint] += delta
    send(1, moved)
    controller.update()
    controller.sample()
    gain = 1 if joint < 5 else (
        (pair.follower_calibration.end_pos[5] - pair.follower_calibration.start_pos[5])
        / (pair.leader_calibration.end_pos[5] - pair.leader_calibration.start_pos[5])
    )
    expected = list(start)
    expected[joint] = round(start[joint] + DIRECTIONS[joint] * delta * gain)
    assert bus.positions == expected
    assert controller.state == "armed"

    send(2, leader)
    controller.update()
    controller.sample()
    assert bus.positions == start
    controller.hold("test re-enable")
    controller.enable("leader")
    controller.update()
    assert bus.positions == start


@pytest.mark.parametrize("directions", [None, [], [1] * 5, [1] * 7, [1, 0, 1, 1, 1, 1],
                                        [1, True, 1, 1, 1, 1], [1, "-1", 1, 1, 1, 1],
                                        [1, 2, 1, 1, 1, 1], "1,-1,-1,1,1,1"])
def test_invalid_direction_profile_rejected_before_connect(tmp_path, directions):
    with pytest.raises(ValueError, match="leader_joint_directions"):
        ArmPairProfile.load(write_profile(tmp_path, directions))


def test_missing_direction_setting_retains_existing_behavior():
    pair = ArmPairProfile.load(ROOT / "tests/fixtures/so101_arm_pair.json")
    assert pair.leader_joint_directions == (1, 1, 1, 1, 1, 1)


def test_direction_settings_are_visible_in_status_and_fault_report(tmp_path):
    pair = ArmPairProfile.load(write_profile(tmp_path))
    limits = list(zip(pair.follower_calibration.start_pos, pair.follower_calibration.end_pos))
    bus = LimitedBus([round((low + high) / 2) for low, high in limits], limits)
    report_path = tmp_path / "fault.json"
    controller = FollowerController(pair, bus, rest_pose_path=tmp_path / "rest.json",
                                    fault_report_path=report_path)
    controller.connect()
    assert controller.status()["leader_joint_directions"] == DIRECTIONS
    controller.fail("test fault")
    assert json.loads(report_path.read_text())["leader_joint_directions"] == DIRECTIONS


def test_file_only_update_preserves_calibrations_connections_and_bom_backup(tmp_path):
    path = write_profile(tmp_path, [1] * 6)
    payload = json.loads(path.read_text())
    payload["leader"]["port"] = "COM8"
    payload["follower"]["port"] = "COM5"
    payload["site_note"] = "机械臂"
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8-sig")
    original = path.read_bytes()
    preview = set_directions(path, [2, 3])
    assert preview["changed"] is False
    assert path.read_bytes() == original
    assert list(tmp_path.iterdir()) == [path]

    result = set_directions(path, [2, 3], apply=True)
    assert result["changed"] is True
    assert Path(result["backup"]).read_bytes() == original
    payload["leader_joint_directions"] = DIRECTIONS
    assert json.loads(path.read_text(encoding="utf-8")) == payload
    assert ArmPairProfile.load(path).leader_joint_directions == tuple(DIRECTIONS)
    assert set_directions(path, [2, 3], apply=True)["changed"] is False
    assert len(list(tmp_path.glob("*.bak"))) == 1  # Re-running never toggles the signs.
    assert set_directions(path, [], apply=True)["changed"] is True
    assert ArmPairProfile.load(path).leader_joint_directions == (1,) * 6


def test_direction_command_uses_active_config_profile(tmp_path):
    path = write_profile(tmp_path, [1] * 6)
    config = tmp_path / "local.toml"
    config.write_text(f'[robot]\nadapter = "so101"\n[robot.options]\nprofile = "{path.as_posix()}"\n')
    result = subprocess.run(
        [sys.executable, str(ROOT / "scripts/set_so101_leader_directions.py"),
         "--config", str(config), "--invert", "2", "3", "--apply"],
        cwd=tmp_path, capture_output=True, text=True,
    )
    assert result.returncode == 0, result.stderr
    assert str(path) in result.stdout
    assert "Restart" in result.stdout
    assert ArmPairProfile.load(path).leader_joint_directions == tuple(DIRECTIONS)
