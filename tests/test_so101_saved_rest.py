"""Rest persistence and real-mesh planning, without any physical motor I/O."""

import json
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_modules/so101/tools"))
from so101_arm_common import ArmCalibration, ArmPairProfile
from so101_follower_service import (
    FakeFollowerBus,
    FollowerController,
    build_rest_return_waypoints,
)
from so101_kinematics import (
    _rest_collision_hulls,
    rendered_link_transforms,
    rendered_minimum_height,
    rest_mesh_minimum_height,
)
from so101_rest_pose import load_rest_pose, rest_pose_path, save_rest_pose

FOLDED_RAW = [1949, 869, 3069, 2602, 2064, 1388]
RAISED = [-45.0890281593407, 110.021978021978, 62.8131868131868,
          76.0659340659341, 4.08791208791209, 0.532978014656895]


@pytest.fixture
def pair(tmp_path):
    calibration = ArmCalibration.from_dict({
        "homing_offset": [183, 482, 1346, -120, 1668, 780], "drive_mode": [0] * 6,
        "start_pos": [739, 854, 854, 812, 0, 1377],
        "end_pos": [3435, 3207, 3069, 3122, 4095, 2878],
        "calib_mode": ["DEGREE"] * 5 + ["LINEAR"], "coordinate_system": "lerobot_urdf",
    })
    return replace(ArmPairProfile.load(ROOT / "tests/fixtures/so101_arm_pair.json"),
                   path=tmp_path / "arm_pair.json", follower_calibration=calibration)


class RecordedBus(FakeFollowerBus):
    def __init__(self, pair, pose):
        super().__init__(pair.follower_calibration.normalized_to_raw(pose))
        self.limits = list(zip(pair.follower_calibration.start_pos, pair.follower_calibration.end_pos))
        self.writes = []

    def read_position_limits(self):
        return self.limits

    def write_positions(self, positions):
        assert all(low <= raw <= high for raw, (low, high) in zip(positions, self.limits))
        self.writes.append(list(positions))
        super().write_positions(positions)


def test_folded_pose_uses_real_mesh_instead_of_empty_box_corners(pair):
    pose = pair.follower_calibration.raw_to_normalized(FOLDED_RAW)
    assert rendered_minimum_height(pose) < -0.015
    assert rest_mesh_minimum_height(pose) == pytest.approx(0.00048034, abs=1e-7)
    base_bottom = np.min(_rest_collision_hulls()["base_link"][:, 2])
    assert rest_mesh_minimum_height(pose) - base_bottom == pytest.approx(0.0028806, abs=1e-7)


def test_hulls_retain_full_mesh_floor_extrema_at_actual_encoder_angles(pair):
    import open3d as o3d

    poses = [pair.follower_calibration.raw_to_normalized(FOLDED_RAW), RAISED,
             [*RAISED[:5], 70.0], [-55, 150, 80, 95, 30, 5]]
    meshes = {}
    for name in rendered_link_transforms(RAISED):
        model = o3d.io.read_triangle_model(str(ROOT / f"robot_modules/so101/assets/{name}.glb"))
        meshes[name] = np.concatenate([np.asarray(item.mesh.vertices) for item in model.meshes])
    for pose in poses:
        expected = min(
            np.min(meshes[name] @ transform[2, :3] + transform[2, 3])
            for name, transform in rendered_link_transforms(pose, measured_angles=True).items()
        )
        assert rest_mesh_minimum_height(pose) == pytest.approx(expected, abs=1e-10)


def test_save_is_local_atomic_backed_up_and_bound_to_calibration(pair):
    assert not save_rest_pose(pair, FOLDED_RAW)["saved"]
    assert not rest_pose_path(pair).exists()
    save_rest_pose(pair, FOLDED_RAW, apply=True)
    original = rest_pose_path(pair).read_bytes()
    second = save_rest_pose(pair, FOLDED_RAW, apply=True)
    assert Path(second["backup"]).read_bytes() == original
    assert load_rest_pose(pair)["raw_positions"] == FOLDED_RAW
    with pytest.raises(RuntimeError, match="different follower"):
        load_rest_pose(replace(pair, follower_serial="different"))
    changed = replace(pair.follower_calibration, homing_offset=(184, 482, 1346, -120, 1668, 780))
    with pytest.raises(RuntimeError, match="different servo calibration"):
        load_rest_pose(replace(pair, follower_calibration=changed))


@pytest.mark.parametrize("raw", [[True, *FOLDED_RAW[1:]], [1949, 853, *FOLDED_RAW[2:]],
                                  [*FOLDED_RAW[:5], 4000]])
def test_invalid_or_out_of_range_pose_cannot_replace_saved_file(pair, raw):
    save_rest_pose(pair, FOLDED_RAW, apply=True)
    before = rest_pose_path(pair).read_bytes()
    with pytest.raises(ValueError if isinstance(raw[0], bool) else RuntimeError):
        save_rest_pose(pair, raw, apply=True)
    assert rest_pose_path(pair).read_bytes() == before


def test_below_floor_pose_is_refused_without_saving(pair):
    bad = pair.follower_calibration.normalized_to_raw([-45, 0, 0, 0, 4, 0.53])
    assert rest_mesh_minimum_height(pair.follower_calibration.raw_to_normalized(bad)) < -0.002
    with pytest.raises(ValueError, match="below the modeled"):
        save_rest_pose(pair, bad, apply=True)
    assert not rest_pose_path(pair).exists()


@pytest.mark.parametrize("base,gripper", [(-85, 0.5), (-25, 0.5), (-45, 60)])
def test_return_reloads_new_pose_and_reaches_exact_folded_encoders(pair, base, gripper):
    clock = [10.0]
    start = [base, *RAISED[1:5], gripper]
    bus = RecordedBus(pair, start)
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    assert controller.rest_pose is None
    save_rest_pose(pair, FOLDED_RAW, apply=True)
    controller.receive({"type": "arm_return_to_rest"})
    assert controller.rest_return_active, controller.status_message
    original_start = controller.rest_return_started_at
    controller.receive({"type": "arm_return_to_rest"})
    assert controller.rest_return_started_at == original_start
    for _ in range(1500):
        clock[0] += 1 / 30
        controller.update()
        controller.sample()
        if not controller.rest_return_active:
            break
    assert controller.state == "hold", controller.status_message
    assert controller.rest_return_progress == 1.0, controller.status_message
    assert bus.positions == FOLDED_RAW
    # Check every command produced by the real controller, not just waypoints.
    assert min(rest_mesh_minimum_height(pair.follower_calibration.raw_to_normalized(raw))
               for raw in bus.writes) >= -0.0006


def test_changed_limits_and_changed_saved_pose_reject_before_any_motor_write(pair):
    save_rest_pose(pair, FOLDED_RAW, apply=True)
    bus = RecordedBus(pair, RAISED)
    controller = FollowerController(pair, bus)
    controller.connect()
    bus.limits[0] = (0, 4095)
    controller.start_rest_return()
    assert "motor limits changed" in controller.status_message
    assert bus.writes == [] and not bus.torque
    bus.limits[0] = (739, 3435)
    saved = json.loads(rest_pose_path(pair).read_text())
    saved["calibration_fingerprint"] = "invalid"
    rest_pose_path(pair).write_text(json.dumps(saved))
    controller.start_rest_return()
    assert "different servo calibration" in controller.status_message
    assert bus.writes == [] and not bus.torque


def test_missing_rest_pose_reports_request_and_requires_no_motor_write(pair):
    bus = RecordedBus(pair, RAISED)
    controller = FollowerController(pair, bus, rest_pose_path=rest_pose_path(pair))
    controller.connect()
    controller.receive({"type": "arm_return_to_rest", "rest_return_request_id": "missing-pose"})
    status = controller.status()
    assert status["rest_return_request_id"] == "missing-pose"
    assert "no rest pose is saved" in status["message"]
    assert not status["rest_return_active"] and not bus.writes and not bus.torque


def test_duplicate_completed_rest_request_cannot_restart_motion(pair):
    save_rest_pose(pair, FOLDED_RAW, apply=True)
    bus = RecordedBus(pair, pair.follower_calibration.raw_to_normalized(FOLDED_RAW))
    controller = FollowerController(pair, bus)
    controller.connect()
    request = {"type": "arm_return_to_rest", "rest_return_request_id": "at-rest"}
    controller.receive(request)
    assert "already at the saved rest" in controller.status()["message"]
    assert controller.status()["rest_return_request_id"] == "at-rest"
    count = len(bus.writes)
    controller.hold("operator stopped")
    controller.receive(request)
    assert len(bus.writes) == count
    assert controller.status()["message"] == "operator stopped"


def test_mesh_route_still_rejects_endpoints_below_floor(pair):
    rest = pair.follower_calibration.raw_to_normalized(FOLDED_RAW)
    with pytest.raises(RuntimeError, match="endpoint is below"):
        build_rest_return_waypoints([-45, 0, 0, 0, 4, 0.53], rest)


def test_stalled_return_stops_even_when_teleop_feedback_setting_is_disabled(pair):
    save_rest_pose(pair, FOLDED_RAW, apply=True)
    clock = [10.0]

    class StalledBus(RecordedBus):
        def write_positions(self, positions):
            measured = list(self.positions)
            super().write_positions(positions)
            self.positions = measured

    bus = StalledBus(pair, RAISED)
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.feedback_settings["following_error_safety_enabled"] = False
    controller.start_rest_return()
    assert controller.rest_return_active
    for _ in range(150):
        clock[0] += 1 / 30
        controller.update()
        controller.sample()
        if not controller.rest_return_active:
            break
    assert controller.following_error_stop
    assert not controller.rest_return_active
    assert controller.state == "hold"
