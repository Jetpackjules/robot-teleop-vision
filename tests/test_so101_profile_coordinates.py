"""Hardware calibration is not a legacy software homing offset."""

from dataclasses import replace
import json
import math
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_modules/so101/tools"))

from so101_arm_common import ArmCalibration, ArmPairProfile, JOINT_DIRECTIONS, JOINT_OFFSETS_DEG  # noqa: E402
from so101_follower_service import FakeFollowerBus, FollowerController  # noqa: E402
from so101_kinematics import JOINT_LIMITS_RAD, overlay_joint_radians  # noqa: E402
from scripts.repair_so101_profile_coordinates import repair_profile  # noqa: E402


def calibration_data():
    return {
        "homing_offset": [183, 482, 1346, -120, 1668, 780],
        "drive_mode": [0] * 6,
        "start_pos": [739, 854, 854, 812, 0, 1377],
        "end_pos": [3435, 3207, 3069, 3122, 4095, 2878],
        "calib_mode": ["DEGREE"] * 5 + ["LINEAR"],
        "coordinate_system": "lerobot_urdf",
    }


def test_hardware_coordinates_match_lerobot_degrees_and_model_joint_frames():
    data = calibration_data()
    calibration = ArmCalibration.from_dict(data)
    for fraction in (0, 0.1, 0.3, 0.5, 0.7, 0.9, 1):
        raw = [round(low + fraction * (high - low)) for low, high in zip(
            data["start_pos"], data["end_pos"], strict=True,
        )]
        normalized = calibration.raw_to_normalized(raw)
        assert calibration.normalized_to_raw(normalized) == raw
        for joint in range(5):
            midpoint = (data["start_pos"][joint] + data["end_pos"][joint]) / 2
            expected_degrees = (raw[joint] - midpoint) * 360 / 4095
            assert normalized[joint] * JOINT_DIRECTIONS[joint] + JOINT_OFFSETS_DEG[joint] == pytest.approx(expected_degrees)
            if JOINT_LIMITS_RAD[joint][0] <= math.radians(expected_degrees) <= JOINT_LIMITS_RAD[joint][1]:
                assert overlay_joint_radians(normalized)[joint] == pytest.approx(math.radians(expected_degrees))
    # Firmware already applied these values: adding them in software a second
    # time must not alter the decoded hardware-calibrated pose.
    without_offsets = replace(calibration, homing_offset=(0,) * 6)
    assert without_offsets.raw_to_normalized(raw) == calibration.raw_to_normalized(raw)


@pytest.mark.parametrize("coordinate_system", ["legacy", "lerobot_urdf"])
def test_unclipped_validation_preserves_out_of_range_gripper_targets(coordinate_system):
    calibration = ArmCalibration.from_dict({**calibration_data(), "coordinate_system": coordinate_system})
    for value in (-10, 110):
        pose = [0, 130, 38, 78, 0, value]
        raw = calibration.normalized_to_raw_unclipped(pose)[5]
        assert not calibration.start_pos[5] <= raw <= calibration.end_pos[5]
        clipped = calibration.normalized_to_raw(pose)[5]
        assert calibration.start_pos[5] <= clipped <= calibration.end_pos[5]


def test_neutral_hardware_pose_and_all_calibration_stages_stay_in_motor_ranges():
    calibration = ArmCalibration.from_dict(calibration_data())
    profile = replace(ArmPairProfile.load(ROOT / "tests/fixtures/so101_arm_pair.json"),
                      follower_calibration=calibration)
    limits = list(zip(calibration.start_pos, calibration.end_pos, strict=True))

    class LimitedBus(FakeFollowerBus):
        def read_position_limits(self):
            return limits

        def write_positions(self, positions):
            assert all(low <= raw <= high for raw, (low, high) in zip(positions, limits, strict=True))
            super().write_positions(positions)

    raw = [1949, 869, 3069, 2602, 2064, 1388]
    clock = [10.0]
    bus = LimitedBus(raw)
    controller = FollowerController(profile, bus, now=lambda: clock[0])
    controller.connect()
    assert controller.follower_normalized == pytest.approx([-52.56, 182.11, 97.36, 125.82, 1.45, 0.73], abs=0.01)
    for mode, attempt in (("axis", 1), ("joints", 1), ("wrist", 1), ("wrist", 2), ("claw", 1)):
        controller.start_calibration_sweep(mode, attempt)
        assert controller.calibration_sweep_active, controller.calibration_rejection
        for _ in range(int(controller.calibration_sweep_duration * 30) + 90):
            clock[0] += 1 / 30
            controller.update()
            controller.sample()
            if not controller.calibration_sweep_active:
                break
        assert controller.calibration_request_state == "completed", controller.status_message
        assert not controller.following_error_stop
        assert bus.positions[1] > raw[1] + 200  # shoulder moves away from its minimum
        assert bus.positions[2] < raw[2] - 200  # elbow moves away from its maximum


def test_changed_hardware_limits_reject_modern_profile_before_motor_writes():
    calibration = ArmCalibration.from_dict(calibration_data())
    profile = replace(ArmPairProfile.load(ROOT / "tests/fixtures/so101_arm_pair.json"),
                      follower_calibration=calibration)

    class ChangedBus(FakeFollowerBus):
        def write_positions(self, positions):
            pytest.fail("Hardware/profile mismatch must not send motor commands")

    bus = ChangedBus([1949, 869, 3069, 2602, 2064, 1388])
    controller = FollowerController(profile, bus)
    controller.connect()
    controller.start_calibration_sweep("axis")
    assert controller.calibration_request_state == "rejected"
    assert "limits changed" in controller.calibration_rejection
    assert not bus.torque


@pytest.mark.parametrize("field,value", [
    ("coordinate_system", "unknown"), ("drive_mode", [1] * 6),
    ("calib_mode", ["DEGREE"] * 6), ("end_pos", [0] * 6),
])
def test_invalid_coordinate_conventions_are_rejected(field, value):
    with pytest.raises(ValueError):
        ArmCalibration.from_dict({**calibration_data(), field: value})


@pytest.fixture
def files(tmp_path):
    profile = json.loads((ROOT / "tests/fixtures/so101_arm_pair.json").read_text())
    data = calibration_data()
    del data["coordinate_system"]
    profile["follower"]["calibration"] = data
    path = tmp_path / "arm_pair.json"
    path.write_text(json.dumps(profile), encoding="utf-8")
    rows = []
    for index, name in enumerate(profile["motor_names"]):
        rows.append({"id": index + 1, "name": name, "errors": {}, "registers": {
            "Homing_Offset": data["homing_offset"][index],
            "Min_Position_Limit": data["start_pos"][index],
            "Max_Position_Limit": data["end_pos"][index],
            "Present_Position": [1949, 869, 3069, 2602, 2064, 1388][index],
        }})
    report = {"port": profile["follower"]["port"], "motors": rows}
    report_path = tmp_path / "diagnostics.json"
    report_path.write_text(json.dumps(report), encoding="utf-8")
    return path, report_path, report


def test_profile_repair_previews_backs_up_preserves_other_settings_and_is_idempotent(files):
    path, report_path, _report = files
    original = path.read_bytes()
    assert not repair_profile(path, report_path)["changed"]
    assert path.read_bytes() == original
    result = repair_profile(path, report_path, apply=True)
    assert Path(result["backup"]).read_bytes() == original
    actual = json.loads(path.read_bytes())
    expected = json.loads(original)
    expected["follower"]["calibration"]["coordinate_system"] = "lerobot_urdf"
    assert actual == expected
    assert not repair_profile(path, report_path, apply=True)["changed"]


@pytest.mark.parametrize("mismatch", ["port", "offset", "range", "id", "read_failure"])
def test_profile_repair_rejects_mismatched_or_incomplete_motor_evidence(files, mismatch):
    path, report_path, report = files
    original = path.read_bytes()
    if mismatch == "port":
        report["port"] = "different-device"
    elif mismatch == "offset":
        report["motors"][0]["registers"]["Homing_Offset"] += 1
    elif mismatch == "range":
        report["motors"][0]["registers"]["Max_Position_Limit"] -= 1
    elif mismatch == "id":
        report["motors"][0]["id"] = 6
    else:
        report["motors"][0]["errors"] = {"Homing_Offset": "read failed"}
    report_path.write_text(json.dumps(report), encoding="utf-8")
    with pytest.raises(ValueError):
        repair_profile(path, report_path, apply=True)
    assert path.read_bytes() == original
