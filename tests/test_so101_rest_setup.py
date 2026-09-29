"""Rest setup actions must persist fresh measurements without actuating the arm."""

from dataclasses import replace
import time

import pytest

from test_so101_saved_rest import FOLDED_RAW, RAISED, RecordedBus, pair  # noqa: F401
from so101_arm_common import ArmProtocolError, validate_browser_arm_message
from so101_follower_service import FollowerController
from so101_rest_pose import load_rest_pose, rest_pose_path, save_rest_pose


def save_request(request_id="save-1"):
    return {"type": "arm_save_rest_pose", "rest_save_request_id": request_id}


def test_save_reads_current_encoders_without_motion_and_duplicates_do_not_resave(pair):
    bus = RecordedBus(pair, RAISED)
    controller = FollowerController(pair, bus)
    controller.connect()
    # Move since the last telemetry snapshot; the action must read again now.
    bus.positions = list(FOLDED_RAW)
    controller.receive(save_request())
    assert load_rest_pose(pair)["raw_positions"] == FOLDED_RAW
    status = controller.status()
    assert status["rest_save_state"] == "saved"
    assert status["rest_pose_available"] and status["rest_pose_save_supported"]
    assert status["rest_save_request_id"] == "save-1"
    assert not bus.writes and not bus.torque and controller.state == "ready"
    saved = rest_pose_path(pair).read_bytes()
    bus.positions = pair.follower_calibration.normalized_to_raw(RAISED)
    controller.receive(save_request())
    assert rest_pose_path(pair).read_bytes() == saved
    assert not list(pair.path.parent.glob("*.bak"))
    controller.receive(save_request("save-2"))
    assert load_rest_pose(pair)["raw_positions"] == bus.positions
    assert next(pair.path.parent.glob("*.bak")).read_bytes() == saved


@pytest.mark.parametrize("failure", ["armed", "calibrating", "returning_rest", "fault", "offline", "read", "limits", "floor"])
def test_rejected_save_preserves_old_pose_and_does_not_actuate(pair, failure):
    save_rest_pose(pair, FOLDED_RAW, apply=True)
    original = rest_pose_path(pair).read_bytes()
    bus = RecordedBus(pair, RAISED)
    controller = FollowerController(pair, bus)
    controller.connect()
    if failure == "offline":
        bus.connected = False
    elif failure == "read":
        def failed_read():
            raise OSError("encoder unavailable")
        bus.read_positions = failed_read
    elif failure == "limits":
        bus.limits[1] = (0, 4095)
    elif failure == "floor":
        bus.positions = pair.follower_calibration.normalized_to_raw([-45, 0, 0, 0, 4, 0.53])
    else:
        controller.state = failure
    controller.receive(save_request())
    assert controller.rest_save_state == "rejected"
    assert "not saved" in controller.rest_save_message
    assert rest_pose_path(pair).read_bytes() == original
    assert not bus.writes and not bus.torque


def test_save_honors_explicit_rest_path_and_status_notices_cli_changes(pair):
    clock = [10.0]
    path = pair.path.with_name("custom-rest.json")
    bus = RecordedBus(pair, RAISED)
    controller = FollowerController(pair, bus, rest_pose_path=path, now=lambda: clock[0])
    controller.connect()
    assert not controller.status()["rest_pose_available"]
    save_rest_pose(pair, FOLDED_RAW, apply=True, path=path)
    clock[0] += 2.1
    assert controller.status()["rest_pose_available"]
    controller.receive(save_request())
    assert load_rest_pose(pair, path)["raw_positions"] == bus.positions
    assert not rest_pose_path(pair).exists()
    path.unlink()
    clock[0] += 2.1
    assert not controller.status()["rest_pose_available"]


def test_saving_counts_as_activity_and_cannot_trigger_immediate_idle_return(pair):
    clock = [10.0]
    bus = RecordedBus(pair, RAISED)
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.idle_return_enabled = True
    clock[0] += 700
    controller.receive(save_request())
    controller.update()
    assert controller.rest_save_state == "saved"
    assert not controller.rest_return_active and not bus.writes and not bus.torque


def test_mixed_conventions_explain_reported_j2_sign_and_preserve_working_override(pair):
    leader = replace(pair.follower_calibration, coordinate_system="legacy",
                     homing_offset=(973, 1494, 1789, 410, -309, 44),
                     start_pos=(685, 860, 870, 929, 0, 1367),
                     end_pos=(3376, 3227, 3065, 3221, 4095, 2641))
    pair = replace(pair, leader_calibration=leader, leader_joint_directions=(1, -1, 1, 1, 1, 1))
    before = leader.raw_to_normalized([2202, 869, 3076, 2669, 1901, 1379])
    after = leader.raw_to_normalized([2198, 2148, 1686, 2562, 1821, 1379])
    assert before[1] == pytest.approx(207.685546875)
    assert after[1] - before[1] == pytest.approx(112.412109375)
    follower_before = pair.follower_calibration.raw_to_normalized([2110, 1364, 2717, 2359, 1064, 1456])
    follower_after = pair.follower_calibration.raw_to_normalized([2119, 1829, 2031, 2163, 1063, 1456])
    assert follower_after[1] - follower_before[1] == pytest.approx(-40.879120879120876)
    assert after[2] < before[2] and follower_after[2] < follower_before[2]

    bus = RecordedBus(pair, RAISED)
    controller = FollowerController(pair, bus)
    controller.connect()
    original = list(bus.positions)
    raw = [2202, 869, 3076, 2669, 1901, 1379]
    def send(seq):
        controller.receive({"type": "arm_command", "seq": seq, "positions": raw,
                            "sent_unix_ms": int(time.time() * 1000)})
    send(1)
    controller.enable("leader")
    assert bus.positions == original
    raw[1] += 10
    send(2)
    controller.update()
    assert bus.positions[1] == original[1] + 10  # No new automatic inversion.
    status = controller.status()
    assert status["leader_joint_directions"] == [1, -1, 1, 1, 1, 1]
    assert status["leader_coordinate_system"] == "legacy"
    assert status["follower_coordinate_system"] == "lerobot_urdf"
    assert "different coordinate conventions" in status["setup_warnings"][0]


@pytest.mark.parametrize("request_id", [None, 1, "", "x" * 129])
def test_save_protocol_requires_bounded_request_id(request_id):
    with pytest.raises(ArmProtocolError):
        validate_browser_arm_message(save_request(request_id))


def test_save_protocol_never_accepts_browser_encoder_values_or_output_paths():
    message = {**save_request(), "positions": [0] * 6, "path": "another-arm.json"}
    assert validate_browser_arm_message(message) == save_request()
