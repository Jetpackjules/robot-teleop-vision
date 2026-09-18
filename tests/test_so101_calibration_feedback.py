"""Calibration must observe physical movement, including during travel."""

import sys
from dataclasses import replace
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_modules/so101/tools"))

import so101_follower_service as service  # noqa: E402
from so101_arm_common import ArmPairProfile  # noqa: E402


@pytest.fixture
def rig(monkeypatch):
    profile = ArmPairProfile.load(ROOT / "tests/fixtures/so101_arm_pair.json")

    class StallingBus(service.FakeFollowerBus):
        stall_joint = None

        def __init__(self, initial):
            super().__init__(initial)
            self.writes = []

        def write_positions(self, positions):
            previous = list(self.positions)
            self.writes.append(list(positions))
            super().write_positions(positions)
            if self.stall_joint is not None:
                self.positions[self.stall_joint] = previous[self.stall_joint]

    bus = StallingBus(profile.follower_calibration.normalized_to_raw([0, 130, 38, 78, -8, 40]))
    clock = [10.0]
    controller = service.FollowerController(profile, bus, now=lambda: clock[0])
    controller.connect()

    def start(joint=0, delta=30.0, moving_seconds=2.0):
        def plan(initial, *_args):
            target = list(initial)
            target[joint] += delta
            return [(target, moving_seconds, "moving test joint"),
                    (target, 2.0, "sampling test joint")]

        monkeypatch.setattr(service, "build_calibration_sweep_waypoints", plan)
        controller.receive({"type": "arm_calibration_sweep", "calibration_request_id": "feedback-test"})
        assert controller.state == "calibrating", controller.calibration_rejection

    return controller, bus, clock, start


@pytest.mark.parametrize("joint", [0, 1, 2, 3, 4, 5])
def test_stalled_joint_cancels_calibration_during_travel_even_with_feedback_disabled(rig, joint):
    controller, bus, clock, start = rig
    bus.stall_joint = joint
    start(joint)
    for _ in range(100):
        clock[0] += 0.05
        # Repeated UI settings cannot clear the mandatory calibration guard.
        controller.receive({"type": "arm_feedback_settings", **{
            name: False for name in controller.feedback_settings
        }})
        controller.update()
        controller.sample()
        if controller.state != "calibrating":
            break
        assert not controller.calibration_pose_settled
    assert clock[0] < 12.0  # stopped before reaching the first sampling pause
    assert controller.calibration_request_state == "cancelled"
    assert controller.following_error_stop
    assert f"servo {joint + 1}" in controller.status_message
    assert controller.state == "hold"
    assert bus.torque  # gravity support remains; the held target is measured
    assert bus.writes[-1] == bus.positions
    assert controller.applied_normalized == controller.follower_normalized
    writes_at_stop = len(bus.writes)
    clock[0] += 0.1
    controller.update()
    assert len(bus.writes) == writes_at_stop


def test_missing_calibration_feedback_disables_torque_before_another_goal(rig):
    controller, bus, clock, start = rig
    start()
    writes_before = len(bus.writes)
    clock[0] += 0.61
    controller.update()
    assert controller.state == "fault"
    assert controller.calibration_request_state == "fault"
    assert "telemetry became stale" in controller.fault
    assert not bus.torque
    assert not controller.calibration_pose_settled
    assert len(bus.writes) == writes_before


def test_sampling_and_completion_wait_for_actual_encoder_pose(rig):
    controller, bus, clock, start = rig
    bus.stall_joint = 0
    controller.profile = replace(controller.profile, max_step=(40.0,) * 6)
    start(delta=20, moving_seconds=0.05)
    clock[0] += 0.05
    controller.update()
    controller.sample()
    clock[0] += 0.05
    controller.update()
    assert controller.applied_normalized == pytest.approx(controller.target_normalized)
    assert not controller.calibration_pose_settled
    # Check the endpoint branch independently of the sustained-error timer.
    clock[0] += 2.0
    controller.sample()
    controller.update()
    assert controller.calibration_request_state == "running"
    bus.stall_joint = None
    clock[0] += 0.05
    controller.update()
    controller.sample()
    clock[0] += 0.05
    controller.update()
    assert controller.calibration_request_state == "completed"


def test_calibration_tracking_does_not_wrap_a_full_encoder_turn(rig):
    controller, _bus, _clock, start = rig
    start()
    assert controller._joint_following_error(0, 380.0, 20.0) == 360.0
