"""Exercise leader teleop against servos with calibrated travel limits."""

from dataclasses import replace
from pathlib import Path
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_modules/so101/tools"))

from so101_arm_common import ArmCalibration, ArmPairProfile
from so101_follower_service import FakeFollowerBus, FollowerController


def calibrated_pair():
    def calibration(low, high):
        return ArmCalibration.from_dict({
            "homing_offset": [0] * 6,
            "drive_mode": [0] * 6,
            "start_pos": low,
            "end_pos": high,
            "calib_mode": ["DEGREE"] * 5 + ["LINEAR"],
            "coordinate_system": "lerobot_urdf",
        })

    return replace(
        ArmPairProfile.load(ROOT / "tests/fixtures/so101_arm_pair.json"),
        leader_calibration=calibration([685, 860, 870, 929, 0, 1367], [3376, 3227, 3065, 3221, 4095, 2641]),
        follower_calibration=calibration([739, 854, 854, 812, 0, 1377], [3435, 3207, 3069, 3122, 4095, 2878]),
    )


class LimitedBus(FakeFollowerBus):
    def __init__(self, positions, limits, *, blocked_joint=None):
        super().__init__(positions)
        self.limits = limits
        self.blocked_joint = blocked_joint
        self.goals = []

    def write_positions(self, positions):
        self.goals.append(list(positions))
        reached = [max(low, min(high, raw)) for raw, (low, high) in zip(positions, self.limits)]
        if self.blocked_joint is not None:
            reached[self.blocked_joint] = self.positions[self.blocked_joint]
        super().write_positions(reached)


def make_controller(tmp_path, joint, direction, *, blocked=False):
    pair = calibrated_pair()
    limits = list(zip(pair.follower_calibration.start_pos, pair.follower_calibration.end_pos))
    start = [round((low + high) / 2) for low, high in limits]
    if not blocked:
        start[joint] = limits[joint][0] + 15 if direction < 0 else limits[joint][1] - 15
    bus = LimitedBus(start, limits, blocked_joint=joint if blocked else None)
    clock = [10.0]
    controller = FollowerController(pair, bus, now=lambda: clock[0], rest_pose_path=tmp_path / "rest.json")
    controller.connect()
    leader = [round((low + high) / 2) for low, high in zip(
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
    assert controller.state == "armed"
    return controller, bus, leader, send


@pytest.mark.parametrize("joint", range(5))
@pytest.mark.parametrize("direction", [-1, 1])
def test_leader_limit_keeps_other_joints_moving_and_reverses_without_dead_zone(tmp_path, joint, direction):
    controller, bus, leader, send = make_controller(tmp_path, joint, direction)
    other = 4 if joint == 0 else 0
    other_start = bus.positions[other]
    for step in range(1, 31):
        moved = list(leader)
        moved[joint] += direction * 12 * step
        moved[other] += 3 * step
        send(step, moved)
        controller.update()
        controller.sample()

    assert all(low <= raw <= high for goal in bus.goals for raw, (low, high) in zip(goal, bus.limits))
    assert controller.state == "armed"
    assert controller.following_error_trip_count == 0
    assert bus.positions[other] == other_start + 90
    boundary = bus.limits[joint][0 if direction < 0 else 1]
    assert bus.positions[joint] == boundary
    assert controller.status()["leader_limited_joints"] == [joint + 1]
    assert "calibrated limit" in controller.status_message

    # Reversing just three ticks must work even after the leader travelled
    # hundreds of ticks beyond the follower's available travel.
    moved[joint] -= direction * 3
    send(31, moved)
    controller.update()
    controller.sample()
    assert bus.positions[joint] == boundary - direction * 3
    assert controller.status()["leader_limited_joints"] == []
    assert controller.state == "armed"


@pytest.mark.parametrize("joint", [1, 2])
def test_in_range_motor_stall_still_holds_arm(tmp_path, joint):
    controller, bus, leader, send = make_controller(tmp_path, joint, 1, blocked=True)
    for step in range(1, 31):
        moved = list(leader)
        moved[joint] += 12 * step
        send(step, moved)
        controller.update()
        controller.sample()
    assert controller.state == "hold"
    assert controller.following_error_trip_count == 1
    assert f"servo {joint + 1}" in controller.status_message
    assert "following error" in controller.status_message
    assert all(low <= raw <= high for goal in bus.goals for raw, (low, high) in zip(goal, bus.limits))
