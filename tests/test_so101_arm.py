from __future__ import annotations

import sys
import time
import math
from pathlib import Path

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import so101_follower_service as follower_service
from so101_arm_common import ArmPairProfile, ArmProtocolError, limit_step, validate_browser_arm_message
from so101_follower_service import (
    CALIBRATION_SAFE_HEIGHT_M,
    FakeFollowerBus,
    FollowerController,
    LeRobotFollowerBus,
    build_base_axis_sweep_waypoints,
    build_calibration_sweep_waypoints,
    build_joint_calibration_sweep_waypoints,
    build_rest_return_waypoints,
    load_rest_pose,
    tool_clearance_metric,
)
from so101_kinematics import (
    BASE_STRAIGHT_NORMALIZED_DEG,
    JOINT_LIMITS_RAD,
    arm_pose,
    joint_rate_step,
    normalized_to_joint_radians,
    overlay_joint_radians,
    resolved_rate_step,
    solve_pose_target,
    tool_rate_step,
    wrist_frame_twist,
)
from migrate_so101_base_zero_registration import ROS_TO_GODOT, _payload_transform, migrate


def profile() -> ArmPairProfile:
    return ArmPairProfile.load(ROOT / "tests" / "fixtures" / "so101_arm_pair.json")


REST_POSE = ROOT / "tests" / "fixtures" / "so101_rest_pose.json"


def test_overlay_base_yaw_tracks_follower_direction():
    centered = [0.0] * 6
    turned_right = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    follower_delta = normalized_to_joint_radians(turned_right)[0] - normalized_to_joint_radians(centered)[0]
    overlay_delta = overlay_joint_radians(turned_right)[0] - overlay_joint_radians(centered)[0]
    assert overlay_delta == pytest.approx(follower_delta)
    assert overlay_delta > 0.0


def test_measured_straight_base_encoder_maps_to_joint_zero():
    straight = [BASE_STRAIGHT_NORMALIZED_DEG, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert normalized_to_joint_radians(straight)[0] == pytest.approx(0.0, abs=1e-7)
    assert overlay_joint_radians(straight)[0] == pytest.approx(0.0, abs=1e-7)


def test_base_zero_registration_migration_preserves_moving_chain():
    payload = {
        "kinematics_version": 9,
        "basis_x": [1.0, 0.0, 0.0],
        "basis_y": [0.0, 1.0, 0.0],
        "basis_z": [0.0, 0.0, 1.0],
        "origin": [0.1, -0.2, 0.3],
    }
    migrated = migrate(payload)
    model_root = np.eye(4)
    model_root[:3, :3] = ROS_TO_GODOT
    from so101_kinematics import JOINT_ORIGINS, _rz, _transform

    shoulder_origin = _transform(*JOINT_ORIGINS[0])
    old_rotation = np.eye(4)
    old_rotation[:3, :3] = _rz(math.radians(BASE_STRAIGHT_NORMALIZED_DEG))
    old_shoulder = _payload_transform(payload) @ model_root @ shoulder_origin @ old_rotation
    new_shoulder = _payload_transform(migrated) @ model_root @ shoulder_origin
    assert migrated["kinematics_version"] == 10
    assert new_shoulder == pytest.approx(old_shoulder, abs=1e-12)


def test_calibration_round_trip_and_pair_mapping():
    pair = profile()
    leader_raw = [2500, 2600, 1600, 2300, 1500, 2600]
    normalized = pair.leader_calibration.raw_to_normalized(leader_raw)
    restored = pair.leader_calibration.normalized_to_raw(normalized)
    assert restored == leader_raw
    mapped_normalized, follower_raw = pair.map_leader_to_follower(leader_raw)
    follower_normalized = pair.follower_calibration.raw_to_normalized(follower_raw)
    assert follower_normalized == pytest.approx(mapped_normalized, abs=0.1)


def test_browser_message_validation_rejects_bad_shape_and_bounds():
    now = 1_000_000
    valid = validate_browser_arm_message(
        {"type": "arm_command", "seq": 7, "sent_unix_ms": now, "positions": [1, 2, 3, 4, 5, 6]},
        now_ms=now,
    )
    assert valid["seq"] == 7
    with pytest.raises(ArmProtocolError):
        validate_browser_arm_message(
            {"type": "arm_command", "seq": 8, "sent_unix_ms": now, "positions": [1, 2]},
            now_ms=now,
        )
    with pytest.raises(ArmProtocolError):
        validate_browser_arm_message(
            {"type": "arm_command", "seq": 8, "sent_unix_ms": now, "positions": [1, 2, 3, 4, 5, 9999]},
            now_ms=now,
        )


def test_keyboard_message_rejects_motion_when_input_is_explicitly_disabled():
    now = int(time.time() * 1000)
    idle = validate_browser_arm_message(
        {
            "type": "arm_cartesian_velocity",
            "seq": 1,
            "sent_unix_ms": now,
            "linear": [0, 0, 0],
            "angular": [0, 0, 0],
            "gripper": 0,
            "precision": False,
            "deadman": False,
        },
        now_ms=now,
    )
    assert idle["deadman"] is False
    assert idle["frame"] == "base"
    with pytest.raises(ArmProtocolError, match="disabled keyboard"):
        validate_browser_arm_message(
            {
                **idle,
                "seq": 2,
                "linear": [1, 0, 0],
            },
            now_ms=now,
        )


def test_wrist_keyboard_frame_validation():
    now = int(time.time() * 1000)
    message = validate_browser_arm_message(
        {
            "type": "arm_cartesian_velocity",
            "seq": 4,
            "sent_unix_ms": now,
            "linear": [0.25, -0.5, 1.0],
            "angular": [0.5, 0.0, -0.25],
            "gripper": 0.0,
            "frame": "wrist",
            "precision": False,
            "deadman": True,
        },
        now_ms=now,
    )
    assert message["frame"] == "wrist"
    assert message["angular_frame"] == "wrist"
    with pytest.raises(ArmProtocolError, match="frame"):
        validate_browser_arm_message({**message, "frame": "camera-ish"}, now_ms=now)


def test_captured_plane_keyboard_message_includes_explicit_wrist_axes():
    now = int(time.time() * 1000)
    message = validate_browser_arm_message(
        {
            "type": "arm_cartesian_velocity",
            "seq": 6,
            "sent_unix_ms": now,
            "linear": [1.0, -0.5, 0.25],
            "angular": [0.0, 0.0, 0.0],
            "wrist": [-0.75, 1.0],
            "gripper": 0.0,
            "frame": "plane",
            "precision": False,
            "deadman": True,
        },
        now_ms=now,
    )
    assert message["frame"] == "plane"
    assert message["angular_frame"] == "base"
    assert message["wrist"] == pytest.approx([-0.75, 1.0])


def test_base_translation_can_request_wrist_relative_rotation():
    now = int(time.time() * 1000)
    message = validate_browser_arm_message(
        {
            "type": "arm_cartesian_velocity",
            "seq": 5,
            "sent_unix_ms": now,
            "linear": [0.0, -1.0, 0.0],
            "angular": [1.0, 0.0, -1.0],
            "gripper": 0.0,
            "frame": "base",
            "angular_frame": "wrist",
            "precision": False,
            "deadman": True,
        },
        now_ms=now,
    )
    assert message["frame"] == "base"
    assert message["angular_frame"] == "wrist"


def test_direct_joint_keyboard_message_validation():
    now = int(time.time() * 1000)
    message = validate_browser_arm_message(
        {
            "type": "arm_joint_velocity",
            "seq": 3,
            "sent_unix_ms": now,
            "velocities": [1, -1, 0.5, 0, 0, -0.25],
            "precision": True,
            "deadman": True,
        },
        now_ms=now,
    )
    assert message["velocities"] == pytest.approx([1, -1, 0.5, 0, 0, -0.25])
    with pytest.raises(ArmProtocolError, match="disabled keyboard"):
        validate_browser_arm_message(
            {
                **message,
                "seq": 4,
                "deadman": False,
            },
            now_ms=now,
        )


def test_tool_keyboard_message_validation():
    now = int(time.time() * 1000)
    message = validate_browser_arm_message(
        {
            "type": "arm_tool_velocity",
            "seq": 5,
            "sent_unix_ms": now,
            "motions": [1, -1, 0.5, 0, 0, -0.25],
            "precision": False,
            "deadman": True,
        },
        now_ms=now,
    )
    assert message["motions"] == pytest.approx([1, -1, 0.5, 0, 0, -0.25])
    with pytest.raises(ArmProtocolError, match="disabled keyboard"):
        validate_browser_arm_message({**message, "seq": 6, "deadman": False}, now_ms=now)


def test_browser_restart_message_is_supported():
    assert validate_browser_arm_message({"type": "arm_restart"}) == {"type": "arm_restart"}


def test_browser_rest_return_messages_are_supported_and_bounded():
    assert validate_browser_arm_message({"type": "arm_return_to_rest"}) == {
        "type": "arm_return_to_rest"
    }
    settings = validate_browser_arm_message(
        {
            "type": "arm_idle_return_settings",
            "enabled": True,
            "timeout_seconds": 600,
        }
    )
    assert settings == {
        "type": "arm_idle_return_settings",
        "enabled": True,
        "timeout_seconds": 600.0,
    }
    with pytest.raises(ArmProtocolError, match="between 60 and 3600"):
        validate_browser_arm_message(
            {
                "type": "arm_idle_return_settings",
                "enabled": True,
                "timeout_seconds": 10,
            }
        )


def test_saved_rest_pose_round_trips_against_current_follower_calibration():
    pair = profile()
    rest = load_rest_pose(pair, REST_POSE)
    assert rest["follower_serial"] == pair.follower_serial
    assert pair.follower_calibration.normalized_to_raw(rest["normalized_positions"]) == rest[
        "raw_positions"
    ]
    assert tool_clearance_metric(rest["normalized_positions"]) > 0.0


def test_rest_return_planner_preserves_saved_endpoint_and_modeled_clearance():
    pair = profile()
    rest = load_rest_pose(pair, REST_POSE)[
        "normalized_positions"
    ]
    start = list(rest)
    start[2] -= 5.0
    waypoints = build_rest_return_waypoints(start, rest)
    assert waypoints[-1][0] == pytest.approx(rest)
    assert sum(item[1] for item in waypoints) >= 0.75


def test_arm_feedback_settings_require_explicit_booleans():
    settings = {
        "type": "arm_feedback_settings",
        "measured_feedback_enabled": True,
        "target_ghost_enabled": False,
        "following_error_safety_enabled": True,
        "freeze_overlay_on_stale_enabled": True,
        "d455_visual_correction_enabled": False,
    }
    assert validate_browser_arm_message(settings) == settings
    with pytest.raises(ArmProtocolError, match="must be boolean"):
        validate_browser_arm_message(
            {**settings, "following_error_safety_enabled": "yes"}
        )


def test_sustained_following_error_rebases_and_holds():
    pair = profile()
    clock = [10.0]
    initial = [0.0, 20.0, -15.0, 5.0, 0.0, 40.0]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.last_command_at = clock[0]
    controller.latest_input_source = "keyboard"
    controller.enable("keyboard")
    # Non-keyboard/leader control retains the conservative explicit Hold;
    # only the browser keyboard path can safely require a key release.
    controller.control_source = "leader"
    controller.applied_normalized = [15.0, *initial[1:]]
    controller.target_normalized = list(controller.applied_normalized)
    clock[0] = 10.4
    controller.sample()
    assert controller.state == "armed"
    clock[0] = 10.8
    controller.sample()
    assert controller.state == "hold"
    assert controller.following_error_stop is True
    assert controller.following_error_trip_count == 1
    assert controller.applied_normalized == pytest.approx(initial, abs=0.1)


def test_gripper_closing_contact_rebases_only_gripper_and_stays_armed():
    pair = profile()
    clock = [10.0]
    initial = [0.0, 20.0, -15.0, 5.0, 0.0, 60.0]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.last_command_at = clock[0]
    controller.latest_input_source = "keyboard"
    controller.enable("keyboard")
    controller.keyboard_gripper = -1.0
    controller.applied_normalized[5] = 35.0
    controller.target_normalized[5] = 35.0
    clock[0] = 10.4
    controller.sample()
    assert controller.state == "armed"
    clock[0] = 10.8
    controller.sample()
    assert controller.state == "armed"
    assert controller.gripper_contact_latched is True
    assert controller.gripper_contact_trip_count == 1
    assert controller.following_error_stop is False
    assert controller.applied_normalized[5] == pytest.approx(initial[5], abs=0.1)

    common = {
        "type": "arm_cartesian_velocity",
        "sent_unix_ms": int(time.time() * 1000),
        "linear": [0.0, 0.0, 0.0],
        "angular": [0.0, 0.0, 0.0],
        "wrist": [0.0, 0.0],
        "precision": False,
        "deadman": True,
    }
    controller.receive({**common, "seq": 1, "gripper": -1.0})
    assert controller.keyboard_gripper == 0.0
    assert controller.gripper_contact_latched is True
    controller.receive({**common, "seq": 2, "gripper": 0.0})
    assert controller.gripper_contact_latched is False
    controller.receive({**common, "seq": 3, "gripper": -1.0})
    assert controller.keyboard_gripper == -1.0


def test_keyboard_arm_contact_rebases_and_requires_release_without_hold():
    pair = profile()
    clock = [10.0]
    initial = [0.0, 20.0, -15.0, 5.0, 0.0, 40.0]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.last_command_at = clock[0]
    controller.latest_input_source = "keyboard"
    controller.enable("keyboard")
    controller.keyboard_linear = [1.0, 0.0, 0.0]
    controller.applied_normalized = [15.0, *initial[1:]]
    controller.target_normalized = list(controller.applied_normalized)
    clock[0] = 10.4
    controller.sample()
    clock[0] = 10.8
    controller.sample()
    assert controller.state == "armed"
    assert controller.arm_contact_latched is True
    assert controller.arm_contact_trip_count == 1
    assert controller.applied_normalized == pytest.approx(initial, abs=0.1)

    common = {
        "type": "arm_cartesian_velocity",
        "sent_unix_ms": int(time.time() * 1000),
        "linear": [1.0, 0.0, 0.0],
        "angular": [0.0, 0.0, 0.0],
        "wrist": [0.0, 0.0],
        "gripper": 0.0,
        "precision": False,
        "deadman": True,
    }
    controller.receive({**common, "seq": 1})
    assert controller.keyboard_linear == [0.0, 0.0, 0.0]
    assert controller.arm_contact_latched is True
    controller.receive({**common, "seq": 2, "linear": [0.0, 0.0, 0.0]})
    assert controller.arm_contact_latched is False
    controller.receive({**common, "seq": 3})
    assert controller.keyboard_linear == [1.0, 0.0, 0.0]


def test_keyboard_controller_starts_at_follower_pose_and_integrates_smoothly():
    pair = profile()
    clock = [10.0]
    initial = [8.0, -12.0, 24.0, -18.0, 15.0, 40.0]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.receive(
        {
            "type": "arm_cartesian_velocity",
            "seq": 1,
            "sent_unix_ms": int(time.time() * 1000),
            "linear": [1.0, 0.0, 0.0],
            "angular": [0.0, 0.0, 0.0],
            "gripper": 0.0,
            "precision": False,
            "deadman": True,
        }
    )
    controller.receive({"type": "arm_enable", "source": "keyboard"})
    starting_target = list(controller.target_normalized)
    assert starting_target == pytest.approx(controller.follower_normalized)
    clock[0] += 1.0 / 30.0
    controller.update()
    assert controller.state == "armed"
    assert controller.target_normalized != pytest.approx(starting_target)
    assert controller.status()["control_source"] == "keyboard"


def test_cartesian_translation_holds_absolute_claw_orientation():
    initial = [8.0, -12.0, 24.0, -18.0, 15.0, 40.0]
    desired_rotation = arm_pose(initial).rotation
    moved = list(initial)
    for _ in range(30):
        moved = resolved_rate_step(
            moved,
            [0.025, -0.01, 0.0],
            [0.0, 0.0, 0.0],
            1.0 / 30.0,
            target_rotation=desired_rotation,
        )
    before = arm_pose(initial)
    after = arm_pose(moved)
    position_delta = sum((a - b) ** 2 for a, b in zip(after.position, before.position, strict=True)) ** 0.5
    rotation_alignment = sum(
        after.rotation[row][column] * desired_rotation[row][column]
        for row in range(3)
        for column in range(3)
    )
    rotation_error = math.acos(max(-1.0, min(1.0, (rotation_alignment - 1.0) * 0.5)))
    assert position_delta > 0.01
    assert rotation_error < math.radians(3.0)


def test_persistent_pose_target_ik_converges_without_joint_branch_jump():
    initial = [-40.0, 117.0, 94.0, 33.0, -157.0, 2.5]
    initial_pose = arm_pose(initial)
    target_position = [
        initial_pose.position[0],
        initial_pose.position[1] - 0.012,
        initial_pose.position[2],
    ]
    solved = list(initial)
    initial_error = math.dist(initial_pose.position, target_position)
    for _ in range(5):
        previous = list(solved)
        solved = solve_pose_target(solved, target_position, initial_pose.rotation)
        assert max(abs(after - before) for after, before in zip(solved[:5], previous[:5], strict=True)) <= 4.01
    assert math.dist(arm_pose(solved).position, target_position) < initial_error * 0.35


def test_cartesian_retraction_stops_before_folding_into_joint_limit():
    pair = profile()
    clock = [10.0]
    initial = [-61.084, 20.654, -89.297, 164.180, -20.918, 2.03]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()

    for sequence in range(1, 30):
        controller.receive(
            {
                "type": "arm_cartesian_velocity",
                "seq": sequence,
                "sent_unix_ms": int(time.time() * 1000),
                "linear": [0.0, -1.0, 0.0],
                "angular": [0.0, 0.0, 0.0],
                "wrist": [0.0, 0.0],
                "gripper": 0.0,
                "frame": "plane",
                "precision": False,
                "deadman": True,
            }
        )
        if sequence == 1:
            controller.receive({"type": "arm_enable", "source": "keyboard"})
        clock[0] += 1.0 / 35.0
        controller.update()
        if controller.state != "armed":
            break
        controller.sample()

    # The redundant two-degree joint-margin stop is gone. This deliberately
    # pathological retraction is still stopped by the independent whole-mesh
    # table-clearance guard before the model folds through the base plane.
    assert controller.state == "hold"
    assert controller.ik_guard_trip_count == 1
    assert "Modeled arm clearance" in controller.status_message
    assert "IK workspace boundary" not in controller.status_message
    joints = normalized_to_joint_radians(controller.target_normalized)
    assert all(
        limits[0] <= joint <= limits[1]
        for joint, limits in zip(joints, JOINT_LIMITS_RAD, strict=True)
    )


def test_task_priority_keeps_forward_and_lateral_motion_distinct():
    initial = [-39.551, 87.1, -47.021, 142.646, -157.5, 2.498]
    desired_rotation = arm_pose(initial).rotation
    deltas = []
    for velocity in ([0.08, 0.0, 0.0], [0.0, -0.08, 0.0]):
        moved = list(initial)
        for _ in range(15):
            moved = resolved_rate_step(
                moved,
                velocity,
                [0.0, 0.0, 0.0],
                1.0 / 30.0,
                target_rotation=desired_rotation,
            )
        before = arm_pose(initial).position
        after = arm_pose(moved).position
        deltas.append([after[index] - before[index] for index in range(3)])
    dot = sum(a * b for a, b in zip(*deltas, strict=True))
    lengths = [sum(value * value for value in delta) ** 0.5 for delta in deltas]
    angle = math.degrees(math.acos(max(-1.0, min(1.0, dot / (lengths[0] * lengths[1])))))
    assert angle > 65.0


def test_direct_joint_keyboard_moves_only_the_selected_joint():
    pair = profile()
    clock = [10.0]
    initial = [0.0, 0.0, 0.0, 0.0, 0.0, 50.0]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.receive(
        {
            "type": "arm_joint_velocity",
            "seq": 1,
            "sent_unix_ms": int(time.time() * 1000),
            "velocities": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "precision": False,
            "deadman": True,
        }
    )
    controller.receive({"type": "arm_enable", "source": "keyboard"})
    before = list(controller.target_normalized)
    clock[0] += 1.0 / 30.0
    controller.update()
    after = controller.target_normalized
    assert after[0] > before[0]
    assert after[1:] == pytest.approx(before[1:])
    assert controller.status()["keyboard_mode"] == "joint"


def test_joint_rate_step_keeps_each_axis_independent():
    initial = [0.0, 0.0, 0.0, 0.0, 0.0, 50.0]
    for joint in range(6):
        velocity = [0.0] * 6
        velocity[joint] = 1.0
        moved = joint_rate_step(initial, velocity, 1.0 / 30.0)
        assert moved[joint] != pytest.approx(initial[joint])
        for other in range(6):
            if other != joint:
                assert moved[other] == pytest.approx(initial[other])


def test_tool_rate_controls_are_distinct_and_base_stays_independent():
    initial = [-35.0, 165.0, 88.0, 125.0, 12.0, 45.0]
    base = tool_rate_step(initial, [1, 0, 0, 0, 0, 0], 1.0 / 30.0)
    reach = tool_rate_step(initial, [0, 1, 0, 0, 0, 0], 1.0 / 30.0)
    lift = tool_rate_step(initial, [0, 0, 1, 0, 0, 0], 1.0 / 30.0)
    assert base[0] > initial[0]
    assert base[1:] == pytest.approx(initial[1:])
    assert reach[1:3] != pytest.approx(initial[1:3])
    assert lift[1:3] != pytest.approx(initial[1:3])
    assert reach[1:3] != pytest.approx(lift[1:3])


def test_tool_keyboard_controller_uses_tool_mode():
    pair = profile()
    clock = [10.0]
    initial = [-35.0, 165.0, 88.0, 125.0, 12.0, 45.0]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.receive(
        {
            "type": "arm_tool_velocity",
            "seq": 1,
            "sent_unix_ms": int(time.time() * 1000),
            "motions": [0, 1, 0, 0, 0, 0],
            "precision": False,
            "deadman": True,
        }
    )
    controller.receive({"type": "arm_enable", "source": "keyboard"})
    before = list(controller.target_normalized)
    clock[0] += 1.0 / 30.0
    controller.update()
    assert controller.target_normalized != pytest.approx(before)
    assert controller.status()["keyboard_mode"] == "tool"


def test_resolved_rate_kinematics_respects_joint_limits():
    pose = [0.0, 0.0, 0.0, 0.0, 0.0, 50.0]
    before = arm_pose(pose)
    after = resolved_rate_step(pose, [0.05, 0.02, 0.01], [0.1, 0.0, -0.1], 1.0 / 30.0)
    assert arm_pose(after).position != pytest.approx(before.position)
    for radians, limits in zip(normalized_to_joint_radians(after), JOINT_LIMITS_RAD, strict=True):
        assert limits[0] <= radians <= limits[1]


def test_wrist_frame_twist_uses_live_tool_orientation():
    current = [-35.0, 165.0, 88.0, 125.0, 12.0, 45.0]
    pose = arm_pose(current)
    linear, angular = wrist_frame_twist(current, [0.0, 0.0, 1.0], [1.0, 0.0, 0.0])
    expected_linear = [pose.rotation[row][2] for row in range(3)]
    expected_angular = [pose.rotation[row][0] for row in range(3)]
    assert linear == pytest.approx(expected_linear)
    assert angular == pytest.approx(expected_angular)


def test_wrist_keyboard_controller_uses_wrist_frame():
    pair = profile()
    clock = [10.0]
    initial = [-35.0, 165.0, 88.0, 125.0, 12.0, 45.0]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.receive(
        {
            "type": "arm_cartesian_velocity",
            "seq": 1,
            "sent_unix_ms": int(time.time() * 1000),
            "linear": [0.0, 0.0, 1.0],
            "angular": [0.0, 0.0, 0.0],
            "gripper": 0.0,
            "frame": "wrist",
            "precision": False,
            "deadman": True,
        }
    )
    controller.receive({"type": "arm_enable", "source": "keyboard"})
    before = list(controller.target_normalized)
    clock[0] += 1.0 / 30.0
    controller.update()
    assert controller.target_normalized != pytest.approx(before)
    assert controller.status()["keyboard_frame"] == "wrist"


def test_captured_plane_lateral_motion_follows_claw_tangent():
    pair = profile()
    clock = [10.0]
    initial = [-40.0, 117.0, 94.0, 33.0, -157.0, 2.5]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.receive(
        {
            "type": "arm_cartesian_velocity",
            "seq": 1,
            "sent_unix_ms": int(time.time() * 1000),
            "linear": [1.0, 0.0, 0.0],
            "angular": [0.0, 0.0, 0.0],
            "wrist": [0.0, 0.0],
            "gripper": 0.0,
            "frame": "plane",
            "precision": False,
            "deadman": True,
        }
    )
    controller.receive({"type": "arm_enable", "source": "keyboard"})
    before_pose = arm_pose(controller.target_normalized).position
    before_joints = list(controller.target_normalized)
    for _ in range(8):
        clock[0] += 1.0 / 30.0
        controller.update()
    after_pose = arm_pose(controller.target_normalized).position
    displacement = [after - before for after, before in zip(after_pose, before_pose, strict=True)]
    tangent_motion = sum(
        value * axis for value, axis in zip(displacement, controller.keyboard_plane_right, strict=True)
    )
    radial_motion = sum(
        value * axis for value, axis in zip(displacement, controller.keyboard_plane_forward, strict=True)
    )
    joint_delta = [abs(after - before) for after, before in zip(controller.target_normalized, before_joints, strict=True)]
    assert tangent_motion > 0.004
    assert abs(radial_motion) < tangent_motion * 0.35
    assert abs(displacement[2]) < tangent_motion * 0.2
    assert joint_delta[0] > max(joint_delta[1:5])


def test_explicit_wrist_roll_updates_telemetry_without_moving_position_joints():
    pair = profile()
    clock = [10.0]
    initial = [-40.0, 117.0, 94.0, 33.0, -157.0, 2.5]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.receive(
        {
            "type": "arm_cartesian_velocity",
            "seq": 1,
            "sent_unix_ms": int(time.time() * 1000),
            "linear": [0.0, 0.0, 0.0],
            "angular": [0.0, 0.0, 0.0],
            "wrist": [0.0, 1.0],
            "gripper": 0.0,
            "frame": "plane",
            "precision": False,
            "deadman": True,
        }
    )
    controller.receive({"type": "arm_enable", "source": "keyboard"})
    before = list(controller.target_normalized)
    for _ in range(8):
        clock[0] += 1.0 / 30.0
        controller.update()
    status = controller.status()
    assert controller.target_normalized[4] > before[4] + 2.0
    assert controller.target_normalized[:3] == pytest.approx(before[:3], abs=0.05)
    assert status["applied_normalized"][4] > before[4]
    assert status["keyboard_wrist"] == pytest.approx([0.0, 1.0])


def test_project_servo_zero_offsets_preserve_current_pose_motion():
    current = [-81.5, 175.0, 94.4, 134.2, 25.3, 0.4]
    joints = normalized_to_joint_radians(current)
    assert joints[1] == pytest.approx(-95.0 * 3.141592653589793 / 180.0, abs=1e-5)
    assert joints[2] == pytest.approx(94.4 * 3.141592653589793 / 180.0, abs=1e-5)
    assert joints[3] == pytest.approx(64.2 * 3.141592653589793 / 180.0, abs=1e-5)
    moved = list(current)
    moved[1] -= 10.0
    assert arm_pose(moved).position != pytest.approx(arm_pose(current).position)

def test_resolved_rate_round_trip_does_not_jump_offset_joints():
    current = [-80.5, 174.4, 95.2, 134.6, 24.9, 0.4]
    moved = resolved_rate_step(current, [0.012, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0 / 30.0)
    assert max(abs(after - before) for after, before in zip(moved[:5], current[:5], strict=True)) <= 4.0
    assert abs(moved[1] - current[1]) < 2.0
    assert abs(moved[3] - current[3]) < 2.0


def test_step_limiter_coordinates_all_joints():
    limited = limit_step([0] * 6, [10, -10, 2, 8, 20, -20], (4, 4, 4, 5, 6, 8))
    assert limited == pytest.approx([3, -3, 0.6, 2.4, 6, -6])


def test_guarded_rest_return_reaches_saved_raw_pose_and_finishes_in_hold():
    pair = profile()
    clock = [10.0]
    rest = load_rest_pose(pair, REST_POSE)
    initial = list(rest["normalized_positions"])
    initial[2] -= 5.0
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(
        pair,
        bus,
        now=lambda: clock[0],
        rest_pose_path=REST_POSE,
    )
    controller.connect()
    controller.receive({"type": "arm_return_to_rest"})
    assert controller.state == "returning_rest"
    assert controller.rest_return_active is True
    for _ in range(300):
        clock[0] += 1.0 / 30.0
        controller.update()
        controller.sample()
        if not controller.rest_return_active:
            break
    assert controller.state == "hold"
    assert controller.rest_return_progress == pytest.approx(1.0)
    assert bus.positions == rest["raw_positions"]


def test_idle_rest_return_is_opt_in_and_requires_torque_enabled():
    pair = profile()
    clock = [10.0]
    rest = load_rest_pose(pair, REST_POSE)
    initial = list(rest["normalized_positions"])
    initial[2] -= 5.0
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(
        pair,
        bus,
        now=lambda: clock[0],
        rest_pose_path=REST_POSE,
    )
    controller.connect()
    clock[0] += 601.0
    controller.update()
    assert controller.rest_return_active is False
    controller.target_normalized = list(initial)
    controller.applied_normalized = list(initial)
    bus.enable_torque()
    controller.torque_enabled = True
    controller.gripper_torque_enabled = True
    controller.state = "hold"
    controller.receive(
        {
            "type": "arm_idle_return_settings",
            "enabled": True,
            "timeout_seconds": 600.0,
        }
    )
    controller.update()
    assert controller.rest_return_active is True
    assert controller.rest_return_reason == "idle"


def test_enable_requires_fresh_matching_pose_and_watchdog_holds():
    pair = profile()
    clock = [10.0]
    target_normalized = [20.0] * 6
    initial_raw = pair.follower_calibration.normalized_to_raw(target_normalized)
    bus = FakeFollowerBus(initial_raw)
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    leader_raw = pair.leader_calibration.normalized_to_raw(target_normalized)
    controller.receive({"type": "arm_command", "seq": 1, "sent_unix_ms": int(time.time() * 1000), "positions": leader_raw})
    controller.receive({"type": "arm_enable"})
    assert controller.state == "armed"
    assert controller.torque_enabled
    clock[0] += pair.watchdog_ms / 1000.0 + 0.001
    controller.update()
    assert controller.state == "hold"
    assert controller.torque_enabled


def test_enable_clutches_mismatched_poses_without_startup_jump_and_release_disables_torque():
    pair = profile()
    clock = [10.0]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw([0.0] * 6))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    leader_raw = pair.leader_calibration.normalized_to_raw([80.0] * 6)
    controller.receive({"type": "arm_command", "seq": 1, "sent_unix_ms": int(time.time() * 1000), "positions": leader_raw})
    controller.receive({"type": "arm_enable"})
    assert controller.state == "armed"
    assert controller.torque_enabled
    assert controller.target_normalized == pytest.approx(controller.follower_normalized)
    controller.update()
    assert bus.positions == pair.follower_calibration.normalized_to_raw(controller.follower_normalized)
    controller.receive({"type": "arm_release_torque"})
    assert not bus.torque
    assert not controller.torque_enabled


def test_hardware_write_guard_reseeds_after_torque_release():
    class StubBus:
        def __init__(self):
            self.disabled = False

        def disable_torque(self, num_retry=0):
            self.disabled = True

    bus = LeRobotFollowerBus("/dev/null", ("joint",))
    bus.bus = StubBus()
    bus.connected = True
    bus.last_written_positions = [2048]
    bus.disable_torque()
    assert bus.bus.disabled
    assert bus.last_written_positions is None


def test_network_stale_command_forces_hold():
    pair = profile()
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw([20.0] * 6))
    controller = FollowerController(pair, bus)
    controller.connect()
    leader_raw = pair.leader_calibration.normalized_to_raw([20.0] * 6)
    controller.receive(
        {
            "type": "arm_command",
            "seq": 1,
            "sent_unix_ms": int(time.time() * 1000) - pair.watchdog_ms - 10,
            "positions": leader_raw,
        }
    )
    assert controller.state == "ready"
    assert "older than watchdog" in controller.status_message


def test_leader_encoder_rollover_is_unwrapped_to_short_motion():
    pair = profile()
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw([20.0] * 6))
    controller = FollowerController(pair, bus)
    controller.connect()
    now_ms = int(time.time() * 1000)
    before = [4090, 2000, 2000, 2000, 2000, 2500]
    after = [5, 2000, 2000, 2000, 2000, 2500]
    controller.receive({"type": "arm_command", "seq": 1, "sent_unix_ms": now_ms, "positions": before})
    first = controller.leader_normalized[0]
    controller.receive({"type": "arm_command", "seq": 2, "sent_unix_ms": now_ms, "positions": after})
    second = controller.leader_normalized[0]
    assert abs(second - first) < 2.0


def test_new_controller_session_can_restart_sequence_after_page_reload():
    pair = profile()
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw([20.0] * 6))
    controller = FollowerController(pair, bus)
    controller.connect()
    now_ms = int(time.time() * 1000)
    first = pair.leader_calibration.normalized_to_raw([10.0] * 6)
    second = pair.leader_calibration.normalized_to_raw([30.0] * 6)
    controller.receive(
        {"type": "arm_command", "control_session": "before-reload", "seq": 400, "sent_unix_ms": now_ms, "positions": first}
    )
    controller.receive(
        {"type": "arm_command", "control_session": "after-reload", "seq": 0, "sent_unix_ms": now_ms, "positions": second}
    )
    assert controller.control_session == "after-reload"
    assert controller.last_seq == 0
    assert controller.leader_raw == second


def test_hardware_restart_recovers_fault_and_reseeds_current_pose():
    class RestartBus(FakeFollowerBus):
        def __init__(self, initial):
            super().__init__(initial)
            self.connect_count = 0
            self.close_count = 0

        def connect(self):
            self.connect_count += 1
            super().connect()

        def close(self, disable_torque=True):
            self.close_count += 1
            super().close(disable_torque)

    pair = profile()
    initial = pair.follower_calibration.normalized_to_raw([10.0, 20.0, 30.0, 40.0, 50.0, 60.0])
    bus = RestartBus(initial)
    controller = FollowerController(pair, bus)
    controller.connect()
    controller.fail("simulated USB write failure")
    controller.hold("operator hold")
    assert "simulated USB write failure" in controller.fault
    controller.receive({"type": "arm_restart"})
    assert bus.connect_count == 2
    assert bus.close_count == 1
    assert controller.state == "hold"
    assert controller.torque_enabled
    assert controller.restart_count == 1
    assert controller.fault == ""
    assert "press Enable Arm" in controller.status_message
    assert controller.applied_normalized == pytest.approx(controller.follower_normalized)


def test_bounded_calibration_sweep_finishes_raised_and_holds():
    pair = profile()
    clock = [10.0]
    initial_normalized = [-35.0, 170.0, 90.0, 135.0, 10.0, 20.0]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial_normalized))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.receive({"type": "arm_calibration_sweep", "action": "start"})
    assert controller.state == "calibrating"
    assert controller.torque_enabled
    expected_measured_poses = sum(
        label.startswith("sampling")
        for _, _, label in controller.calibration_sweep_waypoints
    )
    observed_base_offsets = []
    measured_calibration_poses = []
    was_settled = False
    steps = int(controller.calibration_sweep_duration * 30.0) + 90
    for _ in range(steps):
        clock[0] += 1.0 / 30.0
        controller.update()
        observed_base_offsets.append(controller.applied_normalized[0] - initial_normalized[0])
        if controller.calibration_pose_settled:
            controller.sample()
            assert controller.follower_normalized == pytest.approx(controller.applied_normalized, abs=0.1)
            if not was_settled:
                measured_calibration_poses.append(list(controller.follower_normalized))
        was_settled = controller.calibration_pose_settled
    assert min(observed_base_offsets) >= -40.1
    assert max(observed_base_offsets) <= 40.1
    assert controller.state == "hold"
    assert not controller.calibration_sweep_active
    assert tool_clearance_metric(controller.applied_normalized) >= 0.009
    assert controller.applied_normalized != pytest.approx(initial_normalized, abs=0.3)
    assert expected_measured_poses >= 10
    assert len(measured_calibration_poses) == expected_measured_poses


def test_calibration_sweep_raises_before_lateral_motion_and_preserves_clearance():
    initial = [-19.6, 163.5, 94.2, 133.6, -160.75, 2.42]
    waypoints = build_calibration_sweep_waypoints(initial)
    start_clearance = tool_clearance_metric(initial)
    first_excitation = next(index for index, item in enumerate(waypoints) if item[2].startswith("moving to broad pose"))
    assert first_excitation > 0
    assert all(item[2] == "raising tool" for item in waypoints[:first_excitation])
    excitation_clearances = [
        tool_clearance_metric(pose)
        for pose, _, label in waypoints
        if label.startswith("moving to broad pose")
    ]
    assert min(excitation_clearances) >= start_clearance + 0.009
    assert tool_clearance_metric(waypoints[-1][0]) >= 0.009


def test_calibration_sweep_accepts_absolute_safe_height_from_slightly_low_pose():
    # This is the real pose that previously reached 1.6 cm but was rejected
    # because the planner also demanded a needless 2.5 cm relative gain.
    initial = [-39.023, 101.953, 79.541, 39.902, -153.721, 9.68]
    assert tool_clearance_metric(initial) < 0.010
    waypoints = build_calibration_sweep_waypoints(initial)
    assert waypoints
    assert min(
        tool_clearance_metric(pose)
        for pose, _, label in waypoints
        if label.startswith("moving to broad pose")
    ) >= 0.009


def test_calibration_sweep_uses_broad_geometrically_distinct_poses():
    initial = [-35.0, 170.0, 90.0, 135.0, -165.0, 20.0]
    waypoints = build_calibration_sweep_waypoints(initial)
    poses = [pose for pose, _, label in waypoints if label.startswith("sampling broad pose")]
    assert len(poses) == 5
    ranges = [max(pose[index] for pose in poses + [initial]) - min(pose[index] for pose in poses + [initial]) for index in range(5)]
    assert ranges[0] >= 50.0
    assert ranges[1] >= 40.0
    assert ranges[2] >= 40.0
    assert ranges[3] >= 40.0
    assert ranges[4] >= 30.0


def test_calibration_sweep_is_distinct_when_restarted_from_raised_anchor():
    initial = [0.0, 130.0, 35.0, 75.0, 0.0, 20.0]
    waypoints = build_calibration_sweep_waypoints(initial)
    poses = [pose for pose, _, label in waypoints if label.startswith("sampling broad pose")]
    ranges = [
        max(pose[index] for pose in poses + [initial])
        - min(pose[index] for pose in poses + [initial])
        for index in range(5)
    ]
    assert ranges[0] >= 50.0
    assert ranges[1] >= 40.0
    assert ranges[2] >= 40.0
    assert ranges[3] >= 40.0
    assert ranges[4] >= 30.0


def test_calibration_planner_rejection_keeps_held_torque(monkeypatch):
    pair = profile()
    initial = pair.follower_calibration.normalized_to_raw([0.0, 130.0, 35.0, 75.0, 0.0, 20.0])
    bus = FakeFollowerBus(initial)
    controller = FollowerController(pair, bus)
    controller.connect()
    bus.enable_torque()
    controller.torque_enabled = True
    controller.state = "hold"

    def reject_plan(_starting_pose):
        raise RuntimeError("simulated safe-path rejection")

    monkeypatch.setattr(follower_service, "build_calibration_sweep_waypoints", reject_plan)
    controller.receive({"type": "arm_calibration_sweep", "action": "start"})

    assert controller.state == "hold"
    assert controller.torque_enabled
    assert bus.torque
    assert controller.fault == ""
    assert controller.calibration_rejection == "simulated safe-path rejection"
    assert "without moving" in controller.status_message


def test_joint_calibration_sweep_moves_only_one_servo_at_a_time():
    initial = [-35.0, 170.0, 90.0, 135.0, -165.0, 20.0]
    waypoints = build_joint_calibration_sweep_waypoints(initial)
    anchor = next(pose for pose, _, label in waypoints if label == "sampling joint-calibration anchor")
    sampled = [
        (pose, label)
        for pose, _, label in waypoints
        if label.startswith("sampling servo ")
    ]
    assert {int(label.split()[2]) for _, label in sampled} == {1, 2, 3, 4}
    assert all(
        len([1 for _, label in sampled if int(label.split()[2]) == joint_index]) == 10
        for joint_index in (1, 2, 3)
    )
    assert len([
        1 for _, label in sampled if int(label.split()[2]) == 4
    ]) == 15
    for pose, label in sampled:
        joint_index = int(label.split()[2])
        changed = [
            index
            for index, (value, reference) in enumerate(zip(pose, anchor, strict=True))
            if abs(value - reference) > 0.1
        ]
        allowed = {0, joint_index}
        if joint_index == 4:
            allowed.update({1, 2, 3, 5})
        assert set(changed).issubset(allowed)
    roll_samples = [
        pose for pose, label in sampled if int(label.split()[2]) == 4
    ]
    shoulder_samples = [
        pose for pose, label in sampled if int(label.split()[2]) == 1
    ]
    assert max(pose[1] for pose in shoulder_samples) - min(
        pose[1] for pose in shoulder_samples
    ) >= 30.0
    assert all(
        pose[1:4] == pytest.approx([110.0, 60.0, 75.0])
        for pose in roll_samples
    )
    assert max(pose[4] for pose in roll_samples) - min(pose[4] for pose in roll_samples) >= 60.0
    roll_dwells = [
        duration
        for _, duration, label in waypoints
        if label.startswith("sampling servo 4 wrist_roll")
    ]
    assert roll_dwells == pytest.approx(
        [follower_service.WRIST_CALIBRATION_SAMPLE_DWELL_SECONDS] * 15
    )
    assert all(
        duration > follower_service.JOINT_CALIBRATION_SAMPLE_DWELL_SECONDS
        for duration in roll_dwells
    )
    assert all(
        tool_clearance_metric(pose) >= 0.009
        for pose, _, label in waypoints
        if "joint-calibration" in label or "servo " in label
    )


def test_joint_capture_wrist_coverage_matches_external_five_pose_gate():
    source = (ROOT / "godot/runtime/point_cloud/so101_motion_calibrator.gd").read_text()
    assert "var minimum_distinct := 5 if joint_index == 4 else 3" in source


def test_wrist_calibration_uses_safe_route_and_exposed_negative_roll_range():
    initial = [-33.0, 128.0, 39.0, 78.0, -137.0, 50.0]
    waypoints = follower_service.build_wrist_calibration_sweep_waypoints(initial)

    sampled = [
        pose
        for pose, _, label in waypoints
        if label.startswith("sampling servo 4 wrist_roll")
    ]
    assert len(sampled) == 15
    assert {pose[4] for pose in sampled} == {-140.0, -125.0, -110.0, -95.0, -80.0}
    assert waypoints[-1][0] == pytest.approx(initial)
    planned_poses = [initial, *[pose for pose, _, _ in waypoints]]
    assert min(
        follower_service._path_min_clearance(before, after, 12)
        for before, after in zip(planned_poses, planned_poses[1:])
    ) >= CALIBRATION_SAFE_HEIGHT_M


def test_joint_retry_collects_only_new_complementary_wrist_views():
    initial = [-35.0, 125.0, 40.0, 75.0, -110.0, 50.0]
    waypoints = build_joint_calibration_sweep_waypoints(
        initial,
        view_strategy_attempt=2,
    )
    sampled = [
        pose
        for pose, _, label in waypoints
        if label.startswith("sampling servo 4 wrist_roll")
    ]

    assert len(sampled) == 20
    assert not any(
        label.startswith("sampling servo 1")
        or label.startswith("sampling servo 2")
        or label.startswith("sampling servo 3")
        for _, _, label in waypoints
    )
    assert len({round(pose[0], 1) for pose in sampled}) == 4
    assert {pose[3] for pose in sampled} == {60.0, 65.0, 75.0, 80.0}
    assert all(
        follower_service._path_min_clearance(before, after, 12)
        >= CALIBRATION_SAFE_HEIGHT_M
        for before, after in zip(
            [initial, *[pose for pose, _, _ in waypoints][:-1]],
            [pose for pose, _, _ in waypoints],
            strict=True,
        )
    )


def test_base_axis_sweep_locks_downstream_servos_and_spans_sixty_degrees():
    initial = [-3.0, 130.0, 38.0, 78.0, -8.0, 2.5]
    waypoints = build_base_axis_sweep_waypoints(initial)
    sampled = [
        pose
        for pose, _, label in waypoints
        if label.startswith("sampling servo 0 shoulder_pan")
    ]
    assert len(sampled) == 5
    assert max(pose[0] for pose in sampled) - min(pose[0] for pose in sampled) >= 60.0
    assert all(pose[1:] == sampled[0][1:] for pose in sampled)
    assert all(
        tool_clearance_metric(pose) >= 0.009
        for pose, _, label in waypoints
        if "base-axis" in label or "servo 0" in label
    )


def test_base_axis_retry_uses_interleaved_camera_views_without_moving_other_joints():
    initial = [-3.0, 130.0, 38.0, 78.0, -8.0, 2.5]
    first = build_base_axis_sweep_waypoints(initial, 1)
    retry = build_base_axis_sweep_waypoints(initial, 2)
    first_samples = {
        round(pose[0], 3)
        for pose, _, label in first
        if label.startswith("sampling servo 0 shoulder_pan")
    }
    retry_samples = [
        pose
        for pose, _, label in retry
        if label.startswith("sampling servo 0 shoulder_pan")
    ]
    assert len(retry_samples) == 5
    assert len(first_samples.intersection(round(pose[0], 3) for pose in retry_samples)) <= 1
    assert all(pose[1:] == retry_samples[0][1:] for pose in retry_samples)
    assert all(
        tool_clearance_metric(pose) >= 0.009
        for pose, _, label in retry
        if "base-axis" in label or "servo 0" in label
    )


def test_base_axis_command_reports_only_shoulder_pan_stage():
    pair = profile()
    clock = [10.0]
    initial = [-3.0, 130.0, 38.0, 78.0, -8.0, 2.5]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.receive({"type": "arm_base_axis_sweep", "action": "start"})
    assert controller.calibration_sweep_mode == "axis"
    observed = set()
    steps = int(controller.calibration_sweep_duration * 30.0) + 90
    for _ in range(steps):
        clock[0] += 1.0 / 30.0
        controller.update()
        if controller.calibration_joint_index >= 0:
            observed.add(controller.calibration_joint_index)
    assert observed == {0}
    assert controller.state == "hold"


def test_base_axis_command_routes_camera_view_retry_attempt():
    pair = profile()
    clock = [10.0]
    initial = [-3.0, 130.0, 38.0, 78.0, -8.0, 2.5]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.receive({
        "type": "arm_base_axis_sweep",
        "action": "start",
        "view_strategy_attempt": 2,
    })
    sample_labels = {
        label
        for _, _, label in controller.calibration_sweep_waypoints
        if label.startswith("sampling servo 0 shoulder_pan")
    }
    assert len(sample_labels) == 5
    assert any("alternate-low" in label for label in sample_labels)
    assert any("alternate-high" in label for label in sample_labels)


def test_claw_command_routes_all_five_camera_view_attempts():
    pair = profile()
    clock = [10.0]
    initial = [-3.0, 130.0, 38.0, 78.0, -8.0, 2.5]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.receive({
        "type": "arm_claw_calibration_sweep",
        "action": "start",
        "view_strategy_attempt": 5,
    })
    exposed = [
        pose
        for pose, _, label in controller.calibration_sweep_waypoints
        if label == "moving to exposed claw pose"
    ]
    assert len(exposed) == 1
    assert exposed[0][4] == -50.0


def test_joint_calibration_command_reports_current_servo_stage():
    pair = profile()
    clock = [10.0]
    initial_normalized = [-35.0, 170.0, 90.0, 135.0, 10.0, 20.0]
    bus = FakeFollowerBus(pair.follower_calibration.normalized_to_raw(initial_normalized))
    controller = FollowerController(pair, bus, now=lambda: clock[0])
    controller.connect()
    controller.receive({"type": "arm_joint_calibration_sweep", "action": "start"})
    assert controller.calibration_sweep_mode == "joints"
    observed = set()
    steps = int(controller.calibration_sweep_duration * 30.0) + 90
    for _ in range(steps):
        clock[0] += 1.0 / 30.0
        controller.update()
        if controller.calibration_joint_index >= 0:
            observed.add(controller.calibration_joint_index)
    assert observed == {1, 2, 3, 4}
    assert controller.state == "hold"


def test_joint_calibration_can_restart_from_its_already_raised_anchor():
    anchor = [-2.3, 130.0, 35.0, 75.0, -8.0, 2.0]
    waypoints = build_joint_calibration_sweep_waypoints(anchor)

    sampled = [label for _, _, label in waypoints if label.startswith("sampling servo")]
    assert len(sampled) == 45
    assert all(tool_clearance_metric(pose) >= CALIBRATION_SAFE_HEIGHT_M for pose, _, _ in waypoints)


def test_calibration_sweep_reseeds_stale_write_guard_from_physical_pose():
    class GuardedBus(FakeFollowerBus):
        def __init__(self, initial):
            super().__init__(initial)
            self.last_written = [value + 900 for value in initial]
            self.seed_count = 0

        def reseed_position_guard(self, positions):
            self.last_written = list(positions)
            self.seed_count += 1

        def write_positions(self, positions):
            jumps = [abs(current - previous) for current, previous in zip(positions, self.last_written, strict=True)]
            if any(jump > 256 for jump in jumps):
                raise RuntimeError(f"raw motor jump guard tripped: {jumps}")
            self.last_written = list(positions)
            super().write_positions(positions)

    pair = profile()
    initial = pair.follower_calibration.normalized_to_raw([-20.0, 175.0, 90.0, 95.0, -100.0, 25.0])
    bus = GuardedBus(initial)
    controller = FollowerController(pair, bus)
    controller.connect()
    controller.receive({"type": "arm_calibration_sweep", "action": "start"})
    assert controller.state == "calibrating"
    assert controller.fault == ""
    assert bus.seed_count == 1


def test_linear_gripper_commands_are_clamped_to_calibrated_range():
    pair = profile()
    calibration = pair.follower_calibration
    below = calibration.normalized_to_raw([0, 0, 0, 0, 0, -50])[5]
    above = calibration.normalized_to_raw([0, 0, 0, 0, 0, 150])[5]
    assert below == calibration.start_pos[5]
    assert above == calibration.end_pos[5]
