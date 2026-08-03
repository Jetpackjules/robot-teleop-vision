#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import signal
import socket
import sys
import time
from collections import deque
from pathlib import Path
from typing import Protocol

from so101_arm_common import ArmPairProfile, DEFAULT_PROFILE, limit_step, pose_is_close
from so101_kinematics import (
    arm_pose,
    joint_rate_step,
    rendered_minimum_height,
    solve_pose_target,
    tool_rate_step,
    wrist_frame_twist,
)


CALIBRATION_BROAD_SEGMENT_SECONDS = 2.40
CALIBRATION_SAMPLE_DWELL_SECONDS = 0.60
CALIBRATION_LIFT_STEP_DEGREES = 2.0
CALIBRATION_LIFT_MAX_STEPS = 30
CALIBRATION_SAFE_HEIGHT_M = 0.010
CALIBRATION_LIFT_SEGMENT_SECONDS = 0.18
JOINT_CALIBRATION_SEGMENT_SECONDS = 1.25
JOINT_CALIBRATION_SAMPLE_DWELL_SECONDS = 0.85
WRIST_CALIBRATION_SAMPLE_DWELL_SECONDS = 1.45
BASE_AXIS_SEGMENT_SECONDS = 1.65
# The real shoulder-pan servo can still trail a positive endpoint when the
# interpolation segment ends.  Keep commanding the stationary sample pose long
# enough for the encoder to enter the gross settled gate and for
# Godot to receive multiple fresh depth frames there.
BASE_AXIS_SAMPLE_DWELL_SECONDS = 1.50
RUNTIME_IK_MIN_CLEARANCE_M = 0.010
DEFAULT_REST_POSE = Path(__file__).resolve().with_name("so101_rest_pose.json")
REST_RETURN_ARM_SPEED_DEGREES_PER_SECOND = 12.0
REST_RETURN_GRIPPER_SPEED_PER_SECOND = 18.0
REST_RETURN_CLEARANCE_TOLERANCE_M = 0.001
REST_RETURN_MINIMUM_MODELED_HEIGHT_M = -0.002
REST_RETURN_SETTLE_DEGREES = 0.40


def tool_clearance_metric(normalized: list[float] | tuple[float, ...]) -> float:
    """Return the lowest full-link mesh vertex above the SO-101 base plane."""
    return rendered_minimum_height(normalized)


def _path_min_clearance(start: list[float], end: list[float], samples: int = 12) -> float:
    return min(
        tool_clearance_metric([
            before + (after - before) * amount / samples
            for before, after in zip(start, end, strict=True)
        ])
        for amount in range(samples + 1)
    )


def _build_raised_calibration_prefix(
    starting_pose: list[float],
) -> tuple[list[float], list[tuple[list[float], float, str]]]:
    """Raise the complete rendered arm monotonically above its base plane."""
    start = [float(value) for value in starting_pose]
    raised = list(start)
    lift_path: list[list[float]] = []
    start_clearance = tool_clearance_metric(start)
    for _ in range(CALIBRATION_LIFT_MAX_STEPS):
        candidates: list[tuple[float, list[float]]] = []
        for joint_index in (1, 2, 3):
            for direction in (-1.0, 1.0):
                candidate = list(raised)
                candidate[joint_index] += direction * CALIBRATION_LIFT_STEP_DEGREES
                if abs(candidate[joint_index] - start[joint_index]) > 60.0:
                    continue
                candidates.append((tool_clearance_metric(candidate), candidate))
        best_clearance, best_pose = max(candidates, key=lambda item: item[0])
        current_clearance = tool_clearance_metric(raised)
        if best_clearance <= current_clearance + 0.0002:
            break
        if _path_min_clearance(raised, best_pose, 6) < current_clearance - 0.0002:
            break
        raised = best_pose
        lift_path.append(list(raised))

    raised_clearance = tool_clearance_metric(raised)
    if raised_clearance < CALIBRATION_SAFE_HEIGHT_M:
        raise RuntimeError(
            "could not find a calibration pose that keeps every rendered link above the base plane "
            f"(start={start_clearance * 100.0:.1f} cm, best={raised_clearance * 100.0:.1f} cm)"
        )
    return raised, [
        (pose, CALIBRATION_LIFT_SEGMENT_SECONDS, "raising tool") for pose in lift_path
    ]


def _rest_segment_seconds(start: list[float], end: list[float]) -> float:
    arm_seconds = max(abs(after - before) for before, after in zip(start[:5], end[:5], strict=True)) / REST_RETURN_ARM_SPEED_DEGREES_PER_SECOND
    gripper_seconds = abs(end[5] - start[5]) / REST_RETURN_GRIPPER_SPEED_PER_SECOND
    return max(0.75, arm_seconds, gripper_seconds)


def build_rest_return_waypoints(
    starting_pose: list[float], rest_pose: list[float]
) -> list[tuple[list[float], float, str]]:
    """Plan a slow, modeled-base-safe path to an encoder-defined rest pose.

    This guards the known robot/base geometry. It deliberately does not claim
    to detect people or loose objects, which remain reasons to leave automatic
    idle return disabled.
    """
    start = [float(value) for value in starting_pose]
    rest = [float(value) for value in rest_pose]
    start_height = tool_clearance_metric(start)
    rest_height = tool_clearance_metric(rest)
    if min(start_height, rest_height) < REST_RETURN_MINIMUM_MODELED_HEIGHT_M:
        raise RuntimeError(
            "rest return rejected because an endpoint is below the modeled base plane "
            f"(start={start_height * 1000.0:.1f} mm, rest={rest_height * 1000.0:.1f} mm)"
        )
    allowed_height = min(start_height, rest_height, CALIBRATION_SAFE_HEIGHT_M) - REST_RETURN_CLEARANCE_TOLERANCE_M
    if _path_min_clearance(start, rest, 80) >= allowed_height:
        return [(rest, _rest_segment_seconds(start, rest), "moving slowly to rest")]

    start_raised, start_lift = _build_raised_calibration_prefix(start)
    rest_raised, rest_lift = _build_raised_calibration_prefix(rest)
    if _path_min_clearance(start_raised, rest_raised, 80) < CALIBRATION_SAFE_HEIGHT_M:
        raise RuntimeError("no modeled-base-safe bridge to the saved rest pose is available")
    result: list[tuple[list[float], float, str]] = []
    previous = start
    for pose, _, _ in start_lift:
        result.append((list(pose), _rest_segment_seconds(previous, pose), "raising before rest return"))
        previous = list(pose)
    result.append(
        (list(rest_raised), _rest_segment_seconds(previous, rest_raised), "crossing above the base")
    )
    previous = list(rest_raised)
    rest_lift_poses = [rest, *[list(item[0]) for item in rest_lift]]
    for pose in reversed(rest_lift_poses[:-1]):
        result.append((list(pose), _rest_segment_seconds(previous, pose), "lowering into rest"))
        previous = list(pose)
    return result


def load_rest_pose(profile: ArmPairProfile, path: Path = DEFAULT_REST_POSE) -> dict:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if int(payload.get("version", 0)) != 1:
        raise RuntimeError("unsupported rest-pose file version")
    if str(payload.get("follower_serial", "")) != profile.follower_serial:
        raise RuntimeError("rest pose belongs to a different follower arm")
    raw = payload.get("raw_positions")
    normalized_saved = payload.get("normalized_positions")
    if not isinstance(raw, list) or len(raw) != 6 or any(
        isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 4095
        for value in raw
    ):
        raise RuntimeError("rest pose must contain six valid raw encoder positions")
    if not isinstance(normalized_saved, list) or len(normalized_saved) != 6 or any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        for value in normalized_saved
    ):
        raise RuntimeError("rest pose must contain six finite normalized positions")
    normalized = profile.follower_calibration.raw_to_normalized(raw)
    mismatch = max(
        abs(current - float(saved))
        for current, saved in zip(normalized, normalized_saved, strict=True)
    )
    if mismatch > 2.0:
        raise RuntimeError(
            f"rest pose does not match the current servo calibration ({mismatch:.1f} deg mismatch)"
        )
    result = dict(payload)
    result["raw_positions"] = list(raw)
    result["normalized_positions"] = list(normalized)
    return result


def build_calibration_sweep_waypoints(
    starting_pose: list[float],
) -> list[tuple[list[float], float, str]]:
    """Build a raised, clearance-checked, one-joint-at-a-time sweep."""
    start = [float(value) for value in starting_pose]
    raised, waypoints = _build_raised_calibration_prefix(start)
    # The safety condition is the absolute base plane, not a fixed gain from
    # the starting pose.  Requiring both made a pose starting 9 mm below the
    # modeled plane need 16 mm clearance, even though 10 mm is already safe.
    # The lift path is monotonic, so accepting the absolute safe height does
    # not introduce a downward move through the table.
    minimum_clearance = CALIBRATION_SAFE_HEIGHT_M
    base = raised[0]
    wrist_roll = raised[4]
    broad_poses = [
        [base, 110.0, 15.0, 60.0, wrist_roll, raised[5]],
        [base, 110.0, 55.0, 100.0, wrist_roll, raised[5]],
        [base, 155.0, 25.0, 70.0, wrist_roll, raised[5]],
        [max(-95.0, min(95.0, base + 40.0)), 120.0, 30.0, 75.0, max(-140.0, min(140.0, wrist_roll + 60.0)), raised[5]],
        [max(-95.0, min(95.0, base - 40.0)), 140.0, 25.0, 65.0, wrist_roll, raised[5]],
    ]
    accepted_broad_poses: list[list[float]] = []
    for pose_index, candidate in enumerate(broad_poses, start=1):
        if _path_min_clearance(raised, candidate, 30) < minimum_clearance:
            continue
        accepted_broad_poses.append(candidate)
        waypoints.append((candidate, CALIBRATION_BROAD_SEGMENT_SECONDS, f"moving to broad pose {pose_index}"))
        waypoints.append((list(candidate), CALIBRATION_SAMPLE_DWELL_SECONDS, f"sampling broad pose {pose_index}"))
        waypoints.append((list(raised), CALIBRATION_BROAD_SEGMENT_SECONDS, "returning to raised pose"))
        waypoints.append((list(raised), CALIBRATION_SAMPLE_DWELL_SECONDS, f"sampling return from broad pose {pose_index}"))

    if len(accepted_broad_poses) < 5:
        raise RuntimeError("safe calibration path cannot reach all five broad arm poses")
    planned = [raised, *accepted_broad_poses]
    ranges = [max(pose[index] for pose in planned) - min(pose[index] for pose in planned) for index in range(5)]
    required_ranges = (50.0, 40.0, 40.0, 40.0, 30.0)
    if any(actual < required for actual, required in zip(ranges, required_ranges, strict=True)):
        raise RuntimeError(f"safe calibration path is not geometrically distinct enough: ranges={ranges}")

    # Deliberately remain at the raised pose. Returning to an initially unsafe
    # pose would lower the real gripper back into the table after calibration.
    return waypoints


def build_joint_calibration_sweep_waypoints(
    starting_pose: list[float],
    view_strategy_attempt: int = 1,
) -> list[tuple[list[float], float, str]]:
    """Build a raised sweep that excites one observable arm joint at a time.

    Each pitch joint is solved from the measured revolute axis of the next
    servo, rather than from link appearance. Wrist roll is therefore excited
    too: its motion axis determines wrist-flex zero even though the customized
    camera bracket is deliberately absent from the stock 3D model.
    """
    start = [float(value) for value in starting_pose]
    # Once the complete proximal sweep has been captured, a distal optical
    # disagreement does not justify repeating every joint.  Preserve those
    # frames in Godot and collect only new wrist observations from genuinely
    # different D455 rays.
    if view_strategy_attempt > 1:
        return build_wrist_calibration_sweep_waypoints(
            start,
            view_strategy_attempt,
        )
    # Joint refinement only needs the monotonic raising prefix. Deriving that
    # prefix through the unrelated broad base-registration sweep made a second
    # calibration from an already-raised pose fail its broad-range validation
    # and unnecessarily fault/release the follower.
    raised, waypoints = _build_raised_calibration_prefix(start)
    # Always put the rigid but unmodelled wrist-camera assembly on the back of
    # the claw from the reference D455. The final roll samples then stay in the
    # exposed [-140, -80] degree range: the stock jaws remain visible and the
    # attachment cannot bias their surface fit. Roll alone cannot lower a link.
    anchor_roll = -110.0
    center_pan = max(-35.0, min(35.0, raised[0]))
    # Keep the complete arm lower and extended in the D455's shared frustum.
    # The former compact [130, 35, 75] geometry repeatedly put the distal arm
    # above the image even though it was mechanically safe.
    # 125 degrees keeps the arm lower than the former 130-degree compact pose
    # while preserving a full 30-degree measured shoulder observation span.
    # At 120 degrees the nominal span was only 25 degrees; ordinary servo lag
    # could shrink it below the solver's 24-degree evidence gate, making every
    # automatic retry repeat a trajectory that could never pass.
    anchor = [center_pan, 125.0, 40.0, 75.0, anchor_roll, raised[5]]
    if _path_min_clearance(raised, anchor, 30) < CALIBRATION_SAFE_HEIGHT_M:
        raise RuntimeError("could not reach the attachment-safe joint-calibration pose")

    waypoints.append((anchor, CALIBRATION_BROAD_SEGMENT_SECONDS, "moving to joint-calibration anchor"))
    waypoints.append(
        (list(anchor), JOINT_CALIBRATION_SAMPLE_DWELL_SECONDS, "sampling joint-calibration anchor")
    )

    # Five settled angles per servo make the rotation axis substantially more
    # observable than a low/center/high triplet while preserving the same
    # conservative endpoint range.
    specifications = (
        (1, 15.0, (110.0, 155.0), "shoulder_lift"),
        (2, 15.0, (20.0, 55.0), "elbow_flex"),
        (3, 15.0, (60.0, 100.0), "wrist_flex"),
        # Roll does not change stock-link height. A wider, still low-twist
        # range gives the opposite RealSense view enough parallax to locate
        # this compact axis despite the custom camera bracket.
        (4, 30.0, (-140.0, 140.0), "wrist_roll"),
    )
    for joint_index, amplitude, bounds, name in specifications:
        stage_anchor = list(anchor)
        if joint_index == 4:
            # The compact anchor folds the customized camera bracket and claw
            # against the arm from the D435's viewpoint. The former
            # [100, 0, 30] pose raised the claw above the D455 image. Lower and
            # extend it into the D455 depth frustum for the roll observations.
            # Open the jaw during this stage so its asymmetric articulated
            # surface is visible; a nearly closed jaw leaves wrist zero
            # geometrically ambiguous from a single depth viewpoint.
            stage_anchor[1:4] = [110.0, 60.0, 75.0]
            stage_anchor[5] = 75.0
            if (
                _path_min_clearance(anchor, stage_anchor, 36)
                < CALIBRATION_SAFE_HEIGHT_M
            ):
                raise RuntimeError(
                    "could not reach the exposed wrist-roll observation pose"
                )
            waypoints.append(
                (
                    list(stage_anchor),
                    CALIBRATION_BROAD_SEGMENT_SECONDS,
                    "moving to exposed wrist-roll observation pose",
                )
            )
            waypoints.append(
                (
                    list(stage_anchor),
                    JOINT_CALIBRATION_SAMPLE_DWELL_SECONDS,
                    "sampling exposed wrist-roll observation pose",
                )
            )
        values = (
            max(bounds[0], stage_anchor[joint_index] - amplitude),
            max(bounds[0], stage_anchor[joint_index] - amplitude * 0.5),
            stage_anchor[joint_index],
            min(bounds[1], stage_anchor[joint_index] + amplitude * 0.5),
            min(bounds[1], stage_anchor[joint_index] + amplitude),
        )
        if min(
            stage_anchor[joint_index] - values[0],
            values[-1] - stage_anchor[joint_index],
        ) < 10.0:
            raise RuntimeError(f"joint-calibration range for {name} is too small")
        observation_views = (
            (
                ("view-left", -25.0),
                ("view-right", 25.0),
                # The custom wrist camera can completely hide the stock jaw
                # in one shoulder-pan view.  A third, independently settled
                # D455 view lets the strict solver require two agreeing views
                # without making one unavoidable occlusion fatal.
                ("view-forward", 50.0),
            )
            if joint_index == 4
            else (("view-left", -25.0), ("view-right", 25.0))
        )
        for view_name, pan_offset in observation_views:
            view_anchor = list(stage_anchor)
            view_anchor[0] = center_pan + pan_offset
            if (
                _path_min_clearance(stage_anchor, view_anchor, 24)
                < CALIBRATION_SAFE_HEIGHT_M
            ):
                raise RuntimeError(
                    f"joint-calibration pan path for {name} {view_name} is not clear"
                )
            waypoints.append(
                (
                    view_anchor,
                    JOINT_CALIBRATION_SEGMENT_SECONDS,
                    f"moving to {name} {view_name}",
                )
            )
            for sample_index, value in enumerate(values, start=1):
                candidate = list(view_anchor)
                candidate[joint_index] = value
                if (
                    _path_min_clearance(view_anchor, candidate, 24)
                    < CALIBRATION_SAFE_HEIGHT_M
                ):
                    raise RuntimeError(
                        f"joint-calibration path for {name} {view_name} "
                        f"sample {sample_index} is not clear"
                    )
                waypoints.append(
                    (
                        candidate,
                        JOINT_CALIBRATION_SEGMENT_SECONDS,
                        f"moving servo {joint_index} {name} {view_name} "
                        f"sample-{sample_index}",
                    )
                )
                waypoints.append(
                    (
                        list(candidate),
                        (
                            WRIST_CALIBRATION_SAMPLE_DWELL_SECONDS
                            if joint_index == 4
                            else JOINT_CALIBRATION_SAMPLE_DWELL_SECONDS
                        ),
                        f"sampling servo {joint_index} {name} {view_name} "
                        f"sample-{sample_index}",
                    )
                )
            waypoints.append(
                (
                    list(view_anchor),
                    JOINT_CALIBRATION_SEGMENT_SECONDS,
                    f"returning servo {joint_index} {name} {view_name}",
                )
            )
        waypoints.append(
            (
                list(anchor),
                (
                    CALIBRATION_BROAD_SEGMENT_SECONDS
                    if joint_index == 4
                    else JOINT_CALIBRATION_SEGMENT_SECONDS
                ),
                f"returning {name} to center view",
            )
        )
    return waypoints


def build_wrist_calibration_sweep_waypoints(
    starting_pose: list[float],
    view_strategy_attempt: int = 1,
) -> list[tuple[list[float], float, str]]:
    """Safely expose the stock jaws, sweep roll, then restore the pose."""
    start = [float(value) for value in starting_pose]
    if tool_clearance_metric(start) < CALIBRATION_SAFE_HEIGHT_M:
        raise RuntimeError("the lowered wrist-calibration pose is not clear")
    raised, lift_waypoints = _build_raised_calibration_prefix(start)
    center_pan = max(-60.0, min(60.0, raised[0]))
    # Preserve the requested low tool height while changing wrist flex so the
    # fixed and moving jaw surfaces face the two RealSense cameras instead of
    # being hidden against the table. This is the same clearance-checked
    # observation geometry used by the full outward joint calibration.
    anchor = [center_pan, 110.0, 60.0, 75.0, -110.0, raised[5]]
    if _path_min_clearance(raised, anchor, 36) < CALIBRATION_SAFE_HEIGHT_M:
        raise RuntimeError("could not reach the exposed low wrist pose")
    center_roll = anchor[4]
    values = (-140.0, -125.0, center_roll, -95.0, -80.0)
    waypoints: list[tuple[list[float], float, str]] = list(lift_waypoints)
    waypoints.extend([
        (list(anchor), CALIBRATION_BROAD_SEGMENT_SECONDS, "moving to exposed low wrist pose"),
        (list(anchor), WRIST_CALIBRATION_SAMPLE_DWELL_SECONDS, "sampling exposed low wrist pose"),
    ])
    # The asymmetric views put the same jaw surfaces into complementary camera
    # rays while remaining within the arm's proven shared-frustum pan range.
    if view_strategy_attempt <= 1:
        observation_views = (
            ("view-left", -25.0, 75.0),
            ("view-right", 25.0, 75.0),
            ("view-forward", 50.0, 75.0),
        )
    elif view_strategy_attempt == 2:
        # Pan alone can leave the rigid camera attachment hiding the same jaw
        # face.  Combine four non-overlapping pan rays with complementary
        # wrist-flex angles, all within the already proven [60, 100] sweep.
        observation_views = (
            ("retry-far-left", -50.0, 60.0),
            ("retry-center", 0.0, 80.0),
            ("retry-right", 40.0, 65.0),
            ("retry-far-right", 65.0, 75.0),
        )
    else:
        observation_views = (
            ("retry2-far-left", -60.0, 80.0),
            ("retry2-left", -35.0, 60.0),
            ("retry2-center", 10.0, 75.0),
            ("retry2-right", 60.0, 65.0),
        )
    for view_name, pan_offset, wrist_flex in observation_views:
        view_anchor = list(anchor)
        view_anchor[0] = max(-95.0, min(95.0, center_pan + pan_offset))
        view_anchor[3] = wrist_flex
        if _path_min_clearance(anchor, view_anchor, 24) < CALIBRATION_SAFE_HEIGHT_M:
            raise RuntimeError(f"lowered wrist pan path {view_name} is not clear")
        waypoints.append(
            (list(view_anchor), JOINT_CALIBRATION_SEGMENT_SECONDS, f"moving to wrist_roll {view_name}")
        )
        for sample_index, value in enumerate(values, start=1):
            candidate = list(view_anchor)
            candidate[4] = value
            if (
                _path_min_clearance(view_anchor, candidate, 24)
                < CALIBRATION_SAFE_HEIGHT_M
            ):
                raise RuntimeError(
                    f"wrist-roll path {view_name} sample {sample_index} is not clear"
                )
            waypoints.append(
                (
                    candidate,
                    JOINT_CALIBRATION_SEGMENT_SECONDS,
                    f"moving servo 4 wrist_roll {view_name} sample-{sample_index}",
                )
            )
            waypoints.append(
                (
                    list(candidate),
                    WRIST_CALIBRATION_SAMPLE_DWELL_SECONDS,
                    f"sampling servo 4 wrist_roll {view_name} sample-{sample_index}",
                )
            )
        waypoints.append(
            (list(view_anchor), JOINT_CALIBRATION_SEGMENT_SECONDS, f"returning wrist_roll {view_name}")
        )
    waypoints.append(
        (list(anchor), JOINT_CALIBRATION_SEGMENT_SECONDS, "leaving exposed wrist view")
    )
    waypoints.append(
        (list(raised), CALIBRATION_BROAD_SEGMENT_SECONDS, "returning to raised wrist route")
    )
    lift_poses = [list(pose) for pose, _, _ in lift_waypoints]
    for pose in reversed(lift_poses[:-1]):
        waypoints.append(
            (pose, CALIBRATION_LIFT_SEGMENT_SECONDS, "lowering along safe wrist route")
        )
    waypoints.append(
        (list(start), CALIBRATION_LIFT_SEGMENT_SECONDS, "restoring starting wrist pose")
    )
    return waypoints


def build_claw_calibration_sweep_waypoints(
    starting_pose: list[float],
    view_strategy_attempt: int = 1,
) -> list[tuple[list[float], float, str]]:
    """Expose the jaws to the D455 and sample five measured open states.

    Only the gripper servo moves while samples are taken. The approach pose is
    the same clearance-checked geometry already exercised by wrist calibration,
    with the unmodelled wrist camera kept on the back of the stock jaws.
    """
    start = [float(value) for value in starting_pose]
    raised, waypoints = _build_raised_calibration_prefix(start)
    lift_poses = [list(pose) for pose, _, _ in waypoints]
    center_pan = max(-60.0, min(60.0, raised[0]))
    attempt = max(1, min(5, int(view_strategy_attempt)))
    # Rotate only about the claw's own axis on optical retries. The custom wrist
    # camera hides the stock jaws across most of the old 180-degree retry arc.
    # Native-D455 captures consistently expose both tips near -20 degrees, so
    # retry in overlapping 15-degree views around that measured visibility
    # corridor. This is also strictly inside the previously exercised range.
    wrist_roll_view = {
        1: -20.0,
        2: -5.0,
        3: -35.0,
        4: 10.0,
        5: -50.0,
    }[attempt]
    anchor = [center_pan, 110.0, 60.0, 75.0, wrist_roll_view, start[5]]
    if _path_min_clearance(raised, anchor, 36) < CALIBRATION_SAFE_HEIGHT_M:
        raise RuntimeError("could not reach the exposed claw-calibration pose")
    # The calibrator already captures a depth baseline during preflight. Do not
    # emit a second sample at the approach pose: if the claw starts near the
    # first 5-degree state, that duplicate would consume the pose and leave only
    # four distinct opening samples.
    waypoints.append(
        (list(anchor), CALIBRATION_BROAD_SEGMENT_SECONDS, "moving to exposed claw pose")
    )
    for sample_index, opening in enumerate((5.0, 25.0, 50.0, 75.0, 90.0), start=1):
        candidate = list(anchor)
        candidate[5] = opening
        # Opening the linkage cannot lower an upstream link, but retain the
        # whole-model plane check so a future mesh/kinematics update fails safe.
        if _path_min_clearance(anchor, candidate, 24) < CALIBRATION_SAFE_HEIGHT_M:
            raise RuntimeError(f"claw sample {sample_index} is not clear of the base plane")
        waypoints.extend([
            (
                candidate,
                JOINT_CALIBRATION_SEGMENT_SECONDS,
                f"moving servo 5 gripper sample-{sample_index}",
            ),
            (
                list(candidate),
                JOINT_CALIBRATION_SAMPLE_DWELL_SECONDS,
                f"sampling servo 5 gripper sample-{sample_index}",
            ),
        ])
    restored = list(anchor)
    restored[5] = start[5]
    waypoints.append(
        (
            restored,
            JOINT_CALIBRATION_SEGMENT_SECONDS,
            "returning gripper to its starting opening",
        )
    )
    waypoints.append(
        (list(raised), CALIBRATION_BROAD_SEGMENT_SECONDS, "returning to raised claw route")
    )
    # Only replay the monotonic prefix generated before the exposed anchor.
    # This restores every upstream servo, rather than leaving the arm in the
    # camera-facing calibration pose after a successful or rejected fit.
    for pose in reversed(lift_poses[:-1]):
        waypoints.append(
            (pose, CALIBRATION_LIFT_SEGMENT_SECONDS, "lowering along safe claw route")
        )
    waypoints.append(
        (list(start), CALIBRATION_LIFT_SEGMENT_SECONDS, "restoring starting claw pose")
    )
    return waypoints


def build_base_axis_sweep_waypoints(
    starting_pose: list[float],
    view_strategy_attempt: int = 1,
) -> list[tuple[list[float], float, str]]:
    """Excite only shoulder pan so its fixed revolute axis is observable.

    The downstream joints remain locked in a compact, raised pose.  Five
    settled pan angles provide independent observations of the same rigid
    assembly without conflating base placement with elbow or wrist motion.
    """
    start = [float(value) for value in starting_pose]
    raised, waypoints = _build_raised_calibration_prefix(start)
    center_pan = max(-60.0, min(60.0, raised[0]))
    # This is the same proven low, exposed geometry used by wrist/claw capture.
    # It gives every pan sample substantially more D455 image margin than the
    # old compact raised pose.
    anchor = [center_pan, 110.0, 60.0, 75.0, raised[4], raised[5]]
    if _path_min_clearance(raised, anchor, 30) < CALIBRATION_SAFE_HEIGHT_M:
        raise RuntimeError("could not reach the compact base-axis anchor pose")

    waypoints.append((anchor, CALIBRATION_BROAD_SEGMENT_SECONDS, "moving to base-axis anchor"))
    waypoints.append(
        (list(anchor), BASE_AXIS_SAMPLE_DWELL_SECONDS, "sampling servo 0 shoulder_pan center")
    )
    # Use an asymmetric but equally broad arc. This physical shoulder-pan
    # reliably trails the former +30 endpoint by 5--7 degrees, while the
    # negative side has ample travel. Five measured poses over this 60-degree
    # requested arc preserve axis observability without repeatedly loading an
    # endpoint the servo cannot hold. Calibration always records the fresh
    # physical encoder angle, never these requested values.
    # A retry must not replay the same five image locations.  The capture
    # side retains every usable D455 pose, so these interleaved arcs fill a
    # missing/occluded bucket while keeping the arm in the same proven-safe
    # low, exposed geometry.
    attempt = max(1, min(3, int(view_strategy_attempt)))
    sample_offsets = {
        1: ((0.0, "center"), (-40.0, "low"), (-20.0, "mid-low"), (10.0, "mid-high"), (20.0, "high")),
        2: ((-30.0, "alternate-low"), (-10.0, "alternate-mid-low"), (5.0, "alternate-center"), (30.0, "alternate-mid-high"), (40.0, "alternate-high")),
        3: ((-50.0, "wide-low"), (-25.0, "wide-mid-low"), (15.0, "wide-center"), (35.0, "wide-mid-high"), (50.0, "wide-high")),
    }[attempt]

    # The first center sample used to be emitted separately.  Keep all five
    # samples in one strategy table so retries genuinely target new views.
    waypoints.pop()
    for offset, name in sample_offsets:
        candidate = list(anchor)
        candidate[0] = max(-95.0, min(95.0, center_pan + offset))
        if _path_min_clearance(anchor, candidate, 24) < CALIBRATION_SAFE_HEIGHT_M:
            raise RuntimeError(f"base-axis shoulder-pan path {name} is not clear")
        waypoints.append(
            (
                candidate,
                BASE_AXIS_SEGMENT_SECONDS,
                f"moving servo 0 shoulder_pan {name}",
            )
        )
        waypoints.append(
            (
                list(candidate),
                BASE_AXIS_SAMPLE_DWELL_SECONDS,
                f"sampling servo 0 shoulder_pan {name}",
            )
        )

    # Finish at the compact center pose instead of lowering back toward the
    # starting configuration.
    waypoints.append((list(anchor), BASE_AXIS_SEGMENT_SECONDS, "returning to base-axis anchor"))
    return waypoints


class FollowerBus(Protocol):
    connected: bool

    def connect(self) -> None: ...
    def read_positions(self) -> list[int]: ...
    def reseed_position_guard(self, positions: list[int]) -> None: ...
    def write_positions(self, positions: list[int]) -> None: ...
    def enable_torque(self) -> None: ...
    def disable_torque(self) -> None: ...
    def disable_gripper_torque(self) -> None: ...
    def close(self, disable_torque: bool = True) -> None: ...


class LeRobotFollowerBus:
    def __init__(
        self,
        port: str,
        motor_names: tuple[str, ...],
        ignored_tail_positions: tuple[int, ...] = (),
    ):
        self.port = port
        self.motor_names = motor_names
        self.ignored_tail_positions = ignored_tail_positions
        self.bus = None
        self.connected = False
        self.last_written_positions: list[int] | None = None
        self.max_raw_step = 256

    def connect(self) -> None:
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus

        motors = {
            name: Motor(index + 1, "sts3215", MotorNormMode.RANGE_M100_100)
            for index, name in enumerate(self.motor_names)
        }
        last_error = None
        for _attempt in range(3):
            self.bus = FeetechMotorsBus(port=self.port, motors=motors)
            try:
                self.bus.connect()
                self.connected = True
                self.last_written_positions = None
                return
            except Exception as exc:
                last_error = exc
                try:
                    self.bus.port_handler.closePort()
                except Exception:
                    pass
                time.sleep(0.1)
        raise ConnectionError(f"follower six-motor handshake failed after 3 attempts: {last_error}")

    def read_positions(self) -> list[int]:
        values = self.bus.sync_read("Present_Position", normalize=False, num_retry=2)
        return [int(values[name]) for name in self.motor_names] + list(self.ignored_tail_positions)

    def write_positions(self, positions: list[int]) -> None:
        physical_positions = positions[: len(self.motor_names)]
        if self.last_written_positions is not None:
            jumps = [
                abs(current - previous)
                for current, previous in zip(physical_positions, self.last_written_positions, strict=True)
            ]
            if any(jump > self.max_raw_step for jump in jumps):
                raise RuntimeError(f"raw motor jump guard tripped: {jumps}")
        self.bus.sync_write(
            "Goal_Position",
            dict(zip(self.motor_names, physical_positions, strict=True)),
            normalize=False,
            num_retry=2,
        )
        self.last_written_positions = list(physical_positions)

    def reseed_position_guard(self, positions: list[int]) -> None:
        self.last_written_positions = list(positions[: len(self.motor_names)])

    def enable_torque(self) -> None:
        self.bus.enable_torque(num_retry=2)

    def disable_torque(self) -> None:
        try:
            self.bus.disable_torque(num_retry=2)
        finally:
            # Once torque is off the arm can be moved by hand. The next command
            # must seed the jump guard from that new physical pose.
            self.last_written_positions = None

    def disable_gripper_torque(self) -> None:
        if "gripper" not in self.motor_names:
            return
        self.bus.disable_torque(motors="gripper", num_retry=2)

    def close(self, disable_torque: bool = True) -> None:
        if self.bus is not None and self.connected:
            try:
                self.bus.disconnect(disable_torque=disable_torque)
            except Exception:
                try:
                    if disable_torque:
                        self.bus.disable_torque(num_retry=1)
                finally:
                    self.bus.port_handler.closePort()
        self.connected = False
        self.last_written_positions = None


class FakeFollowerBus:
    def __init__(self, initial: list[int] | None = None):
        self.positions = list(initial or [2048] * 6)
        self.connected = False
        self.torque = False

    def connect(self) -> None:
        self.connected = True

    def read_positions(self) -> list[int]:
        return list(self.positions)

    def reseed_position_guard(self, _positions: list[int]) -> None:
        pass

    def write_positions(self, positions: list[int]) -> None:
        if self.torque:
            self.positions = list(positions)

    def enable_torque(self) -> None:
        self.torque = True

    def disable_torque(self) -> None:
        self.torque = False

    def disable_gripper_torque(self) -> None:
        pass

    def close(self, disable_torque: bool = True) -> None:
        if disable_torque:
            self.torque = False
        self.connected = False


class FollowerController:
    def __init__(
        self,
        profile: ArmPairProfile,
        bus: FollowerBus,
        now=time.monotonic,
        rest_pose_path: Path = DEFAULT_REST_POSE,
    ):
        self.profile = profile
        self.bus = bus
        self.now = now
        self.state = "disconnected"
        self.fault = ""
        self.status_message = "Follower service is starting."
        self.hardware_fault = ""
        self.torque_enabled = False
        self.gripper_torque_enabled = False
        self.last_command_at = 0.0
        self.last_seq = -1
        self.leader_raw: list[int] | None = None
        self.leader_wrapped_normalized: list[float] | None = None
        self.leader_normalized: list[float] | None = None
        self.leader_enable_normalized: list[float] | None = None
        self.follower_enable_normalized: list[float] | None = None
        self.target_normalized: list[float] | None = None
        self.applied_normalized: list[float] | None = None
        self.follower_raw: list[int] = [0] * 6
        self.follower_normalized: list[float] = [0.0] * 6
        self.command_latency_ms: float | None = None
        self.browser_transport_age_ms: float | None = None
        self.started_at = self.now()
        self.update_count = 0
        self.control_session = ""
        self.control_source = "none"
        self.latest_input_source = "none"
        self.keyboard_linear = [0.0, 0.0, 0.0]
        self.keyboard_angular = [0.0, 0.0, 0.0]
        self.keyboard_gripper = 0.0
        self.keyboard_wrist = [0.0, 0.0]
        self.keyboard_linear_smoothed = [0.0, 0.0, 0.0]
        self.keyboard_wrist_smoothed = [0.0, 0.0]
        self.keyboard_plane_forward = [1.0, 0.0, 0.0]
        self.keyboard_plane_right = [0.0, -1.0, 0.0]
        self.keyboard_joint_velocity = [0.0] * 6
        self.keyboard_tool_velocity = [0.0] * 6
        self.keyboard_mode = "joint"
        self.keyboard_frame = "base"
        self.keyboard_angular_frame = "base"
        self.keyboard_precision = False
        self.keyboard_orientation_target: tuple[tuple[float, ...], ...] | None = None
        self.keyboard_position_target: list[float] | None = None
        self.last_integrate_at = self.now()
        self.last_successful_write_at = 0.0
        self.last_successful_read_at = 0.0
        self.write_times: deque[float] = deque()
        self.read_times: deque[float] = deque()
        self.restart_count = 0
        self.restart_completed_unix_ms = 0
        self.calibration_sweep_active = False
        self.calibration_sweep_started_at = 0.0
        self.calibration_sweep_start: list[float] | None = None
        self.calibration_sweep_waypoints: list[tuple[list[float], float, str]] = []
        self.calibration_sweep_duration = 0.0
        self.calibration_sweep_progress = 0.0
        self.calibration_pose_settled = False
        self.calibration_sweep_mode = "base"
        self.calibration_joint_index = -1
        self.calibration_rejection = ""
        self.rest_pose: dict | None = None
        self.rest_pose_fault = ""
        try:
            self.rest_pose = load_rest_pose(profile, rest_pose_path)
        except Exception as exc:
            self.rest_pose_fault = str(exc)
        self.rest_return_active = False
        self.rest_return_started_at = 0.0
        self.rest_return_start: list[float] | None = None
        self.rest_return_waypoints: list[tuple[list[float], float, str]] = []
        self.rest_return_duration = 0.0
        self.rest_return_progress = 0.0
        self.rest_return_reason = ""
        self.idle_return_enabled = False
        self.idle_return_timeout_seconds = 600.0
        self.last_operator_activity_at = self.now()
        self.idle_return_attempted = False
        self.feedback_settings = {
            "measured_feedback_enabled": True,
            "target_ghost_enabled": True,
            "following_error_safety_enabled": True,
            "freeze_overlay_on_stale_enabled": True,
            "d455_visual_correction_enabled": False,
        }
        self.following_error_thresholds = (7.0, 7.0, 7.0, 8.0, 10.0, 12.0)
        self.following_error_normalized = [0.0] * 6
        self.following_error_started_at = 0.0
        self.following_error_trip_count = 0
        self.following_error_stop = False
        self.gripper_contact_latched = False
        self.gripper_contact_trip_count = 0
        self.arm_contact_latched = False
        self.arm_contact_trip_count = 0
        self.ik_guard_trip_count = 0
        self.feedback_read_failures = 0
        self.feedback_read_fault = ""
        self.armed_at = 0.0

    def connect(self) -> None:
        self.bus.connect()
        self.follower_raw = self._read_positions()
        self.follower_normalized = self.profile.follower_calibration.raw_to_normalized(self.follower_raw)
        self.applied_normalized = list(self.follower_normalized)
        self.state = "ready"
        self.fault = ""
        self.hardware_fault = ""
        self.status_message = "Follower connected; choose a control mode and enable."

    def receive(self, message: dict) -> None:
        kind = message.get("type")
        if kind == "arm_restart":
            self.last_operator_activity_at = self.now()
            self.restart_hardware()
            return
        if kind == "arm_return_to_rest":
            self.last_operator_activity_at = self.now()
            self.idle_return_attempted = False
            self.start_rest_return("operator")
            return
        if kind == "arm_idle_return_settings":
            self.idle_return_enabled = bool(message.get("enabled", False))
            self.idle_return_timeout_seconds = max(
                60.0, min(3600.0, float(message.get("timeout_seconds", 600.0)))
            )
            self.status_message = (
                "Automatic idle return enabled; modeled-path checks remain active."
                if self.idle_return_enabled
                else "Automatic idle return disabled."
            )
            return
        if kind == "arm_calibration_sweep":
            action = str(message.get("action", "start"))
            if action == "start":
                self.start_calibration_sweep("base")
            else:
                self.stop_calibration_sweep("automatic calibration sweep stopped")
            return
        if kind == "arm_joint_calibration_sweep":
            action = str(message.get("action", "start"))
            if action == "start":
                self.start_calibration_sweep(
                    "joints",
                    max(1, min(3, int(message.get("view_strategy_attempt", 1)))),
                )
            else:
                self.stop_calibration_sweep("automatic joint-calibration sweep stopped")
            return
        if kind == "arm_base_axis_sweep":
            action = str(message.get("action", "start"))
            if action == "start":
                self.start_calibration_sweep(
                    "axis",
                    max(1, min(3, int(message.get("view_strategy_attempt", 1)))),
                )
            else:
                self.stop_calibration_sweep("automatic base-axis sweep stopped")
            return
        if kind == "arm_wrist_calibration_sweep":
            action = str(message.get("action", "start"))
            if action == "start":
                self.start_calibration_sweep(
                    "wrist",
                    max(1, min(3, int(message.get("view_strategy_attempt", 1)))),
                )
            else:
                self.stop_calibration_sweep("automatic wrist-calibration sweep stopped")
            return
        if kind == "arm_claw_calibration_sweep":
            action = str(message.get("action", "start"))
            if action == "start":
                self.start_calibration_sweep(
                    "claw",
                    max(1, min(5, int(message.get("view_strategy_attempt", 1)))),
                )
            else:
                self.stop_calibration_sweep("automatic claw-calibration sweep stopped")
            return
        if kind == "arm_visual_contact_stop":
            self.sample_feedback_safely()
            self.target_normalized = list(self.follower_normalized)
            self.applied_normalized = list(self.follower_normalized)
            self.bus.reseed_position_guard(self.follower_raw)
            self._write_positions(self.follower_raw)
            self.hold("D455 wrist / claw divergence stopped motion; target rebased.")
            self.following_error_stop = True
            return
        if self.rest_return_active and kind in (
            "arm_command", "arm_cartesian_velocity", "arm_joint_velocity", "arm_tool_velocity", "arm_enable"
        ):
            return
        if self.calibration_sweep_active and kind in (
            "arm_command", "arm_cartesian_velocity", "arm_joint_velocity", "arm_tool_velocity", "arm_enable"
        ):
            return
        if kind == "arm_feedback_settings":
            for key in self.feedback_settings:
                self.feedback_settings[key] = bool(message[key])
            if not self.feedback_settings["following_error_safety_enabled"]:
                self.following_error_started_at = 0.0
                self.following_error_stop = False
            self.status_message = "Arm feedback settings updated."
            return
        incoming_session = str(message.get("control_session", ""))
        if incoming_session and incoming_session != self.control_session:
            if self.control_session:
                self.hold("controller session changed; re-enable required")
            self.control_session = incoming_session
            self.last_seq = -1
            self.leader_raw = None
            self.leader_wrapped_normalized = None
            self.leader_normalized = None
            self.leader_enable_normalized = None
            self.follower_enable_normalized = None
            self.gripper_contact_latched = False
            self.arm_contact_latched = False
        if kind == "arm_command":
            seq = int(message["seq"])
            if seq <= self.last_seq:
                return
            self.last_seq = seq
            previous_leader = list(self.leader_normalized) if self.leader_normalized is not None else None
            self.leader_raw = [int(v) for v in message["positions"]]
            wrapped_normalized, _ = self.profile.map_leader_to_follower(self.leader_raw)
            self.leader_normalized = self._unwrap_leader(wrapped_normalized)
            if self.leader_enable_normalized is not None and self.follower_enable_normalized is not None:
                self.target_normalized = [
                    follower_zero + leader_now - leader_zero
                    for follower_zero, leader_now, leader_zero in zip(
                        self.follower_enable_normalized,
                        self.leader_normalized,
                        self.leader_enable_normalized,
                        strict=True,
                    )
                ]
            else:
                self.target_normalized = list(self.leader_normalized)
            for index, mode in enumerate(self.profile.follower_calibration.calib_mode):
                if mode == "LINEAR":
                    self.target_normalized[index] = max(0.0, min(100.0, self.target_normalized[index]))
            self.last_command_at = self.now()
            received_ms = int(time.time() * 1000)
            self.command_latency_ms = max(0.0, received_ms - int(message.get("sent_unix_ms", received_ms)))
            self.browser_transport_age_ms = max(0.0, float(message.get("transport_age_ms", 0.0)))
            self.latest_input_source = "leader"
            if previous_leader is None or max(
                abs(current - previous)
                for current, previous in zip(self.leader_normalized, previous_leader, strict=True)
            ) > 0.5:
                self.last_operator_activity_at = self.now()
                self.idle_return_attempted = False
            if self.command_latency_ms > self.profile.watchdog_ms:
                self.last_command_at = 0.0
                self.hold("received command is older than watchdog limit")
            return
        if kind == "arm_cartesian_velocity":
            seq = int(message["seq"])
            if seq <= self.last_seq:
                return
            self.last_seq = seq
            incoming_linear = [float(value) for value in message["linear"]]
            incoming_angular = [float(value) for value in message["angular"]]
            incoming_wrist = [float(value) for value in message.get("wrist", [0.0, 0.0])]
            arm_motion_active = any(
                abs(value) > 0.001
                for value in (*incoming_linear, *incoming_angular, *incoming_wrist)
            )
            if arm_motion_active or abs(float(message.get("gripper", 0.0))) > 0.001:
                self.last_operator_activity_at = self.now()
                self.idle_return_attempted = False
            if self.arm_contact_latched and arm_motion_active:
                self.keyboard_linear = [0.0, 0.0, 0.0]
                self.keyboard_angular = [0.0, 0.0, 0.0]
                self.keyboard_wrist = [0.0, 0.0]
            else:
                if not arm_motion_active:
                    self.arm_contact_latched = False
                self.keyboard_linear = incoming_linear
                self.keyboard_angular = incoming_angular
                self.keyboard_wrist = incoming_wrist
            incoming_gripper = float(message.get("gripper", 0.0))
            # Closing against an object is expected gripper contact, not a
            # reason to cancel control of the whole arm.  Once contact is
            # detected, ignore continued close packets until the operator has
            # released the close key (zero) or explicitly commands opening.
            if self.gripper_contact_latched and incoming_gripper < -0.001:
                self.keyboard_gripper = 0.0
            else:
                if incoming_gripper >= -0.001:
                    self.gripper_contact_latched = False
                self.keyboard_gripper = incoming_gripper
            self.keyboard_joint_velocity = [0.0] * 6
            self.keyboard_tool_velocity = [0.0] * 6
            self.keyboard_mode = "cartesian"
            self.keyboard_frame = str(message.get("frame", "base"))
            self.keyboard_angular_frame = str(message.get("angular_frame", self.keyboard_frame))
            self.keyboard_precision = bool(message.get("precision", False))
            self.latest_input_source = "keyboard"
            self.last_command_at = self.now()
            received_ms = int(time.time() * 1000)
            self.command_latency_ms = max(0.0, received_ms - int(message.get("sent_unix_ms", received_ms)))
            self.browser_transport_age_ms = max(0.0, float(message.get("transport_age_ms", 0.0)))
            if self.command_latency_ms > self.profile.watchdog_ms:
                self.last_command_at = 0.0
                self.hold("received keyboard command is older than watchdog limit")
            return
        if kind == "arm_joint_velocity":
            seq = int(message["seq"])
            if seq <= self.last_seq:
                return
            self.last_seq = seq
            incoming_joint_velocity = [float(value) for value in message["velocities"]]
            joint_motion_active = any(abs(value) > 0.001 for value in incoming_joint_velocity)
            if joint_motion_active:
                self.last_operator_activity_at = self.now()
                self.idle_return_attempted = False
            if self.arm_contact_latched and joint_motion_active:
                self.keyboard_joint_velocity = [0.0] * 6
            else:
                if not joint_motion_active:
                    self.arm_contact_latched = False
                self.keyboard_joint_velocity = incoming_joint_velocity
            self.keyboard_linear = [0.0, 0.0, 0.0]
            self.keyboard_angular = [0.0, 0.0, 0.0]
            self.keyboard_gripper = 0.0
            self.keyboard_wrist = [0.0, 0.0]
            self.keyboard_tool_velocity = [0.0] * 6
            self.keyboard_mode = "joint"
            self.keyboard_frame = "base"
            self.keyboard_precision = bool(message.get("precision", False))
            self.latest_input_source = "keyboard"
            self.last_command_at = self.now()
            received_ms = int(time.time() * 1000)
            self.command_latency_ms = max(0.0, received_ms - int(message.get("sent_unix_ms", received_ms)))
            self.browser_transport_age_ms = max(0.0, float(message.get("transport_age_ms", 0.0)))
            if self.command_latency_ms > self.profile.watchdog_ms:
                self.last_command_at = 0.0
                self.hold("received keyboard command is older than watchdog limit")
            return
        if kind == "arm_tool_velocity":
            seq = int(message["seq"])
            if seq <= self.last_seq:
                return
            self.last_seq = seq
            incoming_tool_velocity = [float(value) for value in message["motions"]]
            tool_motion_active = any(abs(value) > 0.001 for value in incoming_tool_velocity)
            if tool_motion_active:
                self.last_operator_activity_at = self.now()
                self.idle_return_attempted = False
            if self.arm_contact_latched and tool_motion_active:
                self.keyboard_tool_velocity = [0.0] * 6
            else:
                if not tool_motion_active:
                    self.arm_contact_latched = False
                self.keyboard_tool_velocity = incoming_tool_velocity
            self.keyboard_joint_velocity = [0.0] * 6
            self.keyboard_linear = [0.0, 0.0, 0.0]
            self.keyboard_angular = [0.0, 0.0, 0.0]
            self.keyboard_gripper = 0.0
            self.keyboard_wrist = [0.0, 0.0]
            self.keyboard_mode = "tool"
            self.keyboard_frame = "base"
            self.keyboard_precision = bool(message.get("precision", False))
            self.latest_input_source = "keyboard"
            self.last_command_at = self.now()
            received_ms = int(time.time() * 1000)
            self.command_latency_ms = max(0.0, received_ms - int(message.get("sent_unix_ms", received_ms)))
            self.browser_transport_age_ms = max(0.0, float(message.get("transport_age_ms", 0.0)))
            if self.command_latency_ms > self.profile.watchdog_ms:
                self.last_command_at = 0.0
                self.hold("received keyboard command is older than watchdog limit")
            return
        if kind == "arm_enable":
            self.last_operator_activity_at = self.now()
            self.idle_return_attempted = False
            self.enable(str(message.get("source", self.latest_input_source)))
        elif kind == "arm_hold":
            self.last_operator_activity_at = self.now()
            self.idle_return_attempted = False
            if self.rest_return_active:
                self.stop_rest_return("operator hold during return to rest")
                return
            if self.calibration_sweep_active:
                self.stop_calibration_sweep("operator hold during automatic calibration")
                return
            self.hold("operator hold")
        elif kind == "arm_release_torque":
            self.release_torque()
        elif kind == "arm_release_gripper_torque":
            self.release_gripper_torque()

    def _unwrap_leader(self, wrapped: list[float]) -> list[float]:
        if self.leader_wrapped_normalized is None or self.leader_normalized is None:
            self.leader_wrapped_normalized = list(wrapped)
            return list(wrapped)
        unwrapped: list[float] = []
        for index, (current, previous_wrapped, previous_unwrapped) in enumerate(
            zip(wrapped, self.leader_wrapped_normalized, self.leader_normalized, strict=True)
        ):
            if self.profile.leader_calibration.calib_mode[index] == "LINEAR":
                unwrapped.append(current)
                continue
            delta = current - previous_wrapped
            while delta > 180.0:
                delta -= 360.0
            while delta < -180.0:
                delta += 360.0
            unwrapped.append(previous_unwrapped + delta)
        self.leader_wrapped_normalized = list(wrapped)
        return unwrapped

    def enable(self, source: str = "leader") -> None:
        if self.state == "fault":
            self.fault = "enable rejected: restart follower service after fault"
            return
        if not self.bus.connected:
            self._reject("follower is not connected")
            return
        if self.now() - self.last_command_at > self.profile.watchdog_ms / 1000.0:
            self._reject(f"{source} data is not fresh")
            return
        self.follower_raw = self._read_positions()
        self.follower_normalized = self.profile.follower_calibration.raw_to_normalized(self.follower_raw)
        if source == "keyboard":
            if self.latest_input_source != "keyboard":
                self._reject("keyboard data is unavailable")
                return
            self.target_normalized = list(self.follower_normalized)
            self.applied_normalized = list(self.follower_normalized)
            self._write_current_pose()
            self.bus.enable_torque()
            self.torque_enabled = True
            self.gripper_torque_enabled = True
            self.control_source = "keyboard"
            initial_pose = arm_pose(self.target_normalized)
            self.keyboard_orientation_target = initial_pose.rotation
            self.keyboard_position_target = list(initial_pose.position)
            tool_position = initial_pose.position
            radial_length = math.hypot(tool_position[0], tool_position[1])
            if radial_length > 0.04:
                self.keyboard_plane_forward = [tool_position[0] / radial_length, tool_position[1] / radial_length, 0.0]
                self.keyboard_plane_right = [self.keyboard_plane_forward[1], -self.keyboard_plane_forward[0], 0.0]
            self.keyboard_linear_smoothed = [0.0, 0.0, 0.0]
            self.keyboard_wrist_smoothed = [0.0, 0.0]
            self.last_integrate_at = self.now()
            self.state = "armed"
            self.armed_at = self.now()
            self.following_error_started_at = 0.0
            self.following_error_stop = False
            self.gripper_contact_latched = False
            self.arm_contact_latched = False
            self.fault = ""
            self.status_message = "Keyboard control is armed."
            return
        if self.target_normalized is None:
            self._reject("leader pose is unavailable")
            return
        _close, _errors = pose_is_close(
            self.follower_normalized,
            self.target_normalized,
            self.profile.start_pose_tolerance,
        )
        if self.leader_normalized is None:
            self._reject("leader pose is unavailable")
            return
        self.leader_enable_normalized = list(self.leader_normalized)
        self.follower_enable_normalized = list(self.follower_normalized)
        self.target_normalized = list(self.follower_normalized)
        self.applied_normalized = list(self.follower_normalized)
        self._write_current_pose()
        self.bus.enable_torque()
        self.torque_enabled = True
        self.gripper_torque_enabled = True
        self.control_source = "leader"
        self.state = "armed"
        self.armed_at = self.now()
        self.following_error_started_at = 0.0
        self.following_error_stop = False
        self.gripper_contact_latched = False
        self.arm_contact_latched = False
        self.fault = ""
        self.status_message = "Leader control is armed."

    def hold(self, reason: str) -> None:
        if self.rest_return_active:
            self.rest_return_active = False
            self.rest_return_waypoints = []
            self.rest_return_duration = 0.0
        if self.state == "fault":
            self.status_message = reason
            self.keyboard_linear = [0.0, 0.0, 0.0]
            self.keyboard_angular = [0.0, 0.0, 0.0]
            self.keyboard_gripper = 0.0
            self.keyboard_joint_velocity = [0.0] * 6
            self.keyboard_tool_velocity = [0.0] * 6
            return
        if self.state == "armed" or self.torque_enabled:
            self.state = "hold"
        else:
            self.state = "ready"
        self.fault = ""
        self.status_message = reason
        self.keyboard_linear = [0.0, 0.0, 0.0]
        self.keyboard_angular = [0.0, 0.0, 0.0]
        self.keyboard_gripper = 0.0
        self.keyboard_joint_velocity = [0.0] * 6
        self.keyboard_tool_velocity = [0.0] * 6

    def release_torque(self) -> None:
        self.rest_return_active = False
        self.rest_return_waypoints = []
        if self.bus.connected:
            self.bus.disable_torque()
        self.torque_enabled = False
        self.gripper_torque_enabled = False
        self.control_source = "none"
        self.state = "ready"
        self.fault = ""
        self.status_message = "Follower torque released."

    def release_gripper_torque(self) -> None:
        if not self.bus.connected:
            self._reject("follower is not connected")
            return
        self.hold("operator requested gripper calibration")
        self.bus.disable_gripper_torque()
        self.gripper_torque_enabled = False
        self.status_message = "Gripper torque released; joints 1-5 remain held."

    def restart_hardware(self) -> None:
        self.restart_count += 1
        self.calibration_sweep_active = False
        self.rest_return_active = False
        self.rest_return_waypoints = []
        self.state = "restarting"
        self.fault = ""
        self.status_message = "Closing and reopening the follower motor bus..."
        self.torque_enabled = False
        self.gripper_torque_enabled = False
        self.control_source = "none"
        self._clear_motion_inputs()
        close_warning = ""
        try:
            self.bus.close(disable_torque=True)
        except Exception as exc:
            close_warning = f"close warning: {exc}"
        time.sleep(0.15)
        try:
            self.bus.connect()
            self.follower_raw = self._read_positions()
            self.follower_normalized = self.profile.follower_calibration.raw_to_normalized(self.follower_raw)
            self.target_normalized = list(self.follower_normalized)
            self.applied_normalized = list(self.follower_normalized)
            self._write_current_pose()
            self.bus.enable_torque()
            self.torque_enabled = True
            self.state = "hold"
            self.fault = ""
            self.hardware_fault = ""
            self.status_message = "Follower connection restarted and holding current pose; press Enable Arm."
            if close_warning:
                self.status_message += f" ({close_warning})"
            self.restart_completed_unix_ms = int(time.time() * 1000)
            self.last_command_at = 0.0
            self.last_seq = -1
            self.leader_raw = None
            self.leader_wrapped_normalized = None
            self.leader_normalized = None
            self.leader_enable_normalized = None
            self.follower_enable_normalized = None
        except Exception as exc:
            try:
                self.bus.close(disable_torque=True)
            except Exception:
                pass
            self.fail(f"follower restart failed: {exc}")

    def start_calibration_sweep(
        self,
        mode: str = "base",
        view_strategy_attempt: int = 1,
    ) -> None:
        if self.state == "fault":
            self.status_message = "Automatic calibration rejected: restart the follower hardware first."
            return
        if not self.bus.connected:
            self._reject("follower is not connected")
            return
        if self.rest_return_active:
            self.stop_rest_return("automatic calibration replaced return to rest")
        try:
            self.follower_raw = self._read_positions()
            self.follower_normalized = self.profile.follower_calibration.raw_to_normalized(self.follower_raw)
            self.calibration_sweep_start = list(self.follower_normalized)
        except Exception as exc:
            self.fail(f"could not read the follower before automatic calibration: {exc}")
            return

        self.calibration_sweep_mode = mode if mode in ("base", "joints", "axis", "wrist", "claw") else "base"
        try:
            if self.calibration_sweep_mode == "joints":
                self.calibration_sweep_waypoints = build_joint_calibration_sweep_waypoints(
                    self.calibration_sweep_start,
                    view_strategy_attempt,
                )
            elif self.calibration_sweep_mode == "wrist":
                self.calibration_sweep_waypoints = build_wrist_calibration_sweep_waypoints(
                    self.calibration_sweep_start,
                    view_strategy_attempt,
                )
            elif self.calibration_sweep_mode == "claw":
                self.calibration_sweep_waypoints = build_claw_calibration_sweep_waypoints(
                    self.calibration_sweep_start,
                    view_strategy_attempt,
                )
            elif self.calibration_sweep_mode == "axis":
                self.calibration_sweep_waypoints = build_base_axis_sweep_waypoints(
                    self.calibration_sweep_start,
                    view_strategy_attempt,
                )
            else:
                self.calibration_sweep_waypoints = build_calibration_sweep_waypoints(
                    self.calibration_sweep_start
                )
            self.calibration_sweep_duration = sum(item[1] for item in self.calibration_sweep_waypoints)
        except Exception as exc:
            self.calibration_sweep_active = False
            self.calibration_sweep_waypoints = []
            self.calibration_sweep_duration = 0.0
            self.calibration_sweep_progress = 0.0
            self.calibration_pose_settled = False
            self.calibration_joint_index = -1
            self.control_source = "none"
            self.state = "hold" if self.torque_enabled else "ready"
            self.fault = ""
            self.calibration_rejection = str(exc)
            self.status_message = f"Automatic calibration rejected without moving the follower: {exc}"
            return

        try:
            self.target_normalized = list(self.follower_normalized)
            self.applied_normalized = list(self.follower_normalized)
            self._write_current_pose()
            self.bus.enable_torque()
        except Exception as exc:
            self.fail(f"could not start automatic calibration sweep: {exc}")
            return
        self.torque_enabled = True
        self.control_source = "calibration"
        self.latest_input_source = "calibration"
        self.calibration_sweep_active = True
        self.calibration_pose_settled = False
        self.calibration_joint_index = -1
        self.calibration_sweep_started_at = self.now()
        self.calibration_sweep_progress = 0.0
        self.state = "calibrating"
        self.fault = ""
        self.calibration_rejection = ""
        self.status_message = (
            "Running raised, clearance-checked SO-101 "
            f"{'joint-alignment' if self.calibration_sweep_mode == 'joints' else 'claw-alignment' if self.calibration_sweep_mode == 'claw' else 'base-axis' if self.calibration_sweep_mode == 'axis' else 'base-registration'} sweep "
            f"({self.calibration_sweep_duration:.1f}s)."
        )

    def stop_calibration_sweep(self, reason: str) -> None:
        if self.calibration_sweep_active and self.applied_normalized is not None:
            self.target_normalized = list(self.applied_normalized)
        self.calibration_sweep_active = False
        self.calibration_sweep_waypoints = []
        self.calibration_sweep_duration = 0.0
        self.calibration_sweep_progress = 0.0
        self.calibration_pose_settled = False
        self.calibration_joint_index = -1
        self.control_source = "none"
        if self.state != "fault":
            self.state = "hold" if self.torque_enabled else "ready"
            self.fault = ""
        self.status_message = reason

    def start_rest_return(self, reason: str = "operator") -> None:
        if self.rest_pose is None:
            self.status_message = f"Return to rest unavailable: {self.rest_pose_fault or 'no rest pose is saved'}."
            return
        if self.state in ("fault", "restarting", "calibrating"):
            self.status_message = f"Return to rest rejected while follower is {self.state}."
            return
        if not self.bus.connected:
            self._reject("follower is not connected")
            return
        try:
            self.follower_raw = self._read_positions()
            self.follower_normalized = self.profile.follower_calibration.raw_to_normalized(self.follower_raw)
            rest = list(self.rest_pose["normalized_positions"])
            close, _ = pose_is_close(
                self.follower_normalized,
                rest,
                (0.8, 0.8, 0.8, 0.8, 0.8, 1.5),
            )
            if close:
                self.target_normalized = list(rest)
                self.applied_normalized = list(rest)
                self.bus.reseed_position_guard(self.rest_pose["raw_positions"])
                self._write_positions(self.rest_pose["raw_positions"])
                self.bus.enable_torque()
                self.torque_enabled = True
                self.gripper_torque_enabled = True
                self.hold("Follower is already at the saved rest pose.")
                self.rest_return_progress = 1.0
                return
            waypoints = build_rest_return_waypoints(self.follower_normalized, rest)
            duration = sum(item[1] for item in waypoints)
            if not waypoints or duration <= 0.0:
                raise RuntimeError("no rest-return trajectory is available")
            self.target_normalized = list(self.follower_normalized)
            self.applied_normalized = list(self.follower_normalized)
            self.bus.reseed_position_guard(self.follower_raw)
            self._write_positions(self.follower_raw)
            self.bus.enable_torque()
        except Exception as exc:
            self.rest_return_active = False
            self.rest_return_waypoints = []
            self.rest_return_duration = 0.0
            self.rest_return_progress = 0.0
            self.state = "hold" if self.torque_enabled else "ready"
            self.fault = ""
            self.status_message = f"Return to rest rejected without moving: {exc}"
            return
        self._clear_motion_inputs()
        self.target_normalized = list(self.follower_normalized)
        self.applied_normalized = list(self.follower_normalized)
        self.rest_return_start = list(self.follower_normalized)
        self.rest_return_waypoints = waypoints
        self.rest_return_duration = duration
        self.rest_return_started_at = self.now()
        self.rest_return_progress = 0.0
        self.rest_return_reason = reason
        self.rest_return_active = True
        self.torque_enabled = True
        self.gripper_torque_enabled = True
        self.control_source = "rest"
        self.latest_input_source = "rest"
        self.state = "returning_rest"
        self.armed_at = self.now()
        self.following_error_started_at = 0.0
        self.following_error_stop = False
        self.fault = ""
        self.status_message = (
            "Returning slowly to the saved rest pose with modeled-clearance and encoder guards."
        )

    def stop_rest_return(self, reason: str) -> None:
        if self.rest_return_active and self.applied_normalized is not None:
            self.target_normalized = list(self.applied_normalized)
        self.rest_return_active = False
        self.rest_return_waypoints = []
        self.rest_return_duration = 0.0
        self.control_source = "none"
        if self.state != "fault":
            self.state = "hold" if self.torque_enabled else "ready"
            self.fault = ""
        self.status_message = reason

    def _update_rest_return(self) -> None:
        if not self.rest_return_active or self.rest_return_start is None:
            return
        elapsed = max(0.0, self.now() - self.rest_return_started_at)
        duration = self.rest_return_duration
        self.rest_return_progress = min(1.0, elapsed / max(0.001, duration))
        if elapsed >= duration:
            self.target_normalized = list(self.rest_return_waypoints[-1][0])
            self.applied_normalized = limit_step(
                self.applied_normalized, self.target_normalized, self.profile.max_step
            )
            self._write_positions(
                self.profile.follower_calibration.normalized_to_raw(self.applied_normalized)
            )
            settled, errors = pose_is_close(
                self.follower_normalized,
                self.target_normalized,
                (2.0, 2.0, 2.0, 2.0, 2.5, 3.0),
            )
            if settled:
                self.rest_return_progress = 1.0
                self.stop_rest_return("Return to rest complete; follower is holding the saved encoder pose.")
            elif elapsed - duration > 4.0:
                self.stop_rest_return(
                    "Return to rest stopped because the measured pose did not settle "
                    f"(largest error {max(errors):.1f} deg)."
                )
            else:
                self.status_message = "Return to rest: waiting for measured servos to settle."
            return
        segment_started_at = 0.0
        segment = 0
        for index, (_, segment_duration, _) in enumerate(self.rest_return_waypoints):
            if elapsed < segment_started_at + segment_duration:
                segment = index
                break
            segment_started_at += segment_duration
        end_pose, segment_duration, label = self.rest_return_waypoints[segment]
        start_pose = (
            self.rest_return_start
            if segment == 0
            else self.rest_return_waypoints[segment - 1][0]
        )
        amount = (elapsed - segment_started_at) / max(0.001, segment_duration)
        amount = amount * amount * (3.0 - 2.0 * amount)
        self.target_normalized = [
            before + (after - before) * amount
            for before, after in zip(start_pose, end_pose, strict=True)
        ]
        self.applied_normalized = limit_step(
            self.applied_normalized, self.target_normalized, self.profile.max_step
        )
        self._write_positions(
            self.profile.follower_calibration.normalized_to_raw(self.applied_normalized)
        )
        self.status_message = f"Return to rest: {label} | {self.rest_return_progress * 100.0:.0f}%"

    def _maybe_start_idle_return(self) -> bool:
        if (
            not self.idle_return_enabled
            or self.idle_return_attempted
            or self.rest_return_active
            or self.calibration_sweep_active
            or self.state not in ("armed", "hold")
            or not self.torque_enabled
            or self.following_error_stop
            or self.arm_contact_latched
            or self.gripper_contact_latched
            or self.now() - self.last_operator_activity_at < self.idle_return_timeout_seconds
        ):
            return False
        self.idle_return_attempted = True
        self.start_rest_return("idle")
        return self.rest_return_active

    def _update_calibration_sweep(self) -> None:
        if not self.calibration_sweep_active or self.calibration_sweep_start is None:
            return
        if not self.calibration_sweep_waypoints or self.calibration_sweep_duration <= 0.0:
            self.stop_calibration_sweep("Automatic calibration stopped: no safe trajectory is available.")
            return
        duration = self.calibration_sweep_duration
        elapsed = max(0.0, self.now() - self.calibration_sweep_started_at)
        self.calibration_sweep_progress = min(1.0, elapsed / duration)
        if elapsed >= duration:
            self.calibration_pose_settled = False
            self.target_normalized = list(self.calibration_sweep_waypoints[-1][0])
            self.applied_normalized = limit_step(
                self.applied_normalized,
                self.target_normalized,
                self.profile.max_step,
            )
            self._write_positions(self.profile.follower_calibration.normalized_to_raw(self.applied_normalized))
            if max(abs(a - b) for a, b in zip(self.applied_normalized, self.target_normalized, strict=True)) <= 0.25:
                self.stop_calibration_sweep("Automatic calibration sweep complete; follower is holding its raised safe pose.")
            return
        segment_started_at = 0.0
        segment = 0
        for index, (_, segment_duration, _) in enumerate(self.calibration_sweep_waypoints):
            if elapsed < segment_started_at + segment_duration:
                segment = index
                break
            segment_started_at += segment_duration
        end_pose, segment_duration, label = self.calibration_sweep_waypoints[segment]
        self.calibration_joint_index = -1
        if "servo " in label:
            try:
                self.calibration_joint_index = int(label.split("servo ", 1)[1].split(" ", 1)[0])
            except (TypeError, ValueError):
                self.calibration_joint_index = -1
        start_pose = self.calibration_sweep_start if segment == 0 else self.calibration_sweep_waypoints[segment - 1][0]
        amount = (elapsed - segment_started_at) / max(0.001, segment_duration)
        amount = amount * amount * (3.0 - 2.0 * amount)
        self.target_normalized = [
            before + (after - before) * amount
            for before, after in zip(start_pose, end_pose, strict=True)
        ]
        self.applied_normalized = limit_step(
            self.applied_normalized,
            self.target_normalized,
            self.profile.max_step,
        )
        at_target = max(
            abs(a - b)
            for a, b in zip(self.applied_normalized, self.target_normalized, strict=True)
        ) <= 0.25
        self.calibration_pose_settled = label.startswith("sampling") and at_target
        # During each sampling dwell, stop issuing writes so a direct encoder
        # read can be taken without colliding with synchronized servo traffic.
        if not self.calibration_pose_settled:
            self._write_positions(self.profile.follower_calibration.normalized_to_raw(self.applied_normalized))
        self.status_message = (
            f"Automatic calibration: {label} | {self.calibration_sweep_progress * 100.0:.0f}%"
        )

    def update(self) -> None:
        if self._maybe_start_idle_return():
            self.update_count += 1
            return
        if self.state == "returning_rest":
            self._update_rest_return()
            self.update_count += 1
            return
        if self.state == "calibrating":
            self._update_calibration_sweep()
            self.update_count += 1
            return
        if self.state == "armed":
            if self.now() - self.last_command_at > self.profile.watchdog_ms / 1000.0:
                self.hold("command watchdog expired")
                return
            if self.target_normalized is None or self.applied_normalized is None:
                self.hold("missing arm target")
                return
            if self.control_source == "keyboard":
                now = self.now()
                dt = max(0.0, min(0.075, now - self.last_integrate_at))
                self.last_integrate_at = now
                precision_scale = 0.32 if self.keyboard_precision else 1.0
                if self.keyboard_mode == "joint":
                    self.target_normalized = joint_rate_step(
                        self.target_normalized,
                        [value * precision_scale for value in self.keyboard_joint_velocity],
                        dt,
                    )
                elif self.keyboard_mode == "tool":
                    self.target_normalized = tool_rate_step(
                        self.target_normalized,
                        [value * precision_scale for value in self.keyboard_tool_velocity],
                        dt,
                    )
                else:
                    smoothing = 1.0 - math.exp(-dt / 0.11) if dt > 0.0 else 0.0
                    self.keyboard_linear_smoothed = [
                        current + (target - current) * smoothing
                        for current, target in zip(self.keyboard_linear_smoothed, self.keyboard_linear, strict=True)
                    ]
                    self.keyboard_wrist_smoothed = [
                        current + (target - current) * smoothing
                        for current, target in zip(self.keyboard_wrist_smoothed, self.keyboard_wrist, strict=True)
                    ]
                    if self.keyboard_frame == "plane":
                        right, forward, up = self.keyboard_linear_smoothed
                        linear = [
                            self.keyboard_plane_right[index] * right * 0.08 * precision_scale
                            + self.keyboard_plane_forward[index] * forward * 0.10 * precision_scale
                            + (up * 0.08 * precision_scale if index == 2 else 0.0)
                            for index in range(3)
                        ]
                    else:
                        linear = [value * 0.09 * precision_scale for value in self.keyboard_linear_smoothed]
                    angular = [value * 0.9 * precision_scale for value in self.keyboard_angular]
                    if self.keyboard_frame == "wrist":
                        linear, angular = wrist_frame_twist(self.target_normalized, linear, angular)
                    elif self.keyboard_angular_frame == "wrist":
                        _, angular = wrist_frame_twist(self.target_normalized, [0.0, 0.0, 0.0], angular)
                    if self.keyboard_position_target is None or self.keyboard_orientation_target is None:
                        current_pose = arm_pose(self.target_normalized)
                        self.keyboard_position_target = list(current_pose.position)
                        self.keyboard_orientation_target = current_pose.rotation
                    self.keyboard_position_target = [
                        position + velocity * dt
                        for position, velocity in zip(self.keyboard_position_target, linear, strict=True)
                    ]
                    previous_target = list(self.target_normalized)
                    candidate_target = solve_pose_target(
                        previous_target,
                        self.keyboard_position_target,
                        self.keyboard_orientation_target,
                    )
                    # solve_pose_target already clamps all five revolute joints
                    # to JOINT_LIMITS_RAD. The former two-degree margin guard
                    # hard-held the entire arm near wrist-roll limits even
                    # though the bounded solver could safely project the
                    # remaining Cartesian motion along that hard limit.
                    previous_clearance = rendered_minimum_height(previous_target)
                    clearance = rendered_minimum_height(candidate_target)
                    if previous_clearance >= RUNTIME_IK_MIN_CLEARANCE_M and clearance < RUNTIME_IK_MIN_CLEARANCE_M:
                        self._stop_on_ik_guard(
                            "Modeled arm clearance stopped motion at %.1f mm; "
                            "target rebased to measured pose."
                            % (clearance * 1000.0)
                        )
                        return
                    self.target_normalized = candidate_target
                    achieved_position = arm_pose(candidate_target).position
                    target_error = [
                        target - achieved
                        for target, achieved in zip(
                            self.keyboard_position_target,
                            achieved_position,
                            strict=True,
                        )
                    ]
                    target_error_length = math.sqrt(sum(value * value for value in target_error))
                    if target_error_length > 0.025:
                        self.keyboard_position_target = [
                            achieved + error * 0.025 / target_error_length
                            for achieved, error in zip(
                                achieved_position,
                                target_error,
                                strict=True,
                            )
                        ]
                    if any(abs(value) > 0.001 for value in self.keyboard_wrist_smoothed):
                        self.target_normalized = joint_rate_step(
                            self.target_normalized,
                            [0.0, 0.0, 0.0, self.keyboard_wrist_smoothed[0], self.keyboard_wrist_smoothed[1], 0.0],
                            dt,
                            rotational_speed_degrees=36.0,
                        )
                        wrist_pose = arm_pose(self.target_normalized)
                        self.keyboard_position_target = list(wrist_pose.position)
                        self.keyboard_orientation_target = wrist_pose.rotation
                    self.target_normalized[5] = max(
                        0.0,
                        min(100.0, self.target_normalized[5] + self.keyboard_gripper * 45.0 * precision_scale * dt),
                    )
            self.applied_normalized = limit_step(
                self.applied_normalized,
                self.target_normalized,
                self.profile.max_step,
            )
            raw = self.profile.follower_calibration.normalized_to_raw(self.applied_normalized)
            self._write_positions(raw)
        self.update_count += 1

    def sample(self) -> None:
        if not self.bus.connected:
            return
        self.follower_raw = self._read_positions()
        self.follower_normalized = self.profile.follower_calibration.raw_to_normalized(self.follower_raw)
        self.feedback_read_failures = 0
        self.feedback_read_fault = ""
        self._update_following_error_guard()

    def sample_feedback_safely(self) -> bool:
        try:
            self.sample()
            return True
        except Exception as exc:
            self.feedback_read_failures += 1
            self.feedback_read_fault = str(exc)[:240]
            return False

    def _joint_following_error(self, index: int, measured: float, commanded: float) -> float:
        error = measured - commanded
        if self.profile.follower_calibration.calib_mode[index] != "LINEAR":
            while error > 180.0:
                error -= 360.0
            while error < -180.0:
                error += 360.0
        return error

    def _update_following_error_guard(self) -> None:
        if self.applied_normalized is None:
            self.following_error_normalized = [0.0] * 6
            self.following_error_started_at = 0.0
            return
        self.following_error_normalized = [
            self._joint_following_error(index, measured, commanded)
            for index, (measured, commanded) in enumerate(
                zip(self.follower_normalized, self.applied_normalized, strict=True)
            )
        ]
        if (
            self.state not in ("armed", "returning_rest")
            or (
                self.state != "returning_rest"
                and not self.feedback_settings["following_error_safety_enabled"]
            )
            or self.now() - self.armed_at < 0.35
        ):
            self.following_error_started_at = 0.0
            return
        violating = [
            index
            for index, (error, threshold) in enumerate(
                zip(self.following_error_normalized, self.following_error_thresholds, strict=True)
            )
            if abs(error) > threshold
        ]
        if not violating:
            self.following_error_started_at = 0.0
            return
        if self.following_error_started_at == 0.0:
            self.following_error_started_at = self.now()
            return
        if self.now() - self.following_error_started_at < 0.35:
            return
        if self.state == "returning_rest":
            joint = max(violating, key=lambda index: abs(self.following_error_normalized[index]))
            self._stop_on_following_error(joint)
            return
        arm_violating = [index for index in violating if index < 5]
        if arm_violating:
            joint = max(arm_violating, key=lambda index: abs(self.following_error_normalized[index]))
            self._stop_on_following_error(joint)
            return
        # Normalized gripper opening increases in the opening direction.  A
        # positive measured-minus-commanded error therefore means the jaws
        # were prevented from closing by an object.  Rebase only the gripper;
        # an obstruction while opening remains a conservative whole-arm stop.
        if self.following_error_normalized[5] > self.following_error_thresholds[5]:
            self._stop_gripper_on_contact()
        else:
            self._stop_on_following_error(5)

    def _stop_gripper_on_contact(self) -> None:
        measured_gripper = self.follower_normalized[5]
        self.target_normalized[5] = measured_gripper
        self.applied_normalized[5] = measured_gripper
        safe_raw = self.profile.follower_calibration.normalized_to_raw(self.applied_normalized)
        # The gripper portion of safe_raw is its actual encoder position and
        # the other five values remain the continuously controlled arm goal.
        self.bus.reseed_position_guard(safe_raw)
        self._write_positions(safe_raw)
        self.keyboard_gripper = 0.0
        self.gripper_contact_latched = True
        self.gripper_contact_trip_count += 1
        self.following_error_started_at = 0.0
        self.following_error_stop = False
        self.status_message = (
            "Gripper contact detected; closing stopped at the measured jaw position "
            "while arm control remains enabled. Release close or command open to reset."
        )

    def _stop_on_following_error(self, joint: int) -> None:
        measured = list(self.follower_normalized)
        self.target_normalized = measured
        self.applied_normalized = measured
        self.bus.reseed_position_guard(self.follower_raw)
        self._write_positions(self.follower_raw)
        if self.control_source == "keyboard" and joint < 5:
            self.keyboard_linear = [0.0, 0.0, 0.0]
            self.keyboard_angular = [0.0, 0.0, 0.0]
            self.keyboard_wrist = [0.0, 0.0]
            self.keyboard_linear_smoothed = [0.0, 0.0, 0.0]
            self.keyboard_wrist_smoothed = [0.0, 0.0]
            self.keyboard_joint_velocity = [0.0] * 6
            self.keyboard_tool_velocity = [0.0] * 6
            measured_pose = arm_pose(measured)
            self.keyboard_position_target = list(measured_pose.position)
            self.keyboard_orientation_target = measured_pose.rotation
            self.arm_contact_latched = True
            self.arm_contact_trip_count += 1
            self.following_error_stop = True
            self.following_error_trip_count += 1
            self.following_error_started_at = 0.0
            self.status_message = (
                "Contact / following error stopped motion at servo %d (%.1f). "
                "Target rebased while keyboard control remains enabled; release movement keys to reset."
                % (joint + 1, self.following_error_normalized[joint])
            )
            return
        self.hold(
            "Contact / following error stopped motion at servo %d (%.1f). "
            "Target rebased to measured pose."
            % (joint + 1, self.following_error_normalized[joint])
        )
        self.keyboard_position_target = None
        self.keyboard_orientation_target = None
        self.following_error_stop = True
        self.following_error_trip_count += 1
        self.following_error_started_at = 0.0

    def _stop_on_ik_guard(self, reason: str) -> None:
        self.sample_feedback_safely()
        measured = list(self.follower_normalized)
        self.target_normalized = measured
        self.applied_normalized = measured
        self.bus.reseed_position_guard(self.follower_raw)
        self._write_positions(self.follower_raw)
        self.hold(reason)
        self.keyboard_position_target = None
        self.keyboard_orientation_target = None
        self.following_error_stop = True
        self.ik_guard_trip_count += 1
        self.following_error_started_at = 0.0

    def _read_positions(self) -> list[int]:
        values = self.bus.read_positions()
        now = self.now()
        self.last_successful_read_at = now
        self.read_times.append(now)
        return values

    def _write_positions(self, positions: list[int]) -> None:
        self.bus.write_positions(positions)
        now = self.now()
        self.last_successful_write_at = now
        self.write_times.append(now)

    def _write_current_pose(self) -> None:
        # A held arm may not have reached its previous goal. Re-seed from the
        # freshly read encoder pose before commanding that same safe pose.
        self.bus.reseed_position_guard(self.follower_raw)
        self._write_positions(self.follower_raw)

    def _clear_motion_inputs(self) -> None:
        self.keyboard_linear = [0.0, 0.0, 0.0]
        self.keyboard_angular = [0.0, 0.0, 0.0]
        self.keyboard_gripper = 0.0
        self.keyboard_wrist = [0.0, 0.0]
        self.keyboard_linear_smoothed = [0.0, 0.0, 0.0]
        self.keyboard_wrist_smoothed = [0.0, 0.0]
        self.keyboard_joint_velocity = [0.0] * 6
        self.keyboard_tool_velocity = [0.0] * 6
        self.keyboard_precision = False
        self.keyboard_angular_frame = "base"
        self.keyboard_orientation_target = None
        self.keyboard_position_target = None
        self.target_normalized = None
        self.applied_normalized = None

    def _recent_rate(self, times: deque[float]) -> int:
        now = self.now()
        while times and now - times[0] > 1.0:
            times.popleft()
        return len(times)

    def status(self) -> dict:
        elapsed = max(0.001, self.now() - self.started_at)
        age_ms = None if self.last_command_at == 0 else max(0.0, (self.now() - self.last_command_at) * 1000.0)
        last_write_age_ms = None if self.last_successful_write_at == 0 else max(0.0, (self.now() - self.last_successful_write_at) * 1000.0)
        last_read_age_ms = None if self.last_successful_read_at == 0 else max(0.0, (self.now() - self.last_successful_read_at) * 1000.0)
        pose = arm_pose(self.follower_normalized)
        return {
            "type": "arm_status",
            "follower_connected": bool(self.bus.connected),
            "follower_serial": self.profile.follower_serial,
            "state": self.state,
            "armed": self.state == "armed",
            "torque_enabled": self.torque_enabled,
            "gripper_torque_enabled": self.gripper_torque_enabled,
            "fault": self.fault,
            "message": self.status_message,
            "hardware_fault": self.hardware_fault,
            "last_seq": self.last_seq,
            "command_age_ms": age_ms,
            "command_latency_ms": self.command_latency_ms,
            "browser_transport_age_ms": self.browser_transport_age_ms,
            "loop_hz": self.update_count / elapsed,
            "write_rate_hz": self._recent_rate(self.write_times),
            "read_rate_hz": self._recent_rate(self.read_times),
            "last_write_age_ms": last_write_age_ms,
            "last_read_age_ms": last_read_age_ms,
            "restart_count": self.restart_count,
            "restart_completed_unix_ms": self.restart_completed_unix_ms,
            "calibration_sweep_active": self.calibration_sweep_active,
            "calibration_sweep_progress": self.calibration_sweep_progress,
            "calibration_pose_settled": self.calibration_pose_settled,
            "calibration_sweep_mode": self.calibration_sweep_mode,
            "calibration_joint_index": self.calibration_joint_index,
            "calibration_rejection": self.calibration_rejection,
            "rest_pose_available": self.rest_pose is not None,
            "rest_pose_fault": self.rest_pose_fault,
            "rest_return_active": self.rest_return_active,
            "rest_return_progress": self.rest_return_progress,
            "rest_return_reason": self.rest_return_reason,
            "idle_return_enabled": self.idle_return_enabled,
            "idle_return_timeout_seconds": self.idle_return_timeout_seconds,
            "idle_seconds": max(0.0, self.now() - self.last_operator_activity_at),
            "leader_raw": self.leader_raw,
            "leader_normalized": self.leader_normalized,
            "target_normalized": self.target_normalized,
            "applied_normalized": self.applied_normalized,
            "follower_raw": self.follower_raw,
            "follower_normalized": self.follower_normalized,
            "measured_feedback_fresh": (
                last_read_age_ms is not None and last_read_age_ms <= 350.0
            ),
            "feedback_settings": dict(self.feedback_settings),
            "feedback_read_failures": self.feedback_read_failures,
            "feedback_read_fault": self.feedback_read_fault,
            "following_error_normalized": list(self.following_error_normalized),
            "following_error_thresholds": list(self.following_error_thresholds),
            "following_error_stop": self.following_error_stop,
            "following_error_trip_count": self.following_error_trip_count,
            "gripper_contact_latched": self.gripper_contact_latched,
            "gripper_contact_trip_count": self.gripper_contact_trip_count,
            "arm_contact_latched": self.arm_contact_latched,
            "arm_contact_trip_count": self.arm_contact_trip_count,
            "ik_guard_trip_count": self.ik_guard_trip_count,
            "alignment_mode": "relative_start",
            "control_session": self.control_session,
            "control_source": self.control_source,
            "latest_input_source": self.latest_input_source,
            "keyboard_mode": self.keyboard_mode,
            "keyboard_frame": self.keyboard_frame,
            "keyboard_angular_frame": self.keyboard_angular_frame,
            "keyboard_wrist": list(self.keyboard_wrist),
            "keyboard_plane_forward": list(self.keyboard_plane_forward),
            "keyboard_plane_right": list(self.keyboard_plane_right),
            "end_effector_position": list(pose.position),
            "end_effector_rotation": [list(row) for row in pose.rotation],
            "sent_unix_ms": int(time.time() * 1000),
        }

    def fail(self, message: str) -> None:
        try:
            if self.bus.connected:
                self.bus.disable_torque()
        except Exception:
            pass
        self.torque_enabled = False
        self.gripper_torque_enabled = False
        self.state = "fault"
        self.hardware_fault = message
        self.fault = f"hardware fault: {message}"
        self.status_message = "Follower hardware stopped; use Restart Arm Connection."
        self.calibration_sweep_active = False
        self.rest_return_active = False
        self.rest_return_waypoints = []

    def _reject(self, reason: str) -> None:
        self.state = "ready" if self.bus.connected else "disconnected"
        self.fault = f"enable rejected: {reason}"
        self.status_message = self.fault


def run_service(
    profile: ArmPairProfile,
    bus: FollowerBus,
    command_port: int,
    status_port: int,
    godot_status_port: int,
    editor_status_port: int,
    hz: float,
    feedback_hz: float,
    hold_on_connect: bool = False,
) -> int:
    controller = FollowerController(profile, bus)
    command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    command_socket.bind(("127.0.0.1", command_port))
    command_socket.setblocking(False)
    status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        controller.connect()
        print(
            "SO-101 startup pose raw=%s normalized=%s"
            % (controller.follower_raw, [round(value, 3) for value in controller.follower_normalized]),
            flush=True,
        )
        if hold_on_connect:
            controller.target_normalized = list(controller.follower_normalized)
            controller.applied_normalized = list(controller.follower_normalized)
            controller._write_current_pose()
            controller.bus.enable_torque()
            controller.torque_enabled = True
            controller.gripper_torque_enabled = True
            controller.state = "hold"
            controller.status_message = "Follower is holding its startup pose."
        print(f"SO-101 follower connected read-only: {profile.follower_port}", flush=True)
        period = 1.0 / max(1.0, hz)
        feedback_period = 1.0 / max(1.0, feedback_hz)
        next_tick = time.monotonic()
        next_sample = next_tick
        next_status = next_tick
        while running:
            while True:
                try:
                    packet, _ = command_socket.recvfrom(8192)
                except BlockingIOError:
                    break
                try:
                    message = json.loads(packet.decode("utf-8"))
                    controller.receive(message)
                except Exception as exc:
                    controller.hold(f"bad local command: {exc}")
            try:
                controller.update()
                now = time.monotonic()
                # Interleave bounded-rate encoder reads with writes so the live model
                # depicts the arm that actually moved, not merely its commanded goal.
                # Read failures are contained and exposed as stale telemetry instead
                # of crashing the safety-isolated follower process.
                can_sample_calibration_pose = (
                    controller.state == "calibrating" and controller.calibration_pose_settled
                )
                can_sample_armed = (
                    controller.state == "armed"
                    and controller.feedback_settings["measured_feedback_enabled"]
                )
                if now >= next_sample and (
                    controller.state not in ("armed", "calibrating", "restarting", "fault")
                    or can_sample_calibration_pose
                    or can_sample_armed
                ):
                    controller.sample_feedback_safely()
                    # Preserve the requested average cadence across the 30 Hz
                    # control-loop ticks. Rebasing to ``now + period`` here
                    # quantizes a 20 Hz request down to every other tick
                    # (15 Hz); carrying the deadline produces alternating
                    # one/two-tick intervals instead.
                    if now - next_sample > feedback_period:
                        next_sample = now + feedback_period
                    else:
                        next_sample += feedback_period
                if (
                    controller.state in ("armed", "returning_rest")
                    and (
                        controller.state == "returning_rest"
                        or controller.feedback_settings["following_error_safety_enabled"]
                    )
                    and controller.last_successful_read_at > 0.0
                    and now - controller.last_successful_read_at > 0.6
                ):
                    controller.applied_normalized = list(controller.follower_normalized)
                    controller.target_normalized = list(controller.follower_normalized)
                    controller.hold(
                        "Motion stopped because measured servo telemetry became stale."
                    )
                if now >= next_status:
                    payload = json.dumps(controller.status(), separators=(",", ":")).encode("utf-8")
                    status_socket.sendto(
                        payload,
                        ("127.0.0.1", status_port),
                    )
                    if godot_status_port > 0 and godot_status_port != status_port:
                        status_socket.sendto(payload, ("127.0.0.1", godot_status_port))
                    if editor_status_port > 0 and editor_status_port not in (status_port, godot_status_port):
                        status_socket.sendto(payload, ("127.0.0.1", editor_status_port))
                    # The overlay and browser status are consumers of the already
                    # computed controller state; publishing them at the control-loop
                    # cadence does not add another servo read or write.
                    next_status = now + period
            except Exception as exc:
                controller.fail(str(exc))
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()
    finally:
        try:
            bus.close(disable_torque=True)
        except Exception as exc:
            print(f"Follower shutdown warning: {exc}", file=sys.stderr, flush=True)
        command_socket.close()
        status_socket.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Safety-isolated SO-101 follower process.")
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE))
    parser.add_argument("--command-port", type=int)
    parser.add_argument("--status-port", type=int)
    parser.add_argument("--godot-status-port", type=int)
    parser.add_argument("--editor-status-port", type=int, default=4252)
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument(
        "--feedback-hz",
        type=float,
        default=20.0,
        help="Measured follower encoder sampling rate while active or holding.",
    )
    parser.add_argument(
        "--hold-on-connect",
        action="store_true",
        help="Hold the current physical pose immediately after connecting.",
    )
    parser.add_argument("--dry-run", action="store_true", help="Use a fake follower and never touch USB hardware.")
    parser.add_argument(
        "--ignore-motor-6",
        action="store_true",
        help="Temporarily control IDs 1-5 and ignore the missing gripper motor (ID 6).",
    )
    args = parser.parse_args()
    profile = ArmPairProfile.load(Path(args.profile))
    command_port = args.command_port or profile.command_port
    status_port = args.status_port or profile.status_port
    godot_status_port = args.godot_status_port or profile.godot_status_port
    bus: FollowerBus
    if args.dry_run:
        bus = FakeFollowerBus(profile.follower_calibration.normalized_to_raw([20, 20, 20, 20, 20, 20]))
        print("SO-101 follower service is in dry-run mode", flush=True)
    else:
        if not Path(profile.follower_port).exists():
            print(f"Follower not found at {profile.follower_port}", file=sys.stderr)
            return 2
        if args.ignore_motor_6:
            virtual_gripper_position = profile.follower_calibration.start_pos[5]
            bus = LeRobotFollowerBus(
                profile.follower_port,
                profile.motor_names[:5],
                ignored_tail_positions=(virtual_gripper_position,),
            )
            print(
                "WARNING: motor 6 is ignored; gripper commands will not be sent",
                file=sys.stderr,
                flush=True,
            )
        else:
            bus = LeRobotFollowerBus(profile.follower_port, profile.motor_names)
    return run_service(
        profile,
        bus,
        command_port,
        status_port,
        godot_status_port,
        args.editor_status_port,
        args.hz,
        args.feedback_hz,
        args.hold_on_connect,
    )


if __name__ == "__main__":
    raise SystemExit(main())
