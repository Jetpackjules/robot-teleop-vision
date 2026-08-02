#!/usr/bin/env python3
"""Fit SO-101 servo zeroes one joint at a time with the robot base locked."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))

ASSET_ROOT = Path(__file__).resolve().parents[1] / "assets" / "robots" / "so101"

from so101_kinematics import (
    GRIPPER_CLOSED_NORMALIZED,
    GRIPPER_LIMITS_RAD,
    GRIPPER_ORIGIN,
    JOINT_ORIGINS,
    _rz,
    _transform,
)


ROS_TO_GODOT = np.asarray(((0.0, -1.0, 0.0), (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)))
JOINT_LINKS = {
    1: "upper_arm_link",
    2: "lower_arm_link",
    3: "wrist_link",
    4: "gripper_link",
}
JOINT_NAMES = {
    1: "shoulder_lift",
    2: "elbow_flex",
    3: "wrist_flex",
    4: "wrist_roll",
}
MAXIMUM_CONVERGENCE_PASSES = 3
MAXIMUM_FIXED_POINT_DELTA_DEGREES = 2.0
REMOTE_ALTERNATIVE_SEPARATION_DEGREES = 15.0
MINIMUM_REMOTE_MOTION_PEAK_RATIO = 1.02
MINIMUM_REFERENCE_WRIST_AXIS_PEAK_RATIO = 1.05
MINIMUM_WRIST_DYNAMIC_CONSTRUCTION_POINTS_PER_POSE = 80
MINIMUM_REFERENCE_WRIST_DYNAMIC_POINTS_PER_POSE = 100
WRIST_DYNAMIC_VOXEL_SIZE_M = 0.005
MINIMUM_REMOTE_MAPPED_MESH_PEAK_RATIO = 1.08
MINIMUM_REFERENCE_MAPPED_MESH_PEAK_RATIO = 1.15
MAXIMUM_STRONG_MESH_DEPTH_LOSS_M = 0.015
MAPPED_MESH_CURRENT_ZERO_SEARCH_RADIUS_DEGREES = 30.0
NO_COMPETING_MESH_PEAK_RATIO = 1.0e9
MAXIMUM_WRIST_ROLL_MESH_LOSS_M = 0.018
MINIMUM_WRIST_ROLL_MESH_SUPPORT_FRACTION = 0.55
MAXIMUM_PARTIAL_WRIST_ROLL_MESH_LOSS_M = 0.025
PARTIAL_WRIST_ROLL_SUPPORT_DISTANCE_M = 0.025
MINIMUM_PARTIAL_WRIST_ROLL_MESH_SUPPORT_FRACTION = 0.40
MINIMUM_WRIST_ROLL_MESH_PEAK_RATIO = 1.06
WRIST_ROLL_REMOTE_ALTERNATIVE_DEGREES = 45.0
MAXIMUM_WRIST_ROLL_VIEW_DISAGREEMENT_DEGREES = 12.0
MAXIMUM_WRIST_ROLL_LOCAL_BASIN_WIDTH_DEGREES = 25.0
WRIST_ROLL_LOCAL_BASIN_LOSS_RATIO = 1.01
MAXIMUM_WRIST_ROLL_FINE_VIEWS_PER_CAMERA = 4
# Overridden by the capture in ``main``. The model-name default keeps imported
# offline helpers and older synthetic captures compatible without embedding a
# physical device serial in this reusable repository.
REFERENCE_CAMERA_SERIAL = "D455"
MINIMUM_WRIST_ROLL_VALIDATING_CAMERAS = 2
MINIMUM_VISIBLE_SETTLED_AXIS_POSES_PER_CAMERA = 3
MINIMUM_VISIBLE_SETTLED_AXIS_POSES_IN_PRIMARY_VIEW = 4
MINIMUM_AXIS_SUPPORTS_PER_VISIBLE_POSE = 8
WRIST_FLEX_CURRENT_ZERO_SEARCH_RADIUS_DEGREES = 30.0
DISTAL_FIXED_JAW_MAXIMUM_LOCAL_Z_M = -0.055
# Both cameras must see coherent rigid motion. Four millimetres is below this
# D435 rig's measured temporal noise floor and turns static table pixels into a
# false wrist axis; 8 mm retains hundreds of real distal points in every
# settled pose while removing that background basin.
WRIST_DYNAMIC_MINIMUM_DISPLACEMENT_M = 0.008
MINIMUM_WRIST_ROLL_MULTIANGLE_VIEWPOINTS = 2
MINIMUM_WRIST_ROLL_MULTIANGLE_POSES_PER_VIEWPOINT = 5
# Live D455 validation with the custom wrist camera places the second exposed
# moving-jaw sample at 21.1--22.1 mm while hidden/clutter samples remain above
# roughly 24 mm.  The former 21 mm edge cut rejected a geometrically perfect
# three-view consensus solely on 0.1 mm of sensor-edge noise.
MAXIMUM_WRIST_ROLL_MOVING_JAW_LOSS_M = 0.0225
MAXIMUM_AUTOMATIC_WRIST_ROLL_CORRECTION_DEGREES = 120.0
# The stock SO-101 GLB's gripper surface zero is a quarter turn behind the
# articulated overlay convention. Depth fitting is performed in the mesh
# convention, then converted once to the joint convention used by Godot.
WRIST_ROLL_MODEL_TO_JOINT_ZERO_DEGREES = 90.0
# The D455's edge noise and the printed bracket surface are not perfectly
# coincident. This narrow shell removes bracket returns without reaching the
# distal fixed jaw, whose fitted region starts more than 55 mm away.
WRIST_CAMERA_MOUNT_EXCLUSION_DISTANCE_M = 0.010
WRIST_CAMERA_MOUNT_ENVELOPE_STEP_DEGREES = 15.0


def registration_basis(payload: dict) -> np.ndarray:
    return np.column_stack((payload["basis_x"], payload["basis_y"], payload["basis_z"]))


def reference_camera_indices(frames: list[dict]) -> list[int]:
    """Return the capture-selected primary calibration camera."""
    cameras = frames[0].get("full_camera_points", []) if frames else []
    indices = [
        index
        for index, camera in enumerate(cameras)
        if REFERENCE_CAMERA_SERIAL in str(camera.get("name", ""))
    ]
    if not indices:
        raise ValueError(
            f"staged calibration requires reference camera {REFERENCE_CAMERA_SERIAL!r}"
        )
    return indices[:1]


def compose_registration_with_parent(
    registration: dict,
    parent: dict | None,
) -> tuple[np.ndarray, np.ndarray]:
    basis = registration_basis(registration)
    origin = np.asarray(registration["origin"], dtype=float)
    if parent is None:
        return basis, origin
    parent_basis = registration_basis(parent)
    parent_origin = np.asarray(parent["origin"], dtype=float)
    return parent_basis @ basis, parent_origin + parent_basis @ origin


def unbounded_chain(
    pose: list[float],
    directions: list[float],
    offsets: list[float],
) -> tuple[dict[str, np.ndarray], list[np.ndarray], list[np.ndarray]]:
    chain = np.eye(4)
    transforms: dict[str, np.ndarray] = {}
    pivots: list[np.ndarray] = []
    axes: list[np.ndarray] = []
    names = ("shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link")
    for index, (xyz, rpy) in enumerate(JOINT_ORIGINS):
        chain = chain @ _transform(xyz, rpy)
        pivots.append(chain[:3, 3].copy())
        axes.append(chain[:3, :3] @ np.asarray((0.0, 0.0, 1.0)))
        rotation = np.eye(4)
        rotation[:3, :3] = _rz(math.radians(pose[index] * directions[index] + offsets[index]))
        chain = chain @ rotation
        transforms[names[index]] = chain.copy()
    return transforms, pivots, axes


def global_point(local_ros: np.ndarray, basis: np.ndarray, origin: np.ndarray) -> np.ndarray:
    return origin + basis @ (ROS_TO_GODOT @ local_ros)


def global_axis(local_ros: np.ndarray, basis: np.ndarray) -> np.ndarray:
    result = basis @ (ROS_TO_GODOT @ local_ros)
    return result / np.linalg.norm(result)


def distinct_frames(frames: list[dict], joint_index: int) -> list[dict]:
    staged = [frame for frame in frames if int(frame.get("calibration_joint_index", -1)) == joint_index]
    if len(staged) < 3:
        raise ValueError(f"{JOINT_NAMES[joint_index]} has only {len(staged)} settled frames")
    staged.sort(key=lambda frame: float(frame["pose"][joint_index]))
    selected: list[dict] = []
    for frame in staged:
        value = float(frame["pose"][joint_index])
        if not selected or value - float(selected[-1]["pose"][joint_index]) >= 1.0:
            selected.append(frame)
    if len(selected) > 5:
        targets = np.linspace(
            float(selected[0]["pose"][joint_index]),
            float(selected[-1]["pose"][joint_index]),
            5,
        )
        normalized: list[dict] = []
        used_ids: set[int] = set()
        for target in targets:
            chosen = min(
                (frame for frame in selected if id(frame) not in used_ids),
                key=lambda frame: abs(
                    float(frame["pose"][joint_index]) - float(target)
                ),
            )
            normalized.append(chosen)
            used_ids.add(id(chosen))
        selected = sorted(
            normalized,
            key=lambda frame: float(frame["pose"][joint_index]),
        )
    if len(selected) < 3:
        raise ValueError(f"{JOINT_NAMES[joint_index]} has fewer than three distinct poses")
    if float(selected[-1]["pose"][joint_index]) - float(selected[0]["pose"][joint_index]) < 24.0:
        raise ValueError(f"{JOINT_NAMES[joint_index]} sweep is too small")
    return selected


def three_distinct_frames(frames: list[dict], joint_index: int) -> list[dict]:
    staged = distinct_frames(frames, joint_index)
    midpoint = (
        float(staged[0]["pose"][joint_index])
        + float(staged[-1]["pose"][joint_index])
    ) * 0.5
    return [
        staged[0],
        min(
            staged,
            key=lambda frame: abs(float(frame["pose"][joint_index]) - midpoint),
        ),
        staged[-1],
    ]


def observation_groups(frames: list[dict], joint_index: int) -> list[list[dict]]:
    """Split a servo sweep by its fixed shoulder-pan observation viewpoint."""
    staged = [
        frame
        for frame in frames
        if int(frame.get("calibration_joint_index", -1)) == joint_index
    ]
    staged.sort(key=lambda frame: float(frame["pose"][0]))
    groups: list[list[dict]] = []
    for frame in staged:
        pan = float(frame["pose"][0])
        if (
            not groups
            or abs(
                pan
                - float(
                    np.median([item["pose"][0] for item in groups[-1]])
                )
            )
            > 3.0
        ):
            groups.append([frame])
        else:
            groups[-1].append(frame)
    result = []
    for group in groups:
        try:
            result.append(distinct_frames(group, joint_index))
        except ValueError:
            continue
    if not result:
        raise ValueError(
            f"{JOINT_NAMES[joint_index]} has no complete observation viewpoint"
        )
    return result


def dynamic_clouds(
    frames: list[dict],
    camera_index: int,
    pivot: np.ndarray,
    base_up: np.ndarray,
    base_origin: np.ndarray,
) -> list[np.ndarray]:
    full = [
        np.asarray(frame["full_camera_points"][camera_index]["points"], dtype=float)
        for frame in frames
    ]
    result = []
    for frame_index, current in enumerate(full):
        other_distances = [
            cKDTree(other).query(current, workers=-1)[0]
            for index, other in enumerate(full)
            if index != frame_index
        ]
        dynamic_score = np.median(np.stack(other_distances, axis=1), axis=1)
        distance_to_axis_region = np.linalg.norm(current - pivot, axis=1)
        height = (current - base_origin) @ base_up
        selected = current[
            (dynamic_score > 0.007)
            & (distance_to_axis_region < 0.46)
            & (height > 0.012)
            & (height < 0.42)
        ]
        if len(selected) < 120:
            raise ValueError(
                f"camera {camera_index} found only {len(selected)} dynamic points"
            )
        result.append(selected)
    return result


def collapse_clouds(
    frames: list[dict],
    clouds: list[np.ndarray],
    joint_index: int,
    pivot: np.ndarray,
    axis: np.ndarray,
    direction: float,
) -> list[np.ndarray]:
    reference = float(frames[1]["pose"][joint_index])
    result = []
    for frame, cloud in zip(frames, clouds, strict=True):
        current = float(frame["pose"][joint_index])
        angle = math.radians(direction * (reference - current))
        rotation = Rotation.from_rotvec(axis * angle).as_matrix()
        result.append((cloud - pivot) @ rotation.T + pivot)
    return result


def collapse_loss(clouds: list[np.ndarray]) -> float:
    pair_losses = []
    for left in range(len(clouds)):
        for right in range(left + 1, len(clouds)):
            distances = np.concatenate(
                (
                    cKDTree(clouds[right]).query(clouds[left], workers=-1)[0],
                    cKDTree(clouds[left]).query(clouds[right], workers=-1)[0],
                )
            )
            distances.sort()
            keep = max(50, int(0.45 * len(distances)))
            pair_losses.append(
                math.sqrt(float(np.mean(np.minimum(distances[:keep], 0.10) ** 2)))
            )
    return float(np.mean(pair_losses))


def posed_mesh(
    raw_points: np.ndarray,
    frame: dict,
    joint_index: int,
    delta: float,
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> np.ndarray:
    candidate_offsets = list(offsets)
    candidate_offsets[joint_index] += delta
    transforms, _, _ = unbounded_chain(frame["pose"], directions, candidate_offsets)
    transform = transforms[JOINT_LINKS[joint_index]]
    local_ros = raw_points @ transform[:3, :3].T + transform[:3, 3]
    local_godot = local_ros @ ROS_TO_GODOT.T
    return local_godot @ basis.T + origin


def posed_moving_jaw(
    raw_points: np.ndarray,
    frame: dict,
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> np.ndarray:
    """Pose the moving jaw using the measured gripper openness.

    The fixed-jaw tip alone has a near quarter-turn ambiguity around the wrist
    axis. The moving jaw is not part of the custom camera bracket and provides
    the missing left/right relationship needed to resolve that orientation.
    """
    transforms, _, _ = unbounded_chain(
        frame["pose"],
        directions,
        offsets,
    )
    transform = transforms["gripper_link"] @ _transform(*GRIPPER_ORIGIN)
    normalized_open = float(frame["pose"][5])
    open_fraction = min(
        1.0,
        max(
            0.0,
            (normalized_open - GRIPPER_CLOSED_NORMALIZED)
            / (100.0 - GRIPPER_CLOSED_NORMALIZED),
        ),
    )
    jaw_angle = (
        GRIPPER_LIMITS_RAD[0]
        + (GRIPPER_LIMITS_RAD[1] - GRIPPER_LIMITS_RAD[0])
        * open_fraction
    )
    jaw_rotation = np.eye(4)
    jaw_rotation[:3, :3] = _rz(jaw_angle)
    transform = transform @ jaw_rotation
    local_ros = raw_points @ transform[:3, :3].T + transform[:3, 3]
    local_godot = local_ros @ ROS_TO_GODOT.T
    return local_godot @ basis.T + origin


def mesh_loss(predicted: np.ndarray, observed: np.ndarray) -> float:
    return mesh_loss_to_tree(predicted, cKDTree(observed))


def mesh_loss_to_tree(
    predicted: np.ndarray,
    tree: cKDTree,
    workers: int = -1,
) -> float:
    distances = tree.query(predicted, workers=workers)[0]
    distances.sort()
    keep = max(300, int(0.65 * len(distances)))
    return math.sqrt(float(np.mean(np.minimum(distances[:keep], 0.10) ** 2)))


def predicted_next_axis(
    frame: dict,
    current_joint_index: int,
    delta: float,
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    candidate_offsets = list(offsets)
    candidate_offsets[current_joint_index] += delta
    _, pivots, axes = unbounded_chain(frame["pose"], directions, candidate_offsets)
    next_joint_index = current_joint_index + 1
    return (
        global_point(pivots[next_joint_index], basis, origin),
        global_axis(axes[next_joint_index], basis),
    )


def next_axis_camera_curve(
    frames: list[dict],
    camera_index: int,
    current_joint_index: int,
    candidates: np.ndarray,
    next_direction: float,
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> list[dict]:
    """Score a parent zero by motion around the next servo's predicted axis.

    A point contributes only when it moved away from its raw location and the
    predicted next-servo rotation maps it close to depth visible at another
    settled pose. Hidden model surfaces and the unmapped attachment never enter
    this score.
    """
    next_joint_index = current_joint_index + 1
    full = [
        np.asarray(frame["full_camera_points"][camera_index]["points"], dtype=float)
        for frame in frames
    ]
    trees = [cKDTree(points) for points in full]
    static_distances = []
    for frame_index, points in enumerate(full):
        distances = [
            trees[other_index].query(points, workers=-1)[0]
            for other_index in range(len(full))
            if other_index != frame_index
        ]
        static_distances.append(np.median(np.stack(distances, axis=1), axis=1))

    base_up = basis[:, 1]
    median_pose = float(
        np.median([frame["pose"][next_joint_index] for frame in frames])
    )
    center_frame = min(
        frames,
        key=lambda frame: abs(
            float(frame["pose"][next_joint_index]) - median_pose
        ),
    )
    results = []
    for delta in candidates:
        pivot, axis = predicted_next_axis(
            center_frame,
            current_joint_index,
            float(delta),
            directions,
            offsets,
            basis,
            origin,
        )
        pose_support = []
        supported_distances: list[float] = []
        for frame_index, (frame, points, static) in enumerate(
            zip(frames, full, static_distances, strict=True)
        ):
            relative = points - pivot
            along_axis = relative @ axis
            radial = np.linalg.norm(
                relative - along_axis[:, None] * axis, axis=1
            )
            height = (points - origin) @ base_up
            region = (
                (radial < 0.15)
                & (height > 0.02)
                & (height < 0.42)
                & (np.linalg.norm(points - origin, axis=1) < 0.55)
            )
            moved_distances = []
            for other_index, other_frame in enumerate(frames):
                if other_index == frame_index:
                    continue
                angle = math.radians(
                    next_direction
                    * (
                        float(other_frame["pose"][next_joint_index])
                        - float(frame["pose"][next_joint_index])
                    )
                )
                rotation = Rotation.from_rotvec(axis * angle).as_matrix()
                predicted = relative @ rotation.T + pivot
                moved_distances.append(
                    trees[other_index].query(predicted, workers=-1)[0]
                )
            moved_candidates = np.stack(moved_distances, axis=1)
            # A chance nearest neighbor in one other depth frame is common in
            # a busy scene. With five settled poses, require support in at
            # least two alternate poses. Three-pose legacy captures retain the
            # weaker one-alternate-pose behavior.
            support_rank = 1 if moved_candidates.shape[1] >= 3 else 0
            moved = np.partition(
                moved_candidates, support_rank, axis=1
            )[:, support_rank]
            supported = (
                region
                & (static > 0.007)
                & (moved < 0.014)
                & (moved < static * 0.80)
            )
            pose_support.append(int(np.count_nonzero(supported)))
            supported_distances.extend(moved[supported].tolist())
        results.append(
            {
                "delta": float(delta),
                "support": int(sum(pose_support)),
                "minimum_pose_support": int(min(pose_support)),
                "pose_support": pose_support,
                "median_residual_m": (
                    float(np.median(supported_distances))
                    if supported_distances
                    else 0.10
                ),
                "pivot": pivot.tolist(),
                "axis": axis.tolist(),
            }
        )
    return results


def fixed_region_axis_camera_curve(
    frames: list[dict],
    camera_index: int,
    current_joint_index: int,
    candidates: np.ndarray,
    fixed_region_delta: float,
    next_direction: float,
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> list[dict]:
    """Score candidate axes against one fixed, cross-camera wrist region.

    Letting every wrist-axis hypothesis select points around itself permits a
    wrong hypothesis to walk onto disoccluded background. A primary-camera
    seed defines the physical wrist region once; both cameras then evaluate
    their own depth evidence inside that same world-space region.
    """
    next_joint_index = current_joint_index + 1
    full = [
        np.asarray(frame["full_camera_points"][camera_index]["points"], dtype=float)
        for frame in frames
    ]
    trees = [cKDTree(points) for points in full]
    static_distances = []
    for frame_index, points in enumerate(full):
        distances = [
            trees[other_index].query(points, workers=-1)[0]
            for other_index in range(len(full))
            if other_index != frame_index
        ]
        static_distances.append(np.median(np.stack(distances, axis=1), axis=1))
    median_pose = float(
        np.median([frame["pose"][next_joint_index] for frame in frames])
    )
    center_frame = min(
        frames,
        key=lambda frame: abs(
            float(frame["pose"][next_joint_index]) - median_pose
        ),
    )
    fixed_pivot, fixed_axis = predicted_next_axis(
        center_frame,
        current_joint_index,
        fixed_region_delta,
        directions,
        offsets,
        basis,
        origin,
    )
    base_up = basis[:, 1]
    fixed_regions = []
    for points in full:
        relative = points - fixed_pivot
        along_axis = relative @ fixed_axis
        radial = np.linalg.norm(
            relative - along_axis[:, None] * fixed_axis, axis=1
        )
        height = (points - origin) @ base_up
        fixed_regions.append(
            (radial < 0.23)
            & (height > 0.01)
            & (height < 0.44)
            & (np.linalg.norm(points - fixed_pivot, axis=1) < 0.32)
        )

    results = []
    for delta in candidates:
        pivot, axis = predicted_next_axis(
            center_frame,
            current_joint_index,
            float(delta),
            directions,
            offsets,
            basis,
            origin,
        )
        pose_support = []
        supported_distances: list[float] = []
        for frame_index, (frame, points, static, region) in enumerate(
            zip(
                frames,
                full,
                static_distances,
                fixed_regions,
                strict=True,
            )
        ):
            relative = points - pivot
            moved_distances = []
            for other_index, other_frame in enumerate(frames):
                if other_index == frame_index:
                    continue
                angle = math.radians(
                    next_direction
                    * (
                        float(other_frame["pose"][next_joint_index])
                        - float(frame["pose"][next_joint_index])
                    )
                )
                rotation = Rotation.from_rotvec(axis * angle).as_matrix()
                predicted = relative @ rotation.T + pivot
                moved_distances.append(
                    trees[other_index].query(predicted, workers=-1)[0]
                )
            moved_candidates = np.stack(moved_distances, axis=1)
            support_rank = 1 if moved_candidates.shape[1] >= 3 else 0
            moved = np.partition(
                moved_candidates, support_rank, axis=1
            )[:, support_rank]
            supported = (
                region
                & (static > 0.004)
                & (moved < 0.018)
                & (moved < static * 0.85)
            )
            pose_support.append(int(np.count_nonzero(supported)))
            supported_distances.extend(moved[supported].tolist())
        results.append(
            {
                "delta": float(delta),
                "support": int(sum(pose_support)),
                "minimum_pose_support": int(min(pose_support)),
                "pose_support": pose_support,
                "median_residual_m": (
                    float(np.median(supported_distances))
                    if supported_distances
                    else 0.10
                ),
                "pivot": pivot.tolist(),
                "axis": axis.tolist(),
            }
        )
    return results


def best_axis_curve_result(curve: list[dict]) -> dict:
    return max(
        curve,
        key=lambda item: (
            item["support"],
            item["minimum_pose_support"],
            -item["median_residual_m"],
        ),
    )


def refine_axis_camera_fit(
    frames: list[dict],
    camera_index: int,
    current_joint_index: int,
    next_direction: float,
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> dict:
    coarse_curve = next_axis_camera_curve(
        frames,
        camera_index,
        current_joint_index,
        np.arange(-180.0, 180.0001, 2.5),
        next_direction,
        directions,
        offsets,
        basis,
        origin,
    )
    coarse = best_axis_curve_result(coarse_curve)
    fine_curve = next_axis_camera_curve(
        frames,
        camera_index,
        current_joint_index,
        np.arange(coarse["delta"] - 4.0, coarse["delta"] + 4.0001, 0.25),
        next_direction,
        directions,
        offsets,
        basis,
        origin,
    )
    best = best_axis_curve_result(fine_curve)
    alternatives = [
        item
        for item in coarse_curve
        if abs(item["delta"] - best["delta"]) >= 15.0
    ]
    alternative = best_axis_curve_result(alternatives)
    best["separated_alternative_support"] = alternative["support"]
    best["separated_alternative_delta"] = alternative["delta"]
    best["support_peak_ratio"] = best["support"] / max(
        1, alternative["support"]
    )
    return best


def refine_fixed_region_axis_fit(
    frames: list[dict],
    camera_index: int,
    current_joint_index: int,
    fixed_region_delta: float,
    candidate_minimum: float,
    candidate_maximum: float,
    next_direction: float,
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> dict:
    coarse_curve = fixed_region_axis_camera_curve(
        frames,
        camera_index,
        current_joint_index,
        np.arange(candidate_minimum, candidate_maximum + 0.0001, 2.5),
        fixed_region_delta,
        next_direction,
        directions,
        offsets,
        basis,
        origin,
    )
    coarse = best_axis_curve_result(coarse_curve)
    fine_minimum = max(candidate_minimum, coarse["delta"] - 4.0)
    fine_maximum = min(candidate_maximum, coarse["delta"] + 4.0)
    fine_curve = fixed_region_axis_camera_curve(
        frames,
        camera_index,
        current_joint_index,
        np.arange(fine_minimum, fine_maximum + 0.0001, 0.25),
        fixed_region_delta,
        next_direction,
        directions,
        offsets,
        basis,
        origin,
    )
    best = best_axis_curve_result(fine_curve)
    alternatives = [
        item
        for item in coarse_curve
        if abs(item["delta"] - best["delta"]) >= 15.0
    ]
    alternative = best_axis_curve_result(alternatives)
    best["separated_alternative_support"] = alternative["support"]
    best["separated_alternative_delta"] = alternative["delta"]
    best["support_peak_ratio"] = best["support"] / max(
        1, alternative["support"]
    )
    return best


def solve_wrist_flex_from_roll_axis(
    all_frames: list[dict],
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> dict:
    """Resolve wrist flex from wrist-roll motion without attachment mesh.

    A revolute axis is geometrically ambiguous modulo 180 degrees. The stock
    servo's calibrated zero convention resolves that line-direction ambiguity;
    only candidates yielding an absolute wrist-flex model offset in
    [-100, 100] degrees are physically plausible.
    """
    joint_index = 3
    next_joint_index = 4
    groups = observation_groups(all_frames, next_joint_index)
    candidate_minimum, candidate_maximum = wrist_flex_local_correction_bounds(
        float(offsets[joint_index])
    )

    # Seed only the spatial crop from the primary D455. It must see the motion
    # in every settled pose; the seed is not itself accepted as calibration.
    seed_candidates = []
    for group_index, frames in enumerate(groups):
        coarse = next_axis_camera_curve(
            frames,
            0,
            joint_index,
            np.arange(candidate_minimum, candidate_maximum + 0.0001, 2.5),
            float(directions[next_joint_index]),
            directions,
            offsets,
            basis,
            origin,
        )
        seed = best_axis_curve_result(coarse)
        seed["group_index"] = group_index
        if seed["support"] >= 100 and seed["minimum_pose_support"] >= 5:
            seed_candidates.append(seed)
    if not seed_candidates:
        raise ValueError("wrist roll is not sufficiently visible in the D455")
    seed = max(
        seed_candidates,
        key=lambda item: (
            item["support"],
            item["minimum_pose_support"],
            -item["median_residual_m"],
        ),
    )
    fixed_region_delta = float(seed["delta"])

    fits_by_group: list[list[dict]] = []
    all_camera_results: list[list[dict]] = [
        [] for _ in camera_indices
    ]
    for group_index, frames in enumerate(groups):
        group_results = []
        for result_index, camera_index in enumerate(camera_indices):
            expected = refine_fixed_region_axis_fit(
                frames,
                camera_index,
                joint_index,
                fixed_region_delta,
                candidate_minimum,
                candidate_maximum,
                float(directions[next_joint_index]),
                directions,
                offsets,
                basis,
                origin,
            )
            wrong = fixed_region_axis_camera_curve(
                frames,
                camera_index,
                joint_index,
                np.asarray([expected["delta"]]),
                fixed_region_delta,
                -float(directions[next_joint_index]),
                directions,
                offsets,
                basis,
                origin,
            )[0]
            expected["camera"] = str(
                frames[0]["full_camera_points"][camera_index]["name"]
            )
            expected["view_pan_degrees"] = float(
                np.median([frame["pose"][0] for frame in frames])
            )
            expected["next_joint"] = JOINT_NAMES[next_joint_index]
            expected["next_joint_direction"] = float(
                directions[next_joint_index]
            )
            expected["wrong_direction_support"] = wrong["support"]
            expected["direction_support_ratio"] = expected["support"] / max(
                1, wrong["support"]
            )
            expected["fixed_region_seed_delta"] = fixed_region_delta
            expected["group_index"] = group_index
            group_results.append(expected)
            all_camera_results[camera_index].append(expected)
        fits_by_group.append(group_results)

    # The transferred spatial crop is only meaningful at the same physical
    # arm viewpoint, so require the two cameras to agree within one group.
    first_max = max(result[0]["support"] for result in fits_by_group)
    second_max = max(result[1]["support"] for result in fits_by_group)
    pairs = []
    for first, second in fits_by_group:
        disagreement = abs(first["delta"] - second["delta"])
        if disagreement > 12.0:
            continue
        quality = (
            first["support"] / max(1, first_max)
            + second["support"] / max(1, second_max)
            + min(first["support_peak_ratio"], 1.25)
            + min(second["support_peak_ratio"], 1.25)
            - disagreement / 24.0
        )
        pairs.append((quality, first, second))
    if not pairs:
        alternatives = [
            [item["delta"] for item in camera_fits]
            for camera_fits in all_camera_results
        ]
        raise ValueError(
            "wrist_flex has no agreeing fixed-region cross-camera viewpoint "
            f"({alternatives})"
        )
    _, first_result, second_result = max(pairs, key=lambda item: item[0])
    camera_results = [first_result, second_result]
    deltas = [result["delta"] for result in camera_results]
    disagreement = max(deltas) - min(deltas)
    consensus = float(np.mean(deltas))
    if any(result["support"] < 60 for result in camera_results):
        raise ValueError("wrist roll has too few fixed-region supports")
    if any(result["minimum_pose_support"] < 8 for result in camera_results):
        raise ValueError("wrist roll is not visible in every settled pose")
    direction_ratios = [
        result["direction_support_ratio"] for result in camera_results
    ]
    if max(direction_ratios) < 1.35 or min(direction_ratios) < 0.70:
        raise ValueError(
            "wrist roll rotation direction is ambiguous "
            f"(ratios={direction_ratios}, deltas={deltas}, "
            f"supports={[result['support'] for result in camera_results]})"
        )
    peak_ratios = [result["support_peak_ratio"] for result in camera_results]
    if max(peak_ratios) < 1.08 or min(peak_ratios) < 0.98:
        raise ValueError("wrist flex has a competing fixed-region solution")
    reference_frames = groups[int(first_result["group_index"])]
    pivot, axis = predicted_next_axis(
        reference_frames[len(reference_frames) // 2],
        joint_index,
        consensus,
        directions,
        offsets,
        basis,
        origin,
    )
    return {
        "joint_index": joint_index,
        "joint": JOINT_NAMES[joint_index],
        "method": "fixed_region_next_servo_revolute_axis",
        "next_joint": JOINT_NAMES[next_joint_index],
        "offset_delta_degrees": consensus,
        "camera_offset_disagreement_degrees": disagreement,
        "next_axis_pivot": pivot.tolist(),
        "next_axis": axis.tolist(),
        "camera_results": camera_results,
        "all_camera_view_results": all_camera_results,
        "fixed_region_seed_delta": fixed_region_delta,
    }


def wrist_dynamic_clouds(
    frames: list[dict],
    camera_index: int,
    rough_pivot: np.ndarray,
    basis: np.ndarray,
    origin: np.ndarray,
) -> list[np.ndarray]:
    full = [
        np.asarray(frame["full_camera_points"][camera_index]["points"], dtype=float)
        for frame in frames
    ]
    result = []
    for frame_index, current in enumerate(full):
        other_distances = [
            cKDTree(other).query(current, workers=-1)[0]
            for index, other in enumerate(full)
            if index != frame_index
        ]
        dynamic_score = np.median(np.stack(other_distances, axis=1), axis=1)
        height = (current - origin) @ basis[:, 1]
        selected = current[
            (dynamic_score > WRIST_DYNAMIC_MINIMUM_DISPLACEMENT_M)
            & (np.linalg.norm(current - rough_pivot, axis=1) < 0.46)
            & (height > 0.01)
            & (height < 0.55)
        ]
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(selected))
        downsampled = np.asarray(
            cloud.voxel_down_sample(WRIST_DYNAMIC_VOXEL_SIZE_M).points
        )
        if (
            len(downsampled)
            < MINIMUM_WRIST_DYNAMIC_CONSTRUCTION_POINTS_PER_POSE
        ):
            raise ValueError(
                f"camera {camera_index} found only {len(downsampled)} "
                "wrist-motion points"
            )
        result.append(downsampled)
    return result


def direct_axis_collapse_camera_fit(
    frames: list[dict],
    camera_index: int,
    current_joint_index: int,
    next_direction: float,
    candidate_minimum: float,
    candidate_maximum: float,
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> dict:
    center_frame = frames[len(frames) // 2]
    rough_pivot, _ = predicted_next_axis(
        center_frame,
        current_joint_index,
        0.0,
        directions,
        offsets,
        basis,
        origin,
    )
    clouds = wrist_dynamic_clouds(
        frames,
        camera_index,
        rough_pivot,
        basis,
        origin,
    )

    def curve(candidates: np.ndarray) -> list[dict]:
        result = []
        for delta in candidates:
            pivot, axis = predicted_next_axis(
                center_frame,
                current_joint_index,
                float(delta),
                directions,
                offsets,
                basis,
                origin,
            )
            expected_loss = collapse_loss(
                collapse_clouds(
                    frames,
                    clouds,
                    current_joint_index + 1,
                    pivot,
                    axis,
                    next_direction,
                )
            )
            result.append(
                {
                    "delta": float(delta),
                    "collapse_loss_m": expected_loss,
                    "pivot": pivot.tolist(),
                    "axis": axis.tolist(),
                }
            )
        return result

    coarse_curve = curve(
        np.arange(candidate_minimum, candidate_maximum + 0.0001, 2.5)
    )
    coarse = min(coarse_curve, key=lambda item: item["collapse_loss_m"])
    fine_curve = curve(
        np.arange(
            max(candidate_minimum, coarse["delta"] - 4.0),
            min(candidate_maximum, coarse["delta"] + 4.0) + 0.0001,
            0.25,
        )
    )
    best = min(fine_curve, key=lambda item: item["collapse_loss_m"])
    alternative = min(
        (
            item
            for item in coarse_curve
            if abs(item["delta"] - best["delta"]) >= 15.0
        ),
        key=lambda item: item["collapse_loss_m"],
    )
    pivot = np.asarray(best["pivot"], dtype=float)
    axis = np.asarray(best["axis"], dtype=float)
    wrong_direction_loss = collapse_loss(
        collapse_clouds(
            frames,
            clouds,
            current_joint_index + 1,
            pivot,
            axis,
            -next_direction,
        )
    )
    best["separated_alternative_delta"] = alternative["delta"]
    best["separated_alternative_loss_m"] = alternative["collapse_loss_m"]
    best["loss_peak_ratio"] = (
        alternative["collapse_loss_m"] / best["collapse_loss_m"]
    )
    best["wrong_direction_loss_m"] = wrong_direction_loss
    best["wrong_direction_loss_ratio"] = (
        wrong_direction_loss / best["collapse_loss_m"]
    )
    best["dynamic_points_per_pose"] = [len(cloud) for cloud in clouds]
    return best


def solve_wrist_flex_from_direct_axis_collapse(
    all_frames: list[dict],
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> dict:
    joint_index = 3
    next_joint_index = 4
    groups = observation_groups(all_frames, next_joint_index)
    camera_indices = reference_camera_indices(all_frames)
    candidate_minimum, candidate_maximum = wrist_flex_local_correction_bounds(
        float(offsets[joint_index])
    )
    fits_by_group = []
    all_camera_results: list[list[dict]] = [
        [] for _ in camera_indices
    ]
    for group_index, frames in enumerate(groups):
        group_results = []
        for result_index, camera_index in enumerate(camera_indices):
            fit = direct_axis_collapse_camera_fit(
                frames,
                camera_index,
                joint_index,
                float(directions[next_joint_index]),
                candidate_minimum,
                candidate_maximum,
                directions,
                offsets,
                basis,
                origin,
            )
            fit["camera"] = str(
                frames[0]["full_camera_points"][camera_index]["name"]
            )
            fit["view_pan_degrees"] = float(
                np.median([frame["pose"][0] for frame in frames])
            )
            fit["group_index"] = group_index
            group_results.append(fit)
            all_camera_results[result_index].append(fit)
        fits_by_group.append(group_results)

    best_loss = min(
        result[0]["collapse_loss_m"] for result in fits_by_group
    )
    candidates = []
    for group_results in fits_by_group:
        result = group_results[0]
        if (
            min(result["dynamic_points_per_pose"])
            < MINIMUM_REFERENCE_WRIST_DYNAMIC_POINTS_PER_POSE
        ):
            continue
        direction_ratio = float(result["wrong_direction_loss_ratio"])
        if direction_ratio < 1.12:
            continue
        quality = (
            best_loss / result["collapse_loss_m"]
            + min(result["loss_peak_ratio"], 1.20)
            + min(direction_ratio, 1.25)
        )
        candidates.append((quality, result))
    if not candidates:
        raise ValueError(
            "wrist_flex has no decisive D455 direct-axis viewpoint "
            f"({[item['delta'] for item in all_camera_results[0]]})"
        )
    _, selected_result = max(candidates, key=lambda item: item[0])
    camera_results = [selected_result]
    deltas = [result["delta"] for result in camera_results]
    disagreement = max(deltas) - min(deltas)
    consensus = float(np.mean(deltas))
    losses = [result["collapse_loss_m"] for result in camera_results]
    if max(losses) > 0.035:
        raise ValueError(f"wrist axis collapse residual is too high ({losses})")
    if any(
        min(result["dynamic_points_per_pose"])
        < MINIMUM_REFERENCE_WRIST_DYNAMIC_POINTS_PER_POSE
        for result in camera_results
    ):
        raise ValueError("wrist motion is not dense in every settled pose")
    peak_ratios = [result["loss_peak_ratio"] for result in camera_results]
    shallow_fixed_point_basin = (
        max(peak_ratios) < MINIMUM_REFERENCE_WRIST_AXIS_PEAK_RATIO
        or min(peak_ratios) < 0.98
    )
    if (
        shallow_fixed_point_basin
        and abs(consensus) > MAXIMUM_FIXED_POINT_DELTA_DEGREES
    ):
        raise ValueError(
            "wrist flex has a competing direct-axis solution "
            f"(ratios={peak_ratios}, deltas={deltas})"
        )
    direction_ratios = [
        float(result["wrong_direction_loss_ratio"])
        for result in camera_results
    ]
    if max(direction_ratios) < 1.12:
        raise ValueError(
            "wrist-roll encoder direction is not decisive in the D455 "
            f"motion view (ratios={direction_ratios})"
        )
    reference_frames = groups[int(selected_result["group_index"])]
    pivot, axis = predicted_next_axis(
        reference_frames[len(reference_frames) // 2],
        joint_index,
        consensus,
        directions,
        offsets,
        basis,
        origin,
    )
    return {
        "joint_index": joint_index,
        "joint": JOINT_NAMES[joint_index],
        "method": "direct_dynamic_next_servo_axis_line_collapse",
        "next_joint": JOINT_NAMES[next_joint_index],
        "offset_delta_degrees": consensus,
        "camera_offset_disagreement_degrees": disagreement,
        "next_axis_pivot": pivot.tolist(),
        "next_axis": axis.tolist(),
        "camera_results": camera_results,
        "all_camera_view_results": all_camera_results,
        "rotation_direction_observable": False,
        "preserved_next_joint_direction": float(directions[next_joint_index]),
        "reference_camera_serial": REFERENCE_CAMERA_SERIAL,
        "fixed_point_only_shallow_basin": shallow_fixed_point_basin,
    }


def mapped_link_depth_losses(
    frames: list[dict],
    joint_index: int,
    delta: float,
    raw_points: np.ndarray,
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
    camera_indices: list[int] | None = None,
) -> list[float]:
    selected = (
        reference_camera_indices(frames)
        if camera_indices is None
        else camera_indices
    )
    per_camera: list[list[float]] = [[] for _ in selected]
    for frame in frames:
        predicted = posed_mesh(
            raw_points,
            frame,
            joint_index,
            delta,
            directions,
            offsets,
            basis,
            origin,
        )
        for result_index, camera_index in enumerate(selected):
            observed = np.asarray(
                frame["full_camera_points"][camera_index]["points"],
                dtype=float,
            )
            distances = np.sort(
                cKDTree(observed).query(predicted, workers=-1)[0]
            )
            keep = max(200, int(0.35 * len(distances)))
            per_camera[result_index].append(
                math.sqrt(
                    float(np.mean(np.minimum(distances[:keep], 0.10) ** 2))
                )
            )
    return [float(np.median(losses)) for losses in per_camera]


def mapped_mesh_arbitration_candidates(
    motion_basin: list[dict],
) -> list[float]:
    """Let mapped stock geometry overturn a false dynamic-motion basin.

    Motion remains broad enough to recover an unknown convention, while the
    independently mapped link also gets a local neighborhood around the
    currently loaded mechanical zero. Restricting mesh scoring to the motion
    basin allowed attachment/clutter motion to pull elbow zero almost 20
    degrees away even though the stock lower-arm surface strongly contradicted
    it.
    """
    candidates = {
        round(float(item["delta"]), 6)
        for item in motion_basin
    }
    candidates.update(
        round(float(delta), 6)
        for delta in np.arange(
            -MAPPED_MESH_CURRENT_ZERO_SEARCH_RADIUS_DEGREES,
            MAPPED_MESH_CURRENT_ZERO_SEARCH_RADIUS_DEGREES + 0.0001,
            2.5,
        )
    )
    return sorted(candidates)


def should_preserve_current_zero_from_mapped_depth(
    fitted_losses: list[float],
    current_zero_losses: list[float],
) -> bool:
    """Avoid changing a supported mechanical zero on a shallow score curve."""
    return (
        max(current_zero_losses) <= MAXIMUM_STRONG_MESH_DEPTH_LOSS_M
        and sum(current_zero_losses) / max(1.0e-9, sum(fitted_losses))
        < MINIMUM_REFERENCE_MAPPED_MESH_PEAK_RATIO
    )


def has_sufficient_cross_camera_settled_axis_pose_coverage(
    camera_pose_support: list[list[int]],
) -> bool:
    """Require a broad primary view and three-pose independent validation."""
    if len(camera_pose_support) < 1 or any(
        len(pose_support) < 5 for pose_support in camera_pose_support
    ):
        return False
    visible_counts = [
        sum(
            support >= MINIMUM_AXIS_SUPPORTS_PER_VISIBLE_POSE
            for support in pose_support
        )
        for pose_support in camera_pose_support
    ]
    if len(visible_counts) == 1:
        return (
            visible_counts[0]
            >= MINIMUM_VISIBLE_SETTLED_AXIS_POSES_IN_PRIMARY_VIEW
        )
    return (
        min(visible_counts)
        >= MINIMUM_VISIBLE_SETTLED_AXIS_POSES_PER_CAMERA
        and max(visible_counts)
        >= MINIMUM_VISIBLE_SETTLED_AXIS_POSES_IN_PRIMARY_VIEW
    )


def wrist_flex_local_correction_bounds(
    current_offset_degrees: float,
) -> tuple[float, float]:
    """Bound automatic refinement away from the near-180-degree line branch."""
    return (
        max(
            -WRIST_FLEX_CURRENT_ZERO_SEARCH_RADIUS_DEGREES,
            -100.0 - current_offset_degrees,
        ),
        min(
            WRIST_FLEX_CURRENT_ZERO_SEARCH_RADIUS_DEGREES,
            100.0 - current_offset_degrees,
        ),
    )


def solve_joint_from_combined_axis_and_mapped_link(
    all_frames: list[dict],
    joint_index: int,
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> dict:
    next_joint_index = joint_index + 1
    groups = observation_groups(all_frames, next_joint_index)
    camera_indices = reference_camera_indices(all_frames)
    coarse_candidates = np.arange(-180.0, 180.0001, 2.5)
    coarse_curves = [
        [
            next_axis_camera_curve(
                frames,
                camera_index,
                joint_index,
                coarse_candidates,
                float(directions[next_joint_index]),
                directions,
                offsets,
                basis,
                origin,
            )
            for frames in groups
        ]
        for camera_index in camera_indices
    ]
    maximum_support = [
        max(
            item["support"]
            for curve in coarse_curves[camera_index]
            for item in curve
        )
        for camera_index in range(len(camera_indices))
    ]
    maximum_minimum_pose = [
        max(
            item["minimum_pose_support"]
            for curve in coarse_curves[camera_index]
            for item in curve
        )
        for camera_index in range(len(camera_indices))
    ]

    def view_quality(item: dict, camera_index: int) -> float:
        return (
            item["support"] / max(1, maximum_support[camera_index])
            + item["minimum_pose_support"]
            / max(1, maximum_minimum_pose[camera_index])
            + max(0.0, 1.0 - item["median_residual_m"] / 0.014)
        )

    combined = []
    for candidate_index, delta in enumerate(coarse_candidates):
        selected = []
        total_quality = 0.0
        for camera_index in range(len(camera_indices)):
            options = [
                (
                    view_quality(
                        coarse_curves[camera_index][group_index][candidate_index],
                        camera_index,
                    ),
                    group_index,
                    coarse_curves[camera_index][group_index][candidate_index],
                )
                for group_index in range(len(groups))
            ]
            quality, group_index, item = max(options, key=lambda value: value[0])
            selected.append((group_index, item))
            total_quality += quality
        combined.append(
            {
                "delta": float(delta),
                "quality": total_quality,
                "selected": selected,
            }
        )
    best_motion = max(combined, key=lambda item: item["quality"])
    motion_basin = [
        item
        for item in combined
        if item["quality"] >= best_motion["quality"] * 0.975
    ]
    link_name = JOINT_LINKS[joint_index]
    mesh = o3d.io.read_triangle_mesh(
        str(ASSET_ROOT / f"{link_name}.glb")
    )
    raw_points = np.asarray(mesh.sample_points_uniformly(1800).points)
    validation_frames = [
        frame
        for frames in groups
        for frame in frames
    ]
    coarse_mesh_results = []
    for delta in mapped_mesh_arbitration_candidates(motion_basin):
        losses = mapped_link_depth_losses(
            validation_frames,
            joint_index,
            delta,
            raw_points,
            directions,
            offsets,
            basis,
            origin,
        )
        coarse_mesh_results.append((sum(losses), delta, losses))
    _, mesh_seed_delta, _ = min(coarse_mesh_results, key=lambda item: item[0])
    fine_candidates = np.arange(
        mesh_seed_delta - 4.0,
        mesh_seed_delta + 4.0001,
        0.25,
    )
    fine_mesh_results = []
    for delta in fine_candidates:
        losses = mapped_link_depth_losses(
            validation_frames,
            joint_index,
            float(delta),
            raw_points,
            directions,
            offsets,
            basis,
            origin,
        )
        fine_mesh_results.append((sum(losses), float(delta), losses))
    _, fitted_delta, depth_losses = min(
        fine_mesh_results, key=lambda item: item[0]
    )
    current_zero_losses = mapped_link_depth_losses(
        validation_frames,
        joint_index,
        0.0,
        raw_points,
        directions,
        offsets,
        basis,
        origin,
    )
    preserved_current_zero = (
        abs(fitted_delta) > 0.001
        and should_preserve_current_zero_from_mapped_depth(
            depth_losses,
            current_zero_losses,
        )
    )
    if preserved_current_zero:
        fitted_delta = 0.0
        depth_losses = current_zero_losses

    camera_results = []
    for result_index, camera_index in enumerate(camera_indices):
        options = []
        for group_index, frames in enumerate(groups):
            expected = next_axis_camera_curve(
                frames,
                camera_index,
                joint_index,
                np.asarray([fitted_delta]),
                float(directions[next_joint_index]),
                directions,
                offsets,
                basis,
                origin,
            )[0]
            options.append((view_quality(expected, result_index), group_index, expected))
        _, group_index, expected = max(options, key=lambda value: value[0])
        frames = groups[group_index]
        wrong = next_axis_camera_curve(
            frames,
            camera_index,
            joint_index,
            np.asarray([fitted_delta]),
            -float(directions[next_joint_index]),
            directions,
            offsets,
            basis,
            origin,
        )[0]
        expected["camera"] = str(
            frames[0]["full_camera_points"][camera_index]["name"]
        )
        expected["view_pan_degrees"] = float(
            np.median([frame["pose"][0] for frame in frames])
        )
        expected["next_joint"] = JOINT_NAMES[next_joint_index]
        expected["next_joint_direction"] = float(
            directions[next_joint_index]
        )
        expected["wrong_direction_support"] = wrong["support"]
        expected["direction_support_ratio"] = expected["support"] / max(
            1, wrong["support"]
        )
        expected["mapped_link_depth_loss_m"] = depth_losses[result_index]
        camera_results.append(expected)

    if any(result["support"] < 60 for result in camera_results):
        raise ValueError(f"{JOINT_NAMES[joint_index]} has too few joint-axis supports")
    if not has_sufficient_cross_camera_settled_axis_pose_coverage(
        [result["pose_support"] for result in camera_results]
    ):
        raise ValueError(
            f"{JOINT_NAMES[joint_index]} lacks broad settled-pose visibility "
            f"({[result['pose_support'] for result in camera_results]})"
        )
    direction_ratios = [
        result["direction_support_ratio"] for result in camera_results
    ]
    if max(direction_ratios) < 1.35 or min(direction_ratios) < 0.70:
        raise ValueError(
            f"{JOINT_NAMES[next_joint_index]} rotation direction is ambiguous"
        )
    if max(depth_losses) > 0.030:
        raise ValueError(
            f"{link_name} mapped depth residual is too high ({depth_losses})"
        )
    alternatives = [
        item
        for item in combined
        if (
            abs(item["delta"] - fitted_delta)
            >= REMOTE_ALTERNATIVE_SEPARATION_DEGREES
        )
    ]
    alternative = max(alternatives, key=lambda item: item["quality"])
    combined_peak_ratio = best_motion["quality"] / max(
        0.0001, alternative["quality"]
    )
    mesh_alternatives = [
        item
        for item in coarse_mesh_results
        if (
            abs(item[1] - fitted_delta)
            >= REMOTE_ALTERNATIVE_SEPARATION_DEGREES
        )
    ]
    if mesh_alternatives:
        alternative_mesh_loss, alternative_mesh_delta, alternative_mesh_losses = min(
            mesh_alternatives,
            key=lambda item: item[0],
        )
        remote_mesh_peak_ratio = alternative_mesh_loss / max(
            0.0001,
            sum(depth_losses),
        )
    else:
        alternative_mesh_delta = None
        alternative_mesh_losses = []
        remote_mesh_peak_ratio = NO_COMPETING_MESH_PEAK_RATIO
    strong_mapped_mesh_separation = (
        max(depth_losses) <= MAXIMUM_STRONG_MESH_DEPTH_LOSS_M
        and remote_mesh_peak_ratio >= MINIMUM_REMOTE_MAPPED_MESH_PEAK_RATIO
    )
    if (
        combined_peak_ratio < MINIMUM_REMOTE_MOTION_PEAK_RATIO
        and not strong_mapped_mesh_separation
        and not preserved_current_zero
    ):
        raise ValueError(
            f"{JOINT_NAMES[joint_index]} absolute zero is ambiguous across "
            f"motion and mapped depth (motion_ratio={combined_peak_ratio:.3f}, "
            f"mesh_ratio={remote_mesh_peak_ratio:.3f}, fitted={fitted_delta}, "
            f"motion_alternative={alternative['delta']}, "
            f"mesh_alternative={alternative_mesh_delta}, "
            f"fitted_depth_losses={depth_losses}, "
            f"alternative_depth_losses={alternative_mesh_losses})"
        )
    reference_frames = groups[
        min(
            range(len(groups)),
            key=lambda index: abs(
                float(np.median([frame["pose"][0] for frame in groups[index]]))
            ),
        )
    ]
    pivot, axis = predicted_next_axis(
        reference_frames[len(reference_frames) // 2],
        joint_index,
        fitted_delta,
        directions,
        offsets,
        basis,
        origin,
    )
    return {
        "joint_index": joint_index,
        "joint": JOINT_NAMES[joint_index],
        "method": "reference_camera_axis_basin_mapped_link_depth",
        "next_joint": JOINT_NAMES[next_joint_index],
        "offset_delta_degrees": fitted_delta,
        "camera_offset_disagreement_degrees": 0.0,
        "next_axis_pivot": pivot.tolist(),
        "next_axis": axis.tolist(),
        "camera_results": camera_results,
        "combined_motion_peak_ratio": combined_peak_ratio,
        "combined_motion_peak_delta": best_motion["delta"],
        "remote_mapped_mesh_peak_ratio": remote_mesh_peak_ratio,
        "remote_mapped_mesh_alternative_delta": alternative_mesh_delta,
        "remote_mapped_mesh_alternative_losses_m": alternative_mesh_losses,
        "mapped_link": link_name,
        "mapped_link_depth_losses_m": depth_losses,
        "current_zero_mapped_link_depth_losses_m": current_zero_losses,
        "preserved_current_zero_due_shallow_evidence": (
            preserved_current_zero
        ),
    }


def solve_joint_from_next_axis(
    all_frames: list[dict],
    joint_index: int,
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> dict:
    if joint_index == 3:
        return solve_wrist_flex_from_direct_axis_collapse(
            all_frames,
            directions,
            offsets,
            basis,
            origin,
        )
    return solve_joint_from_combined_axis_and_mapped_link(
        all_frames,
        joint_index,
        directions,
        offsets,
        basis,
        origin,
    )
    next_joint_index = joint_index + 1
    groups = observation_groups(all_frames, next_joint_index)
    camera_count = len(groups[0][0].get("full_camera_points", []))
    if camera_count < 2:
        raise ValueError("joint-axis solving requires two camera clouds")

    fits_by_camera: list[list[dict]] = []
    for camera_index in range(camera_count):
        group_fits = []
        for frames in groups:
            camera_name = str(
                frames[0]["full_camera_points"][camera_index]["name"]
            )
            expected = refine_axis_camera_fit(
                frames,
                camera_index,
                joint_index,
                float(directions[next_joint_index]),
                directions,
                offsets,
                basis,
                origin,
            )
            # Direction is tested at the axis location found by the expected
            # motion. Letting the wrong sign search a completely different
            # axis can manufacture a second explanation from scene clutter.
            wrong = next_axis_camera_curve(
                frames,
                camera_index,
                joint_index,
                np.asarray([expected["delta"]]),
                -float(directions[next_joint_index]),
                directions,
                offsets,
                basis,
                origin,
            )[0]
            expected["camera"] = camera_name
            expected["view_pan_degrees"] = float(
                np.median([frame["pose"][0] for frame in frames])
            )
            expected["next_joint"] = JOINT_NAMES[next_joint_index]
            expected["next_joint_direction"] = float(
                directions[next_joint_index]
            )
            expected["wrong_direction_support"] = wrong["support"]
            expected["direction_support_ratio"] = expected["support"] / max(
                1, wrong["support"]
            )
            group_fits.append(expected)
        fits_by_camera.append(group_fits)

    # Select the independently observed camera viewpoints as an agreeing pair.
    # A view that only sees disoccluded background can have many accidental
    # supports, but it will not produce the same parent zero as the opposite
    # camera's broadside view.
    pairs = []
    first_max = max(item["support"] for item in fits_by_camera[0])
    second_max = max(item["support"] for item in fits_by_camera[1])
    for first in fits_by_camera[0]:
        for second in fits_by_camera[1]:
            disagreement = abs(first["delta"] - second["delta"])
            if disagreement > 12.0:
                continue
            quality = (
                first["support"] / max(1, first_max)
                + second["support"] / max(1, second_max)
                + min(first["support_peak_ratio"], 1.25)
                + min(second["support_peak_ratio"], 1.25)
                - disagreement / 24.0
            )
            pairs.append((quality, first, second))
    if not pairs:
        alternatives = [
            [item["delta"] for item in camera_fits]
            for camera_fits in fits_by_camera
        ]
        raise ValueError(
            f"{JOINT_NAMES[joint_index]} has no agreeing cross-camera "
            f"observation viewpoints ({alternatives})"
        )
    _, first_result, second_result = max(pairs, key=lambda item: item[0])
    camera_results = [first_result, second_result]

    deltas = [result["delta"] for result in camera_results]
    disagreement = max(deltas) - min(deltas)
    consensus = float(np.mean(deltas))
    if any(result["support"] < 60 for result in camera_results):
        raise ValueError(
            f"{JOINT_NAMES[joint_index]} next-axis motion has too few "
            f"cross-camera supports "
            f"({[result['support'] for result in camera_results]})"
        )
    if any(result["minimum_pose_support"] < 8 for result in camera_results):
        raise ValueError(
            f"{JOINT_NAMES[joint_index]} next-axis motion is not visible "
            "in every settled pose"
        )
    direction_ratios = [
        result["direction_support_ratio"] for result in camera_results
    ]
    # One camera may see mostly end-on or self-occluded motion and therefore
    # carry little sign information. Require one decisive independent sign
    # observation, while rejecting any camera that decisively contradicts it.
    if max(direction_ratios) < 1.35 or min(direction_ratios) < 0.70:
        raise ValueError(
            f"{JOINT_NAMES[next_joint_index]} rotation direction is ambiguous"
        )
    peak_ratios = [result["support_peak_ratio"] for result in camera_results]
    # Cross-camera agreement can disambiguate a broad end-on view. Demand one
    # camera with a separated peak and ensure the other does not prefer a
    # remote alternative; requiring both peaks to be equally sharp rejected
    # repeat captures whose fitted axes agreed within one degree.
    if max(peak_ratios) < 1.05 or min(peak_ratios) < 0.98:
        raise ValueError(
            f"{JOINT_NAMES[joint_index]} absolute zero has a competing "
            f"motion-axis solution (ratios={peak_ratios}, deltas={deltas}, "
            f"supports={[result['support'] for result in camera_results]})"
        )
    reference_frames = min(
        groups,
        key=lambda frames: abs(
            float(np.median([frame["pose"][0] for frame in frames]))
        ),
    )
    pivot, axis = predicted_next_axis(
        reference_frames[len(reference_frames) // 2],
        joint_index,
        consensus,
        directions,
        offsets,
        basis,
        origin,
    )
    return {
        "joint_index": joint_index,
        "joint": JOINT_NAMES[joint_index],
        "method": "next_servo_revolute_axis",
        "next_joint": JOINT_NAMES[next_joint_index],
        "offset_delta_degrees": consensus,
        "camera_offset_disagreement_degrees": disagreement,
        "next_axis_pivot": pivot.tolist(),
        "next_axis": axis.tolist(),
        "camera_results": camera_results,
        "all_camera_view_results": fits_by_camera,
    }


def solve_joint(
    all_frames: list[dict],
    joint_index: int,
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
    raw_points: np.ndarray,
) -> dict:
    frames = three_distinct_frames(all_frames, joint_index)
    _, pivots, axes = unbounded_chain(frames[1]["pose"], directions, offsets)
    pivot = global_point(pivots[joint_index], basis, origin)
    axis = global_axis(axes[joint_index], basis)
    camera_count = len(frames[0].get("full_camera_points", []))
    if camera_count < 2:
        raise ValueError("staged joint solving requires two independent camera clouds")
    camera_results = []
    for camera_index in range(camera_count):
        camera_name = str(frames[0]["full_camera_points"][camera_index]["name"])
        clouds = dynamic_clouds(frames, camera_index, pivot, basis[:, 1], origin)
        sign_losses = {}
        collapsed_candidates = {}
        for direction in (-1.0, 1.0):
            collapsed = collapse_clouds(
                frames, clouds, joint_index, pivot, axis, direction
            )
            collapsed_candidates[direction] = collapsed
            sign_losses[direction] = collapse_loss(collapsed)
        fitted_direction = min(sign_losses, key=sign_losses.get)
        expected_direction = float(directions[joint_index])
        if fitted_direction != expected_direction:
            raise ValueError(
                f"{camera_name} sees {JOINT_NAMES[joint_index]} direction "
                f"{fitted_direction:+.0f}, expected {expected_direction:+.0f}"
            )
        observed = np.concatenate(collapsed_candidates[fitted_direction])
        observed = np.asarray(
            o3d.geometry.PointCloud(o3d.utility.Vector3dVector(observed))
            .voxel_down_sample(0.003)
            .points
        )
        candidates = np.arange(-150.0, 150.0001, 0.25)
        losses = [
            mesh_loss(
                posed_mesh(
                    raw_points,
                    frames[1],
                    joint_index,
                    float(delta),
                    directions,
                    offsets,
                    basis,
                    origin,
                ),
                observed,
            )
            for delta in candidates
        ]
        best_index = int(np.argmin(losses))
        camera_results.append(
            {
                "camera": camera_name,
                "direction": fitted_direction,
                "direction_loss_m": sign_losses[fitted_direction],
                "wrong_direction_loss_ratio": (
                    sign_losses[-fitted_direction] / sign_losses[fitted_direction]
                ),
                "offset_delta_degrees": float(candidates[best_index]),
                "mesh_loss_m": float(losses[best_index]),
                "zero_delta_mesh_loss_m": float(losses[len(candidates) // 2]),
                "dynamic_points": int(sum(len(cloud) for cloud in clouds)),
            }
        )
    deltas = [result["offset_delta_degrees"] for result in camera_results]
    disagreement = max(deltas) - min(deltas)
    consensus = float(np.mean(deltas))
    if disagreement > 8.0:
        raise ValueError(
            f"{JOINT_NAMES[joint_index]} camera offset disagreement is "
            f"{disagreement:.2f} degrees ({deltas})"
        )
    if any(result["wrong_direction_loss_ratio"] < 1.22 for result in camera_results):
        raise ValueError(f"{JOINT_NAMES[joint_index]} rotation direction is ambiguous")
    return {
        "joint_index": joint_index,
        "joint": JOINT_NAMES[joint_index],
        "offset_delta_degrees": consensus,
        "camera_offset_disagreement_degrees": disagreement,
        "pivot": pivot.tolist(),
        "axis": axis.tolist(),
        "camera_results": camera_results,
    }


def solve_outward_chain_to_fixed_point(
    frames: list[dict],
    through_joint: int,
    directions: list[float],
    initial_offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> tuple[list[float], list[dict], dict]:
    """Iterate the staged solve until its own evidence no longer requests a
    material correction.

    A single pass can be biased when the incoming registration has very poor
    downstream zeroes. Merely accepting that pass makes repeated presses drift.
    Each material pass is applied only to the transaction candidate; a final
    complete pass must independently request no more than the fixed-point
    tolerance from any calibrated joint.
    """
    offsets = list(initial_offsets)
    pass_history = []
    validation_results: list[dict] = []
    for pass_index in range(1, MAXIMUM_CONVERGENCE_PASSES + 1):
        # Keep stock-mesh sampling identical between convergence passes.
        o3d.utility.random.seed(23)
        solved_this_pass = []
        try:
            for joint_index in range(1, through_joint + 1):
                solved = solve_joint_from_next_axis(
                    frames,
                    joint_index,
                    directions,
                    offsets,
                    basis,
                    origin,
                )
                solved_this_pass.append(solved)
                # Preserve outward staging within a pass: elbow sees the
                # accepted shoulder candidate, and wrist sees both accepted
                # parent joints.
                offsets[joint_index] += solved["offset_delta_degrees"]
        except ValueError as error:
            raise ValueError(
                f"convergence pass {pass_index} rejected; "
                f"prior pass history={pass_history}: {error}"
            ) from error
        corrections = [
            float(result["offset_delta_degrees"])
            for result in solved_this_pass
        ]
        maximum_correction = max(abs(value) for value in corrections)
        evidence_summaries = []
        for solved in solved_this_pass:
            cameras = solved.get("camera_results", [])
            camera = cameras[0] if cameras else {}
            evidence_summaries.append(
                {
                    "joint": solved["joint"],
                    "loss_peak_ratio": float(
                        camera.get(
                            "loss_peak_ratio",
                            solved.get("remote_mapped_mesh_peak_ratio", 0.0),
                        )
                    ),
                    "wrong_direction_loss_ratio": float(
                        camera.get(
                            "wrong_direction_loss_ratio",
                            camera.get("direction_support_ratio", 0.0),
                        )
                    ),
                    "minimum_dynamic_points_per_pose": int(
                        min(camera.get("dynamic_points_per_pose", [0]))
                    ),
                }
            )
        pass_history.append(
            {
                "pass": pass_index,
                "corrections_degrees": corrections,
                "maximum_correction_degrees": maximum_correction,
                "joint_evidence": evidence_summaries,
            }
        )
        print(
            "SO101_STAGED_CONVERGENCE_PASS "
            + json.dumps(pass_history[-1], separators=(",", ":")),
            flush=True,
        )
        if maximum_correction <= MAXIMUM_FIXED_POINT_DELTA_DEGREES:
            # The pre-pass candidate was independently shown to be a fixed
            # point. Do not apply the small validation residual, which avoids
            # quantization/noise oscillation on repeated button presses.
            for joint_index, correction in enumerate(corrections, start=1):
                offsets[joint_index] -= correction
            validation_results = solved_this_pass
            break
    if not validation_results:
        raise ValueError(
            "staged joint calibration did not converge after "
            f"{MAXIMUM_CONVERGENCE_PASSES} passes "
            f"(history={pass_history})"
        )

    total_deltas = [
        float(value - initial_offsets[index])
        for index, value in enumerate(offsets)
    ]
    for result in validation_results:
        joint_index = int(result["joint_index"])
        result["fixed_point_delta_degrees"] = float(
            result["offset_delta_degrees"]
        )
        result["offset_delta_degrees"] = total_deltas[joint_index]
    convergence = {
        "converged": True,
        "passes": len(pass_history),
        "maximum_fixed_point_delta_degrees": (
            MAXIMUM_FIXED_POINT_DELTA_DEGREES
        ),
        "pass_history": pass_history,
    }
    return offsets, validation_results, convergence


def _nearest_equivalent_degrees(value: float, reference: float) -> float:
    return reference + (value - reference + 180.0) % 360.0 - 180.0


def wrist_roll_evidence_class(
    mesh_loss_m: float,
    strong_support_fraction: float,
    partial_support_fraction: float,
    peak_ratio: float,
    basin_width_degrees: float,
) -> str:
    common_quality = (
        peak_ratio >= MINIMUM_WRIST_ROLL_MESH_PEAK_RATIO
        and basin_width_degrees
        <= MAXIMUM_WRIST_ROLL_LOCAL_BASIN_WIDTH_DEGREES
    )
    if (
        mesh_loss_m <= MAXIMUM_WRIST_ROLL_MESH_LOSS_M
        and strong_support_fraction
        >= MINIMUM_WRIST_ROLL_MESH_SUPPORT_FRACTION
        and common_quality
    ):
        return "strong_full_surface"
    if (
        mesh_loss_m <= MAXIMUM_PARTIAL_WRIST_ROLL_MESH_LOSS_M
        and partial_support_fraction
        >= MINIMUM_PARTIAL_WRIST_ROLL_MESH_SUPPORT_FRACTION
        and common_quality
    ):
        return "unique_partial_surface"
    return "invisible_or_ambiguous"


def select_wrist_roll_cross_camera_pair(
    accepted_by_camera: dict[str, list[dict]],
) -> tuple[list[dict], list[float], float]:
    """Choose the strongest *agreeing* views, not two incompatible rank ones."""
    camera_names = sorted(accepted_by_camera)
    if len(camera_names) != MINIMUM_WRIST_ROLL_VALIDATING_CAMERAS:
        raise ValueError("wrist-roll pair selection requires exactly two cameras")
    pairs = []
    for first in accepted_by_camera[camera_names[0]]:
        first_delta = float(first["offset_delta_degrees"])
        for second in accepted_by_camera[camera_names[1]]:
            second_delta = _nearest_equivalent_degrees(
                float(second["offset_delta_degrees"]),
                first_delta,
            )
            disagreement = abs(first_delta - second_delta)
            if disagreement > MAXIMUM_WRIST_ROLL_VIEW_DISAGREEMENT_DEGREES:
                continue
            partial_count = sum(
                result["evidence_class"] != "strong_full_surface"
                for result in (first, second)
            )
            pairs.append(
                (
                    partial_count,
                    float(first["mesh_loss_m"])
                    + float(second["mesh_loss_m"]),
                    disagreement,
                    [first, second],
                    [first_delta, second_delta],
                )
            )
    if not pairs:
        raise ValueError("wrist-roll has no agreeing cross-camera view pair")
    _, _, disagreement, accepted, unwrapped = min(
        pairs,
        key=lambda item: item[:3],
    )
    return accepted, unwrapped, disagreement


def select_wrist_roll_agreeing_viewpoint_pair(
    viewpoints: list[dict],
) -> tuple[list[dict], list[float], float]:
    """Select two agreeing D455 viewpoints while rejecting optical outliers.

    The custom rigid wrist camera can make one otherwise well-supported mesh
    basin land on the wrong rotational symmetry.  Three physical viewpoints
    are captured so calibration can require two independent estimates to
    agree without allowing the single outlier to veto that consensus.
    """
    pairs = []
    for first_index, first in enumerate(viewpoints):
        first_offset = float(
            first["orientation_camera"]["fitted_offset_degrees"]
        )
        for second in viewpoints[first_index + 1 :]:
            second_offset = _nearest_equivalent_degrees(
                float(
                    second["orientation_camera"][
                        "fitted_offset_degrees"
                    ]
                ),
                first_offset,
            )
            disagreement = abs(first_offset - second_offset)
            if disagreement > MAXIMUM_WRIST_ROLL_VIEW_DISAGREEMENT_DEGREES:
                continue
            pair = [first, second]
            # A low-peak-ratio view may corroborate the absolute zero selected
            # by a strong view, but two individually symmetric/ambiguous views
            # must never validate one another.
            if not any(
                bool(viewpoint.get("visible_in_both_cameras", False))
                for viewpoint in pair
            ):
                continue
            partial_count = sum(
                camera["evidence_class"] != "strong_full_surface"
                for viewpoint in pair
                for camera in viewpoint["camera_results"]
            )
            pairs.append(
                (
                    partial_count,
                    sum(
                        float(viewpoint["combined_mesh_loss_m"])
                        for viewpoint in pair
                    ),
                    disagreement,
                    pair,
                    [first_offset, second_offset],
                )
            )
    if not pairs:
        return [], [], math.inf
    _, _, disagreement, pair, offsets = min(
        pairs,
        key=lambda item: item[:3],
    )
    return pair, offsets, disagreement


def select_wrist_roll_fine_views(
    camera_candidates: list[dict],
) -> list[dict]:
    """Reserve fine-scoring slots for the purpose-built roll observations."""
    ordered = sorted(
        camera_candidates,
        key=lambda item: item["screening_mesh_loss_m"],
    )
    dedicated = [
        item
        for item in ordered
        if int(item["frame"].get("calibration_joint_index", -2)) == 4
    ][:MAXIMUM_WRIST_ROLL_FINE_VIEWS_PER_CAMERA]
    other = [
        item
        for item in ordered
        if int(item["frame"].get("calibration_joint_index", -2)) != 4
    ][:MAXIMUM_WRIST_ROLL_FINE_VIEWS_PER_CAMERA]
    return dedicated + other


def _solve_wrist_roll_zero_from_single_view_legacy(
    frames: list[dict],
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> dict:
    """Fit wrist-roll zero from visible stock-gripper surfaces.

    Wrist-roll motion determines its axis but cannot determine rotation about
    that axis. Fit only the mapped fixed jaw in settled depth frames. Earlier
    one-servo stages and the dedicated five-angle roll observations expose
    different fixed-jaw surfaces without adding another physical sweep. This
    deliberately does not model the customized camera bracket.
    """
    settled_frames = [
        (frame_index, frame)
        for frame_index, frame in enumerate(frames)
        if int(frame.get("calibration_joint_index", -2))
        in (-1, 1, 2, 3, 4)
    ]
    if not settled_frames:
        raise ValueError("wrist-roll zero has no settled depth frame")

    o3d.utility.random.seed(41)
    mesh = o3d.io.read_triangle_mesh(str(ASSET_ROOT / "gripper_link.glb"))
    raw_points = np.asarray(mesh.sample_points_uniformly(2400).points)
    if len(raw_points) < 1000:
        raise ValueError("stock gripper mesh could not be sampled")
    coarse_points = raw_points[::3]

    coarse_candidates = np.arange(-180.0, 180.0, 5.0)
    candidates_by_camera: dict[str, list[dict]] = {}
    for frame_index, frame in settled_frames:
        camera_payloads = frame.get("full_camera_points", [])
        for camera_index, camera_payload in enumerate(camera_payloads):
            observed = np.asarray(camera_payload.get("points", []), dtype=float)
            if len(observed) < 500:
                continue
            camera_name = str(camera_payload.get("name", camera_index))
            tree = cKDTree(observed)
            coarse_curve = []
            for delta in coarse_candidates:
                predicted = posed_mesh(
                    coarse_points,
                    frame,
                    4,
                    float(delta),
                    directions,
                    offsets,
                    basis,
                    origin,
                )
                coarse_curve.append(
                    {
                        "delta": float(delta),
                        "mesh_loss_m": mesh_loss_to_tree(
                            predicted,
                            tree,
                            workers=1,
                        ),
                    }
                )
            coarse = min(coarse_curve, key=lambda item: item["mesh_loss_m"])
            candidates_by_camera.setdefault(camera_name, []).append(
                {
                    "camera": camera_name,
                    "camera_index": camera_index,
                    "capture_frame_index": frame_index,
                    "frame": frame,
                    "observed": observed,
                    "tree": tree,
                    "screening_mesh_loss_m": float(coarse["mesh_loss_m"]),
                }
            )

    if len(candidates_by_camera) < MINIMUM_WRIST_ROLL_VALIDATING_CAMERAS:
        raise ValueError(
            "wrist-roll zero requires settled depth from both RealSense cameras"
        )

    all_view_results = []
    accepted_by_camera: dict[str, list[dict]] = {}
    for camera_name, camera_candidates in candidates_by_camera.items():
        for view in select_wrist_roll_fine_views(camera_candidates):
            frame = view["frame"]
            tree = view["tree"]
            coarse_curve = []
            for delta in coarse_candidates:
                predicted = posed_mesh(
                    raw_points,
                    frame,
                    4,
                    float(delta),
                    directions,
                    offsets,
                    basis,
                    origin,
                )
                coarse_curve.append(
                    {
                        "delta": float(delta),
                        "mesh_loss_m": mesh_loss_to_tree(predicted, tree),
                    }
                )
            coarse = min(coarse_curve, key=lambda item: item["mesh_loss_m"])
            fine_curve = []
            for delta in np.arange(
                coarse["delta"] - 15.0,
                coarse["delta"] + 15.0001,
                0.5,
            ):
                predicted = posed_mesh(
                    raw_points,
                    frame,
                    4,
                    float(delta),
                    directions,
                    offsets,
                    basis,
                    origin,
                )
                fine_curve.append(
                    {
                        "delta": float(delta),
                        "mesh_loss_m": mesh_loss_to_tree(predicted, tree),
                    }
                )
            best = min(fine_curve, key=lambda item: item["mesh_loss_m"])
            local_basin = [
                item
                for item in fine_curve
                if item["mesh_loss_m"]
                <= best["mesh_loss_m"] * WRIST_ROLL_LOCAL_BASIN_LOSS_RATIO
            ]
            basin_minimum = min(item["delta"] for item in local_basin)
            basin_maximum = max(item["delta"] for item in local_basin)
            basin_width = basin_maximum - basin_minimum
            basin_center = (basin_minimum + basin_maximum) * 0.5
            alternatives = [
                item
                for item in coarse_curve
                if abs(
                    _nearest_equivalent_degrees(
                        item["delta"],
                        best["delta"],
                    )
                    - best["delta"]
                )
                >= WRIST_ROLL_REMOTE_ALTERNATIVE_DEGREES
            ]
            alternative = min(
                alternatives,
                key=lambda item: item["mesh_loss_m"],
            )
            predicted = posed_mesh(
                raw_points,
                frame,
                4,
                basin_center,
                directions,
                offsets,
                basis,
                origin,
            )
            distances = tree.query(predicted, workers=-1)[0]
            strong_support_fraction = float(
                np.mean(distances < MAXIMUM_STRONG_MESH_DEPTH_LOSS_M)
            )
            partial_support_fraction = float(
                np.mean(
                    distances < PARTIAL_WRIST_ROLL_SUPPORT_DISTANCE_M
                )
            )
            peak_ratio = alternative["mesh_loss_m"] / max(
                1.0e-9,
                best["mesh_loss_m"],
            )
            evidence_class = wrist_roll_evidence_class(
                float(best["mesh_loss_m"]),
                strong_support_fraction,
                partial_support_fraction,
                float(peak_ratio),
                float(basin_width),
            )
            strong_visible = evidence_class == "strong_full_surface"
            visible = evidence_class != "invisible_or_ambiguous"
            support_for_consensus = (
                strong_support_fraction
                if strong_visible
                else partial_support_fraction
            )
            result = {
                "camera": camera_name,
                # Retain the old key for saved-evidence compatibility while
                # making clear that this may be an earlier settled joint pose.
                "baseline_frame_index": int(view["capture_frame_index"]),
                "capture_frame_index": int(view["capture_frame_index"]),
                "calibration_joint_index": int(
                    frame.get("calibration_joint_index", -2)
                ),
                "pose_wrist_roll_degrees": float(frame["pose"][4]),
                "offset_delta_degrees": float(basin_center),
                "mesh_loss_m": float(best["mesh_loss_m"]),
                "mapped_surface_support_fraction": strong_support_fraction,
                "partial_surface_support_distance_m": (
                    PARTIAL_WRIST_ROLL_SUPPORT_DISTANCE_M
                ),
                "partial_surface_support_fraction": partial_support_fraction,
                "local_orientation_basin_width_degrees": float(basin_width),
                "remote_alternative_delta_degrees": float(
                    alternative["delta"]
                ),
                "remote_alternative_mesh_loss_m": float(
                    alternative["mesh_loss_m"]
                ),
                "mesh_peak_ratio": float(peak_ratio),
                "evidence_class": evidence_class,
                "support_for_consensus": support_for_consensus,
                "visible_stock_gripper": visible,
            }
            all_view_results.append(result)
            if visible:
                accepted_by_camera.setdefault(camera_name, []).append(result)

    missing_cameras = sorted(
        set(candidates_by_camera) - set(accepted_by_camera)
    )
    if missing_cameras:
        raise ValueError(
            "wrist-roll zero lacks unique stock-gripper evidence from "
            + ", ".join(missing_cameras)
        )
    if (
        len(accepted_by_camera)
        < MINIMUM_WRIST_ROLL_VALIDATING_CAMERAS
    ):
        raise ValueError(
            "wrist-roll zero did not validate in both RealSense cameras"
        )

    candidate_summary = {
        camera_name: [
            {
                "delta": float(result["offset_delta_degrees"]),
                "loss": float(result["mesh_loss_m"]),
                "class": str(result["evidence_class"]),
                "frame": int(result["capture_frame_index"]),
                "joint": int(result["calibration_joint_index"]),
            }
            for result in camera_results
        ]
        for camera_name, camera_results in accepted_by_camera.items()
    }
    try:
        accepted, unwrapped, disagreement = (
            select_wrist_roll_cross_camera_pair(accepted_by_camera)
        )
    except ValueError:
        raise ValueError(
            "wrist-roll has no agreeing cross-camera view pair "
            f"(accepted_candidates={candidate_summary})"
        )
    weights = np.asarray(
        [
            float(result["support_for_consensus"])
            * (
                1.0
                if result["evidence_class"] == "strong_full_surface"
                else 0.5
            )
            / max(1.0e-6, float(result["mesh_loss_m"]))
            for result in accepted
        ],
        dtype=float,
    )
    consensus = float(np.average(np.asarray(unwrapped), weights=weights))
    return {
        "joint_index": 4,
        "joint": JOINT_NAMES[4],
        "method": "stationary_stock_gripper_surface_orientation",
        "mapped_link": "gripper_link",
        "unmapped_camera_attachment_excluded": True,
        "camera_attachment_topology": "single_rigid_piece",
        "camera_attachment_fit_policy": "physically_kept_opposite_d455",
        "camera_attachment_model_used": False,
        "offset_delta_degrees": consensus,
        "camera_results": accepted,
        "all_camera_view_results": all_view_results,
        "visible_view_count": len(accepted),
        "validating_camera_count": len(accepted_by_camera),
        "validating_cameras": sorted(accepted_by_camera),
        "two_camera_validation_required": True,
        "visible_view_offset_disagreement_degrees": disagreement,
        "cross_camera_pair_selection": "strong_class_then_total_mesh_loss",
        "preserved_joint_direction": float(directions[4]),
    }


def multiangle_distal_wrist_roll_camera_fit(
    frames: list[dict],
    camera_index: int,
    fitted_direction: float,
    distal_points: np.ndarray,
    moving_jaw_points: np.ndarray,
    wrist_camera_mount_points: np.ndarray,
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> dict:
    """Fit one roll offset across every dedicated angle in one camera."""
    camera_name = str(frames[0]["full_camera_points"][camera_index]["name"])
    observed_clouds = []
    attachment_excluded_points_per_pose = []
    for frame in frames:
        observed = np.asarray(
            frame["full_camera_points"][camera_index]["points"],
            dtype=float,
        )
        if len(wrist_camera_mount_points) > 0:
            # Legacy/offline captures may still provide a measured attachment
            # mesh. Mask its full roll envelope because wrist zero is unknown.
            mount_envelope = np.concatenate(
                [
                    posed_mesh(
                        wrist_camera_mount_points[::3],
                        frame,
                        4,
                        float(delta),
                        directions,
                        offsets,
                        basis,
                        origin,
                    )
                    for delta in np.arange(
                        -180.0,
                        180.0,
                        WRIST_CAMERA_MOUNT_ENVELOPE_STEP_DEGREES,
                    )
                ],
                axis=0,
            )
            mount_tree = cKDTree(mount_envelope)
            near_mount = (
                mount_tree.query(observed, workers=-1)[0]
                <= WRIST_CAMERA_MOUNT_EXCLUSION_DISTANCE_M
            )
        else:
            # The live one-button sweep deliberately keeps the rigid,
            # unmodelled camera assembly behind the claw throughout these
            # samples. It is physically absent from the D455-facing stock-jaw
            # surface, so no guessed geometry should be subtracted.
            near_mount = np.zeros(len(observed), dtype=bool)
        filtered = observed[~near_mount]
        if len(filtered) < 500:
            raise ValueError(
                "wrist-camera bracket exclusion left too little claw depth "
                f"in {camera_name}"
            )
        observed_clouds.append(filtered)
        attachment_excluded_points_per_pose.append(
            int(np.count_nonzero(near_mount))
        )
    trees = [cKDTree(points) for points in observed_clouds]
    candidate_directions = list(directions)
    candidate_directions[4] = fitted_direction
    coarse_points = distal_points[
        ::max(1, len(distal_points) // 900)
    ]

    coarse_moving_jaw_points = moving_jaw_points[
        ::max(1, len(moving_jaw_points) // 900)
    ]

    def score(
        candidate_offset: float,
        points: np.ndarray,
        jaw_points: np.ndarray,
    ) -> dict:
        candidate_offsets = list(offsets)
        candidate_offsets[4] = candidate_offset
        losses = []
        moving_jaw_losses = []
        orientation_losses = []
        strong_supports = []
        partial_supports = []
        for frame, tree in zip(frames, trees, strict=True):
            predicted = posed_mesh(
                points,
                frame,
                4,
                0.0,
                candidate_directions,
                candidate_offsets,
                basis,
                origin,
            )
            distances = tree.query(predicted, workers=-1)[0]
            fixed_loss = mesh_loss_to_tree(predicted, tree)
            moving_jaw = posed_moving_jaw(
                jaw_points,
                frame,
                candidate_directions,
                candidate_offsets,
                basis,
                origin,
            )
            moving_jaw_loss = mesh_loss_to_tree(moving_jaw, tree)
            losses.append(fixed_loss)
            moving_jaw_losses.append(moving_jaw_loss)
            orientation_losses.append(fixed_loss)
            strong_supports.append(
                float(
                    np.mean(
                        distances < MAXIMUM_STRONG_MESH_DEPTH_LOSS_M
                    )
                )
            )
            partial_supports.append(
                float(
                    np.mean(
                        distances
                        < PARTIAL_WRIST_ROLL_SUPPORT_DISTANCE_M
                    )
                )
            )
        exposed_jaw_losses = sorted(moving_jaw_losses)[:2]
        fixed_surface_loss = float(np.median(orientation_losses))
        exposed_jaw_loss = float(np.mean(exposed_jaw_losses))
        return {
            "fitted_offset_degrees": float(candidate_offset),
            "mesh_loss_m": float(np.median(losses)),
            "moving_jaw_mesh_loss_m": float(
                np.median(moving_jaw_losses)
            ),
            "orientation_selection_loss_m": float(
                fixed_surface_loss + 0.5 * exposed_jaw_loss
            ),
            "fixed_surface_orientation_loss_m": fixed_surface_loss,
            "exposed_moving_jaw_orientation_loss_m": exposed_jaw_loss,
            "mapped_surface_support_fraction": float(
                np.median(strong_supports)
            ),
            "partial_surface_support_fraction": float(
                np.median(partial_supports)
            ),
            "per_pose_mesh_losses_m": [float(value) for value in losses],
            "per_pose_moving_jaw_mesh_losses_m": [
                float(value) for value in moving_jaw_losses
            ],
            "per_pose_strong_support_fractions": strong_supports,
            "per_pose_partial_support_fractions": partial_supports,
        }

    coarse_curve = [
        score(
            float(candidate),
            coarse_points,
            coarse_moving_jaw_points,
        )
        for candidate in np.arange(-180.0, 180.0, 5.0)
        if abs(
            _nearest_equivalent_degrees(
                float(candidate)
                + WRIST_ROLL_MODEL_TO_JOINT_ZERO_DEGREES,
                float(offsets[4]),
            )
            - float(offsets[4])
        )
        <= MAXIMUM_AUTOMATIC_WRIST_ROLL_CORRECTION_DEGREES
    ]
    coarse = min(
        coarse_curve,
        key=lambda item: item["orientation_selection_loss_m"],
    )
    seed = float(coarse["fitted_offset_degrees"])
    fine_curve = [
        score(float(candidate), distal_points, moving_jaw_points)
        for candidate in np.arange(seed - 10.0, seed + 10.0001, 0.5)
        if abs(
            _nearest_equivalent_degrees(
                float(candidate)
                + WRIST_ROLL_MODEL_TO_JOINT_ZERO_DEGREES,
                float(offsets[4]),
            )
            - float(offsets[4])
        )
        <= MAXIMUM_AUTOMATIC_WRIST_ROLL_CORRECTION_DEGREES
    ]
    best = min(
        fine_curve,
        key=lambda item: item["orientation_selection_loss_m"],
    )
    prior_raw_offset = _nearest_equivalent_degrees(
        float(offsets[4]) - WRIST_ROLL_MODEL_TO_JOINT_ZERO_DEGREES,
        float(best["fitted_offset_degrees"]),
    )
    prior_score = score(
        prior_raw_offset,
        distal_points,
        moving_jaw_points,
    )
    local_basin = [
        item
        for item in fine_curve
        if item["orientation_selection_loss_m"]
        <= best["orientation_selection_loss_m"]
        * WRIST_ROLL_LOCAL_BASIN_LOSS_RATIO
    ]
    basin_minimum = min(
        item["fitted_offset_degrees"] for item in local_basin
    )
    basin_maximum = max(
        item["fitted_offset_degrees"] for item in local_basin
    )
    basin_width = basin_maximum - basin_minimum
    alternatives = [
        item
        for item in coarse_curve
        if abs(
            _nearest_equivalent_degrees(
                item["fitted_offset_degrees"],
                best["fitted_offset_degrees"],
            )
            - best["fitted_offset_degrees"]
        )
        >= WRIST_ROLL_REMOTE_ALTERNATIVE_DEGREES
    ]
    alternative = min(
        alternatives,
        key=lambda item: item["orientation_selection_loss_m"],
    )
    peak_ratio = float(
        alternative["orientation_selection_loss_m"]
        / max(1.0e-9, best["orientation_selection_loss_m"])
    )
    evidence_class = wrist_roll_evidence_class(
        float(best["mesh_loss_m"]),
        float(best["mapped_surface_support_fraction"]),
        float(best["partial_surface_support_fraction"]),
        peak_ratio,
        float(basin_width),
    )
    raw_model_offset = float(best["fitted_offset_degrees"])
    corrected_joint_offset = (
        raw_model_offset + WRIST_ROLL_MODEL_TO_JOINT_ZERO_DEGREES
    )
    corrected_remote_offset = (
        float(alternative["fitted_offset_degrees"])
        + WRIST_ROLL_MODEL_TO_JOINT_ZERO_DEGREES
    )
    result = dict(best)
    result.update(
        {
            "camera": camera_name,
            "wrist_camera_mount_model_used": False,
            "camera_attachment_fit_policy": (
                "physically_kept_opposite_d455"
            ),
            "attachment_excluded_points_per_pose": (
                attachment_excluded_points_per_pose
            ),
            "attachment_excluded_point_count": int(
                sum(attachment_excluded_points_per_pose)
            ),
            "prior_raw_model_offset_degrees": float(prior_raw_offset),
            "prior_orientation_selection_loss_m": float(
                prior_score["orientation_selection_loss_m"]
            ),
            "orientation_loss_improvement_ratio": float(
                prior_score["orientation_selection_loss_m"]
                / max(1.0e-9, best["orientation_selection_loss_m"])
            ),
            "fitted_direction": float(fitted_direction),
            "raw_model_fitted_offset_degrees": raw_model_offset,
            "fitted_offset_degrees": corrected_joint_offset,
            "model_to_joint_zero_degrees": (
                WRIST_ROLL_MODEL_TO_JOINT_ZERO_DEGREES
            ),
            "local_orientation_basin_width_degrees": float(
                basin_width
            ),
            "remote_alternative_offset_degrees": corrected_remote_offset,
            "remote_alternative_mesh_loss_m": float(
                alternative["mesh_loss_m"]
            ),
            "remote_alternative_orientation_selection_loss_m": float(
                alternative["orientation_selection_loss_m"]
            ),
            "mesh_peak_ratio": peak_ratio,
            "evidence_class": evidence_class,
            "support_for_consensus": float(
                best["mapped_surface_support_fraction"]
                if evidence_class == "strong_full_surface"
                else best["partial_surface_support_fraction"] * 0.5
            ),
            "visible_stock_gripper": (
                evidence_class != "invisible_or_ambiguous"
            ),
            "moving_jaw_orientation_check": False,
            "model_convention_orientation_check": True,
            "moving_jaw_visible_for_orientation": (
                sum(
                    value <= MAXIMUM_WRIST_ROLL_MOVING_JAW_LOSS_M
                    for value in best[
                        "per_pose_moving_jaw_mesh_losses_m"
                    ]
                )
                >= 2
            ),
            "moving_jaw_visible_pose_count": sum(
                value <= MAXIMUM_WRIST_ROLL_MOVING_JAW_LOSS_M
                for value in best[
                    "per_pose_moving_jaw_mesh_losses_m"
                ]
            ),
            "maximum_moving_jaw_orientation_loss_m": (
                MAXIMUM_WRIST_ROLL_MOVING_JAW_LOSS_M
            ),
            "dedicated_pose_count": len(frames),
            "partial_surface_support_distance_m": (
                PARTIAL_WRIST_ROLL_SUPPORT_DISTANCE_M
            ),
        }
    )
    return result


def solve_wrist_roll_zero_from_stock_gripper(
    frames: list[dict],
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
    locked_direction: float | None = None,
) -> dict:
    """Fit in stock-mesh coordinates, then map zero to the Godot joint."""
    groups = observation_groups(frames, 4)
    camera_indices = reference_camera_indices(frames)
    if (
        len(groups) < MINIMUM_WRIST_ROLL_MULTIANGLE_VIEWPOINTS
        or any(
            len(group)
            < MINIMUM_WRIST_ROLL_MULTIANGLE_POSES_PER_VIEWPOINT
            for group in groups
        )
    ):
        raise ValueError(
            "wrist-roll sign/zero requires five angles from two viewpoints"
        )
    dedicated_frames = [frame for group in groups for frame in group]
    if any(
        len(frame.get("full_camera_points", [])) <= camera_indices[0]
        for frame in dedicated_frames
    ):
        raise ValueError(
            "wrist-roll sign/zero requires the reference D455 camera"
        )

    o3d.utility.random.seed(51)
    mesh = o3d.io.read_triangle_mesh(str(ASSET_ROOT / "gripper_link.glb"))
    sampled = np.asarray(mesh.sample_points_uniformly(12000).points)
    distal_points = sampled[
        sampled[:, 2] <= DISTAL_FIXED_JAW_MAXIMUM_LOCAL_Z_M
    ]
    if len(distal_points) < 1000:
        raise ValueError("distal fixed-jaw mesh could not be sampled")
    moving_jaw_mesh = o3d.io.read_triangle_mesh(
        str(ASSET_ROOT / "moving_jaw_so101_v1_link.glb")
    )
    moving_jaw_points = np.asarray(
        moving_jaw_mesh.sample_points_uniformly(8000).points
    )
    if len(moving_jaw_points) < 1000:
        raise ValueError("moving-jaw mesh could not be sampled")
    # The live sweep fixes roll to the D455-exposed negative range before
    # these observations, keeping the rigid camera assembly behind the stock
    # jaws. Do not subtract an approximate attachment mesh from a surface it
    # does not occupy.
    wrist_camera_mount_points = np.empty((0, 3), dtype=float)

    direction_results = []
    valid_directions = []
    candidate_directions = (
        (float(locked_direction),)
        if locked_direction is not None
        else (-1.0, 1.0)
    )
    for candidate_direction in candidate_directions:
        viewpoint_results = []
        valid_viewpoints = []
        for group_index, group in enumerate(groups):
            camera_results = [
                multiangle_distal_wrist_roll_camera_fit(
                    group,
                    camera_index,
                    candidate_direction,
                    distal_points,
                    moving_jaw_points,
                    wrist_camera_mount_points,
                    directions,
                    offsets,
                    basis,
                    origin,
                )
                for camera_index in camera_indices
            ]
            reference = float(
                camera_results[0]["fitted_offset_degrees"]
            )
            unwrapped_offsets = [
                _nearest_equivalent_degrees(
                    float(result["fitted_offset_degrees"]),
                    reference,
                )
                for result in camera_results
            ]
            disagreement = max(unwrapped_offsets) - min(
                unwrapped_offsets
            )
            visible = all(
                result["visible_stock_gripper"]
                for result in camera_results
            )
            # A viewpoint can have a rotationally symmetric remote basin yet
            # still be excellent corroborating evidence once another view
            # supplies absolute orientation. Keep this narrow: low mesh loss,
            # strong mapped support, a narrow local basin, and a physically
            # visible moving jaw are all required.
            corroborating = all(
                float(result["mesh_loss_m"])
                <= MAXIMUM_WRIST_ROLL_MESH_LOSS_M
                and float(result["mapped_surface_support_fraction"])
                >= MINIMUM_WRIST_ROLL_MESH_SUPPORT_FRACTION
                and float(result["local_orientation_basin_width_degrees"])
                <= MAXIMUM_WRIST_ROLL_LOCAL_BASIN_WIDTH_DEGREES
                and bool(result["moving_jaw_visible_for_orientation"])
                for result in camera_results
            )
            orientation_cameras = [
                result
                for result in camera_results
                if result["moving_jaw_visible_for_orientation"]
            ]
            orientation_camera = (
                min(
                    orientation_cameras,
                    key=lambda result: result[
                        "moving_jaw_mesh_loss_m"
                    ],
                )
                if orientation_cameras
                else None
            )
            viewpoint = {
                "group_index": group_index,
                "view_pan_degrees": float(
                    np.median([frame["pose"][0] for frame in group])
                ),
                "camera_results": camera_results,
                "unwrapped_camera_offsets_degrees": unwrapped_offsets,
                "camera_offset_disagreement_degrees": disagreement,
                "combined_mesh_loss_m": float(
                    sum(
                        result["orientation_selection_loss_m"]
                        for result in camera_results
                    )
                ),
                "visible_in_both_cameras": visible,
                "corroborating_stock_gripper": corroborating,
                "orientation_camera": orientation_camera,
                "moving_jaw_orientation_check": (
                    orientation_camera is not None
                ),
                "pose_count": len(group),
            }
            viewpoint_results.append(viewpoint)
            if (
                (visible or corroborating)
                and orientation_camera is not None
                and disagreement
                <= MAXIMUM_WRIST_ROLL_VIEW_DISAGREEMENT_DEGREES
            ):
                valid_viewpoints.append(viewpoint)

        valid_viewpoints.sort(
            key=lambda result: (
                sum(
                    camera["evidence_class"]
                    != "strong_full_surface"
                    for camera in result["camera_results"]
                ),
                result["combined_mesh_loss_m"],
                result["camera_offset_disagreement_degrees"],
            )
        )
        (
            selected_viewpoint_pair,
            viewpoint_offsets,
            viewpoint_disagreement,
        ) = select_wrist_roll_agreeing_viewpoint_pair(valid_viewpoints)
        # A single apparently good view can lock onto the rigid camera bracket
        # or nearby clutter.  The sweep deliberately captures two D455
        # viewpoints; both must independently expose the stock jaws and agree
        # on zero before this result is allowed to overwrite a saved fit.
        selected_viewpoint = (
            selected_viewpoint_pair[0]
            if len(selected_viewpoint_pair)
            >= MINIMUM_WRIST_ROLL_MULTIANGLE_VIEWPOINTS
            else None
        )
        direction_result = {
            "fitted_direction": candidate_direction,
            "viewpoint_results": viewpoint_results,
            "selected_viewpoint": selected_viewpoint,
            "valid_viewpoint_count": len(valid_viewpoints),
            "valid_viewpoint_offsets_degrees": viewpoint_offsets,
            "valid_viewpoint_offset_disagreement_degrees": (
                viewpoint_disagreement
            ),
            "selected_viewpoint_indices": [
                int(viewpoint["group_index"])
                for viewpoint in selected_viewpoint_pair
            ],
            "selected_viewpoint_consensus_degrees": (
                float(np.median(viewpoint_offsets))
                if viewpoint_offsets
                else math.inf
            ),
        }
        direction_results.append(direction_result)
        if selected_viewpoint is not None:
            flattened = dict(direction_result)
            flattened.update(selected_viewpoint)
            valid_directions.append(flattened)

    if not valid_directions:
        diagnostics = [
            {
                "direction": result["fitted_direction"],
                "viewpoints": [
                    {
                        "group": viewpoint["group_index"],
                        "offsets": viewpoint[
                            "unwrapped_camera_offsets_degrees"
                        ],
                        "disagreement": viewpoint[
                            "camera_offset_disagreement_degrees"
                        ],
                        "classes": [
                            camera["evidence_class"]
                            for camera in viewpoint["camera_results"]
                        ],
                    }
                    for viewpoint in result["viewpoint_results"]
                ],
            }
            for result in direction_results
        ]
        raise ValueError(
            "wrist-roll sign/zero has no unambiguous D455 solution "
            f"({diagnostics})"
        )
    valid_directions.sort(
        key=lambda result: (
            sum(
                camera["evidence_class"]
                != "strong_full_surface"
                for camera in result["camera_results"]
            ),
            result["combined_mesh_loss_m"],
            result["camera_offset_disagreement_degrees"],
        )
    )
    if len(valid_directions) > 1:
        best = valid_directions[0]
        alternative = valid_directions[1]
        if (
            alternative["combined_mesh_loss_m"]
            / max(1.0e-9, best["combined_mesh_loss_m"])
            < MINIMUM_WRIST_ROLL_MESH_PEAK_RATIO
        ):
            raise ValueError(
                "wrist-roll encoder direction remains ambiguous "
                f"({[(item['fitted_direction'], item['combined_mesh_loss_m']) for item in valid_directions]})"
            )

    selected = valid_directions[0]
    camera_results = selected["camera_results"]
    orientation_camera = selected["orientation_camera"]
    if orientation_camera is None:
        raise ValueError("wrist-roll has no visible moving-jaw orientation")
    consensus = float(
        selected["selected_viewpoint_consensus_degrees"]
    )
    current_offset = float(offsets[4])
    consensus = _nearest_equivalent_degrees(
        consensus,
        current_offset,
    )
    return {
        "joint_index": 4,
        "joint": JOINT_NAMES[4],
        "method": (
            "multiangle_distal_stock_gripper_sign_and_zero"
        ),
        "mapped_link": "gripper_link",
        "mapped_region": "distal_fixed_jaw",
        "orientation_disambiguation_region": (
            "stock_mesh_to_articulated_joint_convention"
        ),
        "moving_jaw_orientation_check": True,
        "model_convention_orientation_check": True,
        "model_to_joint_zero_degrees": (
            WRIST_ROLL_MODEL_TO_JOINT_ZERO_DEGREES
        ),
        "distal_fixed_jaw_maximum_local_z_m": (
            DISTAL_FIXED_JAW_MAXIMUM_LOCAL_Z_M
        ),
        "unmapped_camera_attachment_excluded": True,
        "camera_attachment_topology": "single_rigid_piece",
        "camera_attachment_fit_policy": "physically_kept_opposite_d455",
        "camera_attachment_model_used": False,
        "fitted_direction": float(selected["fitted_direction"]),
        "fitted_offset_degrees": consensus,
        "offset_delta_degrees": consensus - current_offset,
        "camera_results": camera_results,
        "all_direction_results": direction_results,
        "visible_view_count": int(selected["valid_viewpoint_count"]),
        "validating_camera_count": len(camera_results),
        "validating_cameras": sorted(
            result["camera"] for result in camera_results
        ),
        "two_camera_validation_required": False,
        "reference_camera_validation_required": True,
        "reference_camera_serial": REFERENCE_CAMERA_SERIAL,
        "visible_view_offset_disagreement_degrees": float(
            selected[
                "valid_viewpoint_offset_disagreement_degrees"
            ]
        ),
        "selected_viewpoint_indices": selected[
            "selected_viewpoint_indices"
        ],
        "orientation_camera": str(orientation_camera["camera"]),
        "orientation_camera_moving_jaw_mesh_loss_m": float(
            orientation_camera["moving_jaw_mesh_loss_m"]
        ),
        "zero_consensus_method": (
            "reference_camera_visible_moving_jaw_and_fixed_model_convention"
        ),
        "multiangle_pose_count": len(dedicated_frames),
        "multiangle_viewpoint_count": len(groups),
        "selected_viewpoint_index": int(selected["group_index"]),
        "selected_viewpoint_pose_count": int(
            selected["pose_count"]
        ),
        "selected_view_pan_degrees": float(
            selected["view_pan_degrees"]
        ),
        "preserved_joint_direction": locked_direction is not None,
        "direction_locked_from_dual_camera_motion": (
            locked_direction is not None
        ),
        "direction_locked_from_reference_camera_motion": (
            locked_direction is not None
        ),
    }


def solve_wrist_roll_zero_to_fixed_point(
    frames: list[dict],
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
    locked_direction: float | None = None,
) -> tuple[list[float], list[float], dict]:
    initial_directions = list(directions)
    initial_offsets = list(offsets)
    candidate = solve_wrist_roll_zero_from_stock_gripper(
        frames,
        initial_directions,
        initial_offsets,
        basis,
        origin,
        locked_direction,
    )
    fitted_directions = list(initial_directions)
    fitted_offsets = list(initial_offsets)
    fitted_directions[4] = float(candidate["fitted_direction"])
    fitted_offsets[4] = float(candidate["fitted_offset_degrees"])
    correction = float(candidate["offset_delta_degrees"])
    fixed_point_history = []
    validation = candidate
    fixed_point_delta = math.inf
    for fixed_point_pass in range(1, MAXIMUM_CONVERGENCE_PASSES + 1):
        validation = solve_wrist_roll_zero_from_stock_gripper(
            frames,
            fitted_directions,
            fitted_offsets,
            basis,
            origin,
            locked_direction,
        )
        if float(validation["fitted_direction"]) != fitted_directions[4]:
            raise ValueError(
                "wrist-roll direction did not reach a fixed point "
                f"(initial={fitted_directions[4]}, "
                f"validation={validation['fitted_direction']})"
            )
        fixed_point_delta = float(validation["offset_delta_degrees"])
        fixed_point_history.append(
            {
                "pass": fixed_point_pass,
                "offset_delta_degrees": fixed_point_delta,
                "fitted_offset_degrees": float(
                    validation["fitted_offset_degrees"]
                ),
            }
        )
        if abs(fixed_point_delta) <= MAXIMUM_FIXED_POINT_DELTA_DEGREES:
            break
        fitted_offsets[4] = float(validation["fitted_offset_degrees"])
    else:
        raise ValueError(
            "wrist-roll zero did not reach a fixed point "
            f"(initial={correction}, history={fixed_point_history})"
        )
    validation["initial_correction_degrees"] = correction
    camera_results = candidate.get("camera_results", [])
    if camera_results:
        initial_orientation_fit = camera_results[0]
        optional_diagnostics = {
            "initial_prior_orientation_selection_loss_m": (
                "prior_orientation_selection_loss_m",
                float,
            ),
            "initial_fitted_orientation_selection_loss_m": (
                "orientation_selection_loss_m",
                float,
            ),
            "initial_orientation_loss_improvement_ratio": (
                "orientation_loss_improvement_ratio",
                float,
            ),
            "initial_attachment_excluded_point_count": (
                "attachment_excluded_point_count",
                int,
            ),
        }
        for output_key, (source_key, converter) in optional_diagnostics.items():
            if source_key in initial_orientation_fit:
                validation[output_key] = converter(
                    initial_orientation_fit[source_key]
                )
    validation["fixed_point_delta_degrees"] = fixed_point_delta
    validation["fixed_point_history"] = fixed_point_history
    validation["offset_delta_degrees"] = float(
        fitted_offsets[4] - initial_offsets[4]
    )
    validation["initial_direction"] = float(initial_directions[4])
    validation["fitted_direction"] = float(fitted_directions[4])
    validation["direction_changed"] = (
        fitted_directions[4] != initial_directions[4]
    )
    validation["fitted_offset_degrees"] = float(fitted_offsets[4])
    return fitted_directions, fitted_offsets, validation


def solve_wrist_roll_direction_from_motion(
    frames: list[dict],
    directions: list[float],
    offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
) -> dict:
    """Infer wrist-roll encoder sign by re-aligning repeated physical motion."""
    candidates = []
    rejected = {}
    for candidate_direction in (-1.0, 1.0):
        candidate_directions = list(directions)
        candidate_directions[4] = candidate_direction
        try:
            flex = solve_wrist_flex_from_direct_axis_collapse(
                frames,
                candidate_directions,
                offsets,
                basis,
                origin,
            )
        except ValueError as error:
            rejected[str(candidate_direction)] = str(error)
            continue
        total_loss = float(
            sum(
                result["collapse_loss_m"]
                for result in flex["camera_results"]
            )
        )
        candidates.append(
            {
                "fitted_direction": candidate_direction,
                "combined_collapse_loss_m": total_loss,
                "candidate_wrist_flex_delta_degrees": float(
                    flex["offset_delta_degrees"]
                ),
                "camera_offset_disagreement_degrees": float(
                    flex["camera_offset_disagreement_degrees"]
                ),
                "camera_results": flex["camera_results"],
            }
        )
    if not candidates:
        raise ValueError(
            "wrist-roll encoder direction has no non-contradicting "
            f"D455 motion view ({rejected})"
        )
    candidates.sort(
        key=lambda result: result["combined_collapse_loss_m"]
    )
    if len(candidates) > 1:
        best, alternative = candidates[:2]
        ratio = (
            alternative["combined_collapse_loss_m"]
            / max(1.0e-9, best["combined_collapse_loss_m"])
        )
        if ratio < MINIMUM_WRIST_ROLL_MESH_PEAK_RATIO:
            raise ValueError(
                "wrist-roll encoder direction remains ambiguous in "
                f"D455 motion ({[(item['fitted_direction'], item['combined_collapse_loss_m']) for item in candidates]})"
            )
    selected = dict(candidates[0])
    selected.update(
        {
            "method": (
                "reference_camera_repeated_motion_collapse_direction"
            ),
            "candidate_results": candidates,
            "rejected_candidates": rejected,
            "validating_camera_count": 1,
            "two_camera_validation_required": False,
            "reference_camera_validation_required": True,
            "reference_camera_serial": REFERENCE_CAMERA_SERIAL,
        }
    )
    return selected


def solve_outward_chain_with_coupled_wrist_sign(
    frames: list[dict],
    through_joint: int,
    initial_directions: list[float],
    initial_offsets: list[float],
    basis: np.ndarray,
    origin: np.ndarray,
    prior_wrist_evidence: dict | None = None,
) -> tuple[list[float], list[float], list[dict], dict, dict]:
    """Reach a fixed point between wrist flex and wrist roll sign/zero.

    Wrist-flex mesh fitting depends on the roll pose.  Therefore a roll
    direction discovered after the outward solve must never be committed with
    flex values fitted under the opposite sign.  Refit the outward chain from
    the original mechanical zeros, seeded only with the newly fitted roll
    zero, until the direction and roll seed are both stable.
    """
    directions = list(initial_directions)
    preliminary_wrist_roll = {}
    if through_joint == 3:
        preliminary_wrist_roll = solve_wrist_roll_direction_from_motion(
            frames,
            directions,
            initial_offsets,
            basis,
            origin,
        )
        directions[4] = float(
            preliminary_wrist_roll["fitted_direction"]
        )

    roll_seed = float(initial_offsets[4])
    coupling_history = []
    maximum_coupling_passes = 3
    for coupling_pass in range(1, maximum_coupling_passes + 1):
        outward_seed = list(initial_offsets)
        outward_seed[4] = roll_seed
        outward_directions = list(directions)
        offsets, results, convergence = (
            solve_outward_chain_to_fixed_point(
                frames,
                through_joint,
                outward_directions,
                outward_seed,
                basis,
                origin,
            )
        )
        wrist_roll_zero = {}
        if through_joint != 3:
            return (
                outward_directions,
                offsets,
                results,
                convergence,
                wrist_roll_zero,
            )

        try:
            fitted_directions, fitted_offsets, wrist_roll_zero = (
                solve_wrist_roll_zero_to_fixed_point(
                    frames,
                    outward_directions,
                    offsets,
                    basis,
                    origin,
                    locked_direction=float(outward_directions[4]),
                )
            )
        except ValueError as orientation_error:
            prior = prior_wrist_evidence or {}
            if not validated_prior_wrist_zero_evidence(prior):
                raise orientation_error
            preserved_offset = float(initial_offsets[4])
            wrist_roll_zero = {
                "joint_index": 4,
                "joint": JOINT_NAMES[4],
                "method": (
                    "preserved_validated_prior_wrist_zero_after_d455_motion_sign_check"
                ),
                "mapped_link": "gripper_link",
                "mapped_region": "distal_fixed_jaw",
                "unmapped_camera_attachment_excluded": True,
                "camera_attachment_topology": "single_rigid_piece",
                "camera_attachment_fit_policy": (
                    "physically_kept_opposite_d455"
                ),
                "camera_attachment_model_used": False,
                "fresh_orientation_ambiguous": True,
                "fresh_orientation_rejection": str(orientation_error),
                "orientation_preserved_not_reestimated": True,
                "prior_validation": dict(prior),
                "moving_jaw_orientation_check": True,
                "model_convention_orientation_check": True,
                "fitted_direction": float(outward_directions[4]),
                "fitted_offset_degrees": preserved_offset,
                "offset_delta_degrees": 0.0,
                "fixed_point_delta_degrees": 0.0,
                "camera_results": preliminary_wrist_roll[
                    "camera_results"
                ],
                "validating_camera_count": 1,
                "validating_cameras": [
                    result["camera"]
                    for result in preliminary_wrist_roll[
                        "camera_results"
                    ]
                ],
                "reference_camera_validation_required": True,
                "reference_camera_serial": REFERENCE_CAMERA_SERIAL,
                "direction_locked_from_reference_camera_motion": True,
                "coupling_converged": True,
                "coupling_history": [
                    {
                        "pass": coupling_pass,
                        "outward_wrist_direction": float(
                            outward_directions[4]
                        ),
                        "fitted_wrist_direction": float(
                            outward_directions[4]
                        ),
                        "roll_seed_degrees": preserved_offset,
                        "fitted_roll_zero_degrees": preserved_offset,
                        "roll_seed_residual_degrees": 0.0,
                        "direction_stable": True,
                        "wrist_flex_offset_degrees": float(offsets[3]),
                        "preserved_validated_prior_zero": True,
                    }
                ],
                "preliminary_direction_fit": {
                    "method": preliminary_wrist_roll["method"],
                    "fitted_direction": preliminary_wrist_roll[
                        "fitted_direction"
                    ],
                    "candidate_wrist_flex_delta_degrees": (
                        preliminary_wrist_roll[
                            "candidate_wrist_flex_delta_degrees"
                        ]
                    ),
                },
            }
            fitted_offsets = list(offsets)
            fitted_offsets[4] = preserved_offset
            return (
                outward_directions,
                fitted_offsets,
                results,
                convergence,
                wrist_roll_zero,
            )
        fitted_direction = float(fitted_directions[4])
        fitted_roll_zero = float(fitted_offsets[4])
        direction_stable = (
            fitted_direction == float(outward_directions[4])
        )
        roll_seed_residual = (
            fitted_roll_zero - float(roll_seed)
        )
        coupling_history.append(
            {
                "pass": coupling_pass,
                "outward_wrist_direction": float(
                    outward_directions[4]
                ),
                "fitted_wrist_direction": fitted_direction,
                "roll_seed_degrees": float(roll_seed),
                "fitted_roll_zero_degrees": fitted_roll_zero,
                "roll_seed_residual_degrees": float(
                    roll_seed_residual
                ),
                "direction_stable": direction_stable,
                "wrist_flex_offset_degrees": float(
                    fitted_offsets[3]
                ),
            }
        )
        if (
            direction_stable
            and abs(roll_seed_residual)
            <= MAXIMUM_FIXED_POINT_DELTA_DEGREES
        ):
            wrist_roll_zero["coupling_history"] = coupling_history
            wrist_roll_zero["coupling_converged"] = True
            wrist_roll_zero["preliminary_direction_fit"] = {
                "method": preliminary_wrist_roll["method"],
                "fitted_direction": preliminary_wrist_roll[
                    "fitted_direction"
                ],
                "camera_offset_disagreement_degrees": (
                    preliminary_wrist_roll[
                        "camera_offset_disagreement_degrees"
                    ]
                ),
                "candidate_wrist_flex_delta_degrees": (
                    preliminary_wrist_roll[
                        "candidate_wrist_flex_delta_degrees"
                    ]
                ),
            }
            return (
                fitted_directions,
                fitted_offsets,
                results,
                convergence,
                wrist_roll_zero,
            )
        directions = fitted_directions
        roll_seed = fitted_roll_zero

    raise ValueError(
        "wrist flex/roll sign coupling did not reach a fixed point "
        f"(history={coupling_history})"
    )


def validated_prior_wrist_zero_evidence(prior: dict) -> bool:
    """Return whether a saved intrinsic wrist zero is safe to preserve.

    A relocated base changes the camera/world transform, not the encoder zero.
    Preservation is allowed only after the fresh D455 sweep has independently
    locked motion direction and fresh stock orientation remains ambiguous.
    """
    if not isinstance(prior, dict):
        return False
    method = str(prior.get("method", ""))
    common_fixed_point = (
        abs(float(prior.get("fixed_point_delta_degrees", math.inf)))
        <= MAXIMUM_FIXED_POINT_DELTA_DEGREES
        and bool(prior.get("moving_jaw_orientation_check", False))
    )
    if method == "dual_realsense_native_rgb_tip_projection":
        return common_fixed_point and int(
            prior.get("validating_camera_count", 0)
        ) >= 2
    if (
        method
        == "preserved_validated_prior_wrist_zero_after_d455_motion_sign_check"
    ):
        return (
            common_fixed_point
            and bool(prior.get("orientation_preserved_not_reestimated", False))
            and validated_prior_wrist_zero_evidence(
                prior.get("prior_validation", {})
            )
        )
    if method != "multiangle_distal_stock_gripper_sign_and_zero":
        return False
    if not (
        common_fixed_point
        and bool(prior.get("model_convention_orientation_check", False))
        and bool(prior.get("coupling_converged", False))
        and bool(prior.get("reference_camera_validation_required", False))
        and str(prior.get("reference_camera_serial", ""))
        == REFERENCE_CAMERA_SERIAL
        and int(prior.get("validating_camera_count", 0)) >= 1
        and int(prior.get("multiangle_pose_count", 0)) >= 10
        and int(prior.get("multiangle_viewpoint_count", 0)) >= 2
    ):
        return False
    camera_results = prior.get("camera_results", [])
    reference_results = [
        result
        for result in camera_results
        if isinstance(result, dict)
        and REFERENCE_CAMERA_SERIAL in str(result.get("camera", ""))
    ]
    if not reference_results:
        return False
    for result in reference_results:
        evidence_class = str(result.get("evidence_class", ""))
        strong = (
            evidence_class == "strong_full_surface"
            and float(result.get("mesh_loss_m", math.inf)) <= 0.018
            and float(result.get("mapped_surface_support_fraction", 0.0))
            >= MINIMUM_WRIST_ROLL_MESH_SUPPORT_FRACTION
        )
        partial = (
            evidence_class == "unique_partial_surface"
            and float(result.get("mesh_loss_m", math.inf)) <= 0.025
            and float(
                result.get(
                    "partial_surface_support_distance_m",
                    math.inf,
                )
            )
            <= PARTIAL_WRIST_ROLL_SUPPORT_DISTANCE_M
            and float(result.get("partial_surface_support_fraction", 0.0))
            >= MINIMUM_PARTIAL_WRIST_ROLL_MESH_SUPPORT_FRACTION
        )
        if (
            bool(result.get("visible_stock_gripper", False))
            and int(result.get("moving_jaw_visible_pose_count", 0)) >= 2
            and (strong or partial)
            and float(result.get("mesh_peak_ratio", 0.0))
            >= MINIMUM_WRIST_ROLL_MESH_PEAK_RATIO
            and float(
                result.get(
                    "local_orientation_basin_width_degrees",
                    math.inf,
                )
            )
            <= MAXIMUM_WRIST_ROLL_LOCAL_BASIN_WIDTH_DEGREES
        ):
            return True
    return False


def main() -> int:
    global REFERENCE_CAMERA_SERIAL

    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("registration", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--through-joint", type=int, choices=(1, 2, 3), default=3)
    parser.add_argument(
        "--wrist-only",
        action="store_true",
        help="Fit only wrist-roll direction/zero and preserve all other servo mappings.",
    )
    parser.add_argument(
        "--parent-transform",
        type=Path,
        help="Optional saved parent/world-level transform for a local registration",
    )
    parser.add_argument(
        "--base-fit",
        type=Path,
        help=(
            "Optional base-axis result whose global preview transform replaces "
            "the registration transform for offline transaction diagnostics"
        ),
    )
    args = parser.parse_args()
    capture = json.loads(args.capture.expanduser().read_text())
    REFERENCE_CAMERA_SERIAL = str(
        capture.get("reference_camera", REFERENCE_CAMERA_SERIAL)
    ).strip() or REFERENCE_CAMERA_SERIAL
    registration = json.loads(args.registration.expanduser().read_text())
    parent = (
        json.loads(args.parent_transform.expanduser().read_text())
        if args.parent_transform
        else None
    )
    frames = capture.get("frames", [])
    capture_mode = str(capture.get("calibration_mode", ""))
    wrist_samples = [
        frame
        for frame in frames
        if int(frame.get("calibration_joint_index", -1)) == 4
    ]
    compatible_capture = (
        capture_mode == "joints"
        if not args.wrist_only
        else capture_mode in ("wrist", "joints") and len(wrist_samples) >= 8
    )
    if not compatible_capture:
        raise SystemExit("capture is not a staged joint sweep")
    if args.base_fit:
        base_fit = json.loads(args.base_fit.expanduser().read_text())
        preview = base_fit.get("preview_transform", {})
        basis, origin = compose_registration_with_parent(preview, None)
    else:
        basis, origin = compose_registration_with_parent(registration, parent)
    initial_directions = [
        float(value)
        for value in registration["joint_angle_directions"]
    ]
    initial_offsets = [
        float(value)
        for value in registration["joint_angle_offsets_degrees"]
    ]
    try:
        if args.wrist_only:
            directions, offsets, wrist_roll_zero = (
                solve_wrist_roll_zero_to_fixed_point(
                    frames,
                    initial_directions,
                    initial_offsets,
                    basis,
                    origin,
                )
            )
            results = []
            convergence = {
                "converged": True,
                "passes": 1,
                "wrist_only": True,
            }
        else:
            (
                directions,
                offsets,
                results,
                convergence,
                wrist_roll_zero,
            ) = solve_outward_chain_with_coupled_wrist_sign(
                frames,
                args.through_joint,
                initial_directions,
                initial_offsets,
                basis,
                origin,
                registration.get(
                    "prior_wrist_roll_zero_evidence",
                    registration.get("joint_refinement", {}).get(
                        "wrist_roll_zero",
                        {},
                    ),
                ),
            )
    except ValueError as error:
        print(f"SO101_STAGED_REJECTED {error}", file=sys.stderr)
        return 1
    result = {
        "type": "so101_staged_joint_fit",
        "method": (
            "wrist_roll_only_multiangle_stock_gripper"
            if args.wrist_only
            else "outward_next_servo_revolute_axis_chain"
        ),
        "registration_source": registration.get("registration_source", ""),
        "offset_deltas_degrees": [
            float(offsets[index] - initial_offsets[index])
            for index in range(len(initial_offsets))
        ],
        "fitted_offsets_degrees": offsets,
        "fitted_directions": directions,
        "direction_deltas": [
            float(directions[index] - initial_directions[index])
            for index in range(len(initial_directions))
        ],
        "joints": results,
        "wrist_roll_zero": wrist_roll_zero,
        "convergence": convergence,
        "saved": False,
    }
    # Godot's JSON parser is strict. Refuse non-finite diagnostics here rather
    # than writing a result that the transaction state machine cannot parse.
    encoded = json.dumps(result, indent=2, allow_nan=False)
    if args.output:
        args.output.expanduser().write_text(encoded + "\n")
    print(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
