#!/usr/bin/env python3
"""Fit the SO-101 stock jaws from a five-state D455 motion capture.

This solver is deliberately file-in/file-out. It never talks to the follower
and never writes Godot's live registration; the caller owns validation and the
atomic commit.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


METHOD = "d455_five_state_independent_mesh_fit"


def transform_from_dict(value: dict) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, 0] = np.asarray(value["basis_x"], dtype=np.float64)
    result[:3, 1] = np.asarray(value["basis_y"], dtype=np.float64)
    result[:3, 2] = np.asarray(value["basis_z"], dtype=np.float64)
    result[:3, 3] = np.asarray(value["origin"], dtype=np.float64)
    return result


def transform_points(value: np.ndarray, points: np.ndarray) -> np.ndarray:
    return points @ value[:3, :3].T + value[:3, 3]


def camera_points(frame: dict) -> tuple[str, np.ndarray]:
    candidates: list[tuple[str, np.ndarray]] = []
    for camera in frame.get("full_camera_points", []):
        name = str(camera.get("name", ""))
        points = np.asarray(camera.get("points", []), dtype=np.float64)
        if points.ndim == 2 and points.shape[1] == 3 and len(points):
            candidates.append((name, points[np.isfinite(points).all(axis=1)]))
    for name, points in candidates:
        if "d455" in name.lower():
            return name, points
    raise ValueError("capture frame has no D455 full point cloud")


def rigid_kabsch(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    left, _, right = np.linalg.svd(covariance)
    rotation = right.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right[-1, :] *= -1.0
        rotation = right.T @ left.T
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = target_center - rotation @ source_center
    return result


def rotation_degrees(value: np.ndarray) -> float:
    cosine = np.clip((np.trace(value[:3, :3]) - 1.0) * 0.5, -1.0, 1.0)
    return math.degrees(math.acos(float(cosine)))


def correction_displacement_at_points(value: np.ndarray, points: np.ndarray) -> float:
    """Measure rigid correction where the assembly is, not at world origin."""
    center = np.asarray(points, dtype=np.float64).mean(axis=0, keepdims=True)
    return float(np.linalg.norm(transform_points(value, center)[0] - center[0]))


def fit_shared_rigid(fixed_world: np.ndarray, scenes: list[np.ndarray]) -> tuple[np.ndarray, float, int]:
    lower = fixed_world.min(axis=0) - 0.045
    upper = fixed_world.max(axis=0) + 0.045
    cropped = [
        points[((points >= lower) & (points <= upper)).all(axis=1)]
        for points in scenes
    ]
    cropped = [points for points in cropped if len(points) >= 40]
    if len(cropped) < 4:
        raise ValueError("fewer than four D455 states contain the fixed stock gripper")
    # Keep geometry that persists across every opening. This rejects the
    # articulated jaw before fitting the fixed stock gripper; a custom camera
    # bracket may also persist, but robust model proximity keeps it from
    # becoming the target surface.
    static_mask = np.ones(len(cropped[0]), dtype=bool)
    for points in cropped[1:]:
        distances, _ = cKDTree(points).query(cropped[0], k=1, workers=-1)
        static_mask &= np.isfinite(distances) & (distances <= 0.004)
    static_scene = cropped[0][static_mask]
    scene = static_scene if len(static_scene) >= 80 else np.concatenate(cropped, axis=0)
    tree = cKDTree(scene)
    correction = np.eye(4, dtype=np.float64)
    residual = math.inf
    pair_count = 0
    for _ in range(18):
        transformed = transform_points(correction, fixed_world)
        distances, indices = tree.query(transformed, k=1, workers=-1)
        finite = np.isfinite(distances) & (distances <= 0.035)
        if int(finite.sum()) < 35:
            raise ValueError("insufficient fixed-gripper surface correspondences")
        threshold = float(np.quantile(distances[finite], 0.68))
        keep = finite & (distances <= threshold)
        reverse_tree = cKDTree(transformed)
        reverse_distances, reverse_indices = reverse_tree.query(scene, k=1, workers=-1)
        reverse_finite = np.isfinite(reverse_distances) & (reverse_distances <= 0.035)
        reverse_threshold = float(np.quantile(reverse_distances[reverse_finite], 0.82))
        reverse_keep = reverse_finite & (reverse_distances <= reverse_threshold)
        source_pairs = np.concatenate(
            (transformed[keep], transformed[reverse_indices[reverse_keep]]),
            axis=0,
        )
        target_pairs = np.concatenate(
            (scene[indices[keep]], scene[reverse_keep]),
            axis=0,
        )
        update = rigid_kabsch(source_pairs, target_pairs)
        candidate = update @ correction
        candidate_translation_m = correction_displacement_at_points(candidate, fixed_world)
        candidate_rotation_degrees = rotation_degrees(candidate)
        if candidate_translation_m > 0.035 or candidate_rotation_degrees > 25.0:
            # This is a bounded ICP problem. If a valid in-bound iterate has
            # already been evaluated, retain it at the constraint boundary
            # instead of rejecting the entire five-state capture because the
            # next unconstrained step points toward the omitted camera bracket.
            if pair_count > 0:
                residual = float(np.median(distances[keep]))
                pair_count = int(len(source_pairs))
                break
            raise ValueError(
                "first fixed-gripper correction exceeds the bounded assembly tolerance: "
                f"translation={candidate_translation_m * 1000.0:.1f} mm/35.0 mm, "
                f"rotation={candidate_rotation_degrees:.1f}/25.0 deg"
            )
        correction = candidate
        residual = float(np.median(distances[keep]))
        pair_count = int(len(source_pairs))
        if (
            correction_displacement_at_points(update, transformed) < 0.00005
            and rotation_degrees(update) < 0.04
        ):
            break
    return correction, residual, pair_count


def rotate_about_axis(points: np.ndarray, pivot: np.ndarray, axis: np.ndarray, degrees: float) -> np.ndarray:
    rotation = Rotation.from_rotvec(
        axis / max(np.linalg.norm(axis), 1.0e-12) * math.radians(degrees)
    ).as_matrix()
    return (points - pivot) @ rotation.T + pivot


def fit_jaw_angle(
    frame: dict,
    base: np.ndarray,
    shared: np.ndarray,
    scene: np.ndarray,
) -> tuple[float, float, int]:
    moving_local = np.asarray(
        frame["mesh_points"]["moving_jaw_so101_v1_link"],
        dtype=np.float64,
    )
    moving_world = transform_points(shared, transform_points(base, moving_local))
    joint_frames = frame.get("joint_frames", [])
    if len(joint_frames) < 6:
        raise ValueError("jaw revolute frame is missing")
    pivot_local = np.asarray(joint_frames[5]["pivot"], dtype=np.float64)
    axis_local = np.asarray(joint_frames[5]["axis"], dtype=np.float64)
    pivot = transform_points(shared, transform_points(base, pivot_local[None, :]))[0]
    axis = shared[:3, :3] @ (base[:3, :3] @ axis_local)
    axis /= max(np.linalg.norm(axis), 1.0e-12)
    radial = np.linalg.norm(np.cross(moving_world - pivot, axis), axis=1)
    # Terminal-finger points are much less likely than the hinge to match the
    # fixed jaw or the omitted camera bracket.
    terminal = moving_world[radial >= np.quantile(radial, 0.42)]
    baseline = float(frame["claw_metadata"]["model_gripper_angle_degrees"])
    tree = cKDTree(scene)
    best = (math.inf, baseline, 0)
    for angle in np.arange(-10.0, 86.01, 0.25):
        candidate = rotate_about_axis(terminal, pivot, axis, angle - baseline)
        distances, _ = tree.query(candidate, k=1, workers=-1)
        finite = np.isfinite(distances) & (distances <= 0.045)
        if int(finite.sum()) < 25:
            continue
        threshold = float(np.quantile(distances[finite], 0.70))
        keep = finite & (distances <= threshold)
        score = float(np.mean(np.square(np.minimum(distances[keep], 0.045))))
        if score < best[0]:
            best = (score, float(angle), int(keep.sum()))
    if not math.isfinite(best[0]):
        raise ValueError("moving-jaw surface could not be fitted")
    return best[1], math.sqrt(best[0]), best[2]


def solve(capture: dict) -> dict:
    if capture.get("type") != "so101_motion_capture":
        raise ValueError("input is not an SO-101 motion capture")
    selected: list[tuple[dict, str, np.ndarray]] = []
    for frame in capture.get("frames", []):
        pose = frame.get("pose", [])
        if (
            int(frame.get("calibration_joint_index", -1)) != 5
            or len(pose) < 6
            or "gripper_link" not in frame.get("mesh_points", {})
            or "moving_jaw_so101_v1_link" not in frame.get("mesh_points", {})
        ):
            continue
        name, points = camera_points(frame)
        selected.append((frame, name, points))
    selected.sort(key=lambda item: float(item[0]["pose"][5]))
    distinct: list[tuple[dict, str, np.ndarray]] = []
    for item in selected:
        if not distinct or float(item[0]["pose"][5]) - float(distinct[-1][0]["pose"][5]) >= 5.0:
            distinct.append(item)
    if len(distinct) != 5:
        raise ValueError(f"exactly five distinct D455 claw states are required (got {len(distinct)})")
    parent_pose = np.asarray([item[0]["pose"][:5] for item in distinct], dtype=np.float64)
    if np.ptp(parent_pose, axis=0).max() > 2.0:
        raise ValueError("an upstream arm joint moved during the claw-only fit")

    first = distinct[0][0]
    base = transform_from_dict(first["model_transform"])
    fixed_local = np.asarray(first["mesh_points"]["gripper_link"], dtype=np.float64)
    fixed_world = transform_points(base, fixed_local)
    scenes = [item[2] for item in distinct]
    shared, fixed_residual, fixed_pairs = fit_shared_rigid(fixed_world, scenes)

    fitted: list[float] = []
    jaw_residuals: list[float] = []
    jaw_pairs: list[int] = []
    for frame, _, scene in distinct:
        angle, residual, pairs = fit_jaw_angle(frame, base, shared, scene)
        fitted.append(angle)
        jaw_residuals.append(residual)
        jaw_pairs.append(pairs)
    # A fully closed physical claw is the URDF hard stop. Geometry is least
    # observable there because the two dark fingers nearly touch, so use the
    # known endpoint rather than allowing the fixed jaw to attract it.
    fitted[0] = -10.0
    if any(right - left < 2.0 for left, right in zip(fitted, fitted[1:])):
        raise ValueError(f"fitted jaw states are not strictly monotonic: {fitted}")
    if fitted[-1] < 68.0:
        raise ValueError(f"open jaw fit has insufficient span: {fitted[-1]:.2f} degrees")

    metadata = first["claw_metadata"]
    pre_local = transform_from_dict(metadata["pre_visual_transform_local"])
    corrected_local = transform_from_dict(metadata["corrected_transform_local"])
    current_global = base @ corrected_local
    target_global = shared @ current_global
    pre_global = base @ pre_local
    absolute_correction = np.linalg.inv(pre_global) @ target_global
    absolute_correction[:3, :3] = Rotation.from_matrix(
        absolute_correction[:3, :3]
    ).as_matrix()
    rpy = Rotation.from_matrix(absolute_correction[:3, :3]).as_euler(
        "xyz",
        degrees=True,
    )
    translation = absolute_correction[:3, 3]
    mount = np.asarray(metadata.get("mount_correction_local", [0.0, 0.0, 0.0]), dtype=np.float64)
    median_residual = float(np.median([fixed_residual, *jaw_residuals]))
    confidence = float(np.clip(1.0 - median_residual / 0.050, 0.0, 0.98))
    if confidence < 0.72 or median_residual > 0.018:
        raise ValueError(
            f"D455 claw evidence is uncertain: confidence={confidence:.3f}, "
            f"residual={median_residual * 1000.0:.1f} mm"
        )
    return {
        "type": "so101_claw_visual_fit",
        "method": METHOD,
        "reference_camera": distinct[0][1],
        "gripper_angle_samples_normalized": [
            float(item[0]["pose"][5]) for item in distinct
        ],
        "gripper_angle_samples_degrees": [float(value) for value in fitted],
        "gripper_mount_correction_local": mount.tolist(),
        "gripper_visual_correction_rpy_degrees": rpy.tolist(),
        "gripper_visual_correction_translation_local": translation.tolist(),
        "confidence": confidence,
        "median_residual_m": median_residual,
        "fixed_gripper_residual_m": fixed_residual,
        "jaw_state_residuals_m": jaw_residuals,
        "fixed_correspondences": fixed_pairs,
        "jaw_state_correspondences": jaw_pairs,
        "validation_pose_count": 5,
        "upstream_motion": "none",
        "unmodelled_camera_bracket_excluded": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    try:
        capture = json.loads(arguments.capture.read_text())
        result = solve(capture)
    except Exception as error:
        print(f"SO-101 claw fit rejected: {error}")
        return 1
    arguments.output.write_text(json.dumps(result, indent=2) + "\n")
    print(
        "SO101_CLAW_FIT_OK "
        f"residual={result['median_residual_m'] * 1000.0:.1f}mm "
        f"confidence={result['confidence']:.0%}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
