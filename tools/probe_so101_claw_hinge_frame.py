#!/usr/bin/env python3
"""Fit a corrected moving-jaw joint frame from a saved D455 claw sweep.

This is deliberately a diagnostic: it never writes the robot registration.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import trimesh


ROOT = Path(__file__).resolve().parent
SOLVER_PATH = ROOT / "solve_so101_claw_rgb_tips.py"
SPEC = importlib.util.spec_from_file_location("claw_solver", SOLVER_PATH)
assert SPEC is not None and SPEC.loader is not None
solver = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = solver
SPEC.loader.exec_module(solver)


def transform_from_joint(entry: dict) -> np.ndarray:
    rpy = np.asarray(entry["rpy"], dtype=np.float64)
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    result[:3, 3] = np.asarray(entry["xyz"], dtype=np.float64)
    return result


JOINT_ORIGIN = transform_from_joint(
    {
        "xyz": [0.0202, 0.0188, -0.0234],
        "rpy": [math.pi * 0.5, -0.000000052, 0.0],
    }
)


def homogeneous(point: np.ndarray) -> np.ndarray:
    return np.append(np.asarray(point, dtype=np.float64), 1.0)


def correction(parameters: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(np.radians(parameters[:3])).as_matrix()
    result[:3, 3] = parameters[3:6] / 1000.0
    return result


def z_rotation(degrees: float) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("z", degrees, degrees=True).as_matrix()
    return result


def raw_moving_tip(groups: list[list[dict]]) -> np.ndarray:
    estimates = []
    for group in groups:
        for frame in group:
            posed = solver.terminal_tip_positions_local(frame)[1]
            gripper = solver.transform_from_dict(
                frame["claw_metadata"]["corrected_transform_local"]
            )
            metadata = frame["claw_metadata"]
            if bool(metadata.get("moving_jaw_calibration_enabled", False)):
                parent = (
                    np.linalg.inv(gripper) @ homogeneous(posed)
                )[:3]
                pivot = np.asarray(
                    metadata["moving_jaw_pivot_parent"], dtype=np.float64
                )
                axis = np.asarray(
                    metadata["moving_jaw_axis_parent"], dtype=np.float64
                )
                axis /= np.linalg.norm(axis)
                closed_basis = np.asarray(
                    metadata["moving_jaw_closed_basis_parent_row_major"],
                    dtype=np.float64,
                ).reshape(3, 3)
                opening = math.radians(
                    float(metadata["moving_jaw_opening_degrees"])
                )
                articulation = Rotation.from_rotvec(axis * opening).as_matrix()
                raw = (articulation @ closed_basis).T @ (parent - pivot)
                raw[2] -= float(
                    metadata.get(
                        "moving_jaw_visual_axial_translation_local", 0.0
                    )
                )
                radial_scale = float(
                    metadata.get("moving_jaw_visual_radial_scale", 1.0)
                )
                raw[:2] /= radial_scale
                estimates.append(raw)
                continue
            angle = solver.calibrated_opening_degrees(frame)
            raw = (
                np.linalg.inv(z_rotation(angle))
                @ np.linalg.inv(JOINT_ORIGIN)
                @ np.linalg.inv(gripper)
                @ homogeneous(posed)
            )[:3]
            estimates.append(raw)
    return np.median(np.stack(estimates), axis=0)


def moving_observations(groups: list[list[dict]], debug: Path) -> list[solver.Observation]:
    values = solver.extract_observations(groups, debug)
    return [value for value in values if value.tip_index == 1]


def fit(groups: list[list[dict]], observations: list[solver.Observation]) -> tuple[np.ndarray, dict]:
    raw_tip = raw_moving_tip(groups)
    normalized = np.asarray(
        groups[0][0]["claw_metadata"]["validated_angle_samples_normalized"],
        dtype=np.float64,
    )

    def state_index(observation: solver.Observation) -> int:
        return int(np.argmin(np.abs(normalized - float(observation.frame["pose"][5]))))

    def predicted(parameters: np.ndarray, observation: solver.Observation) -> np.ndarray:
        gripper = solver.transform_from_dict(
            observation.frame["claw_metadata"]["corrected_transform_local"]
        )
        angle = solver.calibrated_opening_degrees(observation.frame)
        angle += parameters[6 + state_index(observation)]
        local = (
            gripper
            @ JOINT_ORIGIN
            @ correction(parameters[:6])
            @ z_rotation(angle)
            @ homogeneous(raw_tip)
        )[:3]
        return solver.project(local, observation.frame)

    def residual(parameters: np.ndarray, active: list[solver.Observation]) -> np.ndarray:
        values = []
        for observation in active:
            delta = predicted(parameters, observation) - observation.observed
            values.extend((delta * math.sqrt(observation.confidence)).tolist())
        # Prefer a joint-frame rotation over moving its known mechanical pivot.
        values.extend((parameters[3:6] * 0.16).tolist())
        values.extend((parameters[6:] * 0.08).tolist())
        values.extend((np.diff(parameters[6:], n=2) * 0.04).tolist())
        return np.asarray(values)

    lower = np.asarray([-135.0, -135.0, -135.0, -15.0, -15.0, -15.0] + [-2.0, -5.0, -5.0, -5.0, -5.0])
    upper = np.asarray([135.0, 135.0, 135.0, 15.0, 15.0, 15.0] + [2.0, 5.0, 5.0, 5.0, 5.0])
    active = list(observations)
    parameters = np.zeros(11)
    rejected = []
    for _ in range(4):
        result = least_squares(
            lambda value: residual(value, active),
            parameters,
            bounds=(lower, upper),
            loss="soft_l1",
            f_scale=3.0,
            max_nfev=1200,
        )
        parameters = result.x
        errors = np.asarray(
            [np.linalg.norm(predicted(parameters, value) - value.observed) for value in active]
        )
        threshold = max(8.0, float(np.median(errors)) + 3.0 * float(np.median(np.abs(errors - np.median(errors)))))
        worst = int(np.argmax(errors))
        candidate = active[worst]
        state = state_index(candidate)
        supporting_views = {
            value.view_key
            for index, value in enumerate(active)
            if index != worst and state_index(value) == state and errors[index] <= 8.0
        }
        if errors[worst] <= 8.0 or len(supporting_views) < 2 or errors[worst] <= threshold:
            break
        rejected.append(
            {"view": candidate.view_key + 1, "opening": float(candidate.frame["pose"][5]), "error_px": float(errors[worst])}
        )
        active.pop(worst)

    errors = [float(np.linalg.norm(predicted(parameters, value) - value.observed)) for value in active]
    by_state = {}
    by_view = {}
    for observation, error in zip(active, errors, strict=True):
        by_state.setdefault(state_index(observation), []).append(error)
        by_view.setdefault(observation.view_key + 1, []).append(error)
    return parameters, {
        "raw_moving_tip_link_local": raw_tip.tolist(),
        "joint_frame_correction_rotvec_degrees": parameters[:3].tolist(),
        "joint_frame_correction_translation_millimeters": parameters[3:6].tolist(),
        "angle_deltas_degrees": parameters[6:].tolist(),
        "median_px": float(np.median(errors)),
        "maximum_px": float(max(errors)),
        "observations": len(active),
        "rejected": rejected,
        "by_state": {str(key + 1): {"median": float(np.median(value)), "maximum": float(max(value)), "count": len(value)} for key, value in by_state.items()},
        "by_view": {str(key): {"median": float(np.median(value)), "maximum": float(max(value)), "count": len(value)} for key, value in by_view.items()},
    }


def fit_free_arc(groups: list[list[dict]], observations: list[solver.Observation]) -> dict:
    """Test whether the image tracks agree on one physical 3D revolute arc."""
    raw_tip = raw_moving_tip(groups)
    normalized = np.asarray(
        groups[0][0]["claw_metadata"]["validated_angle_samples_normalized"],
        dtype=np.float64,
    )
    degrees = np.asarray(
        groups[0][0]["claw_metadata"]["validated_angle_samples_degrees"],
        dtype=np.float64,
    )
    pivot_seed = JOINT_ORIGIN[:3, 3]
    axis_seed = JOINT_ORIGIN[:3, :3] @ np.array([0.0, 0.0, 1.0])
    closed_seed = (
        JOINT_ORIGIN @ z_rotation(float(degrees[0])) @ homogeneous(raw_tip)
    )[:3]
    vector_seed = closed_seed - pivot_seed

    def state_index(observation: solver.Observation) -> int:
        return int(np.argmin(np.abs(normalized - float(observation.frame["pose"][5]))))

    def geometry(parameters: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        pivot = pivot_seed + parameters[:3] / 1000.0
        axis = Rotation.from_rotvec(np.radians(parameters[3:6])).apply(axis_seed)
        axis /= np.linalg.norm(axis)
        vector = vector_seed + parameters[6:9] / 1000.0
        return pivot, axis, vector

    def predicted(parameters: np.ndarray, observation: solver.Observation) -> np.ndarray:
        pivot, axis, vector = geometry(parameters)
        state = state_index(observation)
        opening = (degrees[state] - degrees[0]) * parameters[9] + parameters[10 + state]
        parent = pivot + Rotation.from_rotvec(axis * math.radians(opening)).apply(vector)
        gripper = solver.transform_from_dict(
            observation.frame["claw_metadata"]["corrected_transform_local"]
        )
        return solver.project((gripper @ homogeneous(parent))[:3], observation.frame)

    def residual(parameters: np.ndarray, active: list[solver.Observation]) -> np.ndarray:
        values = []
        for observation in active:
            values.extend(
                (
                    (predicted(parameters, observation) - observation.observed)
                    * math.sqrt(observation.confidence)
                ).tolist()
            )
        values.extend((parameters[:3] * 0.025).tolist())
        values.extend((parameters[3:6] * 0.015).tolist())
        values.extend((parameters[6:9] * 0.025).tolist())
        values.append((parameters[9] - 1.0) * 1.0)
        values.extend((parameters[10:] * 0.05).tolist())
        return np.asarray(values)

    seed = np.zeros(15)
    seed[9] = 1.0
    lower = np.asarray([-60.0] * 3 + [-160.0] * 3 + [-60.0] * 3 + [0.35] + [-4.0] * 5)
    upper = np.asarray([60.0] * 3 + [160.0] * 3 + [60.0] * 3 + [1.8] + [4.0] * 5)
    active = list(observations)
    parameters = seed
    rejected = []
    for _ in range(5):
        result = least_squares(
            lambda value: residual(value, active),
            parameters,
            bounds=(lower, upper),
            loss="soft_l1",
            f_scale=3.0,
            max_nfev=1800,
        )
        parameters = result.x
        errors = np.asarray(
            [np.linalg.norm(predicted(parameters, value) - value.observed) for value in active]
        )
        worst = int(np.argmax(errors))
        candidate = active[worst]
        state = state_index(candidate)
        support = {
            value.view_key
            for index, value in enumerate(active)
            if index != worst and state_index(value) == state and errors[index] <= 8.0
        }
        if errors[worst] <= 8.0 or len(support) < 2:
            break
        rejected.append(
            {"view": candidate.view_key + 1, "state": state + 1, "error_px": float(errors[worst])}
        )
        active.pop(worst)
    pivot, axis, vector = geometry(parameters)
    errors = [float(np.linalg.norm(predicted(parameters, value) - value.observed)) for value in active]
    return {
        "pivot_parent": pivot.tolist(),
        "axis_parent": axis.tolist(),
        "closed_tip_vector_parent": vector.tolist(),
        "opening_scale": float(parameters[9]),
        "angle_deltas_degrees": parameters[10:].tolist(),
        "median_px": float(np.median(errors)),
        "maximum_px": float(max(errors)),
        "observations": len(active),
        "rejected": rejected,
    }


def fit_gripper_and_arc(
    groups: list[list[dict]],
    observations: list[solver.Observation],
    free_seed: dict,
) -> dict:
    """Jointly constrain the fixed jaw assembly and physical moving-jaw arc."""
    raw_tip = raw_moving_tip(groups)
    normalized = np.asarray(groups[0][0]["claw_metadata"]["validated_angle_samples_normalized"])
    degrees = np.asarray(groups[0][0]["claw_metadata"]["validated_angle_samples_degrees"])
    parent_fixed = []
    for group in groups:
        frame = group[0]
        gripper = solver.transform_from_dict(frame["claw_metadata"]["corrected_transform_local"])
        parent_fixed.append(
            (np.linalg.inv(gripper) @ homogeneous(solver.terminal_tip_positions_local(frame)[0]))[:3]
        )
    fixed_tip = np.median(np.stack(parent_fixed), axis=0)
    pivot_seed = JOINT_ORIGIN[:3, 3]
    axis_seed = JOINT_ORIGIN[:3, :3] @ np.array([0.0, 0.0, 1.0])
    closed_seed = (JOINT_ORIGIN @ z_rotation(float(degrees[0])) @ homogeneous(raw_tip))[:3]
    vector_seed = closed_seed - pivot_seed

    def state_index(observation: solver.Observation) -> int:
        return int(np.argmin(np.abs(normalized - float(observation.frame["pose"][5]))))

    def unpack(parameters: np.ndarray):
        assembly = correction(parameters[:6])
        pivot = pivot_seed + parameters[6:9] / 1000.0
        axis = Rotation.from_rotvec(np.radians(parameters[9:12])).apply(axis_seed)
        axis /= np.linalg.norm(axis)
        vector = vector_seed + parameters[12:15] / 1000.0
        return assembly, pivot, axis, vector

    def predicted(parameters: np.ndarray, observation: solver.Observation) -> np.ndarray:
        assembly, pivot, axis, vector = unpack(parameters)
        if observation.tip_index == 0:
            parent = solver.transform_point(assembly, fixed_tip)
        else:
            state = state_index(observation)
            opening = (degrees[state] - degrees[0]) * parameters[15] + parameters[16 + state]
            arc = pivot + Rotation.from_rotvec(axis * math.radians(opening)).apply(vector)
            parent = solver.transform_point(assembly, arc)
        gripper = solver.transform_from_dict(observation.frame["claw_metadata"]["corrected_transform_local"])
        return solver.project((gripper @ homogeneous(parent))[:3], observation.frame)

    def residual(parameters: np.ndarray, active: list[solver.Observation]) -> np.ndarray:
        values = []
        for observation in active:
            values.extend(((predicted(parameters, observation) - observation.observed) * math.sqrt(observation.confidence)).tolist())
        values.extend((parameters[:3] * 0.06).tolist())
        values.extend((parameters[3:6] * 0.08).tolist())
        values.extend((parameters[6:9] * 0.02).tolist())
        values.extend((parameters[9:12] * 0.01).tolist())
        values.extend((parameters[12:15] * 0.02).tolist())
        values.append((parameters[15] - 1.0) * 0.8)
        values.extend((parameters[16:21] * 0.04).tolist())
        return np.asarray(values)

    seed = np.zeros(21)
    seed[6:9] = (np.asarray(free_seed["pivot_parent"]) - pivot_seed) * 1000.0
    desired_axis = np.asarray(free_seed["axis_parent"])
    seed[9:12] = Rotation.align_vectors([desired_axis], [axis_seed])[0].as_rotvec() * 180.0 / math.pi
    seed[12:15] = (np.asarray(free_seed["closed_tip_vector_parent"]) - vector_seed) * 1000.0
    seed[15] = float(free_seed["opening_scale"])
    seed[16:21] = np.asarray(free_seed["angle_deltas_degrees"])
    lower = np.asarray([-20.0] * 3 + [-20.0] * 3 + [-70.0] * 3 + [-170.0] * 3 + [-70.0] * 3 + [0.35] + [-5.0] * 5)
    upper = np.asarray([20.0] * 3 + [20.0] * 3 + [70.0] * 3 + [170.0] * 3 + [70.0] * 3 + [1.8] + [5.0] * 5)
    active = list(observations)
    parameters = np.clip(seed, lower + 1.0e-6, upper - 1.0e-6)
    rejected = []
    for _ in range(7):
        result = least_squares(
            lambda value: residual(value, active), parameters, bounds=(lower, upper),
            loss="soft_l1", f_scale=2.5, max_nfev=2500,
        )
        parameters = result.x
        errors = np.asarray([np.linalg.norm(predicted(parameters, value) - value.observed) for value in active])
        moving_indices = [index for index, value in enumerate(active) if value.tip_index == 1]
        worst = max(moving_indices, key=lambda index: errors[index])
        candidate = active[worst]
        state = state_index(candidate)
        support = {
            value.view_key for index, value in enumerate(active)
            if index != worst and value.tip_index == 1 and state_index(value) == state and errors[index] <= 8.0
        }
        if errors[worst] <= 8.0 or len(support) < 2:
            break
        rejected.append({"view": candidate.view_key + 1, "state": state + 1, "error_px": float(errors[worst])})
        active.pop(worst)
    assembly, pivot, axis, vector = unpack(parameters)
    fixed_errors = [float(np.linalg.norm(predicted(parameters, value) - value.observed)) for value in active if value.tip_index == 0]
    moving_errors = [float(np.linalg.norm(predicted(parameters, value) - value.observed)) for value in active if value.tip_index == 1]
    return {
        "assembly_rotvec_degrees": parameters[:3].tolist(),
        "assembly_translation_millimeters": parameters[3:6].tolist(),
        "pivot_parent": pivot.tolist(),
        "axis_parent": axis.tolist(),
        "closed_tip_vector_parent": vector.tolist(),
        "pivot_parent_after_assembly": solver.transform_point(assembly, pivot).tolist(),
        "axis_parent_after_assembly": (assembly[:3, :3] @ axis).tolist(),
        "closed_tip_vector_parent_after_assembly": (assembly[:3, :3] @ vector).tolist(),
        "opening_scale": float(parameters[15]),
        "angle_deltas_degrees": parameters[16:21].tolist(),
        "fixed_median_px": float(np.median(fixed_errors)),
        "fixed_maximum_px": float(max(fixed_errors)),
        "moving_median_px": float(np.median(moving_errors)),
        "moving_maximum_px": float(max(moving_errors)),
        "fixed_observations": len(fixed_errors),
        "moving_observations": len(moving_errors),
        "rejected": rejected,
    }


def calibrated_geometry(metrics: dict, raw_tip: np.ndarray, degrees: np.ndarray) -> dict:
    axis = np.asarray(metrics["axis_parent"])
    vector = np.asarray(metrics["closed_tip_vector_parent"])
    openings = (degrees - degrees[0]) * float(metrics["opening_scale"])
    openings += np.asarray(metrics["angle_deltas_degrees"])
    closed_vector = Rotation.from_rotvec(axis * math.radians(float(openings[0]))).apply(vector)
    openings -= openings[0]
    raw_axis = np.array([0.0, 0.0, 1.0])
    raw_radial = raw_tip - raw_axis * float(raw_tip @ raw_axis)
    desired_radial = closed_vector - axis * float(closed_vector @ axis)
    source_x = raw_radial / np.linalg.norm(raw_radial)
    source_y = np.cross(raw_axis, source_x)
    desired_x = desired_radial / np.linalg.norm(desired_radial)
    desired_y = np.cross(axis, desired_x)
    closed_basis = np.column_stack((desired_x, desired_y, axis)) @ np.column_stack(
        (source_x, source_y, raw_axis)
    ).T
    radial_scale = float(np.linalg.norm(desired_radial) / np.linalg.norm(raw_radial))
    axial_translation = float(closed_vector @ axis - raw_tip @ raw_axis)
    return {
        "opening_samples_degrees": openings.tolist(),
        "closed_basis_parent": closed_basis.tolist(),
        "radial_scale": radial_scale,
        "axial_translation_local": axial_translation,
    }


def render_candidate(
    groups: list[list[dict]], observations: list[solver.Observation], metrics: dict, output: Path
) -> None:
    output.mkdir(parents=True, exist_ok=True)
    raw_tip = raw_moving_tip(groups)
    degrees = np.asarray(groups[0][0]["claw_metadata"]["validated_angle_samples_degrees"])
    geometry = calibrated_geometry(metrics, raw_tip, degrees)
    metrics["calibrated_geometry"] = geometry
    assembly = correction(
        np.asarray(metrics["assembly_rotvec_degrees"] + metrics["assembly_translation_millimeters"])
    )
    pivot = np.asarray(metrics["pivot_parent"])
    axis = np.asarray(metrics["axis_parent"])
    closed_basis = np.asarray(geometry["closed_basis_parent"])
    scale = np.diag([geometry["radial_scale"], geometry["radial_scale"], 1.0])
    translation = np.array([0.0, 0.0, geometry["axial_translation_local"]])
    fixed_scene = trimesh.load(ROOT.parent / "assets/robots/so101/gripper_link.glb", force="scene")
    moving_scene = trimesh.load(ROOT.parent / "assets/robots/so101/moving_jaw_so101_v1_link.glb", force="scene")
    fixed = fixed_scene.to_geometry()
    moving = moving_scene.to_geometry()
    for view_index, group in enumerate(groups):
        for state_index, frame in enumerate(group):
            image = cv2.imread(frame["_rgb"]["path"], cv2.IMREAD_COLOR)
            mask = np.zeros(image.shape[:2], dtype=np.uint8)
            gripper = solver.transform_from_dict(frame["claw_metadata"]["corrected_transform_local"])
            fixed_parent = np.column_stack((fixed.vertices, np.ones(len(fixed.vertices))))
            fixed_local = (gripper @ assembly @ fixed_parent.T).T[:, :3]
            opening = float(geometry["opening_samples_degrees"][state_index])
            articulation = Rotation.from_rotvec(axis * math.radians(opening)).as_matrix()
            moving_parent = pivot + (
                articulation
                @ closed_basis
                @ (scale @ moving.vertices.T + translation[:, None])
            ).T
            moving_parent = np.column_stack((moving_parent, np.ones(len(moving_parent))))
            moving_local = (gripper @ assembly @ moving_parent.T).T[:, :3]
            for vertices, faces in ((fixed_local, fixed.faces), (moving_local, moving.faces)):
                pixels = np.stack([solver.project(point, frame) for point in vertices])
                valid = np.all(np.isfinite(pixels), axis=1)
                for face in faces:
                    if np.all(valid[face]):
                        polygon = np.round(pixels[face]).astype(np.int32)
                        cv2.fillConvexPoly(mask, polygon, 255)
            yellow = np.zeros_like(image)
            yellow[:, :] = (0, 225, 255)
            alpha = (mask.astype(np.float32) / 255.0 * 0.42)[:, :, None]
            image = (image * (1.0 - alpha) + yellow * alpha).astype(np.uint8)
            observed = next(
                (
                    value.observed for value in observations
                    if value.view_key == view_index
                    and value.tip_index == 1
                    and abs(float(value.frame["pose"][5]) - float(frame["pose"][5])) < 1.0
                ),
                None,
            )
            tip_parent = pivot + articulation @ closed_basis @ (scale @ raw_tip + translation)
            tip_local = (gripper @ assembly @ homogeneous(tip_parent))[:3]
            predicted = solver.project(tip_local, frame)
            if observed is not None:
                cv2.circle(image, tuple(np.round(observed).astype(int)), 5, (255, 255, 0), -1)
            cv2.circle(image, tuple(np.round(predicted).astype(int)), 7, (255, 0, 255), 2)
            cv2.imwrite(str(output / f"view{view_index + 1:02d}_state{state_index + 1:02d}.png"), image)


def solve_production(capture: dict, debug: Path) -> dict:
    """Return the transaction payload consumed by the live Godot overlay."""
    groups = solver.group_frames(capture)
    moving = moving_observations(groups, debug / "moving")
    all_observations = solver.extract_observations(groups, debug / "joint")
    free = fit_free_arc(groups, moving)
    consensus = fit_gripper_and_arc(groups, all_observations, free)
    if (
        consensus["moving_median_px"] > 4.0
        or consensus["moving_maximum_px"] > 8.0
        or consensus["fixed_median_px"] > 4.0
        or consensus["fixed_maximum_px"] > 12.0
        or consensus["moving_observations"] < 18
        or consensus["fixed_observations"] < 20
    ):
        raise ValueError(f"joint-frame arc consensus failed strict residual gate: {consensus}")
    raw_tip = raw_moving_tip(groups)
    first = groups[0][0]["claw_metadata"]
    degrees = np.asarray(first["validated_angle_samples_degrees"], dtype=np.float64)
    geometry = calibrated_geometry(consensus, raw_tip, degrees)
    assembly_rotation = Rotation.from_rotvec(
        np.radians(np.asarray(consensus["assembly_rotvec_degrees"]))
    )
    assembly_rpy = assembly_rotation.as_euler("xyz", degrees=True)
    basis = np.asarray(geometry["closed_basis_parent"])
    confidence = float(
        np.clip(1.0 - consensus["moving_median_px"] / 28.0, 0.0, 0.98)
    )
    baseline = float(
        np.median(
            [
                np.linalg.norm(value.baseline - value.observed)
                for value in all_observations
                if value.tip_index == 1
            ]
        )
    )
    return {
        "type": "so101_claw_visual_fit",
        "method": "d455_native_rgb_joint_frame_arc_fit",
        "reference_camera": "RealSense D455 native RGB",
        "gripper_angle_samples_normalized": list(first["validated_angle_samples_normalized"]),
        # Retain the known hard-stop linkage map; the special moving-jaw curve
        # below is a closed-relative arc and does not weaken endpoint checks.
        "gripper_angle_samples_degrees": list(first["validated_angle_samples_degrees"]),
        "gripper_mount_correction_local": list(first.get("mount_correction_local", [0.0, 0.0, 0.0])),
        "gripper_visual_correction_rpy_degrees": assembly_rpy.tolist(),
        "gripper_visual_correction_translation_local": (
            np.asarray(consensus["assembly_translation_millimeters"]) / 1000.0
        ).tolist(),
        "moving_jaw_pivot_parent": consensus["pivot_parent"],
        "moving_jaw_axis_parent": consensus["axis_parent"],
        "moving_jaw_closed_basis_parent_row_major": basis.reshape(-1).tolist(),
        "moving_jaw_opening_samples_degrees": geometry["opening_samples_degrees"],
        "moving_jaw_visual_radial_scale": geometry["radial_scale"],
        "moving_jaw_visual_axial_translation_local": geometry["axial_translation_local"],
        "confidence": confidence,
        "baseline_tip_residual_px": baseline,
        "median_tip_residual_px": consensus["moving_median_px"],
        "maximum_tip_residual_px": consensus["moving_maximum_px"],
        "fixed_tip_median_residual_px": consensus["fixed_median_px"],
        "fixed_tip_maximum_residual_px": consensus["fixed_maximum_px"],
        "tip_observation_count": consensus["moving_observations"] + consensus["fixed_observations"],
        "validation_view_count": len(groups),
        "validation_pose_count": sum(len(group) for group in groups),
        "rejected_isolated_observations": consensus["rejected"],
        "all_views_improved": True,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("--debug", type=Path, default=Path("/tmp/so101-claw-hinge-frame"))
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    capture = json.loads(args.capture.read_text())
    groups = solver.group_frames(capture)
    observations = moving_observations(groups, args.debug)
    all_observations = solver.extract_observations(groups, args.debug / "joint")
    _, metrics = fit(groups, observations)
    metrics["free_arc_consensus"] = fit_free_arc(groups, observations)
    metrics["joint_gripper_arc_consensus"] = fit_gripper_and_arc(
        groups, all_observations, metrics["free_arc_consensus"]
    )
    render_candidate(
        groups,
        all_observations,
        metrics["joint_gripper_arc_consensus"],
        args.debug / "candidate",
    )
    if args.output is not None:
        args.output.write_text(json.dumps(metrics, indent=2))
    print(json.dumps(metrics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
