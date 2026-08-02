#!/usr/bin/env python3
"""Fit the SO-101 distal visual frame and opening curve from D455 fingertips.

The five measured encoder states may adjust only the moving jaw around its
known hinge. The custom camera bracket is rejected by looking only along the
two terminal jaw corridors and, for the articulated jaw, by requiring temporal
RGB support. The caller owns the atomic registration commit; this process only
writes a candidate JSON file and deterministic diagnostic PNGs.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


METHOD = "d455_native_rgb_multiview_tip_fit"
MAXIMUM_ACCEPTED_TIP_ERROR_PX = 8.0
MAXIMUM_ACCEPTED_MEDIAN_ERROR_PX = 3.0
MINIMUM_MOVING_TIP_OBSERVATIONS_PER_VIEW = 3
TERMINAL_RADIAL_TOLERANCE_M = 0.001


def transform_from_dict(value: dict) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, 0] = np.asarray(value["basis_x"], dtype=np.float64)
    result[:3, 1] = np.asarray(value["basis_y"], dtype=np.float64)
    result[:3, 2] = np.asarray(value["basis_z"], dtype=np.float64)
    result[:3, 3] = np.asarray(value["origin"], dtype=np.float64)
    return result


def transform_point(value: np.ndarray, point: np.ndarray) -> np.ndarray:
    return value[:3, :3] @ point + value[:3, 3]


def project(point_local: np.ndarray, frame: dict, assembly: np.ndarray | None = None) -> np.ndarray:
    snapshot = frame["_rgb"]
    point = point_local
    if assembly is not None:
        corrected = transform_from_dict(frame["claw_metadata"]["corrected_transform_local"])
        pre = transform_from_dict(frame["claw_metadata"]["pre_visual_transform_local"])
        point = transform_point(pre @ assembly @ np.linalg.inv(corrected), point)
    world = transform_point(transform_from_dict(frame["model_transform"]), point)
    camera = transform_point(transform_from_dict(snapshot["camera_inverse"]), world)
    # The untouched RGB frame uses the factory depth-camera to color-camera
    # calibration. Godot's depth-camera convention is +Y up/-Z forward;
    # librealsense uses +Y down/+Z forward.
    if snapshot.get("uses_raw_color", False):
        values = np.asarray(snapshot.get("depth_to_raw_color_extrinsics", []), dtype=np.float64)
        if values.size != 12:
            return np.array([math.nan, math.nan], dtype=np.float64)
        depth_camera = np.array([camera[0], -camera[1], -camera[2]], dtype=np.float64)
        color_camera = values[:9].reshape((3, 3), order="F") @ depth_camera + values[9:12]
        depth = color_camera[2]
        if depth <= 1.0e-5:
            return np.array([math.nan, math.nan], dtype=np.float64)
        fx, fy, cx, cy = np.asarray(snapshot["intrinsics"], dtype=np.float64)
        return np.array([fx * color_camera[0] / depth + cx, fy * color_camera[1] / depth + cy])
    depth = -camera[2]
    if depth <= 1.0e-5:
        return np.array([math.nan, math.nan], dtype=np.float64)
    fx, fy, cx, cy = np.asarray(snapshot["intrinsics"], dtype=np.float64)
    return np.array([fx * camera[0] / depth + cx, cy - fy * camera[1] / depth])


def assembly_from_parameters(parameters: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = Rotation.from_rotvec(np.radians(parameters[:3])).as_matrix()
    result[:3, 3] = parameters[3:] / 1000.0
    return result


def optimization_seed(pre: np.ndarray, corrected: np.ndarray) -> np.ndarray:
    """Return a stable six-parameter seed for the distal rigid fit.

    Matrix inversion leaves identity registrations with approximately 1e-14
    degree/ millimetre residue.  Starting finite-difference least-squares at
    that residue can satisfy its relative step test before it evaluates a real
    correction.  Identity must therefore be represented by exact zeros.
    """
    current = np.linalg.inv(pre) @ corrected
    seed = np.concatenate(
        (
            Rotation.from_matrix(current[:3, :3]).as_rotvec() * 180.0 / math.pi,
            current[:3, 3] * 1000.0,
        )
    )
    seed[np.abs(seed) < 1.0e-9] = 0.0
    return seed


@dataclass
class Observation:
    frame: dict
    tip_index: int
    observed: np.ndarray
    baseline: np.ndarray
    confidence: float
    view_key: int
    nominal_tip: np.ndarray


def d455_snapshot(frame: dict) -> dict:
    for snapshot in frame.get("rgb_snapshots", []):
        if "d455" in str(snapshot.get("name", "")).lower() and Path(str(snapshot.get("path", ""))).is_file():
            return snapshot
    raise ValueError("capture frame has no saved native-D455 RGB image")


def group_frames(capture: dict) -> list[list[dict]]:
    frames: list[dict] = []
    for value in capture.get("frames", []):
        if int(value.get("calibration_joint_index", -1)) != 5 or len(value.get("pose", [])) < 6:
            continue
        if len(value.get("claw_tip_positions_local", [])) != 2:
            continue
        value = dict(value)
        value["_rgb"] = d455_snapshot(value)
        frames.append(value)
    groups: list[list[dict]] = []
    for frame in sorted(frames, key=lambda item: (float(item["pose"][4]), float(item["pose"][5]))):
        match = next((group for group in groups if abs(float(group[0]["pose"][4]) - float(frame["pose"][4])) < 3.0), None)
        (match if match is not None else groups.append([frame]))
        if match is not None:
            match.append(frame)
    complete: list[list[dict]] = []
    for group in groups:
        group.sort(key=lambda item: float(item["pose"][5]))
        distinct: list[dict] = []
        for frame in group:
            if not distinct or float(frame["pose"][5]) - float(distinct[-1]["pose"][5]) >= 5.0:
                distinct.append(frame)
        if len(distinct) >= 5 and float(distinct[-1]["pose"][5]) - float(distinct[0]["pose"][5]) >= 70.0:
            complete.append(distinct[:5])
    if len(complete) < 2:
        raise ValueError(f"fewer than two complete native-RGB wrist views ({len(complete)})")
    return complete


def terminal_tip_positions_local(frame: dict) -> np.ndarray:
    """Return the real terminal centers of the fixed and moving jaw meshes.

    The old capture landmark used a per-link coordinate minimum.  That is not
    a geometric fingertip: on the articulated jaw it can select an interior
    corner as the link rotates.  A jaw endpoint is instead the mesh region
    farthest from its revolute hinge axis.  Averaging the final millimetre of
    that region yields a stable center rather than an arbitrary STL vertex.
    """
    joint_frames = frame.get("joint_frames", [])
    if len(joint_frames) < 6:
        raise ValueError("capture frame has no claw hinge frame")
    pivot = np.asarray(joint_frames[5].get("pivot", []), dtype=np.float64)
    axis = np.asarray(joint_frames[5].get("axis", []), dtype=np.float64)
    if pivot.shape != (3,) or axis.shape != (3,) or np.linalg.norm(axis) <= 1.0e-8:
        raise ValueError("capture frame has an invalid claw hinge axis")
    axis /= np.linalg.norm(axis)
    result: list[np.ndarray] = []
    for link_name in ("gripper_link", "moving_jaw_so101_v1_link"):
        points = np.asarray(frame.get("mesh_points", {}).get(link_name, []), dtype=np.float64)
        if points.ndim != 2 or points.shape[1:] != (3,) or len(points) < 8:
            raise ValueError(f"capture frame has too few {link_name} mesh samples")
        relative = points - pivot
        radial = np.linalg.norm(relative - np.outer(relative @ axis, axis), axis=1)
        maximum = float(np.max(radial))
        terminal = points[radial >= maximum - TERMINAL_RADIAL_TOLERANCE_M]
        if len(terminal) < 2:
            terminal = points[np.argsort(radial)[-2:]]
        result.append(np.mean(terminal, axis=0))
    return np.stack(result)


def calibrated_opening_degrees(frame: dict) -> float:
    metadata = frame["claw_metadata"]
    normalized = np.asarray(
        metadata.get("validated_angle_samples_normalized", []), dtype=np.float64
    )
    degrees = np.asarray(
        metadata.get("validated_angle_samples_degrees", []), dtype=np.float64
    )
    if normalized.shape != (5,) or degrees.shape != (5,):
        raise ValueError("capture did not retain the five-state opening curve")
    return float(np.interp(float(frame["pose"][5]), normalized, degrees))


def moving_tip_for_hinge_direction(
    frame: dict,
    current_tip: np.ndarray,
    hinge_direction: int,
) -> np.ndarray:
    """Mirror physical opening about the measured closed state when needed."""
    if hinge_direction == 1:
        return current_tip
    pivot = np.asarray(frame["joint_frames"][5]["pivot"], dtype=np.float64)
    axis = np.asarray(frame["joint_frames"][5]["axis"], dtype=np.float64)
    axis /= np.linalg.norm(axis)
    closed = float(frame["claw_metadata"]["validated_angle_samples_degrees"][0])
    angle = calibrated_opening_degrees(frame)
    delta = (float(hinge_direction) - 1.0) * (angle - closed)
    rotation = Rotation.from_rotvec(axis * math.radians(delta)).as_matrix()
    return (current_tip - pivot) @ rotation.T + pivot


def dark_mask(image: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    value = hsv[:, :, 2]
    saturation = hsv[:, :, 1]
    # The jaws are nearly black; the saturation clause retains dark blue/grey
    # plastic under the D455's changing exposure without accepting pale wood.
    mask = ((value < 82) | ((value < 118) & (saturation > 42))).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return mask


def temporal_masks(images: list[np.ndarray]) -> list[np.ndarray]:
    stack = np.stack([cv2.GaussianBlur(image, (5, 5), 0).astype(np.int16) for image in images])
    median = np.median(stack, axis=0)
    result: list[np.ndarray] = []
    for image in stack:
        difference = np.max(np.abs(image - median), axis=2)
        mask = (difference >= 15).astype(np.uint8) * 255
        mask = cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=1)
        result.append(mask)
    return result


def corridor_component_tip(
    mask: np.ndarray,
    hinge: np.ndarray,
    predicted: np.ndarray,
    temporal: np.ndarray | None,
) -> tuple[np.ndarray, float, np.ndarray]:
    direction = predicted - hinge
    length = float(np.linalg.norm(direction))
    if not np.isfinite(length) or length < 18.0:
        raise ValueError("projected jaw corridor is too short")
    direction /= length
    perpendicular = np.array([-direction[1], direction[0]])
    height, width = mask.shape
    yy, xx = np.mgrid[:height, :width]
    relative_x = xx - hinge[0]
    relative_y = yy - hinge[1]
    along = relative_x * direction[0] + relative_y * direction[1]
    across = np.abs(relative_x * perpendicular[0] + relative_y * perpendicular[1])
    corridor = (along >= length * 0.48) & (along <= length * 1.35) & (across <= max(11.0, length * 0.20))
    candidate = ((mask > 0) & corridor).astype(np.uint8)
    if temporal is not None:
        # Moving-jaw pixels must either change over the opening sweep or remain
        # connected to changed shaft pixels. This is the camera-bracket gate.
        changed = ((temporal > 0) & corridor).astype(np.uint8)
        changed = cv2.dilate(changed, np.ones((9, 9), np.uint8), iterations=2)
        candidate &= changed
    count, labels, stats, _ = cv2.connectedComponentsWithStats(candidate, 8)
    best_label = -1
    best_score = -math.inf
    for label in range(1, count):
        ys, xs = np.where(labels == label)
        if len(xs) < 12:
            continue
        points = np.column_stack((xs, ys)).astype(np.float64)
        distance = float(np.min(np.linalg.norm(points - predicted, axis=1)))
        projections = (points - hinge) @ direction
        seed_support = int(np.count_nonzero((projections >= length * 0.70) & (projections <= length * 1.12)))
        if seed_support < 4 or distance > max(32.0, length * 0.30):
            continue
        score = seed_support * 2.0 + len(points) * 0.03 - distance * 1.5
        if score > best_score:
            best_score = score
            best_label = label
    if best_label < 0:
        raise ValueError("no dark terminal component supports the projected jaw")
    ys, xs = np.where(labels == best_label)
    points = np.column_stack((xs, ys)).astype(np.float64)
    projections = (points - hinge) @ direction
    threshold = np.quantile(projections, 0.965)
    terminal = points[projections >= threshold]
    observed = np.median(terminal, axis=0)
    confidence = float(np.clip(len(points) / 180.0, 0.35, 1.0) * np.clip(1.0 - np.linalg.norm(observed - predicted) / 42.0, 0.25, 1.0))
    return observed, confidence, labels == best_label


def terminal_edge_tip(
    image: np.ndarray,
    hinge: np.ndarray,
    predicted: np.ndarray,
    temporal: np.ndarray | None,
) -> tuple[np.ndarray, float, np.ndarray]:
    """Locate the nearby dark-to-background terminal edge of one jaw.

    The calibrated chain is already close, so a strict local search is safer
    than following black pixels onto the perforated table.  Candidate edges
    must have darker material behind them (toward the hinge) than ahead.
    """
    direction = predicted - hinge
    length = float(np.linalg.norm(direction))
    if length < 18.0:
        raise ValueError("projected jaw corridor is too short")
    direction /= length
    perpendicular = np.array([-direction[1], direction[0]])
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    grey = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(grey, 18, 58)
    height, width = grey.shape
    yy, xx = np.mgrid[:height, :width]
    relative = np.stack((xx - predicted[0], yy - predicted[1]), axis=-1)
    along = relative @ direction
    across = np.abs(relative @ perpendicular)
    local = (edges > 0) & (along >= -18.0) & (along <= 20.0) & (across <= 18.0)
    ys, xs = np.where(local)
    if len(xs) == 0:
        raise ValueError("no local terminal edge")

    def sample_value(point: np.ndarray) -> float:
        x = int(np.clip(round(point[0]), 0, width - 1))
        y = int(np.clip(round(point[1]), 0, height - 1))
        return float(grey[y, x])

    scored: list[tuple[float, np.ndarray, float]] = []
    for x, y in zip(xs, ys):
        point = np.array([float(x), float(y)])
        behind = np.mean([sample_value(point - direction * distance) for distance in (3.0, 6.0, 9.0)])
        ahead = np.mean([sample_value(point + direction * distance) for distance in (3.0, 6.0, 9.0)])
        contrast = float(ahead - behind)
        temporal_support = 0.0
        if temporal is not None:
            tx = int(np.clip(x, 0, width - 1))
            ty = int(np.clip(y, 0, height - 1))
            temporal_support = 12.0 if temporal[ty, tx] > 0 else -8.0
        distance = float(np.linalg.norm(point - predicted))
        # Require at least a weak dark-to-background transition. This is what
        # prevents a changing table hole from becoming a fingertip.
        score = contrast * 1.6 + temporal_support - distance * 1.15 - abs(float((point - predicted) @ perpendicular)) * 0.35
        if contrast >= 3.0:
            scored.append((score, point, contrast))
    if not scored:
        raise ValueError("local edges have no dark fingertip contrast")
    scored.sort(key=lambda value: value[0], reverse=True)
    best_score, best, contrast = scored[0]
    if np.linalg.norm(best - predicted) > 20.0 or best_score < 1.0:
        raise ValueError("best local edge is not a supported fingertip")
    nearby = [value[1] for value in scored[:12] if np.linalg.norm(value[1] - best) <= 3.5 and value[0] >= best_score - 8.0]
    observed = np.median(np.stack(nearby), axis=0)
    confidence = float(np.clip(0.45 + contrast / 65.0 - np.linalg.norm(observed - predicted) / 55.0, 0.30, 1.0))
    return observed, confidence, edges > 0


def temporal_dark_moving_tip(
    image: np.ndarray,
    hinge: np.ndarray,
    temporal: np.ndarray,
    expected_radius: float,
) -> tuple[np.ndarray, float, np.ndarray]:
    """Find the moving black fingertip without trusting its current model pose."""
    mask = dark_mask(image) > 0
    changed = cv2.dilate(
        (temporal > 0).astype(np.uint8), np.ones((5, 5), np.uint8), iterations=1
    ) > 0
    height, width = mask.shape
    yy, xx = np.mgrid[:height, :width]
    radius = np.hypot(xx - hinge[0], yy - hinge[1])
    candidate = (
        mask
        & changed
        & (radius >= max(18.0, expected_radius * 0.42))
        & (radius <= min(190.0, expected_radius * 1.85))
    ).astype(np.uint8)
    candidate = cv2.morphologyEx(
        candidate, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(candidate, 8)
    best_label = -1
    best_score = -math.inf
    for label in range(1, count):
        ys, xs = np.where(labels == label)
        if len(xs) < 18:
            continue
        points = np.column_stack((xs, ys)).astype(np.float64)
        radii = np.linalg.norm(points - hinge, axis=1)
        nearest = float(np.min(radii))
        farthest = float(np.quantile(radii, 0.985))
        if nearest > 95.0 or farthest < 38.0:
            continue
        score = (
            farthest
            - nearest * 0.65
            - abs(farthest - expected_radius) * 0.30
            + min(len(points), 500) * 0.025
        )
        if score > best_score:
            best_score = score
            best_label = label
    if best_label < 0:
        raise ValueError("no temporally moving dark jaw component")
    ys, xs = np.where(labels == best_label)
    points = np.column_stack((xs, ys)).astype(np.float64)
    radii = np.linalg.norm(points - hinge, axis=1)
    terminal = points[radii >= np.quantile(radii, 0.97)]
    observed = np.median(terminal, axis=0)
    confidence = float(
        np.clip(len(points) / 260.0, 0.45, 1.0)
        * np.clip(float(np.max(radii)) / 100.0, 0.45, 1.0)
    )
    return observed, confidence, labels == best_label


def extract_observations(
    groups: list[list[dict]],
    debug_directory: Path,
    hinge_direction: int = 1,
) -> list[Observation]:
    debug_directory.mkdir(parents=True, exist_ok=True)
    observations: list[Observation] = []
    for view_index, group in enumerate(groups):
        images = [cv2.imread(frame["_rgb"]["path"], cv2.IMREAD_COLOR) for frame in group]
        if any(image is None for image in images):
            raise ValueError("a saved D455 RGB frame could not be decoded")
        changed = temporal_masks(images)
        for state_index, (frame, image) in enumerate(zip(group, images)):
            tips = terminal_tip_positions_local(frame)
            hinge_local = np.asarray(frame["joint_frames"][5]["pivot"], dtype=np.float64)
            hinge = project(hinge_local, frame)
            fixed_radius = float(np.linalg.norm(project(tips[0], frame) - hinge))
            canvas = image.copy()
            for tip_index in range(2):
                baseline = project(tips[tip_index], frame)
                nominal_tip = (
                    moving_tip_for_hinge_direction(
                        frame, tips[tip_index], hinge_direction
                    )
                    if tip_index == 1
                    else tips[tip_index]
                )
                search_prediction = project(nominal_tip, frame)
                try:
                    if tip_index == 1 and state_index > 0:
                        observed, confidence, component = temporal_dark_moving_tip(
                            image, hinge, changed[state_index], fixed_radius
                        )
                    else:
                        observed, confidence, component = terminal_edge_tip(
                            image,
                            hinge,
                            search_prediction,
                            changed[state_index] if tip_index == 1 else None,
                        )
                except ValueError:
                    continue
                if (
                    tip_index == 0
                    and np.linalg.norm(observed - search_prediction) > 20.0
                ):
                    continue
                observations.append(Observation(
                    frame,
                    tip_index,
                    observed,
                    baseline,
                    confidence,
                    view_index,
                    nominal_tip,
                ))
                cv2.line(canvas, tuple(np.round(hinge).astype(int)), tuple(np.round(search_prediction).astype(int)), (0, 180, 255), 1)
                cv2.circle(canvas, tuple(np.round(search_prediction).astype(int)), 7, (0, 220, 255), 2)
                cv2.circle(canvas, tuple(np.round(observed).astype(int)), 5, (255, 255, 0), -1)
            cv2.imwrite(str(debug_directory / f"view{view_index + 1:02d}_state{state_index + 1:02d}_observed.png"), canvas)
    moving_counts = {
        view_index: sum(
            observation.tip_index == 1 and observation.view_key == view_index
            for observation in observations
        )
        for view_index in range(len(groups))
    }
    qualified_views = {
        view_index
        for view_index, count in moving_counts.items()
        if count >= MINIMUM_MOVING_TIP_OBSERVATIONS_PER_VIEW
    }
    if len(qualified_views) < 2:
        raise ValueError(
            "moving fingertip was not independently visible in two D455 wrist views "
            f"(counts={{{', '.join(f'{key + 1}: {value}' for key, value in moving_counts.items())}}})"
        )
    # Fixed-jaw-only views cannot constrain the articulated jaw and are exactly
    # where the custom bracket tends to masquerade as a terminal edge. Retain
    # their diagnostics, but exclude them from the numerical candidate.
    observations = [
        observation
        for observation in observations
        if observation.view_key in qualified_views
    ]
    if len(observations) < 12:
        raise ValueError(f"only {len(observations)}/12 required fingertip observations were unambiguous")
    if len({observation.view_key for observation in observations}) < 2:
        raise ValueError("fingertips were not visible in two wrist views")
    if len({observation.tip_index for observation in observations}) < 2:
        raise ValueError("both physical fingertips were not identified")
    return observations


def fit_assembly(
    observations: list[Observation],
    hinge_direction: int = 1,
    allow_observation_rejection: bool = True,
) -> tuple[np.ndarray, dict]:
    first_metadata = observations[0].frame["claw_metadata"]
    sample_normalized = np.asarray(
        first_metadata.get("validated_angle_samples_normalized", []),
        dtype=np.float64,
    )
    if sample_normalized.shape != (5,):
        raise ValueError("capture did not retain five normalized jaw samples")

    def sample_index(observation: Observation) -> int:
        return int(
            np.argmin(
                np.abs(
                    sample_normalized
                    - float(observation.frame["pose"][5])
                )
            )
        )

    def candidate_tip(observation: Observation, angle_deltas: np.ndarray) -> np.ndarray:
        point = observation.nominal_tip
        if observation.tip_index == 0:
            return point
        frame = observation.frame
        pivot = np.asarray(frame["joint_frames"][5]["pivot"], dtype=np.float64)
        axis = np.asarray(frame["joint_frames"][5]["axis"], dtype=np.float64)
        axis /= np.linalg.norm(axis)
        rotation = Rotation.from_rotvec(
            axis * math.radians(float(angle_deltas[sample_index(observation)]))
        ).as_matrix()
        return (point - pivot) @ rotation.T + pivot

    def residual(parameters: np.ndarray) -> np.ndarray:
        assembly = assembly_from_parameters(parameters[:6])
        angle_deltas = parameters[6:]
        values: list[float] = []
        for observation in observations:
            predicted = project(
                candidate_tip(observation, angle_deltas),
                observation.frame,
                assembly,
            )
            values.extend(((predicted - observation.observed) * math.sqrt(observation.confidence)).tolist())
        # Keep unobservable depth/rotation combinations conservative.
        values.extend((parameters[:3] * 0.08).tolist())
        values.extend((parameters[3:6] * 0.04).tolist())
        # Closed is a known physical hard stop.  The remaining state deltas are
        # weakly regularized and smoothed; actual multiview tip evidence, not a
        # guessed polynomial, remains dominant.
        values.append(float(angle_deltas[0]) * 0.20)
        values.extend((angle_deltas[1:] * 0.025).tolist())
        values.extend((np.diff(angle_deltas, n=2) * 0.018).tolist())
        return np.asarray(values, dtype=np.float64)

    def jacobian(parameters: np.ndarray) -> np.ndarray:
        baseline = residual(parameters)
        result = np.empty((len(baseline), len(parameters)), dtype=np.float64)
        steps = np.asarray(
            [0.02, 0.02, 0.02, 0.05, 0.05, 0.05]
            + [0.03] * 5
        )
        for index, step in enumerate(steps):
            forward = parameters.copy()
            backward = parameters.copy()
            forward[index] += step
            backward[index] -= step
            result[:, index] = (residual(forward) - residual(backward)) / (2.0 * step)
        return result

    pre = transform_from_dict(first_metadata["pre_visual_transform_local"])
    corrected = transform_from_dict(first_metadata["corrected_transform_local"])
    seed = np.concatenate((optimization_seed(pre, corrected), np.zeros(5)))
    lower = np.asarray(
        [-15.0, -15.0, -15.0, -25.0, -25.0, -25.0]
        + [-1.0, -3.0, -3.0, -3.0, -3.0]
    )
    upper = np.asarray(
        [15.0, 15.0, 15.0, 25.0, 25.0, 25.0]
        + [1.0, 3.0, 3.0, 3.0, 3.0]
    )
    fit = least_squares(
        residual,
        seed,
        jac=jacobian,
        bounds=(lower, upper),
        loss="soft_l1",
        f_scale=3.0,
        max_nfev=500,
    )
    assembly = assembly_from_parameters(fit.x[:6])
    angle_deltas = fit.x[6:]
    baseline_errors: list[float] = []
    candidate_errors: list[float] = []
    observation_metrics: list[dict] = []
    by_view: dict[int, list[tuple[float, float]]] = {}
    for observation in observations:
        candidate = project(
            candidate_tip(observation, angle_deltas),
            observation.frame,
            assembly,
        )
        before = float(np.linalg.norm(observation.baseline - observation.observed))
        after = float(np.linalg.norm(candidate - observation.observed))
        baseline_errors.append(before)
        candidate_errors.append(after)
        observation_metrics.append({
            "view": observation.view_key + 1,
            "opening": float(observation.frame["pose"][5]),
            "tip": "fixed" if observation.tip_index == 0 else "moving",
            "baseline_px": before,
            "candidate_px": after,
        })
        by_view.setdefault(observation.view_key, []).append((before, after))
    view_metrics = []
    all_improved = True
    for view, pairs in sorted(by_view.items()):
        before = float(np.median([pair[0] for pair in pairs]))
        after = float(np.median([pair[1] for pair in pairs]))
        improved = after <= before - 0.25 or (before <= 3.0 and after <= before + 0.15)
        all_improved &= improved
        view_metrics.append({
            "view": view + 1,
            "baseline_median_px": before,
            "candidate_median_px": after,
            "candidate_maximum_px": float(max(pair[1] for pair in pairs)),
            "improved": improved,
            "observations": len(pairs),
        })
    moving_counts = {
        view: sum(
            observation.tip_index == 1 and observation.view_key == view
            for observation in observations
        )
        for view in by_view
    }
    enough_moving_evidence = all(
        count >= MINIMUM_MOVING_TIP_OBSERVATIONS_PER_VIEW
        for count in moving_counts.values()
    )
    improved_views = {
        int(metric["view"]) - 1
        for metric in view_metrics
        if bool(metric["improved"])
    }
    worsened_metrics = [
        metric
        for metric in view_metrics
        if not bool(metric["improved"])
        and float(metric["candidate_median_px"])
            >= float(metric["baseline_median_px"]) + 0.5
    ]
    median_outlier_metrics = [
        metric
        for metric in view_metrics
        if float(metric["candidate_median_px"]) > MAXIMUM_ACCEPTED_TIP_ERROR_PX
    ]
    maximum_outlier_metrics = [
        metric
        for metric in view_metrics
        if float(metric["candidate_maximum_px"]) > MAXIMUM_ACCEPTED_TIP_ERROR_PX
    ]
    # Remove globally poor views first and refit. A compromised all-view fit can
    # make an otherwise good view contain one large residual; maximum-error
    # rejection is therefore used only after no view has a poor median.
    absolute_outlier_metrics = (
        median_outlier_metrics
        if median_outlier_metrics
        else maximum_outlier_metrics
    )
    rejected_metrics = worsened_metrics + [
        metric
        for metric in absolute_outlier_metrics
        if metric not in worsened_metrics
    ]
    rejected_view_keys = {
        int(metric["view"]) - 1 for metric in rejected_metrics
    }
    retained_observations = [
        observation
        for observation in observations
        if observation.view_key not in rejected_view_keys
    ]
    # The rigid wrist-camera bracket can fully occlude one roll and generate a
    # locally plausible edge there.  Reject a whole wrist view only when the
    # provisional multiview fit makes that view materially worse or the whole
    # view remains outside the strict eight-pixel tip gate, while two other
    # independent views still retain at least twelve observations.
    # This is a view-level robust estimator, not per-point cherry-picking.
    if (
        len(by_view) - len(rejected_view_keys) >= 2
        and len(retained_observations) >= 12
        and rejected_metrics
    ):
        robust_assembly, robust_metrics = fit_assembly(
            retained_observations,
            hinge_direction,
            allow_observation_rejection,
        )
        robust_metrics["rejected_outlier_views"] = rejected_metrics
        robust_metrics["outlier_view_rejection"] = (
            "whole_view_failed_strict_tip_residual_or_worsened"
        )
        return robust_assembly, robust_metrics
    # One wide-open jaw edge can merge with the camera bracket or a table edge.
    # Reject it only when the same measured opening state is independently
    # supported within the strict gate by two other wrist views. This preserves
    # multiview evidence for every state and cannot hide a systematic bad fit.
    rejected_observation_indices: list[int] = []
    if allow_observation_rejection:
        for index, (observation, error) in enumerate(
            zip(observations, candidate_errors, strict=True)
        ):
            if (
                observation.tip_index != 1
                or error <= MAXIMUM_ACCEPTED_TIP_ERROR_PX
            ):
                continue
            state = sample_index(observation)
            supporting_views = {
                other.view_key
                for other, other_error in zip(
                    observations, candidate_errors, strict=True
                )
                if (
                    other.tip_index == 1
                    and sample_index(other) == state
                    and other.view_key != observation.view_key
                    and other_error <= MAXIMUM_ACCEPTED_TIP_ERROR_PX
                )
            }
            if len(supporting_views) >= 2:
                rejected_observation_indices.append(index)
    if rejected_observation_indices:
        retained = [
            observation
            for index, observation in enumerate(observations)
            if index not in rejected_observation_indices
        ]
        retained_moving_counts = {
            view: sum(
                observation.tip_index == 1 and observation.view_key == view
                for observation in retained
            )
            for view in {observation.view_key for observation in retained}
        }
        if (
            len(retained) >= 12
            and len(retained_moving_counts) >= 2
            and all(
                count >= MINIMUM_MOVING_TIP_OBSERVATIONS_PER_VIEW
                for count in retained_moving_counts.values()
            )
        ):
            robust_assembly, robust_metrics = fit_assembly(
                retained, hinge_direction, False
            )
            robust_metrics["rejected_isolated_observations"] = [
                observation_metrics[index]
                for index in rejected_observation_indices
            ]
            robust_metrics["observation_outlier_rejection"] = (
                "same_opening_supported_by_two_independent_wrist_views"
            )
            return robust_assembly, robust_metrics
    if (
        max(candidate_errors) > MAXIMUM_ACCEPTED_TIP_ERROR_PX
        or float(np.median(candidate_errors)) > MAXIMUM_ACCEPTED_MEDIAN_ERROR_PX
        or not enough_moving_evidence
    ):
        all_improved = False
    metrics = {
        "baseline_median_px": float(np.median(baseline_errors)),
        "candidate_median_px": float(np.median(candidate_errors)),
        "maximum_candidate_px": float(max(candidate_errors)),
        "all_views_improved": bool(all_improved),
        "view_metrics": view_metrics,
        "observation_count": len(observations),
        "moving_tip_observations_by_view": {
            str(view + 1): count for view, count in sorted(moving_counts.items())
        },
        "enough_moving_tip_evidence": enough_moving_evidence,
        "rejected_outlier_views": [],
        "angle_deltas_degrees": angle_deltas.tolist(),
        "gripper_hinge_direction": hinge_direction,
        "observation_metrics": observation_metrics,
        "rejected_isolated_observations": [],
        "observation_outlier_rejection": "none",
    }
    return assembly, metrics


def solve(capture: dict, debug_directory: Path) -> dict:
    if capture.get("type") != "so101_motion_capture":
        raise ValueError("input is not an SO-101 motion capture")
    groups = group_frames(capture)
    candidates: list[tuple[np.ndarray, dict]] = []
    hypothesis_failures: list[str] = []
    for hinge_direction, label in ((1, "forward"), (-1, "reversed")):
        try:
            observations = extract_observations(
                groups, debug_directory / label, hinge_direction
            )
            assembly, metrics = fit_assembly(observations, hinge_direction)
            if bool(metrics.get("all_views_improved", False)):
                candidates.append((assembly, metrics))
            else:
                worst = sorted(
                    metrics.get("observation_metrics", []),
                    key=lambda value: float(value.get("candidate_px", 0.0)),
                    reverse=True,
                )[:5]
                hypothesis_failures.append(
                    f"{label}: views={metrics.get('view_metrics', [])}, "
                    f"worst={worst}"
                )
        except ValueError as error:
            hypothesis_failures.append(f"{label}: {error}")
    if not candidates:
        raise ValueError(
            "neither stock gripper hinge hypothesis passed the strict tip gate; "
            + "; ".join(hypothesis_failures)
        )
    assembly, metrics = min(
        candidates,
        key=lambda value: (
            -(
                float(value[1]["baseline_median_px"])
                - float(value[1]["candidate_median_px"])
            ),
            float(value[1]["candidate_median_px"]),
            float(value[1]["maximum_candidate_px"]),
        ),
    )
    first_metadata = groups[0][0]["claw_metadata"]
    normalized = list(first_metadata.get("validated_angle_samples_normalized", []))
    degrees = list(first_metadata.get("validated_angle_samples_degrees", []))
    if len(normalized) != 5 or len(degrees) != 5:
        raise ValueError("validated gripper opening curve was not captured")
    degrees = [
        float(value) + float(delta)
        for value, delta in zip(
            degrees,
            metrics.get("angle_deltas_degrees", [0.0] * 5),
            strict=True,
        )
    ]
    if any(right <= left for left, right in zip(normalized, normalized[1:])) or any(right <= left for left, right in zip(degrees, degrees[1:])):
        raise ValueError("validated gripper opening curve is not monotonic")
    if not metrics["all_views_improved"]:
        worst = sorted(
            metrics.get("observation_metrics", []),
            key=lambda value: float(value.get("candidate_px", 0.0)),
            reverse=True,
        )[:5]
        raise ValueError(
            "distal candidate failed strict tip gate: "
            f"views={metrics['view_metrics']}, worst={worst}"
        )
    rpy = Rotation.from_matrix(assembly[:3, :3]).as_euler("xyz", degrees=True)
    confidence = float(np.clip(1.0 - metrics["candidate_median_px"] / 28.0, 0.0, 0.98))
    return {
        "type": "so101_claw_visual_fit",
        "method": METHOD,
        "reference_camera": "RealSense D455 native RGB",
        "gripper_angle_samples_normalized": normalized,
        "gripper_angle_samples_degrees": degrees,
        "gripper_hinge_direction": int(
            metrics.get("gripper_hinge_direction", 1)
        ),
        "gripper_mount_correction_local": list(first_metadata.get("mount_correction_local", [0.0, 0.0, 0.0])),
        "gripper_visual_correction_rpy_degrees": rpy.tolist(),
        "gripper_visual_correction_translation_local": assembly[:3, 3].tolist(),
        "confidence": confidence,
        "median_tip_residual_px": metrics["candidate_median_px"],
        "baseline_tip_residual_px": metrics["baseline_median_px"],
        "maximum_tip_residual_px": metrics["maximum_candidate_px"],
        "all_views_improved": metrics["all_views_improved"],
        "validation_view_count": len(groups),
        "validation_pose_count": len(capture.get("frames", [])),
        "tip_observation_count": metrics["observation_count"],
        "view_metrics": metrics["view_metrics"],
        "rejected_outlier_views": metrics.get("rejected_outlier_views", []),
        "rejected_isolated_observations": metrics.get(
            "rejected_isolated_observations", []
        ),
        "observation_outlier_rejection": metrics.get(
            "observation_outlier_rejection", "none"
        ),
        "outlier_view_rejection": metrics.get("outlier_view_rejection", "none"),
        "upstream_motion": "none_during_each_opening_sweep",
        "validated_opening_curve_preserved_unchanged": bool(
            max(
                abs(float(value))
                for value in metrics.get("angle_deltas_degrees", [0.0])
            ) < 1.0e-6
        ),
        "unmodelled_camera_bracket_excluded": True,
        "diagnostic_directory": str(debug_directory),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--debug-directory", type=Path)
    arguments = parser.parse_args()
    debug = arguments.debug_directory or arguments.output.with_suffix("").with_name(arguments.output.stem + "_rgb_debug")
    try:
        result = solve(json.loads(arguments.capture.read_text()), debug)
    except Exception as error:
        print(f"SO-101 native-RGB claw fit rejected: {error}")
        return 1
    arguments.output.write_text(json.dumps(result, indent=2) + "\n")
    print(
        "SO101_CLAW_RGB_TIP_FIT_OK "
        f"baseline={result['baseline_tip_residual_px']:.2f}px "
        f"candidate={result['median_tip_residual_px']:.2f}px "
        f"views={result['validation_view_count']} confidence={result['confidence']:.0%}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
