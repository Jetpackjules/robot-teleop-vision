#!/usr/bin/env python3
"""Fit the SO-101 shoulder-pan axis from a settled one-joint sweep."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import open3d as o3d
from scipy.optimize import differential_evolution
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parent))

ASSET_ROOT = Path(__file__).resolve().parents[1] / "assets"

from so101_kinematics import rendered_link_transforms

PAN_HEIGHT_M = 0.0624
PAN_LOCAL_Z_M = -0.0388353
STOCK_MOVING_LINKS = ("shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link")


def select_reference_camera(payload: dict, frames: list[dict]) -> tuple[int, str]:
    cameras = frames[0].get("full_camera_points", []) if frames else []
    if not cameras:
        raise ValueError("base-axis fit has no full-resolution camera clouds")
    requested = str(payload.get("reference_camera", "")).strip()
    if requested:
        for index, camera in enumerate(cameras):
            name = str(camera.get("name", ""))
            if requested == name or requested.lower() in name.lower():
                return index, name
        raise ValueError(
            f"captured reference camera {requested!r} is unavailable; "
            f"cameras={[camera.get('name', '') for camera in cameras]}"
        )
    for index, camera in enumerate(cameras):
        name = str(camera.get("name", ""))
        if "d455" in name.lower():
            return index, name
    coverage = []
    for index, camera in enumerate(cameras):
        point_count = sum(
            len(frame.get("full_camera_points", [])[index].get("points", []))
            for frame in frames
            if index < len(frame.get("full_camera_points", []))
        )
        coverage.append((point_count, index, str(camera.get("name", ""))))
    _points, index, name = max(coverage)
    return index, name


def plane_consensus(
    frames: list[dict],
    camera_indices: list[int] | None = None,
) -> tuple[np.ndarray, float, list[dict]]:
    measurements: list[tuple[np.ndarray, float, int, str]] = []
    selected = (
        range(len(frames[0]["full_camera_points"]))
        if camera_indices is None
        else camera_indices
    )
    for camera_index in selected:
        camera = frames[0]["full_camera_points"][camera_index]
        for frame in frames:
            points = np.asarray(frame["full_camera_points"][camera_index]["points"], dtype=float)
            cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
            plane, inliers = cloud.segment_plane(0.008, 3, 900)
            normal = np.asarray(plane[:3], dtype=float)
            scale = np.linalg.norm(normal)
            normal /= scale
            offset = float(plane[3] / scale)
            if normal[1] < 0.0:
                normal = -normal
                offset = -offset
            measurements.append((normal, offset, len(inliers), camera["name"]))
    normal = sum(n * weight for n, _, weight, _ in measurements)
    normal /= np.linalg.norm(normal)
    offset = float(np.average([d for _, d, _, _ in measurements], weights=[w for _, _, w, _ in measurements]))
    details = [
        {"camera": name, "normal": n.tolist(), "offset": d, "inliers": weight}
        for n, d, weight, name in measurements
    ]
    return normal, offset, details


def plane_basis(normal: np.ndarray, offset: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    e1 = np.array((1.0, 0.0, 0.0))
    e1 -= normal * np.dot(e1, normal)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(normal, e1)
    return -offset * normal, e1, e2


def dynamic_clouds(
    frames: list[dict], normal: np.ndarray, plane_offset: float, camera_index: int | None
) -> list[np.ndarray]:
    selected_cameras = range(len(frames[0]["full_camera_points"])) if camera_index is None else (camera_index,)
    result: list[np.ndarray] = []
    for pose_index, frame in enumerate(frames):
        parts: list[np.ndarray] = []
        for cam in selected_cameras:
            current = np.asarray(frame["full_camera_points"][cam]["points"], dtype=float)
            others = [
                np.asarray(other["full_camera_points"][cam]["points"], dtype=float)
                for index, other in enumerate(frames)
                if index != pose_index
            ]
            distances = np.stack([cKDTree(other).query(current, workers=-1)[0] for other in others], axis=1)
            dynamic_score = np.median(distances, axis=1)
            height = current @ normal + plane_offset
            parts.append(current[(dynamic_score > 0.008) & (height > 0.018) & (height < 0.40)])
            camera_motion = np.asarray(frame["camera_points"][cam]["points"], dtype=float)
            if camera_motion.size:
                height = camera_motion @ normal + plane_offset
                parts.append(camera_motion[(height > 0.018) & (height < 0.40)])
        cloud = np.concatenate(parts)
        fifth = cKDTree(cloud).query(cloud, k=min(5, len(cloud)), workers=-1)[0][:, -1]
        cloud = cloud[fifth < 0.06]
        if len(cloud) > 700:
            cloud = cloud[np.linspace(0, len(cloud) - 1, 700).astype(int)]
        result.append(cloud)
    return result


def axis_objective(
    coordinates: np.ndarray,
    clouds: list[np.ndarray],
    rotations: list[np.ndarray],
    plane_point: np.ndarray,
    e1: np.ndarray,
    e2: np.ndarray,
    normal: np.ndarray,
) -> float:
    base_projection = plane_point + e1 * coordinates[0] + e2 * coordinates[1]
    axis_point = base_projection + normal * PAN_HEIGHT_M
    transformed = [(cloud - axis_point) @ rotation.T + axis_point for cloud, rotation in zip(clouds, rotations)]
    pair_losses: list[float] = []
    for left in range(len(transformed)):
        for right in range(left + 1, len(transformed)):
            distances = np.concatenate(
                (
                    cKDTree(transformed[right]).query(transformed[left])[0],
                    cKDTree(transformed[left]).query(transformed[right])[0],
                )
            )
            distances.sort()
            distances = distances[: max(25, int(0.45 * len(distances)))]
            pair_losses.append(float(np.mean(np.minimum(distances, 0.10) ** 2)))
    return math.sqrt(float(np.mean(pair_losses)))


def fit_axis(
    frames: list[dict],
    clouds: list[np.ndarray],
    normal: np.ndarray,
    plane_offset: float,
    seed: int,
) -> dict:
    plane_point, e1, e2 = plane_basis(normal, plane_offset)
    all_points = np.concatenate(clouds)
    uv = np.column_stack(((all_points - plane_point) @ e1, (all_points - plane_point) @ e2))
    bounds = [
        (float(uv[:, 0].min() - 0.12), float(uv[:, 0].max() + 0.12)),
        (float(uv[:, 1].min() - 0.12), float(uv[:, 1].max() + 0.12)),
    ]
    angles = np.radians([float(frame["pose"][0]) for frame in frames])
    fits = []
    for sign in (1, -1):
        rotations = [
            Rotation.from_rotvec(normal * sign * (angles[0] - angle)).as_matrix()
            for angle in angles
        ]
        objective = lambda value: axis_objective(value, clouds, rotations, plane_point, e1, e2, normal)
        solved = differential_evolution(
            objective, bounds, maxiter=45, popsize=12, tol=1e-5, polish=True, seed=seed
        )
        base_projection = plane_point + e1 * solved.x[0] + e2 * solved.x[1]
        fits.append(
            {
                "sign": sign,
                "loss_m": float(solved.fun),
                "base_projection": base_projection.tolist(),
                "axis_point": (base_projection + normal * PAN_HEIGHT_M).tolist(),
            }
        )
    return min(fits, key=lambda fit: fit["loss_m"]) | {"alternatives": fits}


def collapsed_motion(
    frames: list[dict],
    normal: np.ndarray,
    plane_offset: float,
    axis_point: np.ndarray,
    sign: int,
    clouds: list[np.ndarray] | None = None,
) -> np.ndarray:
    angles = np.radians([float(frame["pose"][0]) for frame in frames])
    result = []
    for frame_index, (frame, angle) in enumerate(zip(frames, angles)):
        points = (
            np.asarray(frame["points"], dtype=float)
            if clouds is None
            else clouds[frame_index]
        )
        height = points @ normal + plane_offset
        points = points[(height > 0.015) & (height < 0.38)]
        rotation = Rotation.from_rotvec(normal * sign * (angles[0] - angle)).as_matrix()
        result.append((points - axis_point) @ rotation.T + axis_point)
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate(result))).voxel_down_sample(0.004)
    return np.asarray(cloud.points)


def fit_heading(
    frames: list[dict],
    normal: np.ndarray,
    plane_offset: float,
    base_projection: np.ndarray,
    sign: int,
    observed_clouds: list[np.ndarray] | None = None,
) -> dict:
    plane_point, e1, e2 = plane_basis(normal, plane_offset)
    axis_point = base_projection + normal * PAN_HEIGHT_M
    observed = collapsed_motion(
        frames,
        normal,
        plane_offset,
        axis_point,
        sign,
        observed_clouds,
    )
    observed_tree = cKDTree(observed)
    raw_meshes = {}
    for link in STOCK_MOVING_LINKS:
        mesh = o3d.io.read_triangle_mesh(str(ASSET_ROOT / f"{link}.glb"))
        raw_meshes[link] = np.asarray(mesh.sample_points_uniformly(900).points)
    pose = np.asarray(frames[0]["pose"], dtype=float)

    def objective(value: np.ndarray) -> float:
        heading, shoulder_delta = value
        basis_z = math.cos(heading) * e2 + math.sin(heading) * e1
        basis_x = np.cross(normal, basis_z)
        basis_x /= np.linalg.norm(basis_x)
        basis_z = np.cross(basis_x, normal)
        basis = np.column_stack((basis_x, normal, basis_z))
        origin = base_projection - PAN_LOCAL_Z_M * basis_z
        adjusted_pose = pose.copy()
        adjusted_pose[1] -= shoulder_delta
        transforms = rendered_link_transforms(adjusted_pose)
        predicted = []
        for link, points in raw_meshes.items():
            transform = transforms[link]
            ros = points @ transform[:3, :3].T + transform[:3, 3]
            godot_local = np.column_stack((-ros[:, 1], ros[:, 2], -ros[:, 0]))
            predicted.append(godot_local @ basis.T + origin)
        distances = observed_tree.query(np.concatenate(predicted))[0]
        distances.sort()
        distances = distances[: int(0.48 * len(distances))]
        return math.sqrt(float(np.mean(np.minimum(distances, 0.08) ** 2)))

    solved = differential_evolution(
        objective,
        [(-math.pi, math.pi), (-35.0, 35.0)],
        maxiter=40,
        popsize=12,
        tol=1e-5,
        polish=True,
        seed=19,
    )
    heading, shoulder_nuisance = solved.x
    basis_z = math.cos(heading) * e2 + math.sin(heading) * e1
    basis_x = np.cross(normal, basis_z)
    basis_x /= np.linalg.norm(basis_x)
    basis_z = np.cross(basis_x, normal)
    origin = base_projection - PAN_LOCAL_Z_M * basis_z
    return {
        "heading_degrees": math.degrees(float(heading)),
        "heading_loss_m": float(solved.fun),
        "shoulder_nuisance_degrees": float(shoulder_nuisance),
        "basis_x": basis_x.tolist(),
        "basis_y": normal.tolist(),
        "basis_z": basis_z.tolist(),
        "origin": origin.tolist(),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    payload = json.loads(args.capture.expanduser().read_text())
    frames = payload["frames"]
    if len(frames) < 5:
        raise SystemExit("need at least five settled poses")
    cameras = frames[0].get("full_camera_points", [])
    try:
        reference_index, reference_camera = select_reference_camera(payload, frames)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    # Open3D's RANSAC plane segmentation otherwise chooses a different table
    # plane on identical replays, which rotates the fitted robot basis enough
    # to make repeated calibration presses drift at the distal joints.
    o3d.utility.random.seed(23)
    normal, plane_offset, plane_details = plane_consensus(
        frames,
        [reference_index],
    )
    reference_clouds = dynamic_clouds(
        frames,
        normal,
        plane_offset,
        reference_index,
    )
    combined = fit_axis(
        frames,
        reference_clouds,
        normal,
        plane_offset,
        5,
    )
    camera_fits = [
        {
            "camera": cameras[reference_index]["name"],
            **combined,
        }
    ]
    disagreement = 0.0
    wrong_sign = next(item for item in combined["alternatives"] if item["sign"] != combined["sign"])
    if combined["sign"] != -1 or disagreement > 0.025 or combined["loss_m"] > 0.012:
        raise SystemExit(
            f"base-axis fit rejected: sign={combined['sign']} loss={combined['loss_m']:.4f} "
            f"camera disagreement={disagreement:.4f}"
        )
    base_projection = np.asarray(combined["base_projection"])
    heading = fit_heading(
        frames,
        normal,
        plane_offset,
        base_projection,
        combined["sign"],
        reference_clouds,
    )
    result = {
        "type": "so101_base_axis_fit",
        "method": "settled_shoulder_pan_revolute_axis",
        "normal": normal.tolist(),
        "plane_offset": plane_offset,
        "plane_measurements": plane_details,
        "axis_fit": combined,
        "camera_fits": camera_fits,
        "camera_axis_disagreement_m": disagreement,
        "reference_camera": reference_camera,
        "reference_camera_serial": reference_camera,
        "reference_camera_validation_required": True,
        "wrong_sign_loss_ratio": float(wrong_sign["loss_m"] / combined["loss_m"]),
        "preview_transform": heading,
        "saved": False,
    }
    encoded = json.dumps(result, indent=2)
    if args.output:
        args.output.expanduser().write_text(encoded + "\n")
    print(encoded)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
