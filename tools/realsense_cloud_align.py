#!/usr/bin/env python3
"""RealSense-to-RealSense point-cloud alignment worker.

This captures two RealSense cameras by serial, builds view-coordinate point
clouds, optionally uses RGB texture/features, and writes a serial-keyed
alignment result for Godot to apply.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import os
import sys
import time
from pathlib import Path
from argparse import Namespace

import numpy as np
import pyrealsense2 as rs


PROFILE_DEPTH = {
    "fast60": (848, 480, 60),
    "viewer30": (848, 480, 30),
    "highres30": (1280, 720, 30),
}


def write_result(path: str, payload: dict) -> None:
    if not path:
        print(json.dumps(payload, indent=2), flush=True)
        return
    result_path = Path(path)
    result_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def fail(args: argparse.Namespace, status: str, details: dict | None = None) -> int:
    write_result(
        args.result_path,
        {
            "type": "realsense_cloud_alignment",
            "ok": False,
            "method": args.mode,
            "status": status,
            "reference_camera": f"realsense:{args.reference_serial}",
            "target_camera": f"realsense:{args.target_serial}",
            "details": details or {},
            "timestamp": time.time(),
        },
    )
    print(status, file=sys.stderr, flush=True)
    return 1


def parse_transform(text: str) -> np.ndarray:
    if not text:
        return np.eye(4, dtype=np.float64)
    payload = json.loads(text)
    if "matrix" in payload:
        matrix = np.asarray(payload["matrix"], dtype=np.float64)
        if matrix.shape == (4, 4):
            return matrix
    r = np.asarray(payload.get("R", np.eye(3)), dtype=np.float64)
    t = np.asarray(payload.get("T", [0.0, 0.0, 0.0]), dtype=np.float64)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = r.reshape(3, 3)
    transform[:3, 3] = t.reshape(3)
    return transform


def profile_tuple(profile: str) -> tuple[int, int, int]:
    return PROFILE_DEPTH.get(str(profile).strip().lower(), PROFILE_DEPTH["viewer30"])


def capture_depth(serial: str, profile: str, frames: int, timeout_s: float, capture_color: bool) -> tuple[np.ndarray, np.ndarray | None, rs.intrinsics, dict]:
    width, height, fps = profile_tuple(profile)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    if capture_color:
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    started = False
    try:
        pipeline_profile = pipeline.start(config)
        started = True
        align_to_depth = rs.align(rs.stream.depth) if capture_color else None
        depth_stream = pipeline_profile.get_stream(rs.stream.depth).as_video_stream_profile()
        intrinsics = depth_stream.get_intrinsics()
        depth_sensor = pipeline_profile.get_device().first_depth_sensor()
        depth_scale = float(depth_sensor.get_depth_scale())

        deadline = time.time() + max(1.0, float(timeout_s))
        frame_timeout_ms = int(max(1000, min(5000, float(timeout_s) * 1000.0)))
        samples: list[np.ndarray] = []
        color_samples: list[np.ndarray] = []
        while time.time() < deadline and len(samples) < max(1, int(frames)):
            try:
                frame_set = pipeline.wait_for_frames(frame_timeout_ms)
            except RuntimeError:
                continue
            if align_to_depth is not None:
                frame_set = align_to_depth.process(frame_set)
            depth_frame = frame_set.get_depth_frame()
            if not depth_frame:
                continue
            depth = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
            if np.isfinite(depth).any():
                samples.append(depth)
                if capture_color:
                    color_frame = frame_set.get_color_frame()
                    if color_frame:
                        color_bgr = np.asanyarray(color_frame.get_data())
                        color_samples.append(color_bgr[:, :, ::-1].copy())
        if not samples:
            raise RuntimeError("no depth frames received")
        if len(samples) == 1:
            depth_m = samples[0]
        else:
            stack = np.stack(samples, axis=0)
            stack[stack <= 0.0] = np.nan
            depth_m = np.nanmedian(stack, axis=0).astype(np.float32)
            depth_m[~np.isfinite(depth_m)] = 0.0
        color_rgb = None
        if capture_color and color_samples:
            color_rgb = color_samples[-1]
        return depth_m, color_rgb, intrinsics, {
            "width": int(width),
            "height": int(height),
            "fps": int(fps),
            "frames": int(len(samples)),
            "color_frames": int(len(color_samples)),
            "depth_scale": depth_scale,
        }
    finally:
        if started:
            pipeline.stop()


def depth_to_view_points(depth_m, intrinsics, min_depth, max_depth, stride, max_points):
    points, _colors = depth_to_view_points_and_colors(depth_m, None, intrinsics, min_depth, max_depth, stride, max_points)
    return points


def depth_to_view_points_and_colors(depth_m, color_rgb, intrinsics, min_depth, max_depth, stride, max_points):
    stride = max(1, int(stride))
    rows = np.arange(0, depth_m.shape[0], stride, dtype=np.int32)
    cols = np.arange(0, depth_m.shape[1], stride, dtype=np.int32)
    sampled = depth_m[np.ix_(rows, cols)].astype(np.float32, copy=False)
    valid = np.isfinite(sampled) & (sampled >= float(min_depth)) & (sampled <= float(max_depth))
    if int(valid.sum()) <= 0:
        return None, None
    grid_x = cols[np.newaxis, :].astype(np.float32)
    grid_y = rows[:, np.newaxis].astype(np.float32)
    z = sampled
    x = (grid_x - float(intrinsics.ppx)) * z / max(1e-6, float(intrinsics.fx))
    y = (grid_y - float(intrinsics.ppy)) * z / max(1e-6, float(intrinsics.fy))
    points = np.stack([x, -y, -z], axis=2)[valid].astype(np.float64, copy=False)
    colors = None
    if color_rgb is not None:
        sampled_color = color_rgb[np.ix_(rows, cols)]
        colors = sampled_color[valid].astype(np.float64, copy=False) / 255.0
    if max_points > 0 and points.shape[0] > max_points:
        indices = np.linspace(0, points.shape[0] - 1, int(max_points), dtype=np.int64)
        points = points[indices]
        if colors is not None:
            colors = colors[indices]
    return points, colors


def transform_delta(initial: np.ndarray, solved: np.ndarray) -> tuple[float, float]:
    delta = solved @ np.linalg.inv(initial)
    translation_m = float(np.linalg.norm(delta[:3, 3]))
    trace = float(np.trace(delta[:3, :3]))
    cos_theta = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    return translation_m, float(math.degrees(math.acos(cos_theta)))


def transform_payload(transform: np.ndarray) -> dict:
    return {
        "R": transform[:3, :3].tolist(),
        "T": transform[:3, 3].tolist(),
    }


def make_cloud(o3d, points: np.ndarray):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    return cloud


def make_colored_cloud(o3d, points: np.ndarray, colors: np.ndarray | None):
    cloud = make_cloud(o3d, points)
    if colors is not None and colors.shape[0] == points.shape[0]:
        cloud.colors = o3d.utility.Vector3dVector(colors)
    return cloud


def downsample_with_normals(o3d, cloud, voxel: float):
    down = cloud.voxel_down_sample(float(voxel))
    if len(down.points) < 100:
        down = cloud
    down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=float(voxel) * 2.5, max_nn=40))
    return down


def preprocess(o3d, points: np.ndarray, voxel: float):
    cloud = make_cloud(o3d, points)
    down = downsample_with_normals(o3d, cloud, voxel)
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=float(voxel) * 5.0, max_nn=100),
    )
    return cloud, down, fpfh


def icp_scale_schedule(base_voxel: float) -> list[float]:
    base = max(0.006, float(base_voxel))
    return sorted(
        {
            round(max(0.006, base), 4),
            round(max(0.006, base * 0.55), 4),
            round(max(0.006, base * 0.32), 4),
        },
        reverse=True,
    )


def run_icp_multiscale(o3d, source, target, init: np.ndarray, args: argparse.Namespace):
    transform = init
    stages: list[dict] = []
    estimation = "point_to_plane"
    last_result = None
    user_max_corr = float(args.max_correspondence)
    for scale in icp_scale_schedule(float(args.voxel)):
        source_down = downsample_with_normals(o3d, source, scale)
        target_down = downsample_with_normals(o3d, target, scale)
        if len(source_down.points) < 100 or len(target_down.points) < 100:
            continue
        scale_corr = user_max_corr if user_max_corr > 0.0 else max(scale * 2.2, 0.012)
        scale_iterations = int(args.icp_iterations) if scale >= float(args.voxel) * 0.9 else int(args.fine_icp_iterations)
        try:
            result = o3d.pipelines.registration.registration_icp(
                source_down,
                target_down,
                scale_corr,
                transform,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=scale_iterations),
            )
            stage_estimation = "point_to_plane"
        except Exception:
            result = o3d.pipelines.registration.registration_icp(
                source_down,
                target_down,
                scale_corr,
                transform,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=scale_iterations),
            )
            stage_estimation = "point_to_point"
        transform = np.asarray(result.transformation, dtype=np.float64)
        last_result = result
        estimation = stage_estimation
        stages.append(
            {
                "voxel_m": scale,
                "max_correspondence_m": scale_corr,
                "source_down_points": int(len(source_down.points)),
                "target_down_points": int(len(target_down.points)),
                "fitness": float(result.fitness),
                "rmse": float(result.inlier_rmse),
                "iterations": scale_iterations,
                "estimation": stage_estimation,
            }
        )
    if last_result is None:
        raise RuntimeError("ICP refinement was too sparse at every scale")
    return last_result, transform, estimation, stages


def run_colored_icp_multiscale(o3d, source, target, init: np.ndarray, args: argparse.Namespace):
    if not source.has_colors() or not target.has_colors():
        raise RuntimeError("colored ICP needs RGB color from both RealSense cameras")
    transform = init
    stages: list[dict] = []
    last_result = None
    user_max_corr = float(args.max_correspondence)
    for scale in icp_scale_schedule(float(args.voxel)):
        source_down = downsample_with_normals(o3d, source, scale)
        target_down = downsample_with_normals(o3d, target, scale)
        if len(source_down.points) < 100 or len(target_down.points) < 100:
            continue
        scale_corr = user_max_corr if user_max_corr > 0.0 else max(scale * 2.2, 0.012)
        scale_iterations = int(args.icp_iterations) if scale >= float(args.voxel) * 0.9 else int(args.fine_icp_iterations)
        result = o3d.pipelines.registration.registration_colored_icp(
            source_down,
            target_down,
            scale_corr,
            transform,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(lambda_geometric=float(args.color_icp_geometry_weight)),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=scale_iterations),
        )
        transform = np.asarray(result.transformation, dtype=np.float64)
        last_result = result
        stages.append(
            {
                "voxel_m": scale,
                "max_correspondence_m": scale_corr,
                "source_down_points": int(len(source_down.points)),
                "target_down_points": int(len(target_down.points)),
                "fitness": float(result.fitness),
                "rmse": float(result.inlier_rmse),
                "iterations": scale_iterations,
                "estimation": "colored_icp",
            }
        )
    if last_result is None:
        raise RuntimeError("colored ICP refinement was too sparse at every scale")
    return last_result, transform, "colored_icp", stages


def transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    return (transform[:3, :3] @ points.T).T + transform[:3, 3]


def rotation_matrix(axis: int, degrees: float) -> np.ndarray:
    angle = math.radians(float(degrees))
    c = math.cos(angle)
    s = math.sin(angle)
    r = np.eye(4, dtype=np.float64)
    if axis == 0:
        r[:3, :3] = np.asarray([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)
    elif axis == 1:
        r[:3, :3] = np.asarray([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)
    else:
        r[:3, :3] = np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return r


def rotation_about_point_matrix(axis: int, degrees: float, center: np.ndarray) -> np.ndarray:
    to_origin = np.eye(4, dtype=np.float64)
    from_origin = np.eye(4, dtype=np.float64)
    center = np.asarray(center, dtype=np.float64).reshape(3)
    to_origin[:3, 3] = -center
    from_origin[:3, 3] = center
    return from_origin @ rotation_matrix(axis, degrees) @ to_origin


def translation_matrix(axis: int, meters: float) -> np.ndarray:
    t = np.eye(4, dtype=np.float64)
    t[axis, 3] = float(meters)
    return t


def score_transform(o3d, source_down, target_tree, target_count: int, transform: np.ndarray, max_distance: float) -> tuple[float, dict]:
    transformed = transform_points(transform, np.asarray(source_down.points))
    if transformed.shape[0] <= 0 or target_count <= 0:
        return float("inf"), {"fitness": 0.0, "rmse": 999.0, "inliers": 0}
    original_count = int(transformed.shape[0])
    if transformed.shape[0] > 8000:
        indices = np.linspace(0, transformed.shape[0] - 1, 8000, dtype=np.int64)
        transformed = transformed[indices]
    max_distance = max(0.002, float(max_distance))
    max_distance_sq = max_distance * max_distance
    distances: list[float] = []
    for point in transformed:
        _k, _idx, dist_sq = target_tree.search_knn_vector_3d(point, 1)
        if dist_sq and dist_sq[0] <= max_distance_sq:
            distances.append(math.sqrt(float(dist_sq[0])))
    inliers = len(distances)
    if inliers <= 0:
        return float("inf"), {"fitness": 0.0, "rmse": 999.0, "inliers": 0}
    distances_np = np.asarray(distances, dtype=np.float64)
    rmse = float(np.sqrt(np.mean(distances_np * distances_np)))
    fitness = float(inliers) / float(max(1, transformed.shape[0]))
    coverage = float(inliers) / float(max(1, target_count))
    # Reward overlap, but make bad nearest-neighbor distances expensive.
    score = rmse + max_distance * (1.0 - min(1.0, fitness)) * 0.75 + max_distance * (1.0 - min(1.0, coverage)) * 0.25
    return score, {"fitness": fitness, "coverage": coverage, "rmse": rmse, "inliers": inliers, "evaluated_points": int(transformed.shape[0]), "source_points": original_count}


def score_cloud_alignment(o3d, source, target, transform: np.ndarray, voxel: float, max_distance: float) -> tuple[float, dict]:
    source_down = downsample_with_normals(o3d, source, voxel)
    target_down = downsample_with_normals(o3d, target, voxel)
    if len(source_down.points) < 100 or len(target_down.points) < 100:
        raise RuntimeError(f"score clouds too sparse source={len(source_down.points)} target={len(target_down.points)}")
    target_tree = o3d.geometry.KDTreeFlann(target_down)
    return score_transform(o3d, source_down, target_tree, len(target_down.points), transform, max_distance)


def extract_dominant_planes(o3d, cloud, max_planes: int = 7, distance_threshold: float = 0.025) -> list[dict]:
    remaining = cloud.voxel_down_sample(0.018)
    planes: list[dict] = []
    total = max(1, len(remaining.points))
    for _i in range(max_planes):
        if len(remaining.points) < 500:
            break
        model, inliers = remaining.segment_plane(
            distance_threshold=float(distance_threshold),
            ransac_n=3,
            num_iterations=1200,
        )
        if len(inliers) < max(250, total * 0.025):
            break
        points = np.asarray(remaining.points)
        inlier_points = points[np.asarray(inliers, dtype=np.int64)]
        normal = np.asarray(model[:3], dtype=np.float64)
        norm = float(np.linalg.norm(normal))
        if norm <= 1e-8:
            remaining = remaining.select_by_index(inliers, invert=True)
            continue
        normal /= norm
        centroid = inlier_points.mean(axis=0)
        planes.append({
            "normal": normal,
            "centroid": centroid,
            "count": int(len(inliers)),
            "model": [float(v) for v in model],
        })
        remaining = remaining.select_by_index(inliers, invert=True)
    return sorted(planes, key=lambda item: int(item["count"]), reverse=True)


def rotation_from_vectors(source_vectors: np.ndarray, target_vectors: np.ndarray) -> np.ndarray:
    h = source_vectors.T @ target_vectors
    u, _s, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    return r


def solve_plane_initial(o3d, source, target, args: argparse.Namespace) -> tuple[np.ndarray, dict]:
    source_planes = extract_dominant_planes(o3d, source, max_planes=int(args.plane_max_planes), distance_threshold=float(args.plane_distance_threshold))
    target_planes = extract_dominant_planes(o3d, target, max_planes=int(args.plane_max_planes), distance_threshold=float(args.plane_distance_threshold))
    if len(source_planes) < 3 or len(target_planes) < 3:
        raise RuntimeError(f"plane alignment needs at least 3 planes source={len(source_planes)} target={len(target_planes)}")
    source_down = downsample_with_normals(o3d, source, float(args.plane_score_voxel))
    target_down = downsample_with_normals(o3d, target, float(args.plane_score_voxel))
    target_tree = o3d.geometry.KDTreeFlann(target_down)
    target_count = len(target_down.points)
    best_transform = None
    best_score = float("inf")
    best_stats: dict = {}
    best_match: dict = {}
    plane_limit = min(len(source_planes), len(target_planes), int(args.plane_max_planes), 4)

    def consider_candidate(candidate: np.ndarray, match: dict) -> None:
        nonlocal best_transform, best_score, best_stats, best_match
        score, stats = score_transform(o3d, source_down, target_tree, target_count, candidate, float(args.plane_score_distance))
        if score < best_score:
            best_score = score
            best_transform = candidate
            best_stats = stats
            best_match = dict(match)
            best_match["score"] = score

    for src_combo in itertools.combinations(range(plane_limit), 3):
        src_normals = np.asarray([source_planes[i]["normal"] for i in src_combo], dtype=np.float64)
        if abs(float(np.linalg.det(src_normals))) < 0.025:
            continue
        src_centroids = np.asarray([source_planes[i]["centroid"] for i in src_combo], dtype=np.float64)
        for tgt_combo in itertools.combinations(range(plane_limit), 3):
            tgt_base_normals = np.asarray([target_planes[i]["normal"] for i in tgt_combo], dtype=np.float64)
            if abs(float(np.linalg.det(tgt_base_normals))) < 0.025:
                continue
            tgt_centroids = np.asarray([target_planes[i]["centroid"] for i in tgt_combo], dtype=np.float64)
            for perm in itertools.permutations(range(3)):
                tgt_normals_perm = tgt_base_normals[list(perm)]
                tgt_centroids_perm = tgt_centroids[list(perm)]
                for signs in itertools.product([-1.0, 1.0], repeat=3):
                    tgt_normals = tgt_normals_perm * np.asarray(signs, dtype=np.float64).reshape(3, 1)
                    r = rotation_from_vectors(src_normals, tgt_normals)
                    a = tgt_normals
                    b = np.asarray([
                        float(tgt_normals[i].dot(tgt_centroids_perm[i]) - tgt_normals[i].dot(r @ src_centroids[i]))
                        for i in range(3)
                    ], dtype=np.float64)
                    try:
                        t, *_ = np.linalg.lstsq(a, b, rcond=None)
                    except Exception:
                        continue
                    candidate = np.eye(4, dtype=np.float64)
                    candidate[:3, :3] = r
                    candidate[:3, 3] = t
                    consider_candidate(candidate, {
                        "kind": "three_plane",
                        "source_planes": list(src_combo),
                        "target_planes": [int(tgt_combo[i]) for i in perm],
                        "signs": [float(v) for v in signs],
                    })
    for src_combo in itertools.combinations(range(plane_limit), 2):
        src_normals_raw = [source_planes[i]["normal"] for i in src_combo]
        if abs(float(np.dot(src_normals_raw[0], src_normals_raw[1]))) > 0.985:
            continue
        src_centroids = np.asarray([source_planes[i]["centroid"] for i in src_combo], dtype=np.float64)
        src_basis_0 = src_normals_raw[0] / max(1e-8, np.linalg.norm(src_normals_raw[0]))
        src_basis_1 = src_normals_raw[1] - src_basis_0 * float(np.dot(src_basis_0, src_normals_raw[1]))
        src_basis_1 /= max(1e-8, np.linalg.norm(src_basis_1))
        src_basis_2 = np.cross(src_basis_0, src_basis_1)
        src_basis = np.asarray([src_basis_0, src_basis_1, src_basis_2], dtype=np.float64)
        for tgt_combo in itertools.combinations(range(plane_limit), 2):
            tgt_normals_base = [target_planes[i]["normal"] for i in tgt_combo]
            tgt_centroids_base = np.asarray([target_planes[i]["centroid"] for i in tgt_combo], dtype=np.float64)
            for perm in itertools.permutations(range(2)):
                for signs in itertools.product([-1.0, 1.0], repeat=2):
                    tgt0 = tgt_normals_base[perm[0]] * signs[0]
                    tgt1 = tgt_normals_base[perm[1]] * signs[1]
                    if abs(float(np.dot(tgt0, tgt1))) > 0.985:
                        continue
                    tgt_basis_0 = tgt0 / max(1e-8, np.linalg.norm(tgt0))
                    tgt_basis_1 = tgt1 - tgt_basis_0 * float(np.dot(tgt_basis_0, tgt1))
                    tgt_basis_1 /= max(1e-8, np.linalg.norm(tgt_basis_1))
                    tgt_basis_2 = np.cross(tgt_basis_0, tgt_basis_1)
                    tgt_basis = np.asarray([tgt_basis_0, tgt_basis_1, tgt_basis_2], dtype=np.float64)
                    r = rotation_from_vectors(src_basis, tgt_basis)
                    tgt_centroids = tgt_centroids_base[list(perm)]
                    t = tgt_centroids.mean(axis=0) - r @ src_centroids.mean(axis=0)
                    candidate = np.eye(4, dtype=np.float64)
                    candidate[:3, :3] = r
                    candidate[:3, 3] = t
                    consider_candidate(candidate, {
                        "kind": "two_plane_centroid",
                        "source_planes": list(src_combo),
                        "target_planes": [int(tgt_combo[i]) for i in perm],
                        "signs": [float(v) for v in signs],
                    })
    if best_transform is None:
        raise RuntimeError("plane alignment found no non-degenerate plane match")
    return best_transform, {
        "source_plane_count": len(source_planes),
        "target_plane_count": len(target_planes),
        "source_planes": [{"count": p["count"], "normal": p["normal"].tolist(), "centroid": p["centroid"].tolist()} for p in source_planes],
        "target_planes": [{"count": p["count"], "normal": p["normal"].tolist(), "centroid": p["centroid"].tolist()} for p in target_planes],
        "best_match": best_match,
        "score_stats": best_stats,
    }


def run_axis_search_refine(o3d, source, target, init: np.ndarray, args: argparse.Namespace):
    voxel = max(0.006, float(args.axis_search_voxel or args.voxel))
    source_down = downsample_with_normals(o3d, source, voxel)
    target_down = downsample_with_normals(o3d, target, voxel)
    if len(source_down.points) < 100 or len(target_down.points) < 100:
        raise RuntimeError(f"axis search clouds too sparse source={len(source_down.points)} target={len(target_down.points)}")
    target_tree = o3d.geometry.KDTreeFlann(target_down)
    max_distance = float(args.axis_search_score_distance or args.max_correspondence or voxel * 2.5)
    current = np.asarray(init, dtype=np.float64).copy()
    best_score, best_stats = score_transform(o3d, source_down, target_tree, len(target_down.points), current, max_distance)
    transformed_centroid = transform_points(current, np.asarray(source_down.points)).mean(axis=0)
    target_centroid = np.asarray(target_down.points).mean(axis=0)
    centroid_candidate = translation_matrix(0, target_centroid[0] - transformed_centroid[0]) @ current
    centroid_candidate = translation_matrix(1, target_centroid[1] - transformed_centroid[1]) @ centroid_candidate
    centroid_candidate = translation_matrix(2, target_centroid[2] - transformed_centroid[2]) @ centroid_candidate
    centroid_score, centroid_stats = score_transform(o3d, source_down, target_tree, len(target_down.points), centroid_candidate, max_distance)
    if centroid_score < best_score:
        current = centroid_candidate
        best_score = centroid_score
        best_stats = centroid_stats
    stages: list[dict] = [{
        "pass": 0,
        "kind": "initial",
        "score": best_score,
        **best_stats,
    }]

    rotation_span = float(args.axis_search_rotation_deg)
    translation_span = float(args.axis_search_translation_m)
    min_rotation = float(args.axis_search_min_rotation_deg)
    min_translation = float(args.axis_search_min_translation_m)
    passes = int(args.axis_search_passes)
    min_improvement = float(args.axis_search_min_improvement)
    samples = max(5, int(args.axis_search_samples))
    if samples % 2 == 0:
        samples += 1
    for pass_index in range(1, passes + 1):
        improved_this_pass = False
        for kind, axes, span in [
            ("rotation", [0, 1, 2], rotation_span),
            ("translation", [0, 1, 2], translation_span),
        ]:
            for axis in axes:
                chosen = None
                for offset in np.linspace(-span, span, samples):
                    if abs(float(offset)) <= 1e-12:
                        continue
                    if kind == "rotation":
                        current_points = transform_points(current, np.asarray(source_down.points))
                        rotation_center = current_points.mean(axis=0)
                        delta = rotation_about_point_matrix(axis, float(offset), rotation_center)
                    else:
                        rotation_center = None
                        delta = translation_matrix(axis, float(offset))
                    candidate = delta @ current
                    score, stats = score_transform(o3d, source_down, target_tree, len(target_down.points), candidate, max_distance)
                    if score + min_improvement < best_score:
                        chosen = (candidate, score, stats, float(offset), rotation_center)
                        best_score = score
                if chosen is not None:
                    current, best_score, best_stats, offset, rotation_center = chosen
                    improved_this_pass = True
                    stage = {
                        "pass": pass_index,
                        "kind": kind,
                        "axis": axis,
                        "offset": offset,
                        "span": span,
                        "score": best_score,
                        **best_stats,
                    }
                    if rotation_center is not None:
                        stage["rotation_center"] = [float(rotation_center[0]), float(rotation_center[1]), float(rotation_center[2])]
                    stages.append(stage)
        rotation_span *= 0.5
        translation_span *= 0.5
        if (not improved_this_pass) and rotation_span <= min_rotation and translation_span <= min_translation:
            break
    return current, stages, best_score, best_stats


def rigid_transform(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source_centroid = source.mean(axis=0)
    target_centroid = target.mean(axis=0)
    src_centered = source - source_centroid
    tgt_centered = target - target_centroid
    h = src_centered.T @ tgt_centered
    u, _s, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T
    t = target_centroid - r @ source_centroid
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = r
    transform[:3, 3] = t
    return transform


def point_from_pixel(depth_m: np.ndarray, intrinsics, x: float, y: float, min_depth: float, max_depth: float):
    ix = int(round(x))
    iy = int(round(y))
    if ix < 0 or iy < 0 or iy >= depth_m.shape[0] or ix >= depth_m.shape[1]:
        return None
    radius = 3
    y0 = max(0, iy - radius)
    y1 = min(depth_m.shape[0], iy + radius + 1)
    x0 = max(0, ix - radius)
    x1 = min(depth_m.shape[1], ix + radius + 1)
    patch = depth_m[y0:y1, x0:x1]
    valid_patch = patch[np.isfinite(patch) & (patch >= min_depth) & (patch <= max_depth)]
    if valid_patch.size <= 0:
        return None
    depth = float(np.median(valid_patch))
    if not np.isfinite(depth) or depth < min_depth or depth > max_depth:
        return None
    px = (float(ix) - float(intrinsics.ppx)) * depth / max(1e-6, float(intrinsics.fx))
    py = (float(iy) - float(intrinsics.ppy)) * depth / max(1e-6, float(intrinsics.fy))
    return np.asarray([px, -py, -depth], dtype=np.float64)


def solve_rgb_feature_initial(
    args: argparse.Namespace,
    ref_depth: np.ndarray,
    ref_color: np.ndarray | None,
    ref_intr,
    ref_min_depth: float,
    ref_max_depth: float,
    tgt_depth: np.ndarray,
    tgt_color: np.ndarray | None,
    tgt_intr,
    tgt_min_depth: float,
    tgt_max_depth: float,
) -> tuple[np.ndarray, dict]:
    if ref_color is None or tgt_color is None:
        raise RuntimeError("RGB feature alignment needs color from both cameras")
    try:
        import cv2
    except Exception as exc:
        raise RuntimeError(f"OpenCV/cv2 is required for RGB feature alignment: {exc}") from exc

    ref_gray = cv2.equalizeHist(cv2.cvtColor(ref_color, cv2.COLOR_RGB2GRAY))
    tgt_gray = cv2.equalizeHist(cv2.cvtColor(tgt_color, cv2.COLOR_RGB2GRAY))
    method = str(getattr(args, "feature_method", "sift")).lower()
    if method == "sift" and hasattr(cv2, "SIFT_create"):
        detector = cv2.SIFT_create(nfeatures=int(args.feature_count), contrastThreshold=0.018, edgeThreshold=12)
        norm = cv2.NORM_L2
    elif method == "akaze" and hasattr(cv2, "AKAZE_create"):
        detector = cv2.AKAZE_create(threshold=0.0006)
        norm = cv2.NORM_HAMMING
    else:
        method = "orb"
        detector = cv2.ORB_create(nfeatures=int(args.feature_count), scaleFactor=1.2, nlevels=10, fastThreshold=7)
        norm = cv2.NORM_HAMMING
    ref_kp, ref_desc = detector.detectAndCompute(ref_gray, None)
    tgt_kp, tgt_desc = detector.detectAndCompute(tgt_gray, None)
    if ref_desc is None or tgt_desc is None or len(ref_kp) < 12 or len(tgt_kp) < 12:
        raise RuntimeError(f"not enough {method} RGB features reference={len(ref_kp)} target={len(tgt_kp)}")

    matcher = cv2.BFMatcher(norm, crossCheck=False)
    raw_matches = matcher.knnMatch(tgt_desc, ref_desc, k=2)
    reverse_matches = matcher.knnMatch(ref_desc, tgt_desc, k=2)
    reverse_best: dict[int, int] = {}
    for pair in reverse_matches:
        if len(pair) < 2:
            continue
        best, second = pair
        if best.distance < float(args.feature_ratio) * second.distance:
            reverse_best[best.queryIdx] = best.trainIdx
    matched_source: list[np.ndarray] = []
    matched_target: list[np.ndarray] = []
    matched_pixels: list[tuple[float, float, float, float]] = []
    for pair in raw_matches:
        if len(pair) < 2:
            continue
        best, second = pair
        if best.distance >= float(args.feature_ratio) * second.distance:
            continue
        if reverse_best.get(best.trainIdx, -1) != best.queryIdx:
            continue
        tgt_pt = tgt_kp[best.queryIdx].pt
        ref_pt = ref_kp[best.trainIdx].pt
        source_3d = point_from_pixel(tgt_depth, tgt_intr, tgt_pt[0], tgt_pt[1], tgt_min_depth, tgt_max_depth)
        target_3d = point_from_pixel(ref_depth, ref_intr, ref_pt[0], ref_pt[1], ref_min_depth, ref_max_depth)
        if source_3d is None or target_3d is None:
            continue
        matched_source.append(source_3d)
        matched_target.append(target_3d)
        matched_pixels.append((float(tgt_pt[0]), float(tgt_pt[1]), float(ref_pt[0]), float(ref_pt[1])))

    source = np.asarray(matched_source, dtype=np.float64)
    target = np.asarray(matched_target, dtype=np.float64)
    if source.shape[0] < 6:
        raise RuntimeError(f"not enough depth-valid {method} RGB feature matches: {source.shape[0]}")

    rng = np.random.default_rng(12345)
    best_inliers = np.zeros(source.shape[0], dtype=bool)
    best_transform = np.eye(4, dtype=np.float64)
    threshold = float(args.feature_ransac_threshold)
    iterations = int(args.feature_ransac_iterations)
    sample_size = min(4, source.shape[0])
    for _i in range(iterations):
        sample = rng.choice(source.shape[0], size=sample_size, replace=False)
        candidate = rigid_transform(source[sample], target[sample])
        transformed = (candidate[:3, :3] @ source.T).T + candidate[:3, 3]
        errors = np.linalg.norm(transformed - target, axis=1)
        inliers = errors <= threshold
        if int(inliers.sum()) > int(best_inliers.sum()):
            best_inliers = inliers
            best_transform = candidate
    if int(best_inliers.sum()) < 6:
        raise RuntimeError(f"{method} RGB-D feature RANSAC found too few 3D inliers: {int(best_inliers.sum())}/{source.shape[0]}")
    refined = rigid_transform(source[best_inliers], target[best_inliers])
    transformed = (refined[:3, :3] @ source[best_inliers].T).T + refined[:3, 3]
    rmse = float(np.sqrt(np.mean(np.sum((transformed - target[best_inliers]) ** 2, axis=1))))
    return refined, {
        "reference_features": int(len(ref_kp)),
        "target_features": int(len(tgt_kp)),
        "feature_method": method,
        "raw_match_sets": int(len(raw_matches)),
        "mutual_ratio_matches": int(source.shape[0]),
        "depth_valid_matches": int(source.shape[0]),
        "inliers": int(best_inliers.sum()),
        "ransac_threshold_m": threshold,
        "feature_rmse_m": rmse,
        "inlier_ratio": float(best_inliers.sum()) / float(max(1, source.shape[0])),
    }


def solve_alignment(
    args: argparse.Namespace,
    source_points: np.ndarray,
    target_points: np.ndarray,
    source_colors: np.ndarray | None = None,
    target_colors: np.ndarray | None = None,
    feature_initial: np.ndarray | None = None,
    feature_details: dict | None = None,
) -> tuple[np.ndarray, str, dict]:
    import open3d as o3d

    voxel = float(args.voxel)
    max_corr = float(args.max_correspondence or voxel * 3.0)
    source, source_down, source_fpfh = preprocess(o3d, source_points, voxel)
    target, target_down, target_fpfh = preprocess(o3d, target_points, voxel)
    source_colored = make_colored_cloud(o3d, source_points, source_colors)
    target_colored = make_colored_cloud(o3d, target_points, target_colors)
    if len(source_down.points) < 100 or len(target_down.points) < 100:
        raise RuntimeError(f"downsample too sparse source={len(source_down.points)} target={len(target_down.points)}")

    initial = parse_transform(args.initial_transform)
    global_fitness = 0.0
    global_rmse = 0.0
    init = initial
    if feature_initial is not None:
        init = feature_initial
    plane_details = None
    if args.mode in ("plane_rigid", "plane_align"):
        init, plane_details = solve_plane_initial(o3d, source, target, args)
    if args.mode == "auto":
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down,
            target_down,
            source_fpfh,
            target_fpfh,
            True,
            max_corr,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            4,
            [
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(max_corr),
            ],
            o3d.pipelines.registration.RANSACConvergenceCriteria(int(args.ransac_iterations), 0.999),
        )
        init = np.asarray(result.transformation, dtype=np.float64)
        global_fitness = float(result.fitness)
        global_rmse = float(result.inlier_rmse)

    if args.mode == "plane_rigid":
        solved = init
        delta_translation_m, delta_rotation_deg = transform_delta(initial, solved)
        details = {
            "voxel_m": voxel,
            "max_correspondence_m": max_corr,
            "source_points": int(source_points.shape[0]),
            "target_points": int(target_points.shape[0]),
            "source_down_points": int(len(source_down.points)),
            "target_down_points": int(len(target_down.points)),
            "global_fitness": 0.0,
            "global_rmse": 0.0,
            "feature_initial": feature_details or {},
            "plane_initial": plane_details or {},
            "icp_fitness": float((plane_details or {}).get("score_stats", {}).get("fitness", 0.0)),
            "icp_rmse": float((plane_details or {}).get("score_stats", {}).get("rmse", 999.0)),
            "icp_estimation": "plane_rigid",
            "icp_stages": [],
            "delta_translation_m": delta_translation_m,
            "delta_rotation_deg": delta_rotation_deg,
        }
        status = (
            f"RealSense plane rigid alignment applied | "
            f"plane_fitness={details['icp_fitness']:.3f} plane_rmse={details['icp_rmse']:.4f} "
            f"delta={delta_translation_m:.3f}m/{delta_rotation_deg:.1f}deg"
        )
        return solved, status, details
    if args.mode == "color_refine":
        icp, solved, estimation, icp_stages = run_colored_icp_multiscale(o3d, source_colored, target_colored, init, args)
    elif args.mode == "axis_search":
        solved, axis_stages, axis_score, axis_stats = run_axis_search_refine(o3d, source, target, init, args)
        icp = type("AxisSearchResult", (), {
            "fitness": axis_stats.get("fitness", 0.0),
            "inlier_rmse": axis_stats.get("rmse", 999.0),
        })()
        estimation = "axis_search"
        icp_stages = axis_stages
    else:
        icp, solved, estimation, icp_stages = run_icp_multiscale(o3d, source, target, init, args)
    delta_translation_m, delta_rotation_deg = transform_delta(initial, solved)
    label = "RGB feature align" if args.mode == "rgb_feature" else ("plane align" if args.mode == "plane_align" else ("colored refine" if args.mode == "color_refine" else ("axis search" if args.mode == "axis_search" else args.mode)))
    status = (
        f"RealSense cloud {label} applied | "
        f"global_fitness={global_fitness:.3f} global_rmse={global_rmse:.4f} "
        f"fine_icp_fitness={float(icp.fitness):.3f} fine_icp_rmse={float(icp.inlier_rmse):.4f} "
        f"delta={delta_translation_m:.3f}m/{delta_rotation_deg:.1f}deg"
    )
    details = {
        "voxel_m": voxel,
        "max_correspondence_m": max_corr,
        "source_points": int(source_points.shape[0]),
        "target_points": int(target_points.shape[0]),
        "source_down_points": int(len(source_down.points)),
        "target_down_points": int(len(target_down.points)),
        "global_fitness": global_fitness,
        "global_rmse": global_rmse,
        "feature_initial": feature_details or {},
        "plane_initial": plane_details or {},
        "icp_fitness": float(icp.fitness),
        "icp_rmse": float(icp.inlier_rmse),
        "icp_estimation": estimation,
        "icp_stages": icp_stages,
        "delta_translation_m": delta_translation_m,
        "delta_rotation_deg": delta_rotation_deg,
    }
    return solved, status, details


def benchmark_candidates(args: argparse.Namespace, tgt_points, ref_points, tgt_colors, ref_colors, ref_depth, ref_color, ref_intr, ref_min_depth, ref_max_depth, tgt_depth, tgt_color, tgt_intr, tgt_min_depth, tgt_max_depth) -> tuple[np.ndarray, str, dict]:
    ground_truth = parse_transform(Path(args.benchmark_ground_truth).read_text(encoding="utf-8"))
    candidates: list[dict] = []
    candidates.append({"name": "keep_initial_static", "mode": "keep_initial", "voxel": 0.025, "score_distance": 0.055})
    for voxel in [0.018, 0.025, 0.035, 0.055, 0.075]:
        candidates.append({"name": "refine", "mode": "refine", "voxel": voxel})
        candidates.append({
            "name": "guarded_refine",
            "mode": "guarded_refine",
            "voxel": voxel,
            "guard_score_distance": max(0.035, voxel * 2.2),
            "guard_max_translation_m": 0.06,
            "guard_max_rotation_deg": 4.0,
        })
    for voxel in [0.025, 0.035, 0.055]:
        candidates.append({"name": "auto", "mode": "auto", "voxel": voxel, "ransac_iterations": 25000})
    if ref_colors is not None and tgt_colors is not None:
        for weight in [0.90, 0.968, 0.99]:
            candidates.append({"name": "color_refine", "mode": "color_refine", "color_icp_geometry_weight": weight})
    if ref_color is not None and tgt_color is not None:
        for method in ["sift", "akaze", "orb"]:
            for ratio in [0.68, 0.76, 0.84]:
                candidates.append({
                    "name": f"{method}_feature_rigid",
                    "mode": "rgb_feature_rigid",
                    "feature_method": method,
                    "feature_ratio": ratio,
                    "feature_ransac_threshold": 0.055,
                    "feature_ransac_iterations": 5000,
                })
        for method in ["sift", "akaze"]:
            candidates.append({
                "name": f"{method}_feature_icp",
                "mode": "rgb_feature",
                "feature_method": method,
                "feature_ratio": 0.76,
                "feature_ransac_threshold": 0.055,
                "feature_ransac_iterations": 5000,
            })
    for voxel in [0.025, 0.035, 0.055]:
        candidates.append({
            "name": "plane_rigid",
            "mode": "plane_rigid",
            "voxel": voxel,
            "plane_score_voxel": voxel,
            "plane_score_distance": max(0.05, voxel * 2.5),
        })
        candidates.append({
            "name": "plane_align",
            "mode": "plane_align",
            "voxel": voxel,
            "plane_score_voxel": voxel,
            "plane_score_distance": max(0.05, voxel * 2.5),
        })
    for rotation_deg, translation_m, score_distance in [
        (15.0, 0.08, 0.035),
        (30.0, 0.16, 0.055),
        (60.0, 0.28, 0.085),
        (90.0, 0.40, 0.110),
    ]:
        candidates.append({
            "name": "axis_search",
            "mode": "axis_search",
            "axis_search_rotation_deg": rotation_deg,
            "axis_search_translation_m": translation_m,
            "axis_search_score_distance": score_distance,
            "axis_search_samples": 7,
            "axis_search_passes": 3,
        })

    results: list[dict] = []
    best_transform = None
    best_result = None
    for index, candidate in enumerate(candidates):
        print(f"benchmark candidate {index + 1}/{len(candidates)}: {candidate}", flush=True)
        candidate_args = Namespace(**vars(args))
        for key, value in candidate.items():
            if key != "name":
                setattr(candidate_args, key, value)
        try:
            feature_initial = None
            feature_details = None
            if candidate_args.mode == "keep_initial":
                import open3d as o3d
                source = make_colored_cloud(o3d, tgt_points, tgt_colors)
                target = make_colored_cloud(o3d, ref_points, ref_colors)
                score, score_stats = score_cloud_alignment(
                    o3d,
                    source,
                    target,
                    parse_transform(args.initial_transform),
                    float(candidate.get("voxel", args.voxel)),
                    float(candidate.get("score_distance", args.max_correspondence or args.voxel * 2.5)),
                )
                transform = parse_transform(args.initial_transform)
                status = "Static baseline kept current transform | score=%.5f fitness=%.3f rmse=%.4f" % (
                    score,
                    float(score_stats.get("fitness", 0.0)),
                    float(score_stats.get("rmse", 999.0)),
                )
                details = {
                    "score": score,
                    "score_stats": score_stats,
                    "icp_fitness": float(score_stats.get("fitness", 0.0)),
                    "icp_rmse": float(score_stats.get("rmse", 999.0)),
                    "delta_translation_m": 0.0,
                    "delta_rotation_deg": 0.0,
                }
            elif candidate_args.mode == "guarded_refine":
                import open3d as o3d
                source = make_colored_cloud(o3d, tgt_points, tgt_colors)
                target = make_colored_cloud(o3d, ref_points, ref_colors)
                initial = parse_transform(args.initial_transform)
                baseline_score, baseline_stats = score_cloud_alignment(
                    o3d,
                    source,
                    target,
                    initial,
                    float(candidate_args.voxel),
                    float(candidate.get("guard_score_distance", candidate_args.max_correspondence or candidate_args.voxel * 2.5)),
                )
                raw_transform, raw_status, raw_details = solve_alignment(candidate_args, tgt_points, ref_points, tgt_colors, ref_colors, None, None)
                raw_score, raw_stats = score_cloud_alignment(
                    o3d,
                    source,
                    target,
                    raw_transform,
                    float(candidate_args.voxel),
                    float(candidate.get("guard_score_distance", candidate_args.max_correspondence or candidate_args.voxel * 2.5)),
                )
                raw_delta_m, raw_delta_deg = transform_delta(initial, raw_transform)
                accept = (
                    raw_score < baseline_score * 0.985
                    and raw_delta_m <= float(candidate.get("guard_max_translation_m", 0.06))
                    and raw_delta_deg <= float(candidate.get("guard_max_rotation_deg", 4.0))
                )
                transform = raw_transform if accept else initial
                status = "Guarded refine %s | baseline=%.5f raw=%.5f raw_delta=%.3fm/%.2fdeg" % (
                    "accepted" if accept else "kept static baseline",
                    baseline_score,
                    raw_score,
                    raw_delta_m,
                    raw_delta_deg,
                )
                details = dict(raw_details)
                details.update({
                    "guard_accepted": accept,
                    "guard_baseline_score": baseline_score,
                    "guard_baseline_stats": baseline_stats,
                    "guard_raw_score": raw_score,
                    "guard_raw_stats": raw_stats,
                    "guard_raw_status": raw_status,
                })
            elif candidate_args.mode in ("rgb_feature", "rgb_feature_rigid"):
                feature_initial, feature_details = solve_rgb_feature_initial(
                    candidate_args,
                    ref_depth,
                    ref_color,
                    ref_intr,
                    ref_min_depth,
                    ref_max_depth,
                    tgt_depth,
                    tgt_color,
                    tgt_intr,
                    tgt_min_depth,
                    tgt_max_depth,
                )
                if candidate_args.mode == "rgb_feature_rigid":
                    transform = feature_initial
                    translation_delta_m, rotation_delta_deg = transform_delta(parse_transform(args.initial_transform), transform)
                    status = "RGB-D %s rigid feature alignment | inliers=%d rmse=%.4f delta=%.3fm/%.1fdeg" % (
                        str(feature_details.get("feature_method", "feature")),
                        int(feature_details.get("inliers", 0)),
                        float(feature_details.get("feature_rmse_m", 999.0)),
                        translation_delta_m,
                        rotation_delta_deg,
                    )
                    details = {
                        "feature_initial": feature_details or {},
                        "delta_translation_m": translation_delta_m,
                        "delta_rotation_deg": rotation_delta_deg,
                        "icp_fitness": float(feature_details.get("inlier_ratio", 0.0)),
                        "icp_rmse": float(feature_details.get("feature_rmse_m", 999.0)),
                        "icp_estimation": "rgbd_feature_rigid",
                    }
                else:
                    transform, status, details = solve_alignment(candidate_args, tgt_points, ref_points, tgt_colors, ref_colors, feature_initial, feature_details)
            elif candidate_args.mode in ("plane_rigid", "plane_align"):
                transform, status, details = solve_alignment(candidate_args, tgt_points, ref_points, tgt_colors, ref_colors, None, None)
            elif candidate_args.mode not in ("keep_initial", "guarded_refine"):
                transform, status, details = solve_alignment(candidate_args, tgt_points, ref_points, tgt_colors, ref_colors, feature_initial, feature_details)
            translation_error_m, rotation_error_deg = transform_delta(ground_truth, transform)
            result = {
                "index": index,
                "candidate": candidate,
                "ok": True,
                "translation_error_m": translation_error_m,
                "rotation_error_deg": rotation_error_deg,
                "score": translation_error_m + rotation_error_deg * 0.02,
                "status": status,
                "details": details,
                "transform": transform_payload(transform),
            }
        except Exception as exc:
            result = {
                "index": index,
                "candidate": candidate,
                "ok": False,
                "error": str(exc),
                "score": 999.0,
            }
        results.append(result)
        if result.get("ok") and (best_result is None or float(result["score"]) < float(best_result["score"])):
            best_result = result
            best_transform = parse_transform(json.dumps(result["transform"]))
        progress_status = "Benchmark running %d/%d" % (index + 1, len(candidates))
        if best_result is not None:
            progress_status += " | best %s %.3fm/%.2fdeg" % (
                str(best_result["candidate"].get("name", best_result["candidate"].get("mode", ""))),
                float(best_result["translation_error_m"]),
                float(best_result["rotation_error_deg"]),
            )
        write_result(
            args.result_path,
            {
                "type": "realsense_cloud_alignment",
                "ok": False,
                "method": "benchmark",
                "status": progress_status,
                "reference_camera": f"realsense:{args.reference_serial}",
                "target_camera": f"realsense:{args.target_serial}",
                "details": {
                    "partial": True,
                    "candidate_count": len(candidates),
                    "completed": index + 1,
                    "latest": result,
                    "best": best_result or {},
                    "results": sorted(results, key=lambda item: float(item.get("score", 999.0)))[:10],
                },
                "timestamp": time.time(),
            },
        )
    if best_result is None or best_transform is None:
        raise RuntimeError("alignment benchmark found no successful candidates")
    status = "Benchmark best %s | error=%.3fm/%.2fdeg | tried=%d" % (
        str(best_result["candidate"].get("name", best_result["candidate"].get("mode", ""))),
        float(best_result["translation_error_m"]),
        float(best_result["rotation_error_deg"]),
        len(candidates),
    )
    details = {
        "ground_truth": transform_payload(ground_truth),
        "best": best_result,
        "results": sorted(results, key=lambda item: float(item.get("score", 999.0)))[:20],
        "candidate_count": len(candidates),
    }
    return best_transform, status, details


def solve_guarded_refine(args: argparse.Namespace, tgt_points, ref_points, tgt_colors, ref_colors) -> tuple[np.ndarray, str, dict]:
    import open3d as o3d

    initial = parse_transform(args.initial_transform)
    source = make_colored_cloud(o3d, tgt_points, tgt_colors)
    target = make_colored_cloud(o3d, ref_points, ref_colors)
    score_distance = max(0.035, float(args.voxel) * 2.2)
    baseline_score, baseline_stats = score_cloud_alignment(o3d, source, target, initial, float(args.voxel), score_distance)
    refine_args = Namespace(**vars(args))
    refine_args.mode = "refine"
    raw_transform, raw_status, raw_details = solve_alignment(refine_args, tgt_points, ref_points, tgt_colors, ref_colors, None, None)
    raw_score, raw_stats = score_cloud_alignment(o3d, source, target, raw_transform, float(args.voxel), score_distance)
    raw_delta_m, raw_delta_deg = transform_delta(initial, raw_transform)
    accept = raw_score < baseline_score * 0.985 and raw_delta_m <= 0.06 and raw_delta_deg <= 4.0
    transform = raw_transform if accept else initial
    status = "Guarded RealSense refine %s | baseline=%.5f raw=%.5f raw_delta=%.3fm/%.2fdeg" % (
        "accepted",
        baseline_score,
        raw_score,
        raw_delta_m,
        raw_delta_deg,
    ) if accept else "Guarded RealSense refine kept static baseline | baseline=%.5f raw=%.5f raw_delta=%.3fm/%.2fdeg" % (
        baseline_score,
        raw_score,
        raw_delta_m,
        raw_delta_deg,
    )
    details = dict(raw_details)
    details.update({
        "guard_accepted": accept,
        "guard_baseline_score": baseline_score,
        "guard_baseline_stats": baseline_stats,
        "guard_raw_score": raw_score,
        "guard_raw_stats": raw_stats,
        "guard_raw_status": raw_status,
        "guard_max_translation_m": 0.06,
        "guard_max_rotation_deg": 4.0,
    })
    return transform, status, details


def update_registry(args: argparse.Namespace, result_payload: dict) -> None:
    if result_payload.get("method") == "cloud_benchmark":
        return
    if not args.registry_path or not result_payload.get("ok"):
        return
    registry_path = Path(args.registry_path)
    try:
        registry = json.loads(registry_path.read_text(encoding="utf-8")) if registry_path.exists() else {}
    except Exception:
        registry = {}
    if not isinstance(registry, dict):
        registry = {}
    registry["type"] = "camera_alignment_registry"
    registry["reference_camera"] = result_payload["reference_camera"]
    registry["updated_at"] = time.time()
    transforms = registry.get("transforms", {})
    if not isinstance(transforms, dict):
        transforms = {}
    transforms[result_payload["target_camera"]] = {
        "relative_to": result_payload["reference_camera"],
        "method": result_payload["method"],
        "R": result_payload["R"],
        "T": result_payload["T"],
        "status": result_payload["status"],
        "details": result_payload.get("details", {}),
        "updated_at": time.time(),
    }
    registry["transforms"] = transforms
    registry_path.parent.mkdir(parents=True, exist_ok=True)
    registry_path.write_text(json.dumps(registry, indent=2), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="RealSense cloud alignment.")
    parser.add_argument("--reference-serial", required=True)
    parser.add_argument("--target-serial", required=True)
    parser.add_argument("--reference-profile", default="viewer30")
    parser.add_argument("--target-profile", default="viewer30")
    parser.add_argument("--mode", choices=("auto", "refine", "guarded_refine", "color_refine", "rgb_feature", "rgb_feature_rigid", "plane_rigid", "plane_align", "axis_search", "benchmark"), default="refine")
    parser.add_argument("--initial-transform", default="")
    parser.add_argument("--result-path", required=True)
    parser.add_argument("--registry-path", default="")
    parser.add_argument("--min-depth", type=float, default=0.2)
    parser.add_argument("--max-depth", type=float, default=4.5)
    parser.add_argument("--reference-min-depth", type=float, default=None)
    parser.add_argument("--reference-max-depth", type=float, default=None)
    parser.add_argument("--target-min-depth", type=float, default=None)
    parser.add_argument("--target-max-depth", type=float, default=None)
    parser.add_argument("--stride", type=int, default=2)
    parser.add_argument("--max-points", type=int, default=90000)
    parser.add_argument("--frames", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--voxel", type=float, default=0.035)
    parser.add_argument("--max-correspondence", type=float, default=0.0)
    parser.add_argument("--ransac-iterations", type=int, default=60000)
    parser.add_argument("--icp-iterations", type=int, default=80)
    parser.add_argument("--fine-icp-iterations", type=int, default=120)
    parser.add_argument("--color-icp-geometry-weight", type=float, default=0.968)
    parser.add_argument("--feature-method", choices=("sift", "akaze", "orb"), default="sift")
    parser.add_argument("--feature-count", type=int, default=5000)
    parser.add_argument("--feature-ratio", type=float, default=0.78)
    parser.add_argument("--feature-ransac-threshold", type=float, default=0.045)
    parser.add_argument("--feature-ransac-iterations", type=int, default=1200)
    parser.add_argument("--axis-search-translation-m", type=float, default=0.25)
    parser.add_argument("--axis-search-rotation-deg", type=float, default=45.0)
    parser.add_argument("--axis-search-min-translation-m", type=float, default=0.0025)
    parser.add_argument("--axis-search-min-rotation-deg", type=float, default=0.125)
    parser.add_argument("--axis-search-passes", type=int, default=7)
    parser.add_argument("--axis-search-voxel", type=float, default=0.02)
    parser.add_argument("--axis-search-score-distance", type=float, default=0.055)
    parser.add_argument("--axis-search-min-improvement", type=float, default=0.00015)
    parser.add_argument("--axis-search-samples", type=int, default=13)
    parser.add_argument("--plane-max-planes", type=int, default=7)
    parser.add_argument("--plane-distance-threshold", type=float, default=0.025)
    parser.add_argument("--plane-score-voxel", type=float, default=0.035)
    parser.add_argument("--plane-score-distance", type=float, default=0.085)
    parser.add_argument("--benchmark-ground-truth", default="")
    args = parser.parse_args()

    if args.reference_serial == args.target_serial:
        return fail(args, "reference and target RealSense serials must be different")
    try:
        import open3d  # noqa: F401
    except Exception as exc:
        return fail(args, f"Open3D is required for cloud alignment: {exc}")

    try:
        needs_color = args.mode in ("color_refine", "rgb_feature", "rgb_feature_rigid", "benchmark")
        capture_fallback = ""
        try:
            ref_depth, ref_color, ref_intr, ref_meta = capture_depth(args.reference_serial, args.reference_profile, args.frames, args.timeout, needs_color)
            tgt_depth, tgt_color, tgt_intr, tgt_meta = capture_depth(args.target_serial, args.target_profile, args.frames, args.timeout, needs_color)
        except Exception as exc:
            if args.mode != "benchmark" or not needs_color:
                raise
            capture_fallback = f"color capture failed, benchmark fell back to depth-only: {exc}"
            ref_depth, ref_color, ref_intr, ref_meta = capture_depth(args.reference_serial, args.reference_profile, args.frames, args.timeout, False)
            tgt_depth, tgt_color, tgt_intr, tgt_meta = capture_depth(args.target_serial, args.target_profile, args.frames, args.timeout, False)
        ref_min_depth = args.min_depth if args.reference_min_depth is None else args.reference_min_depth
        ref_max_depth = args.max_depth if args.reference_max_depth is None else args.reference_max_depth
        tgt_min_depth = args.min_depth if args.target_min_depth is None else args.target_min_depth
        tgt_max_depth = args.max_depth if args.target_max_depth is None else args.target_max_depth
        ref_points, ref_colors = depth_to_view_points_and_colors(ref_depth, ref_color, ref_intr, ref_min_depth, ref_max_depth, args.stride, args.max_points)
        tgt_points, tgt_colors = depth_to_view_points_and_colors(tgt_depth, tgt_color, tgt_intr, tgt_min_depth, tgt_max_depth, args.stride, args.max_points)
        if ref_points is None or tgt_points is None or ref_points.shape[0] < 500 or tgt_points.shape[0] < 500:
            return fail(
                args,
                f"not enough depth points reference={0 if ref_points is None else ref_points.shape[0]} target={0 if tgt_points is None else tgt_points.shape[0]}",
                {"reference_capture": ref_meta, "target_capture": tgt_meta},
            )
        feature_initial = None
        feature_details = None
        if args.mode == "benchmark":
            if not args.benchmark_ground_truth:
                return fail(args, "benchmark mode needs --benchmark-ground-truth")
            transform, status, details = benchmark_candidates(
                args,
                tgt_points,
                ref_points,
                tgt_colors,
                ref_colors,
                ref_depth,
                ref_color,
                ref_intr,
                ref_min_depth,
                ref_max_depth,
                tgt_depth,
                tgt_color,
                tgt_intr,
                tgt_min_depth,
                tgt_max_depth,
            )
        elif args.mode == "guarded_refine":
            transform, status, details = solve_guarded_refine(args, tgt_points, ref_points, tgt_colors, ref_colors)
        elif args.mode in ("rgb_feature", "rgb_feature_rigid"):
            feature_initial, feature_details = solve_rgb_feature_initial(
                args,
                ref_depth,
                ref_color,
                ref_intr,
                ref_min_depth,
                ref_max_depth,
                tgt_depth,
                tgt_color,
                tgt_intr,
                tgt_min_depth,
                tgt_max_depth,
            )
            if args.mode == "rgb_feature_rigid":
                transform = feature_initial
                delta_translation_m, delta_rotation_deg = transform_delta(parse_transform(args.initial_transform), transform)
                status = "RGB-D %s rigid feature alignment applied | inliers=%d rmse=%.4f delta=%.3fm/%.1fdeg" % (
                    str(feature_details.get("feature_method", "feature")),
                    int(feature_details.get("inliers", 0)),
                    float(feature_details.get("feature_rmse_m", 999.0)),
                    delta_translation_m,
                    delta_rotation_deg,
                )
                details = {
                    "feature_initial": feature_details,
                    "delta_translation_m": delta_translation_m,
                    "delta_rotation_deg": delta_rotation_deg,
                    "icp_fitness": float(feature_details.get("inlier_ratio", 0.0)),
                    "icp_rmse": float(feature_details.get("feature_rmse_m", 999.0)),
                    "icp_estimation": "rgbd_feature_rigid",
                }
            else:
                transform, status, details = solve_alignment(args, tgt_points, ref_points, tgt_colors, ref_colors, feature_initial, feature_details)
        else:
            transform, status, details = solve_alignment(args, tgt_points, ref_points, tgt_colors, ref_colors, feature_initial, feature_details)
        details["reference_capture"] = ref_meta
        details["target_capture"] = tgt_meta
        details["reference_depth_range_m"] = [float(ref_min_depth), float(ref_max_depth)]
        details["target_depth_range_m"] = [float(tgt_min_depth), float(tgt_max_depth)]
        if capture_fallback:
            details["capture_fallback"] = capture_fallback
            status = f"{status} | {capture_fallback}"
        payload = {
            "type": "realsense_cloud_alignment",
            "ok": True,
            "method": f"cloud_{args.mode}",
            "status": status,
            "reference_camera": f"realsense:{args.reference_serial}",
            "target_camera": f"realsense:{args.target_serial}",
            "R": transform[:3, :3].tolist(),
            "T": transform[:3, 3].tolist(),
            "details": details,
            "timestamp": time.time(),
        }
        write_result(args.result_path, payload)
        update_registry(args, payload)
        print(status, flush=True)
        return 0
    except Exception as exc:
        return fail(args, f"RealSense cloud alignment failed: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
