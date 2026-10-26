"""Joint RGB-D camera registration, with a fixed reference and fresh-frame validation.

Consumes the existing capture stream; never opens a camera or writes calibration.
Godot owns the transaction and commits the selected world transforms atomically.
"""

from __future__ import annotations

import itertools
import json
import time

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

FLIP = np.diag([1.0, -1.0, -1.0, 1.0])
MIN_MATCHES = 25


def transform(pose, points):
    return points @ pose[:3, :3].T + pose[:3, 3]


def project(intrinsics, points):
    return points[:, :2] / points[:, 2, None] * intrinsics[:2] + intrinsics[2:]


def matrix(payload):
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = payload["R"], payload["T"]
    if (
        not np.isfinite(pose).all()
        or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5)
        or not np.isclose(np.linalg.det(pose[:3, :3]), 1, atol=1e-5)
    ):
        raise ValueError("Invalid rigid camera transform")
    return pose


class Camera:
    def __init__(self, metadata, color, depth, minimum=0.2, maximum=3.0):
        self.meta, self.image = metadata, color
        self.e = np.eye(4)
        kd = np.asarray(metadata["intrinsics"], dtype=float)
        self.k = kd.copy()
        if metadata.get("color_space") == "original_rgb":
            projection = metadata["color_projection"]
            if projection.get("model") != "pinhole":
                raise ValueError("Joint refinement needs calibrated pinhole RGB projection")
            values = projection["extrinsics"]
            self.e[:3, :3] = np.array(values[:9]).reshape(3, 3, order="F")
            self.e[:3, 3] = values[9:]
            self.k = np.asarray(projection["intrinsics"], dtype=float)
            if list(projection["size"]) != [color.shape[1], color.shape[0]]:
                raise ValueError("RGB dimensions do not match projection intrinsics")
        elif color.shape[:2] != depth.shape:
            raise ValueError("Aligned RGB dimensions do not match depth")
        if (
            kd.shape != (4,)
            or self.k.shape != (4,)
            or not np.isfinite(np.r_[kd, self.k, self.e.ravel()]).all()
            or np.any(np.r_[kd[:2], self.k[:2]] <= 0)
        ):
            raise ValueError("Invalid RGB-D intrinsics")
        self.pixel_scale = color.shape[1] / 1920.0
        y, x = np.indices(depth.shape)
        z = depth.astype(np.float32) * 0.001
        valid = (z > minimum) & (z < maximum)
        points = np.stack([(x - kd[2]) * z / kd[0], (y - kd[3]) * z / kd[1], z], -1)
        points = transform(self.e, points[valid])
        pixels = project(self.k, points)
        valid = (points[:, 2] > 0.1) & np.isfinite(pixels).all(1)
        valid &= (pixels[:, 0] >= 0) & (pixels[:, 0] < color.shape[1])
        valid &= (pixels[:, 1] >= 0) & (pixels[:, 1] < color.shape[0])
        self.points, self.pixels = points[valid], pixels[valid]
        if len(self.points) < 100:
            raise ValueError("Insufficient valid depth for joint refinement")
        self.tree = cKDTree(self.pixels)

    def lift(self, pixels):
        if len(pixels) == 0:
            return np.empty((0, 3)), np.empty(0)
        distances, indices = self.tree.query(pixels, k=40)
        points, errors = [], []
        for pixel, distance, index in zip(pixels, distances, indices):
            keep = distance < max(3.0, 14 * self.pixel_scale)
            if keep.sum() < 6:
                points.append([np.nan] * 3)
                errors.append(np.inf)
                continue
            q, uv = self.points[index[keep]], self.pixels[index[keep]]
            z = q[:, 2]
            if np.percentile(z, 90) - np.percentile(z, 10) > 0.035:
                points.append([np.nan] * 3)
                errors.append(np.inf)
                continue
            a = np.column_stack([(uv - self.k[2:]) / self.k[:2], np.ones(len(q))])
            weights = np.exp(-0.5 * (distance[keep] / max(2.0, 8 * self.pixel_scale)) ** 2)
            fit = np.linalg.lstsq(a * weights[:, None], weights / z, rcond=None)[0]
            residual = z - 1 / (a @ fit)
            mad = max(0.0006, float(np.median(np.abs(residual - np.median(residual)))))
            good = np.abs(residual - np.median(residual)) < 3 * mad
            if good.sum() < 6:
                points.append([np.nan] * 3)
                errors.append(np.inf)
                continue
            fit = np.linalg.lstsq(
                a[good] * weights[good, None], weights[good] / z[good], rcond=None
            )[0]
            ray = np.r_[(pixel - self.k[2:]) / self.k[:2], 1.0]
            value = 1 / (ray @ fit)
            points.append(ray * value if 0.1 < value < 10 else [np.nan] * 3)
            errors.append(float(np.sqrt(np.mean((z[good] - 1 / (a[good] @ fit)) ** 2))))
        return np.asarray(points), np.asarray(errors)


def connected(count, pairs):
    reached = {0}
    for _ in range(count):
        for pair in pairs:
            i, j = pair["i"], pair["j"]
            if i in reached or j in reached:
                reached.update([i, j])
    return len(reached) == count


def updated(base, x):
    poses = [base[0].copy()]
    for i in range(1, len(base)):
        delta = x[(i - 1) * 6 : i * 6]
        pose = base[i].copy()
        pose[:3, :3] = Rotation.from_rotvec(delta[:3]).as_matrix() @ pose[:3, :3]
        pose[:3, 3] += delta[3:]
        poses.append(pose)
    return poses


def pair_errors(cameras, poses, pair):
    i, j = pair["i"], pair["j"]
    a, b = poses[i], poses[j]
    pa, pb = pair["pa"], pair["pb"]
    forward = project(cameras[i].k, transform(np.linalg.inv(a) @ b, pb)) - pair["ua"]
    reverse = project(cameras[j].k, transform(np.linalg.inv(b) @ a, pa)) - pair["ub"]
    # Normalize image errors to a 1920-pixel-wide RGB image across stream profiles.
    forward /= cameras[i].pixel_scale
    reverse /= cameras[j].pixel_scale
    spatial = transform(a, pa) - transform(b, pb)
    return forward, reverse, spatial


def metrics(cameras, poses, pairs):
    result = []
    for pair in pairs:
        forward, reverse, spatial = pair_errors(cameras, poses, pair)
        error = (np.linalg.norm(forward, axis=1) + np.linalg.norm(reverse, axis=1)) / 2
        result.append(
            {
                "pair": [pair["i"], pair["j"]],
                "matches": len(error),
                "median_reprojection_px_1920": float(np.median(error)),
                "p90_reprojection_px_1920": float(np.percentile(error, 90)),
                "median_3d_mm": float(np.median(np.linalg.norm(spatial, axis=1)) * 1000),
            }
        )
    return result


def refine(cameras, base, pairs):
    if not connected(len(cameras), pairs):
        raise ValueError("Joint feature matches do not connect every enabled camera")

    def residual(x):
        poses, values = updated(base, x), []
        for pair in pairs:
            forward, reverse, spatial = pair_errors(cameras, poses, pair)
            weight = np.sqrt(np.maximum(0.1, pair["scores"]))
            # Balance image cells so repeated lettering cannot dominate the fit.
            uv = pair["ua"] / cameras[pair["i"]].pixel_scale
            cells = (uv[:, 0] // 240).astype(int) + 8 * (uv[:, 1] // 180).astype(int)
            _, inverse, counts = np.unique(cells, return_inverse=True, return_counts=True)
            weight /= np.sqrt(np.maximum(1, counts[inverse] / 12))
            values.extend(
                [
                    (forward / 2.5 * weight[:, None]).ravel(),
                    (reverse / 2.5 * weight[:, None]).ravel(),
                    (spatial / 0.006 * 0.3 * weight[:, None]).ravel(),
                ]
            )
        values.append(x / 0.04)
        return np.concatenate(values)

    fit = least_squares(
        residual,
        np.zeros(6 * (len(base) - 1)),
        loss="soft_l1",
        f_scale=1,
        x_scale=0.01,
        bounds=(-0.035, 0.035),
        max_nfev=150,
        ftol=1e-10,
        xtol=1e-10,
        gtol=1e-8,
    )
    if not fit.success or not np.isfinite(fit.x).all():
        raise ValueError("Joint RGB-D optimizer did not converge")
    for delta in fit.x.reshape(-1, 6):
        if np.linalg.norm(delta[3:]) > 0.025 or np.linalg.norm(delta[:3]) > 0.05:
            raise ValueError("Joint correction exceeded its bounded refinement range")
    return updated(base, fit.x)


def score(values):
    return sum(v["median_reprojection_px_1920"] * v["matches"] for v in values) / sum(
        v["matches"] for v in values
    )


def no_regression(after, before):
    return all(
        a["median_reprojection_px_1920"] <= b["median_reprojection_px_1920"] * 1.03 + 0.25
        and a["p90_reprojection_px_1920"] <= b["p90_reprojection_px_1920"] * 1.03 + 0.5
        and a["median_3d_mm"] <= b["median_3d_mm"] * 1.25 + 1.0
        for a, b in zip(after, before)
    )


def select(cameras, initial, candidate, previous, validation):
    before, after = metrics(cameras, initial, validation), metrics(cameras, candidate, validation)
    selected, poses = "initial", initial
    if no_regression(after, before) and score(after) < score(before) - 0.05:
        selected, poses = "refined", candidate
    chosen = after if selected == "refined" else before
    prior = metrics(cameras, previous, validation) if previous is not None else None
    if prior is not None and (
        not no_regression(chosen, prior) or score(chosen) >= score(prior) - 0.05
    ):
        selected, poses = "previous", previous
    elif any(
        v["median_reprojection_px_1920"] > 12 or v["p90_reprojection_px_1920"] > 35 for v in chosen
    ):
        raise ValueError(
            "Fresh-frame image alignment remains too imprecise; saved alignment preserved"
        )
    return selected, poses, {"initial": before, "refined": after, "previous": prior}


class Matcher:
    def __init__(self, model):
        import onnxruntime as ort

        options = ort.SessionOptions()
        options.intra_op_num_threads, options.inter_op_num_threads = 4, 1
        self.session = ort.InferenceSession(
            str(model), sess_options=options, providers=["CPUExecutionProvider"]
        )

    def match(self, a, b, rotation):
        image = np.rot90(b.image, rotation // 90).copy()
        h, w = image.shape[:2]
        images = np.stack(
            [
                cv2.cvtColor(
                    cv2.resize(im, (960, 544), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY
                ).astype(np.float32)
                / 255
                for im in (image, a.image)
            ]
        )[:, None]
        keypoints, matches, scores = self.session.run(None, {"images": images})
        good = scores >= 0.05
        matches, scores = matches[good], scores[good]
        ub = keypoints[0, matches[:, 1]].astype(float) * [w / 960, h / 544]
        ua = keypoints[1, matches[:, 2]].astype(float) * [
            a.image.shape[1] / 960,
            a.image.shape[0] / 544,
        ]
        x, y = ub.T.copy()
        oh, ow = b.image.shape[:2]
        if rotation == 90:
            ub = np.column_stack([ow - 1 - y, x])
        elif rotation == 180:
            ub = np.column_stack([ow - 1 - x, oh - 1 - y])
        elif rotation == 270:
            ub = np.column_stack([y, oh - 1 - x])
        return ua, ub, scores


def pair_data(cameras, poses, i, j, matched):
    ua, ub, scores = matched
    pa, ea = cameras[i].lift(ua)
    pb, eb = cameras[j].lift(ub)
    good = np.isfinite(pa).all(1) & np.isfinite(pb).all(1) & (ea < 0.005) & (eb < 0.005)
    good &= np.linalg.norm(transform(poses[i], pa) - transform(poses[j], pb), axis=1) < 0.045
    return {
        "i": i,
        "j": j,
        "ua": ua[good],
        "ub": ub[good],
        "pa": pa[good],
        "pb": pb[good],
        "scores": scores[good],
    }


def capture_sets(source_url, ids, folder, timeout=30):
    import struct

    from tools.lan_remote_view_server import MultipartRawFrameReader
    from tools.workspace_stream_crop import _decode_depth

    reader = MultipartRawFrameReader(source_url)
    sets, cached, baseline = [], {}, None
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            _, packet = reader.read_part()
            if packet[:5] != b"RGBD1":
                raise ValueError("Capture source did not send RGB-D")
            length = struct.unpack_from("<I", packet, 5)[0]
            metadata, offset = json.loads(packet[9 : 9 + length]), 9 + length
            for camera in metadata["cameras"]:
                c, d = camera["color_length"], camera["depth_length"]
                color, depth = packet[offset : offset + c], packet[offset + c : offset + c + d]
                offset += c + d
                if camera["id"] in ids and c and d:
                    cached[camera["id"]] = (camera, color, _decode_depth(depth, camera).copy())
            if not all(serial in cached for serial in ids):
                continue
            sequences = {serial: cached[serial][0]["sequence"] for serial in ids}
            # Discard the initial cached packet; both datasets must be fresh for every camera.
            if baseline is None:
                baseline = sequences
                continue
            if any(sequences[serial] == baseline[serial] for serial in ids):
                continue
            frame = []
            destination = folder / f"frame-{len(sets)}"
            destination.mkdir()
            for serial in ids:
                camera, color, depth = cached[serial]
                (destination / (serial + ".json")).write_text(json.dumps(camera), encoding="utf-8")
                (destination / (serial + ".jpg")).write_bytes(color)
                np.savez_compressed(destination / (serial + ".npz"), depth=depth)
                image = cv2.imdecode(np.frombuffer(color, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError("Invalid captured RGB image")
                frame.append((camera, image, depth))
            sets.append(frame)
            baseline = sequences
            if len(sets) == 2:
                return sets
    finally:
        reader.close()
    raise ValueError("Timed out waiting for fresh RGB-D from every enabled camera")


def run(request, folder, progress=lambda text: None, frames=None, matcher=None):
    ids = request["cameras"]
    if len(ids) < 2 or len(ids) != len(set(ids)) or ids[0] != request["reference_camera"]:
        raise ValueError("Invalid fixed-reference camera rig")
    if any(
        not isinstance(s, str)
        or not 1 <= len(s) <= 64
        or any(not (c.isascii() and (c.isalnum() or c in "-_")) for c in s)
        for s in ids
    ):
        raise ValueError("Invalid camera serial")
    progress("Capturing two fresh RGB-D datasets")
    frames = frames if frames is not None else capture_sets(request["source_url"], ids, folder)
    cameras = [
        [
            Camera(*frame, request.get("min_depth_m", 0.2), request.get("max_depth_m", 3.0))
            for frame in dataset
        ]
        for dataset in frames
    ]
    initial_world = [matrix(request["initial_transforms"][serial]) for serial in ids]
    reference_color_world = initial_world[0] @ FLIP @ np.linalg.inv(cameras[0][0].e)

    def color_poses(world, dataset):
        return [
            np.linalg.inv(reference_color_world) @ pose @ FLIP @ np.linalg.inv(camera.e)
            for pose, camera in zip(world, dataset)
        ]

    initial = color_poses(initial_world, cameras[0])
    previous_world = request.get("previous_transforms", {})
    previous = (
        color_poses([matrix(previous_world[s]) for s in ids], cameras[0])
        if all(s in previous_world for s in ids)
        else None
    )
    for a, b in zip(cameras[0], cameras[1]):
        if not np.allclose(a.e, b.e) or not np.allclose(a.k, b.k):
            raise ValueError("Camera intrinsics changed between fit and validation captures")
    matcher = matcher or Matcher(request["model_path"])
    pairs, validation, rotations = [], [], {}
    for i, j in itertools.combinations(range(len(ids)), 2):
        progress(f"Matching RGB-D overlap {ids[i]} / {ids[j]}")
        best, best_rotation = None, 0
        for rotation in (0, 270, 90, 180):
            pair = pair_data(
                cameras[0], initial, i, j, matcher.match(cameras[0][i], cameras[0][j], rotation)
            )
            if best is None or len(pair["ua"]) > len(best["ua"]):
                best, best_rotation = pair, rotation
        if len(best["ua"]) < MIN_MATCHES:
            continue
        fresh = pair_data(
            cameras[1], initial, i, j, matcher.match(cameras[1][i], cameras[1][j], best_rotation)
        )
        if len(fresh["ua"]) < MIN_MATCHES:
            continue
        pairs.append(best)
        validation.append(fresh)
        rotations[f"{ids[i]} / {ids[j]}"] = best_rotation
    if not connected(len(ids), pairs):
        raise ValueError(
            "Fresh feature overlap does not connect every camera; saved alignment preserved"
        )
    progress(f"Jointly refining all {len(ids)} cameras")
    candidate = refine(cameras[0], initial, pairs)
    progress("Validating the joint result against fresh frames and the saved alignment")
    selected, poses, checks = select(cameras[1], initial, candidate, previous, validation)
    output = {}
    for serial, camera, pose in zip(ids, cameras[0], poses):
        world = reference_color_world @ pose @ camera.e @ FLIP
        output[serial] = {"R": world[:3, :3].tolist(), "T": world[:3, 3].tolist()}
    # Preserve the reference exactly, including the original floating point representation.
    output[ids[0]] = request["initial_transforms"][ids[0]]
    return {
        "ok": True,
        "request_id": request["request_id"],
        "selected": selected,
        "reference_camera": ids[0],
        "transforms": output,
        "details": {
            "validation": checks,
            "rotations_ccw_deg": rotations,
            "independent_capture": True,
            "matched_pairs": len(pairs),
        },
    }
