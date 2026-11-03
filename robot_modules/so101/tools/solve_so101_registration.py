"""Place the captured, fixed-kinematics arm; never estimate servo offsets.

The model samples come from Godot's actual overlay at each measured pose,
including saved joint signs, offsets, jaw geometry and assembly corrections.
Only a common table-plane translation and heading are fitted. The existing
motion-axis fit supplies the support plane and shoulder-pan pivot, not yaw.
"""
from __future__ import annotations

import argparse
import copy
import json
import logging
import math
from pathlib import Path

import numpy as np
from scipy.optimize import minimize
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

TRANSLATION_LIMIT_M = 0.025
LOGGER = logging.getLogger(__name__)
LINKS = ("base_link", "shoulder_link", "upper_arm_link", "lower_arm_link",
         "wrist_link", "gripper_link", "moving_jaw_so101_v1_link")


def matrix(value: dict) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = np.column_stack([value[f"basis_{a}"] for a in "xyz"])
    result[:3, 3] = value["origin"]
    if not np.isfinite(result).all() or not np.allclose(
        result[:3, :3].T @ result[:3, :3], np.eye(3), atol=1e-4
    ) or np.linalg.det(result[:3, :3]) < 0.99:
        raise ValueError("capture has an invalid rigid transform")
    return result


def points(value) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.ndim != 2 or result.shape[1] != 3 or not np.isfinite(result).all():
        raise ValueError("capture has invalid 3D points")
    return result


def prepare_snapshot(snapshot: dict) -> dict:
    """Prepare constant camera/depth data once, outside optimizer evaluations."""
    prepared = dict(snapshot)
    prepared["_inverse"] = matrix(snapshot["camera_inverse"])
    cols, rows = int(snapshot["cols"]), int(snapshot["rows"])
    depths = np.asarray(snapshot["depths"], dtype=float)
    valid = np.asarray(snapshot["valid"], dtype=bool)
    if cols <= 0 or rows <= 0 or len(depths) != cols * rows or len(valid) != len(depths):
        raise ValueError("capture has a malformed depth visibility image")
    prepared["_depths"], prepared["_valid"] = depths, valid
    return prepared


def visible(world: np.ndarray, snapshot: dict | None) -> np.ndarray:
    if snapshot is None:
        # Old capture files remain usable, with geometric coverage gates.
        return np.ones(len(world), dtype=bool)
    if "_inverse" not in snapshot:
        snapshot = prepare_snapshot(snapshot)
    inverse = snapshot["_inverse"]
    camera = world @ inverse[:3, :3].T + inverse[:3, 3]
    z = -camera[:, 2]
    fx, fy, cx, cy = snapshot["intrinsics"]
    stride = max(1, int(snapshot["grid_stride"]))
    step = max(1, int(snapshot["sample_step"]))
    safe_z = np.maximum(z, 1e-6)
    col = np.floor((camera[:, 0] * fx / safe_z + cx) / (stride * step)).astype(int)
    row = np.floor((-camera[:, 1] * fy / safe_z + cy) / (stride * step)).astype(int)
    cols, rows = int(snapshot["cols"]), int(snapshot["rows"])
    mask = (z > 0.15) & (col >= 0) & (col < cols) & (row >= 0) & (row < rows)
    index = np.clip(row, 0, rows - 1) * cols + np.clip(col, 0, cols - 1)
    depths, valid = snapshot["_depths"], snapshot["_valid"]
    # Do not demand matches from the back of a link or behind another object.
    # Free-space mismatches in front of measured depth remain in the score.
    return mask & valid[index] & (depths[index] >= z - 0.025)


class PlacementFit:
    def __init__(self, payload: dict, evidence: dict):
        self.frames = payload["frames"]
        if len(self.frames) < 5:
            raise ValueError("whole-arm alignment requires five distinct measured poses")
        poses = np.asarray([frame["pose"] for frame in self.frames], dtype=float)
        if poses.shape[1] != 6 or not np.isfinite(poses).all() or np.ptp(poses[:, 0]) < 45:
            raise ValueError("insufficient measured shoulder-pan coverage")
        self.normal = np.array(evidence["normal"], dtype=float, copy=True)
        if self.normal.shape != (3,) or not np.isfinite(self.normal).all() or np.linalg.norm(self.normal) < 0.5:
            raise ValueError("support plane has an invalid normal")
        self.normal /= np.linalg.norm(self.normal)
        if self.normal[1] < 0.25:
            raise ValueError("support plane is not upright")
        self.axis = np.asarray(evidence["axis_fit"]["axis_point"], dtype=float)
        self.reference = str(evidence["reference_camera"])
        initial = matrix(self.frames[0]["model_transform"])
        up = initial[:3, 1]
        cross = np.cross(up, self.normal)
        correction = Rotation.from_rotvec(
            cross / max(np.linalg.norm(cross), 1e-12)
            * math.acos(float(np.clip(up @ self.normal, -1, 1)))
        ).as_matrix()
        self.seed = correction @ initial[:3, :3]
        self.e1, self.e2 = self.seed[:, 0], self.seed[:, 2]
        self.pivot = np.asarray(self.frames[0]["joint_frames"][0]["pivot"], dtype=float)
        self.models, self.views, self.motion_views = [], [], []
        for frame in self.frames:
            model = {name: points(value) for name, value in frame["mesh_points"].items()
                     if name in LINKS and len(value) >= 8}
            # A high-resolution claw lookup can enlarge Godot's shared mesh
            # cache between poses. Bound each link here as well, so a captured
            # dense cache cannot multiply every optimizer evaluation's cost.
            for name, local in model.items():
                if len(local) > 220:
                    rng = np.random.default_rng(23)
                    indices = ((np.arange(220) + rng.random(220)) * len(local) / 220).astype(int)
                    model[name] = local[indices]
            if len(model) < 4:
                raise ValueError("capture lacks the calibrated whole-arm mesh samples")
            self.models.append(model)
            snapshots = {s["name"]: prepare_snapshot(s) for s in frame.get("depth_snapshots", [])}
            views = {}
            for camera in frame["full_camera_points"]:
                cloud = points(camera["points"])
                height = (cloud - self.axis) @ self.normal
                cloud = cloud[(height > -0.065) & (height < 0.65)
                              & (np.linalg.norm(cloud - self.axis, axis=1) < 0.7)]
                if len(cloud) >= 25:
                    views[camera["name"]] = (cKDTree(cloud), snapshots.get(camera["name"]))
            if not views:
                raise ValueError("no usable camera depth in a pose")
            self.views.append(views)
            motion = {}
            for camera in frame.get("camera_points", []):
                if not camera.get("points"):
                    continue
                cloud = points(camera["points"])
                height = (cloud - self.axis) @ self.normal
                # Remove table/disocclusion evidence, independently of a tested heading.
                cloud = cloud[(height > 0.025) & (height < 0.65)
                              & (np.linalg.norm(cloud - self.axis, axis=1) < 0.7)]
                if len(cloud) > 600:
                    cloud = cloud[np.linspace(0, len(cloud) - 1, 600).astype(int)]
                motion[camera["name"]] = cloud
            self.motion_views.append(motion)
        if not any(self.reference in view for view in self.views):
            raise ValueError("selected reference camera is missing from the capture")
        # No single pose may fit and validate itself. Keep measured viewpoints
        # aside, then require agreement at both fitted and withheld poses.
        self.validation = list(range(1, len(self.frames), 3))
        self.training = [i for i in range(len(self.frames)) if i not in self.validation]

    def transform(self, value: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        basis = Rotation.from_rotvec(self.normal * value[0]).as_matrix() @ self.seed
        origin = self.axis - basis @ self.pivot + self.e1 * value[1] + self.e2 * value[2]
        return basis, origin

    def score(self, value, cameras, indices, details=False):
        basis, origin = self.transform(value)
        losses, reports = [], []
        for frame_index in indices:
            world_links = {name: local @ basis.T + origin
                           for name, local in self.models[frame_index].items()}
            pose_views = []
            for camera_name in cameras:
                if camera_name not in self.views[frame_index]:
                    continue
                tree, snapshot = self.views[frame_index][camera_name]
                links = {}
                for name, world in world_links.items():
                    mask = visible(world, snapshot)
                    count = int(np.count_nonzero(mask))
                    if count < 3:
                        continue
                    distances = tree.query(world[mask])[0]
                    support = float(np.mean(distances < 0.018))
                    links[name] = {"samples": count, "support": support,
                                   "squared_residual_m2": float(np.mean(np.minimum(distances, 0.08) ** 2))}
                report = {"pose": frame_index, "camera": camera_name, "links": links,
                          **self.summarize_links([links])}
                pose_views.append(report)
            # Missing views and hidden links contribute no error. Coverage and
            # validation belong to the union of observable links across cameras,
            # rather than requiring every camera to see three links on its own.
            summary = self.summarize_links([report["links"] for report in pose_views])
            loss = summary["loss_m"] if summary["loss_m"] is not None else 0.10
            # Optimize with the same combined-coverage requirement used for
            # validation. Otherwise a wrong heading can hide most of the arm
            # and win on the one small fragment still in front of a camera.
            coverage_deficit = max(0.0, 1 - summary["coverage_weight"] / 2.5,
                                   1 - summary["visible_samples"] / 48)
            losses.append(loss + 0.10 * coverage_deficit)
            if details:
                reports.extend(pose_views)
        score = float(np.mean(losses)) if losses else 0.10
        return (score, reports) if details else score

    @staticmethod
    def summarize_links(views):
        combined = {}
        for view in views:
            for name, link in view.items():
                combined.setdefault(name, []).append(link)
        losses, weights, supported, contradicted = [], [], [], []
        for name, observations in combined.items():
            counts = np.array([o["samples"] for o in observations], dtype=float)
            support = float(np.average([o["support"] for o in observations], weights=counts))
            residual = float(np.sqrt(np.average(
                [o["squared_residual_m2"] for o in observations], weights=counts)))
            # A tiny visible sliver must not carry a whole link's weight or veto
            # a fit. Keep well-observed links balanced even across dense cameras.
            weight = min(1.0, float(counts.sum()) / 32)
            losses.append(residual + 0.015 * (1 - support))
            weights.append(weight)
            if counts.sum() >= 8 and support >= 0.45 and residual < 0.025:
                supported.append(name)
            if counts.sum() >= 32 and support < 0.20 and residual > 0.035:
                contradicted.append(name)
        return {"loss_m": float(np.average(losses, weights=weights)) if weights else None,
                "visible_samples": sum(o["samples"] for obs in combined.values() for o in obs),
                "coverage_weight": float(sum(weights)), "supported_links": supported,
                "contradicted_links": contradicted}

    def pose_reports(self, reports):
        return [{"pose": index, **self.summarize_links(
            [r["links"] for r in reports if r["pose"] == index])}
                for index in range(len(self.frames))]

    @staticmethod
    def passes_pose_validation(loss, poses):
        return loss <= 0.022 and all(
            r["loss_m"] is not None and r["loss_m"] <= 0.03
            and r["visible_samples"] >= 48 and r["coverage_weight"] >= 1.5
            and len(r["supported_links"]) >= 3 and not r["contradicted_links"]
            for r in poses)

    def validate_motion(self, value, cameras):
        """Require observed moving geometry to agree too, when capture has enough.

        This checks fixed hypotheses only. Withheld poses never fit parameters.
        Legacy/low-motion captures retain the original ambiguity requirement.
        """
        basis, origin = self.transform(value)
        reports = []
        for index, motion in enumerate(self.motion_views):
            observed = [motion[name] for name in cameras if name in motion and len(motion[name])]
            if not observed or sum(map(len, observed)) < 80:
                continue
            tree = cKDTree(np.concatenate(list(self.models[index].values())) @ basis.T + origin)
            distances = tree.query(np.concatenate(observed))[0]
            trimmed = np.sort(np.minimum(distances, 0.08))[:max(1, int(len(distances) * 0.7))]
            reports.append({"pose": index, "samples": len(distances),
                            "support": float(np.mean(distances < 0.025)),
                            "trimmed_loss_m": float(np.mean(trimmed))})
        available = (len(reports) >= 2 and any(r["pose"] in self.validation for r in reports)
                     and any(r["pose"] in self.training for r in reports))
        return {"available": available,
                "valid": not available or all(r["support"] >= 0.45 and r["trimmed_loss_m"] <= 0.025
                                              for r in reports),
                "pose_results": reports}

    def optimize(self):
        available = sorted(set().union(*(view.keys() for view in self.views)) - {self.reference})
        cameras = [self.reference, *available]
        objective = lambda value: self.score(value, cameras, self.training)
        coarse = [np.array([angle, 0.0, 0.0]) for angle in np.linspace(-math.pi, math.pi, 24, endpoint=False)]
        coarse.sort(key=objective)
        LOGGER.info("Whole-arm coarse best: %.2f mm", objective(coarse[0]) * 1000)
        candidates = []
        for seed in coarse[:6]:
            result = minimize(objective, seed, method="Powell", bounds=[
                (seed[0] - math.radians(20), seed[0] + math.radians(20)),
                (-TRANSLATION_LIMIT_M, TRANSLATION_LIMIT_M),
                (-TRANSLATION_LIMIT_M, TRANSLATION_LIMIT_M),
            ], options={"maxiter": 20, "maxfev": 400, "xtol": 1e-4, "ftol": 1e-4})
            candidates.append((float(result.fun), result.x))
            LOGGER.info("Whole-arm hypothesis: %.2f mm, %d evaluations", result.fun * 1000, result.nfev)
        candidates.sort(key=lambda item: item[0])
        best_loss, best = candidates[0]
        excluded = []
        cameras = [self.reference]
        for name in available:
            loss, reports = self.score(best, [name], self.training, True)
            observed = [r for r in reports if r["loss_m"] is not None]
            samples = sum(r["visible_samples"] for r in observed)
            camera_loss = (float(np.average([r["loss_m"] for r in observed],
                           weights=[r["visible_samples"] for r in observed])) if observed else 0.10)
            if samples >= 16 and camera_loss < 0.022 and not any(r["contradicted_links"] for r in observed):
                cameras.append(name)
            else:
                excluded.append({"camera": name, "loss_m": camera_loss,
                                 "reason": "insufficient whole-arm agreement with the reference"})
        if excluded:
            # A disagreeing auxiliary camera must not drag the reference fit.
            # Re-search the accepted set rather than only nudging a biased seed.
            objective = lambda value: self.score(value, cameras, self.training)
            coarse.sort(key=objective)
            rescored = []
            for seed in coarse[:6]:
                result = minimize(objective, seed, method="Powell", bounds=[
                    (seed[0] - math.radians(20), seed[0] + math.radians(20)),
                    (-TRANSLATION_LIMIT_M, TRANSLATION_LIMIT_M),
                    (-TRANSLATION_LIMIT_M, TRANSLATION_LIMIT_M)],
                    options={"maxiter": 20, "maxfev": 400, "xtol": 1e-4, "ftol": 1e-4})
                rescored.append((float(result.fun), result.x))
            rescored.sort(key=lambda item: item[0])
            best_loss, best = rescored[0]
            candidates = rescored
        # Validate fixed training hypotheses before asking whether two placements
        # are plausible. Withheld poses reject hypotheses, never fit parameters.
        rejected_headings, validated = [], []
        for training_loss, candidate in candidates:
            candidate_loss, candidate_reports = self.score(
                candidate, cameras, range(len(self.frames)), True)
            candidate_poses = self.pose_reports(candidate_reports)
            candidate_motion = self.validate_motion(candidate, cameras)
            if (np.max(np.abs(candidate[1:])) > TRANSLATION_LIMIT_M - 0.0005
                    or not self.passes_pose_validation(candidate_loss, candidate_poses)
                    or not candidate_motion["valid"]):
                rejected_headings.append({
                    "yaw_degrees": math.degrees(float(candidate[0])),
                    "training_loss_m": training_loss, "validation_loss_m": candidate_loss,
                    "pose_results": candidate_poses, "motion_validation": candidate_motion,
                    "reason": "failed placement or motion validation"})
                LOGGER.info("Competing heading %.2f degrees rejected by placement validation",
                            math.degrees(float(candidate[0])))
            else:
                validated.append((training_loss, candidate))
        if not validated:
            raise ValueError("whole-arm placement failed link coverage, moving-arm evidence, or withheld-pose validation")
        best_loss, best = validated[0]
        for loss, alternative in validated[1:]:
            separation = abs(math.atan2(math.sin(alternative[0] - best[0]), math.cos(alternative[0] - best[0])))
            if separation > math.radians(15) and loss < best_loss + max(0.0015, best_loss * 0.12):
                raise ValueError("whole-arm heading is ambiguous; previous placement retained")
        if len(cameras) > 1:
            reference_loss = self.score(best, [self.reference], self.training)
            result = minimize(lambda v: self.score(v, cameras, self.training), best,
                              method="Powell", bounds=[(best[0] - 0.15, best[0] + 0.15),
                              (-TRANSLATION_LIMIT_M, TRANSLATION_LIMIT_M),
                              (-TRANSLATION_LIMIT_M, TRANSLATION_LIMIT_M)],
                              options={"maxiter": 20, "maxfev": 400, "xtol": 1e-4, "ftol": 1e-4})
            if self.score(result.x, [self.reference], self.training) > reference_loss + 0.002:
                raise ValueError("camera consensus worsened reference alignment")
            best = result.x
        LOGGER.info("Whole-arm candidate: yaw %.2f degrees, pivot translation %.1f / %.1f mm",
                    math.degrees(float(best[0])), best[1] * 1000, best[2] * 1000)
        if np.max(np.abs(best[1:])) > TRANSLATION_LIMIT_M - 0.0005:
            raise ValueError("whole-arm fit reached the pivot translation limit")
        loss, reports = self.score(best, cameras, range(len(self.frames)), True)
        poses = self.pose_reports(reports)
        if not self.passes_pose_validation(loss, poses):
            raise ValueError("whole-arm placement failed link coverage or withheld-pose validation")
        motion_validation = self.validate_motion(best, cameras)
        if not motion_validation["valid"]:
            raise ValueError("whole-arm placement disagrees with observed moving-arm geometry; previous placement retained")
        basis, origin = self.transform(best)
        return {"ok": True, "type": "so101_rigid_registration",
                "method": "fixed_kinematics_whole_arm_registration",
                "kinematics_token": self.payload_token,
                "preview_transform": {**{f"basis_{a}": basis[:, i].tolist() for i, a in enumerate("xyz")},
                                      "origin": origin.tolist()},
                "confidence": max(0.0, min(1.0, 1 - loss / 0.04)),
                "metrics": {"loss_m": loss, "cameras": cameras, "excluded_cameras": excluded,
                            "rejected_heading_alternatives": rejected_headings,
                            "motion_validation": motion_validation,
                            "validation_poses": self.validation, "pose_camera_results": reports,
                            "pose_results": poses, "visibility_scoring": "observed_links_across_cameras",
                            "yaw_correction_degrees": math.degrees(float(best[0])),
                            "pivot_translation_m": best[1:].tolist(), "joint_mapping_changed": False},
                "status": "Whole-arm placement aligned; joint mapping and claw geometry preserved."}


def fit_registration(payload: dict, evidence: dict) -> dict:
    fit = PlacementFit(payload, evidence)
    fit.payload_token = str(payload.get("kinematics_token", ""))
    if not fit.payload_token:
        raise ValueError("capture lacks a joint-mapping/registration concurrency token")
    result = fit.optimize()
    result["capture_started_unix_ms"] = payload.get("capture_started_unix_ms", 0)
    result = refine_validated_placement(payload, evidence, result, fit)
    return result


def refine_validated_placement(payload, evidence, coarse, fit):
    """Retain a valid placement if the optional finer pass is unavailable/worse."""
    result = copy.deepcopy(coarse)
    if not all(frame.get("link_visual_transforms") for frame in payload["frames"]):
        result["metrics"]["fine_refinement"] = {"status": "not_available", "reason": "capture lacks exact visual transforms"}
        return result
    try:
        from refine_so101_registration import propose_refinement
        proposed, metadata = propose_refinement(payload, evidence, coarse, fit.training, fit.validation)
        if proposed is not None:
            # Fine fitting never bypasses the original geometry/coverage gates,
            # pivot bounds, token checks or atomic-save path.
            displacement = proposed[:3, :3] @ fit.pivot + proposed[:3, 3] - fit.axis
            shift = np.array([displacement @ fit.e1, displacement @ fit.e2])
            original_transform = fit.transform
            try:
                fit.transform = lambda _: (proposed[:3, :3], proposed[:3, 3])
                loss, reports = fit.score(None, coarse["metrics"]["cameras"], range(len(fit.frames)), True)
                motion_validation = fit.validate_motion(None, coarse["metrics"]["cameras"])
            finally:
                fit.transform = original_transform
            poses = fit.pose_reports(reports)
            valid = (np.max(np.abs(shift)) < TRANSLATION_LIMIT_M - .0005
                     and motion_validation["valid"]
                     and loss <= min(.022, coarse["metrics"]["loss_m"] + .002)
                     and all(p["loss_m"] is not None and p["loss_m"] <= .03
                             and p["visible_samples"] >= 48 and p["coverage_weight"] >= 1.5
                             and len(p["supported_links"]) >= 3 and not p["contradicted_links"] for p in poses))
            if valid:
                basis = proposed[:3, :3]
                result["preview_transform"] = {**{f"basis_{a}": basis[:, i].tolist() for i, a in enumerate("xyz")},
                                               "origin": proposed[:3, 3].tolist()}
                result["confidence"] = max(0.0, min(1.0, 1 - loss / .04))
                result["metrics"].update(loss_m=loss, pose_camera_results=reports, pose_results=poses,
                                          motion_validation=motion_validation,
                                          pivot_translation_m=shift.tolist(), yaw_correction_degrees=math.degrees(float(
                                              Rotation.from_matrix(basis @ fit.seed.T).as_rotvec() @ fit.normal)))
                metadata["status"] = "accepted"
            else:
                metadata.update(status="coarse_retained", reason="fine candidate failed original placement gates")
        result["metrics"]["fine_refinement"] = metadata
    except (ImportError, KeyError, OSError, RuntimeError, ValueError) as error:
        LOGGER.warning("Fine registration unavailable; validated whole-arm placement retained: %s", error)
        result["metrics"]["fine_refinement"] = {"status": "coarse_retained", "reason": str(error)}
    return result


def solve(payload: dict) -> dict:
    # Reuse the existing measured-axis and table-plane evidence. Its old model
    # heading and default servo offsets are deliberately never evaluated here.
    from solve_so101_base_axis import select_axis_evidence
    return fit_registration(payload, select_axis_evidence(payload))


def main():
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        result = solve(json.loads(args.capture.read_text()))
    except (ValueError, KeyError, RuntimeError) as error:
        result = {"ok": False, "status": str(error), "type": "so101_rigid_registration"}
        print(result["status"])
    args.output.write_text(json.dumps(result, indent=2))
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
