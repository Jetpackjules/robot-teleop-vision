"""Optional fixed-kinematics surface/edge pass after validated arm placement.

This module only returns a candidate. The registration caller retains the
validated placement unless both training and withheld views improve.
"""
from __future__ import annotations

import logging
from pathlib import Path

import numpy as np
import open3d as o3d
import trimesh
from scipy.ndimage import binary_dilation, binary_erosion, distance_transform_edt
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

LOGGER = logging.getLogger(__name__)


def affine(value):
    out = np.eye(4)
    out[:3, :3] = np.column_stack([value['basis_' + a] for a in 'xyz'])
    out[:3, 3] = value['origin']
    if not np.isfinite(out).all() or np.linalg.det(out[:3, :3]) <= 0:
        raise ValueError('invalid captured visual transform')
    return out


class SurfaceViews:
    def __init__(self, payload, evidence, previous, cameras, assets=None):
        self.previous = affine(previous['preview_transform'])
        self.normal = np.asarray(evidence['normal'], dtype=float)
        self.normal /= np.linalg.norm(self.normal)
        self.pivot = np.asarray(payload['frames'][0]['joint_frames'][0]['pivot'], dtype=float)
        self.links = sorted(payload['frames'][0]['mesh_points'])
        assets = Path(assets) if assets else Path(__file__).resolve().parents[1] / 'assets'
        meshes = {}
        for name in self.links:
            if name not in {'base_link', 'shoulder_link', 'upper_arm_link', 'lower_arm_link',
                            'wrist_link', 'gripper_link', 'moving_jaw_so101_v1_link'}:
                raise ValueError('unknown arm mesh')
            mesh = trimesh.load(assets / (name + '.glb'), force='scene', process=False).to_geometry()
            mesh.merge_vertices()
            meshes[name] = mesh
        self.scenes, self.views = [], []
        for frame in payload['frames']:
            scene = o3d.t.geometry.RaycastingScene(nthreads=2)
            for name in self.links:
                mesh = meshes[name]
                t = affine(frame['link_visual_transforms'][name])
                vertices = mesh.vertices @ t[:3, :3].T + t[:3, 3]
                scene.add_triangles(o3d.core.Tensor(vertices.astype('float32')),
                                    o3d.core.Tensor(mesh.faces.astype('uint32')))
            # The exact imported geometry must agree with the captured coarse
            # samples. Never refine a different assembly or joint configuration.
            for name in self.links:
                q = np.asarray(frame['mesh_points'][name], dtype='float32')
                distance = scene.compute_distance(o3d.core.Tensor(q), nthreads=2).numpy()
                if len(distance) < 8 or np.max(distance) > .002:
                    raise ValueError('surface geometry differs from captured arm samples')
            self.scenes.append(scene)
            views = []
            for snap in frame.get('depth_snapshots', []):
                if snap['name'] not in cameras:
                    continue
                inv = affine(snap['camera_inverse'])
                origin = -inv[:3, :3].T @ inv[:3, 3]
                fx, fy, cx, cy = snap['intrinsics']
                step = snap['grid_stride'] * snap['sample_step']
                rows, cols = int(snap['rows']), int(snap['cols'])
                if rows <= 0 or cols <= 0 or fx <= 0 or fy <= 0 or step <= 0:
                    raise ValueError('invalid camera sampling geometry')
                v, u = np.indices((rows, cols))
                # Capture stores the centre pixel of each subsampled cell.
                camera = np.column_stack([((u.ravel() + .5) * step - cx) / fx,
                                          -((v.ravel() + .5) * step - cy) / fy, -np.ones(u.size)])
                measured = np.asarray(snap['depths'], dtype=float).ravel()
                valid = np.asarray(snap['valid'], dtype=bool).ravel()
                if len(measured) != u.size or len(valid) != u.size:
                    raise ValueError('invalid depth image size')
                directions = camera @ inv[:3, :3]
                points = origin + directions * measured[:, None]
                view = {'origin': origin, 'directions': directions, 'depth': measured,
                        'valid': valid & np.isfinite(measured), 'points': points,
                        'shape': u.shape, 'name': snap['name'], 'focal_grid': .5 * (fx + fy) / step}
                front, _ = self.render(len(self.scenes) - 1, view, self.previous)
                finite = np.isfinite(front).reshape(u.shape)
                if not finite.any():
                    continue
                _, nearest = distance_transform_edt(~finite, return_indices=True)
                filled = front.reshape(u.shape)[tuple(nearest)].ravel()
                roi = binary_dilation(finite, iterations=8).ravel()
                observed = view['valid'] & roi & (np.abs(measured - filled) < .055)
                observed &= points @ self.normal > self.previous[:3, 3] @ self.normal + .025
                # A camera seeing no useful arm fragment contributes no penalty.
                if observed.sum() < 8:
                    continue
                mask = observed.reshape(u.shape)
                view.update(baseline_depth=front, observed=observed, roi=roi,
                            observed_edge=mask & ~binary_erosion(mask),
                            outside_observed=distance_transform_edt(~mask))
                views.append(view)
            if not views:
                raise ValueError('no observable surface fragment in a captured pose')
            self.views.append(views)

    def render(self, fi, view, transform, roi=None):
        basis, origin = transform[:3, :3], transform[:3, 3]
        directions = view['directions'] if roi is None else view['directions'][roi]
        local_origin = (view['origin'] - origin) @ basis
        local_directions = directions @ basis
        rays = np.column_stack([np.broadcast_to(local_origin, local_directions.shape), local_directions])
        out = self.scenes[fi].cast_rays(o3d.core.Tensor(rays.astype('float32')), nthreads=2)
        return out['t_hit'].numpy().astype(float), out['geometry_ids'].numpy()


class FinePlacement:
    def __init__(self, views, training, validation):
        self.views = views
        self.normal, self.pivot = views.normal, views.pivot
        self.basis = views.previous[:3, :3]
        self.pivot_world = self.basis @ self.pivot + views.previous[:3, 3]
        self.training, self.validation = list(training), list(validation)

    def transform(self, value):
        yaw, x, z = value
        basis = Rotation.from_rotvec(self.normal * np.radians(yaw)).as_matrix() @ self.basis
        origin = self.pivot_world - basis @ self.pivot + self.basis[:, 0] * x / 1000 + self.basis[:, 2] * z / 1000
        out = np.eye(4)
        out[:3, :3], out[:3, 3] = basis, origin
        return out

    @staticmethod
    def robust(residual, scale=.008):
        return scale * (np.sqrt(1 + (np.minimum(np.abs(residual), .08) / scale) ** 2) - 1)

    def objective(self, value, indices=None, details=False):
        transform = self.transform(value)
        reports = []
        for fi in self.training if indices is None else indices:
            for view in self.views.views[fi]:
                roi = view['roi']
                front_roi, ids_roi = self.views.render(fi, view, transform, roi)
                front, ids = np.full(len(roi), np.inf), np.full(len(roi), -1, dtype=np.int64)
                front[roi], ids[roi] = front_roi, ids_roi
                has_model = np.isfinite(front)
                visible = view['valid'] & has_model & (view['depth'] >= front - .025)
                model_mask = has_model.reshape(view['shape'])
                edge = model_mask & ~binary_erosion(model_mask) & visible.reshape(view['shape'])
                depth_losses, edge_losses = [], []
                for li in range(len(self.views.links)):
                    selected = visible & (ids == li)
                    if selected.sum() >= 5:
                        depth_losses.append(np.mean(self.robust(view['depth'][selected] - front[selected])))
                    selected_edge = edge & (ids.reshape(view['shape']) == li)
                    if selected_edge.sum() >= 3:
                        spacing = np.median(view['depth'][selected_edge.ravel()]) / view['focal_grid']
                        edge_losses.append(np.mean(self.robust(view['outside_observed'][selected_edge] * spacing)))
                # Fixed observed foreground prevents a fit winning by hiding
                # previously observable surfaces. All joints stay fixed.
                local = (view['points'][view['observed']] - transform[:3, 3]) @ transform[:3, :3]
                closest = self.views.scenes[fi].compute_closest_points(o3d.core.Tensor(local.astype('float32')), nthreads=2)
                distances = np.linalg.norm(closest['points'].numpy() - local, axis=1)
                geometry = closest['geometry_ids'].numpy()
                surface_losses = [np.mean(self.robust(distances[geometry == li]))
                                  for li in range(len(self.views.links)) if (geometry == li).sum() >= 5]
                missing = distance_transform_edt(~model_mask)[view['observed_edge']]
                spacing = np.median(view['depth'][view['observed']]) / view['focal_grid']
                reverse = np.mean(self.robust(missing * spacing))
                surface = float(np.mean(surface_losses)) if surface_losses else .08
                depth = float(np.mean(depth_losses)) if depth_losses else .08
                silhouette = .5 * (float(np.mean(edge_losses)) if edge_losses else .08) + .5 * reverse
                old_visible = view['valid'] & np.isfinite(view['baseline_depth']) & (view['depth'] >= view['baseline_depth'] - .025)
                coverage = visible.sum() / max(1, old_visible.sum())
                loss = .5 * surface + .3 * depth + .2 * silhouette + .05 * max(0, .75 - coverage)
                reports.append({'pose': fi, 'camera': view['name'], 'loss': float(loss), 'coverage': float(coverage)})
        score = float(np.mean([r['loss'] for r in reports])) if reports else .1
        return (score, reports) if details else score


def improves(before, after, reports_before, reports_after):
    """Require a meaningful improvement on the independently withheld poses."""
    if not np.isfinite([before, after]).all() or after >= before - max(.00005, before * .01):
        return False
    for pose in {r['pose'] for r in reports_before}:
        old = np.mean([r['loss'] for r in reports_before if r['pose'] == pose])
        new = [r['loss'] for r in reports_after if r['pose'] == pose]
        if not new or not np.isfinite(new).all() or np.mean(new) > old + .0002:
            return False
    return True


def propose_refinement(payload, evidence, previous, training, validation):
    views = SurfaceViews(payload, evidence, previous, previous['metrics']['cameras'])
    fit = FinePlacement(views, training, validation)
    baseline_training = fit.objective([0, 0, 0])
    baseline_validation, old_reports = fit.objective([0, 0, 0], validation, True)
    results = []
    for start in [[0, 0, 0], [2, 0, 0], [-2, 0, 0]]:
        result = minimize(fit.objective, start, method='Powell', bounds=[(-8, 8), (-12, 12), (-12, 12)],
                          options={'maxfev': 650, 'maxiter': 30, 'xtol': .03, 'ftol': 1e-5})
        if np.isfinite(result.fun):
            results.append(result)
        LOGGER.info('Fine arm hypothesis: %.2f mm', result.fun * 1000)
    if not results:
        raise ValueError('surface refinement did not produce a finite candidate')
    # Hypothesis selection and all optimization use training views only.
    best = min(results, key=lambda r: r.fun)
    validation_after, new_reports = fit.objective(best.x, validation, True)
    accepted = (best.fun < baseline_training - max(.00005, baseline_training * .01)
                and improves(baseline_validation, validation_after, old_reports, new_reports))
    metadata = {'status': 'candidate' if accepted else 'coarse_retained',
                'reason': 'training and withheld views improved' if accepted else 'no reliable improvement on withheld views',
                'training_before': baseline_training, 'training_after': float(best.fun),
                'validation_before': baseline_validation, 'validation_after': validation_after,
                'delta_degrees_mm': best.x.tolist(), 'joint_settings_changed': False}
    return (fit.transform(best.x) if accepted else None), metadata
