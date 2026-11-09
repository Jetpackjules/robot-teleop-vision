import copy
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.ndimage import binary_dilation, binary_erosion, distance_transform_edt

o3d = pytest.importorskip("open3d")
pytest.importorskip("trimesh")
TOOLS = Path(__file__).resolve().parents[1] / "robot_modules/so101/tools"
sys.path.insert(0, str(TOOLS))
from refine_so101_registration import FinePlacement, SurfaceViews, improves


def synthetic_views():
    views = SurfaceViews.__new__(SurfaceViews)
    views.previous = np.eye(4)
    views.previous[:3, 3] = [0, .12, -.9]
    views.links = ['known_fixed_body']
    views.data = {'evidence': {'normal': [0, 1, 0]},
                  'frames': [{'joint_frames': [{'pivot': [0, 0, 0]}], 'pose': [i * 15, 10, 20, 30, 40, 50]}
                             for i in range(5)]}
    mesh = o3d.geometry.TriangleMesh.create_box(.15, .2, .05)
    mesh.translate([-.075, -.1, -.025])
    scene = o3d.t.geometry.RaycastingScene(nthreads=1)
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    views.scenes = [scene] * 5
    views.normal = np.array([0., 1., 0.])
    views.pivot = np.zeros(3)
    fit = FinePlacement(views, [0, 2, 3], [1, 4])
    true_parameters = [2, 5, -4]
    truth = fit.transform(true_parameters)
    views.views = []
    y, x = np.indices((60, 80))
    directions = np.column_stack([((x.ravel() + .5) - 40) / 110,
                                   -((y.ravel() + .5) - 30) / 110, -np.ones(x.size)])
    for fi in range(5):
        view = {'origin': np.array([0, .12, 0]), 'directions': directions,
                'name': 'synthetic-camera', 'shape': x.shape, 'focal_grid': 110, 'valid': np.ones(x.size, dtype=bool)}
        depth, _ = views.render(fi, view, truth)
        observed = np.isfinite(depth)
        view['depth'] = np.where(observed, depth, 2.0)
        view['points'] = view['origin'] + directions * view['depth'][:, None]
        baseline, _ = views.render(fi, view, views.previous)
        mask = observed.reshape(x.shape)
        view.update(baseline_depth=baseline, observed=observed,
                    observed_edge=mask & ~binary_erosion(mask), outside_observed=distance_transform_edt(~mask),
                    roi=binary_dilation(np.isfinite(baseline).reshape(x.shape), iterations=8).ravel())
        views.views.append([view])
    return views, fit, true_parameters


def test_known_surface_and_edges_prefer_the_true_rigid_placement():
    _, fit, truth = synthetic_views()
    assert fit.objective(truth) < 1e-6
    assert fit.objective([0, 0, 0]) > fit.objective(truth) + .0001


def test_refinement_preserves_joint_states_dimensions_and_table_plane():
    views, fit, _ = synthetic_views()
    before = copy.deepcopy(views.data)
    transform = fit.transform([7, 10, -8])
    assert views.data == before
    assert transform[:3, :3].T @ transform[:3, :3] == pytest.approx(np.eye(3), abs=1e-12)
    assert transform[:3, 1] == pytest.approx([0, 1, 0])
    assert transform[1, 3] == pytest.approx(views.previous[1, 3])


def test_default_search_does_not_read_withheld_poses():
    views, fit, _ = synthetic_views()
    class ForbiddenView:
        def __iter__(self):
            pytest.fail('Optimization read a withheld pose')
    views.views[1] = ForbiddenView()
    views.views[4] = ForbiddenView()
    assert np.isfinite(fit.objective([0, 0, 0]))


def test_refinement_requires_improvement_on_withheld_poses():
    old = [{"pose": 1, "loss": .01}, {"pose": 4, "loss": .01}]
    assert not improves(.01, .01001, old, old)
    assert not improves(.01, .00999, old, old)
    good = [{"pose": 1, "loss": .009}, {"pose": 4, "loss": .009}]
    assert improves(.01, .009, old, good)
    bad_pose = [{"pose": 1, "loss": .006}, {"pose": 4, "loss": .011}]
    assert not improves(.01, .0085, old, bad_pose)
