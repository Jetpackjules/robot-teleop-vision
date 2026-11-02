"""Registration uses captured calibrated geometry, with only one rigid placement."""
import copy
import sys
import types
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

TOOLS = Path(__file__).resolve().parents[1] / "robot_modules/so101/tools"
sys.path.insert(0, str(TOOLS))
import solve_so101_registration as registration


def transform(basis, origin):
    return {**{f"basis_{axis}": basis[:, i].tolist() for i, axis in enumerate("xyz")},
            "origin": np.asarray(origin).tolist()}


def capture(yaw=43.0, secondary_error=0.0, noise=0.001):
    rng = np.random.default_rng(21)
    pivot = np.array([0.0, 0.0624, -0.0388353])
    normal = Rotation.from_rotvec([0.03, 0.0, -0.02]).apply([0, 1, 0])
    tilt = Rotation.from_rotvec([0.03, 0.0, -0.02]).as_matrix()
    basis = Rotation.from_rotvec(normal * np.radians(yaw)).as_matrix() @ tilt
    axis = np.array([0.18, 0.07, -0.43])
    origin = axis - basis @ pivot
    evidence = {"normal": normal.tolist(), "axis_fit": {"axis_point":
                (axis + tilt[:, 0] * 0.009 - tilt[:, 2] * 0.006).tolist()},
                "reference_camera": "primary"}
    frames = []
    # These sampled link positions represent an already configured, bent arm.
    # They deliberately do not come from the solver's/default SO-101 FK.
    centers = [(0, .035, 0), (.01, .08, -.04), (.045, .15, -.11),
               (.06, .23, -.06), (.04, .27, .01), (.015, .25, .09), (.03, .23, .1)]
    raw = {name: rng.normal(0, [.009, .018, .012], (35, 3)) + center
           for name, center in zip(registration.LINKS, centers)}
    for pan in [-35, -18, 0, 19, 37]:
        turn = Rotation.from_rotvec([0, np.radians(pan), 0]).as_matrix()
        mesh = {name: ((p - pivot) @ turn.T + pivot if name != "base_link" else p)
                for name, p in raw.items()}
        observed = np.concatenate(list(mesh.values())) @ basis.T + origin
        observed += rng.normal(0, noise, observed.shape)
        views = [{"name": "primary", "points": observed.tolist()},
                 {"name": "secondary", "points": (observed + secondary_error).tolist()}]
        frames.append({"pose": [pan, 97, -23, 38, 12, 45],
                       "mesh_points": {name: p.tolist() for name, p in mesh.items()},
                       "model_transform": transform(tilt, origin),
                       "joint_frames": [{"pivot": pivot.tolist()}], "full_camera_points": views})
    return {"frames": frames, "kinematics_token": "saved-nondefault-model"}, evidence, basis, origin


def test_recovers_large_yaw_and_small_translation_without_changing_geometry():
    payload, evidence, basis, origin = capture()
    before = copy.deepcopy(payload)
    result = registration.fit_registration(payload, evidence)
    fitted = registration.matrix(result["preview_transform"])
    assert fitted[:3, :3] == pytest.approx(basis, abs=0.008)
    assert fitted[:3, 3] == pytest.approx(origin, abs=0.002)
    assert result["metrics"]["cameras"] == ["primary", "secondary"]
    assert result["metrics"]["validation_poses"]
    assert result["metrics"]["joint_mapping_changed"] is False
    assert payload == before
    assert not any("offset" in key or "direction" in key for key in result)


def test_misaligned_auxiliary_camera_cannot_pull_the_arm_away():
    payload, evidence, basis, origin = capture(yaw=-38, secondary_error=0.14)
    result = registration.fit_registration(payload, evidence)
    fitted = registration.matrix(result["preview_transform"])
    assert fitted[:3, :3] == pytest.approx(basis, abs=0.008)
    assert fitted[:3, 3] == pytest.approx(origin, abs=0.002)
    assert result["metrics"]["cameras"] == ["primary"]
    assert result["metrics"]["excluded_cameras"][0]["camera"] == "secondary"


def test_dense_mesh_cache_is_bounded_for_every_pose():
    payload, evidence, *_ = capture()
    dense = payload["frames"][2]["mesh_points"]
    for name, values in dense.items():
        dense[name] = np.repeat(values, 100, axis=0).tolist()
    before = copy.deepcopy(payload)
    fit = registration.PlacementFit(payload, evidence)
    assert all(len(points) <= 220 for model in fit.models for points in model.values())
    assert payload == before


def test_withheld_pose_mismatch_is_rejected():
    payload, evidence, *_ = capture()
    # Pose 1 is withheld. Training can be perfect while this pose disagrees.
    for camera in payload["frames"][1]["full_camera_points"]:
        camera["points"] = (np.asarray(camera["points"]) + [0.10, 0, -0.10]).tolist()
    with pytest.raises(ValueError, match="validation|coverage"):
        registration.fit_registration(payload, evidence)


def test_unobservable_heading_is_rejected():
    payload, evidence, *_ = capture(noise=0)
    circle = np.column_stack([0.09 * np.cos(np.linspace(0, 2*np.pi, 240, endpoint=False)),
                              np.full(240, .15),
                              0.09 * np.sin(np.linspace(0, 2*np.pi, 240, endpoint=False))])
    for frame in payload["frames"]:
        frame["mesh_points"] = {name: circle.tolist() for name in registration.LINKS}
        cloud = circle @ registration.matrix(frame["model_transform"])[:3, :3].T
        cloud += registration.matrix(frame["model_transform"])[:3, 3]
        for camera in frame["full_camera_points"]:
            camera["points"] = cloud.tolist()
    with pytest.raises(ValueError, match="ambiguous|limit|validation"):
        registration.fit_registration(payload, evidence)


def test_visibility_excludes_occlusion_but_keeps_free_space_errors():
    snapshot = {"camera_inverse": transform(np.eye(3), [0, 0, 0]),
                "intrinsics": [10, 10, 1, 1], "grid_stride": 1, "sample_step": 1,
                "cols": 3, "rows": 3, "depths": [1.0] * 9, "valid": [1] * 9}
    assert registration.visible(np.array([[0, 0, -1], [0, 0, -1.2], [0, 0, -.8]]), snapshot).tolist() == [True, False, True]


def test_visibility_reuses_prepared_depth_without_changing_capture(monkeypatch):
    snapshot = {"camera_inverse": transform(np.eye(3), [0, 0, 0]),
                "intrinsics": [10, 10, 1, 1], "grid_stride": 1, "sample_step": 1,
                "cols": 3, "rows": 3, "depths": [1.0] * 9, "valid": [1] * 9}
    original = copy.deepcopy(snapshot)
    prepared = registration.prepare_snapshot(snapshot)
    monkeypatch.setattr(registration, "matrix", lambda *_: pytest.fail("repeated camera conversion"))
    assert registration.visible(np.array([[0, 0, -1]]), prepared).tolist() == [True]
    assert snapshot == original


def test_complementary_partial_views_validate_as_one_arm(monkeypatch):
    payload, evidence, basis, origin = capture(yaw=0, noise=0)
    names = list(registration.LINKS)
    # Depth is valid only at each camera's observed subset. No single camera
    # sees three links, and the reference is entirely absent in one pose.
    for fi, frame in enumerate(payload["frames"]):
        frame["full_camera_points"], frame["depth_snapshots"] = [], []
        for ci, selected in enumerate([names[:2], names[2:4], names[4:6], names[6:]]):
            if fi == 1 and ci == 0:
                continue
            camera = "primary" if ci == 0 else f"part-{ci}"
            cloud = np.concatenate([frame["mesh_points"][name] for name in selected]) @ basis.T + origin
            # Each fragment gets its own camera crop, containing no other links.
            frame["full_camera_points"].append({"name": camera, "points": cloud.tolist()})
            frame["depth_snapshots"].append({"name": camera, "selected": selected})
    # Isolate the aggregation rule from the separately tested projection math.
    # The masks represent cameras with disjoint crops of the known arm.
    snapshots = [frame.pop("depth_snapshots") for frame in payload["frames"]]
    fit = registration.PlacementFit(payload, evidence)
    from scipy.spatial import cKDTree
    for fi, frame in enumerate(payload["frames"]):
        for snapshot in snapshots[fi]:
            camera = snapshot["name"]
            cloud = np.concatenate([frame["mesh_points"][name] for name in snapshot["selected"]]) @ basis.T + origin
            tree, _ = fit.views[fi][camera]
            fit.views[fi][camera] = (tree, cKDTree(cloud))
    monkeypatch.setattr(registration, "visible", lambda world, snapshot: snapshot.query(world)[0] < 1e-6)
    # The pivot evidence intentionally includes a small local translation.
    value = np.array([0.0, -0.009, 0.006])
    loss, views = fit.score(value, ["primary", "part-1", "part-2", "part-3"], range(5), True)
    poses = fit.pose_reports(views)
    assert loss < 1e-6
    assert all(len(v["supported_links"]) <= 2 for v in views)
    assert all(len(p["supported_links"]) >= 5 for p in poses)
    assert all(p["coverage_weight"] >= 1.5 for p in poses)


def test_tiny_visible_fragment_has_less_weight_and_cannot_veto():
    good = {"samples": 100, "support": 1.0, "squared_residual_m2": 0.001**2}
    bad = {"samples": 3, "support": 0.0, "squared_residual_m2": 0.06**2}
    result = registration.PlacementFit.summarize_links([{"upper_arm_link": good, "base_link": bad}])
    assert result["loss_m"] < 0.008
    assert result["contradicted_links"] == []


def test_empty_view_contributes_no_mismatch_and_true_visible_conflict_remains():
    good = {"samples": 100, "support": 1.0, "squared_residual_m2": 0.001**2}
    conflict = {"samples": 100, "support": 0.0, "squared_residual_m2": 0.06**2}
    summarize = registration.PlacementFit.summarize_links
    assert summarize([{}, {"wrist_link": good}]) == summarize([{"wrist_link": good}])
    assert summarize([{"base_link": conflict}])["contradicted_links"] == ["base_link"]


def test_visibility_ignores_cropped_and_missing_depth_without_ignoring_free_space():
    snapshot = {"camera_inverse": transform(np.eye(3), [0, 0, 0]),
                "intrinsics": [10, 10, 1, 1], "grid_stride": 1, "sample_step": 1,
                "cols": 3, "rows": 3, "depths": [1.0] * 9,
                "valid": [1, 1, 1, 1, 0, 1, 1, 1, 1]}
    world = np.array([[0, 0, -1], [1, 0, -1], [0, 1, -1],
                      [0, 0, 1], [.1, 0, -1], [.1, 0, -.8], [.1, 0, -1.2]])
    assert registration.visible(world, snapshot).tolist() == [False, False, False, False, True, True, False]


@pytest.mark.parametrize("failure", ["worse", "unavailable", "bounds"])
def test_optional_refinement_preserves_the_validated_placement(monkeypatch, failure):
    payload, evidence, basis, origin = capture(noise=0)
    for frame in payload["frames"]:
        frame["link_visual_transforms"] = {"captured": True}
    fit = registration.PlacementFit(payload, evidence)
    coarse = {"preview_transform": transform(basis, origin), "kinematics_token": "unchanged",
              "metrics": {"cameras": ["primary", "secondary"], "loss_m": 0.0}, "confidence": 1.0}
    before = copy.deepcopy(coarse)
    def propose(*_):
        if failure == "unavailable":
            raise ImportError("optional dependency unavailable")
        if failure == "worse":
            return None, {"status": "coarse_retained", "reason": "withheld views did not improve"}
        bad = registration.matrix(coarse["preview_transform"])
        bad[:3, 3] += .1
        return bad, {"status": "candidate"}
    monkeypatch.setitem(sys.modules, "refine_so101_registration", types.SimpleNamespace(propose_refinement=propose))
    result = registration.refine_validated_placement(payload, evidence, coarse, fit)
    assert result["preview_transform"] == before["preview_transform"]
    assert result["kinematics_token"] == before["kinematics_token"]
    assert result["metrics"]["fine_refinement"]["status"] == "coarse_retained"
    assert coarse == before


@pytest.mark.parametrize("damage", ["missing_model", "short_sweep", "missing_token"])
def test_insufficient_or_unbound_capture_fails_closed(damage):
    payload, evidence, *_ = capture()
    if damage == "missing_model":
        payload["frames"][0]["mesh_points"] = {}
    elif damage == "short_sweep":
        for frame in payload["frames"]:
            frame["pose"][0] = 0
    else:
        payload.pop("kinematics_token")
    with pytest.raises(ValueError):
        registration.fit_registration(payload, evidence)
