"""Exercise the real mesh fit in the same joint coordinates used by Godot."""

import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import pytest
from scipy.spatial import cKDTree

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_modules/so101/tools"))
import solve_so101_staged_joints as staged


class SingleWorkerTree(cKDTree):
    def query(self, *args, **kwargs):
        # Thread creation dominates these small clouds. Keep the real query.
        kwargs["workers"] = 1
        return super().query(*args, **kwargs)


@pytest.mark.parametrize("true_offset", [0.0, -45.0])
def test_mesh_fit_result_reproduces_observed_gripper_without_quarter_turn(monkeypatch, true_offset):
    monkeypatch.setattr(staged, "cKDTree", SingleWorkerTree)
    o3d.utility.random.seed(51)
    fixed_mesh = o3d.io.read_triangle_mesh(str(staged.ASSET_ROOT / "gripper_link.glb"))
    fixed = np.asarray(fixed_mesh.sample_points_uniformly(3000).points)
    fixed = fixed[fixed[:, 2] <= staged.DISTAL_FIXED_JAW_MAXIMUM_LOCAL_Z_M]
    moving_mesh = o3d.io.read_triangle_mesh(str(staged.ASSET_ROOT / "moving_jaw_so101_v1_link.glb"))
    moving = np.asarray(moving_mesh.sample_points_uniformly(1500).points)
    directions = [1.0, -1.0, 1.0, 1.0, 1.0, 1.0]
    offsets = [40.4296875, 80.0, 0.0, -70.0, true_offset, 0.0]
    frames = []
    for roll in [-100.0, -80.0, -60.0, -40.0, -20.0]:
        frame = {"pose": [-40.0, 130.0, 38.0, 78.0, roll, 25.0]}
        observed = np.concatenate([
            staged.posed_mesh(fixed, frame, 4, 0, directions, offsets, np.eye(3), np.zeros(3)),
            staged.posed_moving_jaw(moving, frame, directions, offsets, np.eye(3), np.zeros(3)),
        ])
        frame["full_camera_points"] = [{"name": "RealSense D435 B", "points": observed}]
        frames.append(frame)
    initial_offsets = [*offsets[:4], 0.0, 0.0]
    fit = staged.multiangle_distal_wrist_roll_camera_fit(
        frames, 0, 1.0, fixed, moving, np.empty((0, 3)),
        directions, initial_offsets, np.eye(3), np.zeros(3),
    )
    assert fit["raw_model_fitted_offset_degrees"] == pytest.approx(true_offset, abs=0.51)
    assert fit["fitted_offset_degrees"] == pytest.approx(true_offset, abs=0.51)
    applied_offsets = [*initial_offsets[:4], fit["fitted_offset_degrees"], 0.0]
    predicted = staged.posed_mesh(fixed, frames[0], 4, 0, directions, applied_offsets, np.eye(3), np.zeros(3))
    actual = staged.posed_mesh(fixed, frames[0], 4, 0, directions, offsets, np.eye(3), np.zeros(3))
    assert np.max(np.linalg.norm(predicted - actual, axis=1)) < 0.0003
