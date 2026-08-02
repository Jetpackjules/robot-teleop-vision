from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from solve_so101_claw_visual import correction_displacement_at_points, solve  # noqa: E402
from solve_so101_claw_rgb_tips import optimization_seed  # noqa: E402
import probe_so101_claw_hinge_frame as hinge_probe  # noqa: E402
from so101_follower_service import (  # noqa: E402
    CALIBRATION_SAFE_HEIGHT_M,
    build_claw_calibration_sweep_waypoints,
    tool_clearance_metric,
)


def transform_dictionary(value: np.ndarray) -> dict:
    return {
        "basis_x": value[:3, 0].tolist(),
        "basis_y": value[:3, 1].tolist(),
        "basis_z": value[:3, 2].tolist(),
        "origin": value[:3, 3].tolist(),
    }


def rotate(points: np.ndarray, degrees: float) -> np.ndarray:
    return points @ Rotation.from_euler("z", degrees, degrees=True).as_matrix().T


def box_surface(rng: np.random.Generator, center: list[float], size: list[float], count: int) -> np.ndarray:
    points = rng.uniform(-0.5, 0.5, (count, 3)) * np.asarray(size)
    faces = rng.integers(0, 3, count)
    signs = rng.choice((-0.5, 0.5), count)
    points[np.arange(count), faces] = signs * np.asarray(size)[faces]
    return points + np.asarray(center)


def synthetic_capture() -> tuple[dict, list[float], np.ndarray]:
    rng = np.random.default_rng(12)
    fixed = np.concatenate(
        (
            box_surface(rng, [0.050, 0.000, 0.015], [0.080, 0.025, 0.030], 700),
            box_surface(rng, [0.078, 0.012, 0.033], [0.025, 0.014, 0.020], 300),
        ),
        axis=0,
    )
    jaw_zero = box_surface(rng, [0.065, 0.000, 0.000], [0.065, 0.013, 0.018], 850)
    normalized = [5.0, 25.0, 50.0, 75.0, 90.0]
    model_angles = [-10.0, 12.0, 35.0, 60.0, 76.0]
    physical_angles = [-10.0, 15.5, 41.5, 66.25, 79.5]
    shared = np.eye(4)
    shared[:3, :3] = Rotation.from_euler("xyz", [3.0, -2.0, 4.0], degrees=True).as_matrix()
    shared[:3, 3] = [0.004, -0.003, 0.006]
    frames = []
    for opening, model_angle, physical_angle in zip(
        normalized,
        model_angles,
        physical_angles,
        strict=True,
    ):
        model_jaw = rotate(jaw_zero, model_angle)
        physical = np.concatenate((fixed, rotate(jaw_zero, physical_angle)), axis=0)
        physical = physical @ shared[:3, :3].T + shared[:3, 3]
        scene = physical + rng.normal(0.0, 0.00035, physical.shape)
        scene = np.repeat(scene, 2, axis=0)
        scene = np.concatenate(
            (
                scene,
                rng.uniform([0.20, 0.20, -0.02], [0.35, 0.35, 0.02], (500, 3)),
            ),
            axis=0,
        )
        joint_frames = [{"pivot": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0]} for _ in range(6)]
        frames.append(
            {
                "pose": [0.0, 110.0, 60.0, 75.0, -110.0, opening],
                "calibration_joint_index": 5,
                "full_camera_points": [{"name": "RealSense D455 synthetic", "points": scene.tolist()}],
                "mesh_points": {
                    "gripper_link": fixed.tolist(),
                    "moving_jaw_so101_v1_link": model_jaw.tolist(),
                },
                "joint_frames": joint_frames,
                "model_transform": transform_dictionary(np.eye(4)),
                "claw_metadata": {
                    "pre_visual_transform_local": transform_dictionary(np.eye(4)),
                    "corrected_transform_local": transform_dictionary(np.eye(4)),
                    "model_gripper_angle_degrees": model_angle,
                    "mount_correction_local": [0.0, 0.0, 0.0],
                },
            }
        )
    return {"type": "so101_motion_capture", "frames": frames}, physical_angles, shared


def test_five_state_solver_recovers_rigid_frame_and_opening_curve() -> None:
    capture, physical_angles, shared = synthetic_capture()
    result = solve(capture)
    assert result["type"] == "so101_claw_visual_fit"
    assert result["validation_pose_count"] == 5
    np.testing.assert_allclose(
        result["gripper_angle_samples_degrees"],
        physical_angles,
        atol=2.0,
    )
    np.testing.assert_allclose(
        result["gripper_visual_correction_translation_local"],
        shared[:3, 3],
        atol=0.002,
    )
    assert result["confidence"] >= 0.72
    assert result["median_residual_m"] <= 0.018


def test_world_space_rotation_bound_measures_displacement_at_gripper() -> None:
    center = np.asarray([0.1, 2.35, -0.7])
    points = center + np.asarray([
        [-0.04, 0.0, 0.0],
        [0.04, 0.0, 0.0],
        [0.0, -0.02, 0.0],
        [0.0, 0.02, 0.0],
    ])
    rotation = Rotation.from_euler("z", 12.0, degrees=True).as_matrix()
    correction = np.eye(4)
    correction[:3, :3] = rotation
    correction[:3, 3] = center - rotation @ center + [0.004, 0.0, 0.0]
    assert np.linalg.norm(correction[:3, 3]) > 0.2
    assert correction_displacement_at_points(correction, points) < 0.005


def test_claw_sweep_samples_only_gripper_after_exposed_anchor() -> None:
    initial = [0.0, 130.0, 35.0, 75.0, -110.0, 42.0]
    waypoints = build_claw_calibration_sweep_waypoints(initial)
    sampled = [
        pose
        for pose, _, label in waypoints
        if label.startswith("sampling servo 5")
    ]
    assert [pose[5] for pose in sampled] == [5.0, 25.0, 50.0, 75.0, 90.0]
    assert all(pose[:5] == sampled[0][:5] for pose in sampled)
    assert all(tool_clearance_metric(pose) >= CALIBRATION_SAFE_HEIGHT_M for pose in sampled)
    assert waypoints[-1][0] == initial


def test_rgb_tip_optimizer_rounds_near_identity_seed_to_exact_zero() -> None:
    pre = np.eye(4)
    pre[:3, :3] = Rotation.from_euler(
        "xyz", [23.0, -11.0, 41.0], degrees=True
    ).as_matrix()
    pre[:3, 3] = [0.12, -0.07, 0.33]
    corrected = pre.copy()
    seed = optimization_seed(pre, corrected)
    np.testing.assert_array_equal(seed, np.zeros(6))


def test_claw_optical_retry_changes_only_wrist_view_before_sampling() -> None:
    initial = [0.0, 130.0, 35.0, 75.0, -110.0, 42.0]
    views = []
    for attempt in (1, 2, 3, 4, 5):
        sampled = [
            pose
            for pose, _, label in build_claw_calibration_sweep_waypoints(initial, attempt)
            if label.startswith("sampling servo 5")
        ]
        assert len(sampled) == 5
        assert all(pose[:4] == sampled[0][:4] for pose in sampled)
        assert all(pose[4] == sampled[0][4] for pose in sampled)
        assert all(tool_clearance_metric(pose) >= CALIBRATION_SAFE_HEIGHT_M for pose in sampled)
        views.append(sampled[0][4])
    assert views == [-20.0, -5.0, -35.0, 10.0, -50.0]


def test_installed_moving_jaw_geometry_can_be_inverted_for_recalibration(
    monkeypatch,
) -> None:
    raw_tip = np.asarray([-0.011, -0.082, 0.019])
    pivot = np.asarray([0.019, -0.017, -0.004])
    axis = np.asarray([-0.990, -0.102, -0.097])
    axis /= np.linalg.norm(axis)
    closed_basis = Rotation.from_euler(
        "xyz", [87.0, -4.0, -96.0], degrees=True
    ).as_matrix()
    opening_degrees = 46.5
    radial_scale = 1.187
    axial_translation = 0.0182
    visual_tip = raw_tip.copy()
    visual_tip[:2] *= radial_scale
    visual_tip[2] += axial_translation
    posed_tip = pivot + (
        Rotation.from_rotvec(axis * np.radians(opening_degrees)).as_matrix()
        @ closed_basis
        @ visual_tip
    )
    metadata = {
        "corrected_transform_local": transform_dictionary(np.eye(4)),
        "moving_jaw_calibration_enabled": True,
        "moving_jaw_pivot_parent": pivot.tolist(),
        "moving_jaw_axis_parent": axis.tolist(),
        "moving_jaw_closed_basis_parent_row_major": closed_basis.reshape(-1).tolist(),
        "moving_jaw_opening_degrees": opening_degrees,
        "moving_jaw_visual_radial_scale": radial_scale,
        "moving_jaw_visual_axial_translation_local": axial_translation,
    }
    frame = {"claw_metadata": metadata}
    monkeypatch.setattr(
        hinge_probe.solver,
        "terminal_tip_positions_local",
        lambda _frame: np.stack((np.zeros(3), posed_tip)),
    )
    recovered = hinge_probe.raw_moving_tip([[frame]])
    np.testing.assert_allclose(recovered, raw_tip, atol=1.0e-9)
