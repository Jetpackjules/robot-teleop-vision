"""Sparse wrist observations must not hide usable views from later sweeps."""

import sys
from pathlib import Path

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1] / "robot_modules/so101/tools"
sys.path.insert(0, str(TOOLS))
import solve_so101_staged_joints as staged


def motion_frames(point_counts):
    frames = []
    for group_index, count in enumerate(point_counts):
        for roll in (-30.0, 0.0, 30.0):
            # Separated planes give real dynamic point sets of known size,
            # surviving the production displacement, height and voxel filters.
            points = [
                [(i % 11) * 0.01 - 0.05, 0.15 + (i // 11) * 0.01, roll * 0.002]
                for i in range(count)
            ]
            frames.append(
                {
                    "calibration_joint_index": 4,
                    "pose": [group_index * 20.0, 0.0, 0.0, 0.0, roll, 0.0],
                    "full_camera_points": [
                        {"name": "D435 A", "points": []},
                        {"name": "D435 B", "points": points},
                    ],
                }
            )
    return frames


def install_axis_fits(monkeypatch, overrides=None, expected_direction=None):
    calls = []
    overrides = overrides or {}
    monkeypatch.setattr(staged, "REFERENCE_CAMERA_SERIAL", "D435 B")
    monkeypatch.setattr(
        staged, "predicted_next_axis", lambda *_: (np.zeros(3), np.array([0.0, 1.0, 0.0]))
    )

    def fit(frames, camera_index, _joint, _sign, _low, _high, _directions, _offsets, basis, origin):
        assert camera_index == 1  # Keep the camera chosen by the base solve.
        group_index = int(frames[0]["pose"][0] / 20)
        calls.append(group_index)
        clouds = staged.wrist_dynamic_clouds(frames, camera_index, origin, basis, origin)
        return {
            "delta": -9.0,
            "collapse_loss_m": 0.005,
            "loss_peak_ratio": 1.20,
            "wrong_direction_loss_ratio": (
                0.9 if expected_direction is not None and _sign != expected_direction else 1.25
            ),
            "dynamic_points_per_pose": [len(cloud) for cloud in clouds],
            **overrides.get(group_index, {}),
        }

    monkeypatch.setattr(staged, "direct_axis_collapse_camera_fit", fit)
    return calls


def solve(frames):
    return staged.solve_wrist_flex_from_direct_axis_collapse(
        frames, [1.0] * 6, [0.0] * 6, np.eye(3), np.zeros(3)
    )


@pytest.mark.parametrize("counts, selected_group", [([77, 110], 1), ([110, 77], 0)])
def test_sparse_view_does_not_abort_other_observation_groups(monkeypatch, counts, selected_group):
    calls = install_axis_fits(monkeypatch)
    result = solve(motion_frames(counts))

    assert calls == [0, 1]
    assert result["camera_results"][0]["group_index"] == selected_group
    assert result["camera_results"][0]["dynamic_points_per_pose"] == [110] * 3
    assert result["reference_camera_serial"] == "D435 B"
    rejected = result["rejected_camera_views"]
    assert len(rejected) == 1
    assert rejected[0]["camera"] == "D435 B"
    assert "77 wrist-motion points" in rejected[0]["reason"]


def test_all_sparse_views_still_reject_with_each_view_diagnostic(monkeypatch):
    calls = install_axis_fits(monkeypatch)
    with pytest.raises(ValueError) as error:
        solve(motion_frames([77, 60]))

    assert calls == [0, 1]
    assert "77 wrist-motion points" in str(error.value)
    assert "60 wrist-motion points" in str(error.value)
    assert "D435 B" in str(error.value)
    assert "D455" not in str(error.value)


@pytest.mark.parametrize(
    "bad_fit, reason",
    [
        ({"dynamic_points_per_pose": [110, 99, 110]}, "not dense"),
        ({"wrong_direction_loss_ratio": 1.11}, "not decisive"),
        ({"loss_peak_ratio": 1.04}, "competing direct-axis"),
        ({"collapse_loss_m": 0.036}, "residual is too high"),
        ({"collapse_loss_m": float("nan")}, "invalid scores"),
        ({"collapse_loss_m": 0.0}, "invalid scores"),
        ({"loss_peak_ratio": float("inf")}, "invalid scores"),
    ],
)
def test_invalid_fit_is_rejected_before_ranking_views(monkeypatch, bad_fit, reason):
    # Make the bad view outrank the good one under the old quality formula.
    install_axis_fits(
        monkeypatch,
        {
            0: {"collapse_loss_m": 0.001, **bad_fit},
            1: {
                "collapse_loss_m": 0.034,
                "loss_peak_ratio": 1.05,
                "wrong_direction_loss_ratio": 1.12,
            },
        },
    )
    result = solve(motion_frames([110, 110]))

    assert result["camera_results"][0]["group_index"] == 1
    assert reason in result["rejected_camera_views"][0]["reason"]

    # The fallback is another validated view, never acceptance of the bad fit.
    with pytest.raises(ValueError, match=reason):
        solve(motion_frames([110]))


def test_shallow_basin_is_allowed_only_at_the_existing_fixed_point(monkeypatch):
    install_axis_fits(monkeypatch, {0: {"delta": 2.0, "loss_peak_ratio": 1.01}})
    result = solve(motion_frames([110]))
    assert result["fixed_point_only_shallow_basin"] is True
    assert result["offset_delta_degrees"] == 2.0


def test_sparse_construction_does_not_lower_final_density_threshold(monkeypatch):
    install_axis_fits(monkeypatch)
    with pytest.raises(ValueError, match="not dense"):
        solve(motion_frames([80, 99]))


def test_wrist_sign_error_uses_selected_reference_camera(monkeypatch):
    calls = install_axis_fits(monkeypatch)
    with pytest.raises(ValueError) as error:
        staged.solve_wrist_roll_direction_from_motion(
            motion_frames([77, 60]), [1.0] * 6, [0.0] * 6, np.eye(3), np.zeros(3)
        )
    assert calls == [0, 1, 0, 1]
    assert "D435 B" in str(error.value)
    assert "D455" not in str(error.value)


def test_wrist_sign_can_use_later_valid_view_and_still_reject_wrong_direction(monkeypatch):
    calls = install_axis_fits(monkeypatch, expected_direction=-1.0)
    result = staged.solve_wrist_roll_direction_from_motion(
        motion_frames([77, 110]), [1.0] * 6, [0.0] * 6, np.eye(3), np.zeros(3)
    )
    assert calls == [0, 1, 0, 1]
    assert result["fitted_direction"] == -1.0
    assert "not decisive" in result["rejected_candidates"]["1.0"]
    assert result["camera_results"][0]["group_index"] == 1
    assert "77 wrist-motion points" in result["rejected_camera_views"][0]["reason"]


def test_fixed_region_solver_uses_both_cameras_with_selected_reference_first(monkeypatch):
    monkeypatch.setattr(staged, "REFERENCE_CAMERA_SERIAL", "D435 B")
    seed_calls = []
    fit_calls = []
    fit = {
        "delta": 0.0,
        "support": 150,
        "minimum_pose_support": 10,
        "median_residual_m": 0.005,
        "support_peak_ratio": 1.2,
    }

    def coarse(_frames, camera_index, *_):
        seed_calls.append(camera_index)
        return [dict(fit)]

    def refine(_frames, camera_index, *_):
        fit_calls.append(camera_index)
        return dict(fit)

    monkeypatch.setattr(staged, "next_axis_camera_curve", coarse)
    monkeypatch.setattr(staged, "best_axis_curve_result", lambda curve: curve[0])
    monkeypatch.setattr(staged, "refine_fixed_region_axis_fit", refine)
    monkeypatch.setattr(staged, "fixed_region_axis_camera_curve", lambda *_: [{"support": 50}])
    monkeypatch.setattr(
        staged, "predicted_next_axis", lambda *_: (np.zeros(3), np.array([0.0, 1.0, 0.0]))
    )
    result = staged.solve_wrist_flex_from_roll_axis(
        motion_frames([110]), [1.0] * 6, [0.0] * 6, np.eye(3), np.zeros(3)
    )
    assert seed_calls == [1]
    assert fit_calls == [1, 0]
    assert [item["camera"] for item in result["camera_results"]] == ["D435 B", "D435 A"]
