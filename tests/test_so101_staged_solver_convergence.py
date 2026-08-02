import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools import solve_so101_staged_joints as staged
from tools.so101_kinematics import rendered_link_transforms


def test_moving_jaw_orientation_check_uses_measured_openness():
    pose = [0.0, 0.0, 0.0, 0.0, 15.0, 88.0]
    points = np.asarray(
        ((0.0, 0.0, 0.0), (0.01, -0.02, 0.03)),
        dtype=float,
    )
    predicted = staged.posed_moving_jaw(
        points,
        {"pose": pose},
        [1.0, -1.0, 1.0, 1.0, 1.0, 1.0],
        [40.4296875, 80.0, 0.0, -70.0, 0.0, 0.0],
        staged.ROS_TO_GODOT.T,
        np.zeros(3),
    )
    transform = rendered_link_transforms(pose)[
        "moving_jaw_so101_v1_link"
    ]
    expected = points @ transform[:3, :3].T + transform[:3, 3]

    assert predicted == pytest.approx(expected)


def test_outward_chain_iterates_to_a_fixed_point(monkeypatch):
    target = [0.0, 12.0, -7.0, 4.0, 0.0, 0.0]

    def solve(_frames, joint_index, _directions, offsets, _basis, _origin):
        return {
            "joint_index": joint_index,
            "joint": staged.JOINT_NAMES[joint_index],
            "offset_delta_degrees": target[joint_index] - offsets[joint_index],
        }

    monkeypatch.setattr(staged, "solve_joint_from_next_axis", solve)
    offsets, results, convergence = staged.solve_outward_chain_to_fixed_point(
        [],
        3,
        [1.0] * 6,
        [0.0] * 6,
        np.eye(3),
        np.zeros(3),
    )

    assert offsets == pytest.approx(target)
    assert convergence["passes"] == 2
    assert convergence["pass_history"][0]["corrections_degrees"] == pytest.approx(
        target[1:4]
    )
    assert convergence["pass_history"][1]["maximum_correction_degrees"] == 0.0
    assert [result["offset_delta_degrees"] for result in results] == pytest.approx(
        target[1:4]
    )
    assert [result["fixed_point_delta_degrees"] for result in results] == [
        0.0,
        0.0,
        0.0,
    ]


def test_outward_chain_rejects_a_nonconvergent_candidate(monkeypatch):
    def solve(_frames, joint_index, _directions, _offsets, _basis, _origin):
        return {
            "joint_index": joint_index,
            "joint": staged.JOINT_NAMES[joint_index],
            "offset_delta_degrees": 3.0,
        }

    monkeypatch.setattr(staged, "solve_joint_from_next_axis", solve)
    with pytest.raises(ValueError, match="did not converge after 3 passes"):
        staged.solve_outward_chain_to_fixed_point(
            [],
            3,
            [1.0] * 6,
            [0.0] * 6,
            np.eye(3),
            np.zeros(3),
        )


def test_mapped_mesh_arbitration_can_overturn_false_motion_basin():
    candidates = staged.mapped_mesh_arbitration_candidates(
        [{"delta": 82.5}, {"delta": 85.0}]
    )

    assert 82.5 in candidates
    assert 85.0 in candidates
    assert -30.0 in candidates
    assert 0.0 in candidates
    assert 30.0 in candidates
    assert len(candidates) == len(set(candidates))


def test_supported_current_zero_is_preserved_on_shallow_mesh_curve():
    assert staged.should_preserve_current_zero_from_mapped_depth(
        fitted_losses=[0.0043, 0.0051],
        current_zero_losses=[0.0047, 0.0050],
    )


def test_uniquely_better_mesh_evidence_can_change_current_zero():
    assert not staged.should_preserve_current_zero_from_mapped_depth(
        fitted_losses=[0.0040, 0.0045],
        current_zero_losses=[0.0070, 0.0075],
    )


def test_axis_pose_coverage_allows_partial_independent_camera_view():
    assert staged.has_sufficient_cross_camera_settled_axis_pose_coverage(
        [
            [14, 12, 10, 9, 8],
            [0, 7, 10, 12, 14],
        ]
    )


def test_axis_pose_coverage_rejects_fewer_than_three_independent_poses():
    assert not staged.has_sufficient_cross_camera_settled_axis_pose_coverage(
        [
            [14, 12, 10, 9, 8],
            [0, 0, 7, 12, 14],
        ]
    )


def test_wrist_flex_refinement_excludes_near_opposite_branch():
    assert staged.wrist_flex_local_correction_bounds(-60.75) == pytest.approx(
        (-30.0, 30.0)
    )


def test_wrist_flex_refinement_respects_absolute_mechanical_limit():
    assert staged.wrist_flex_local_correction_bounds(85.0) == pytest.approx(
        (-30.0, 15.0)
    )


def test_wrist_roll_zero_applies_large_correction_then_validates(monkeypatch):
    calls = []

    def solve(
        _frames,
        _directions,
        offsets,
        _basis,
        _origin,
        _locked_direction=None,
    ):
        calls.append(list(offsets))
        target = -105.0
        return {
            "fitted_direction": -1.0,
            "fitted_offset_degrees": target,
            "offset_delta_degrees": target - offsets[4],
            "camera_results": [{"camera": "D455"}],
        }

    monkeypatch.setattr(
        staged,
        "solve_wrist_roll_zero_from_stock_gripper",
        solve,
    )
    directions, offsets, result = staged.solve_wrist_roll_zero_to_fixed_point(
        [],
        [1.0] * 6,
        [0.0] * 6,
        np.eye(3),
        np.zeros(3),
    )

    assert directions[4] == -1.0
    assert offsets[4] == pytest.approx(-105.0)
    assert calls[0][4] == 0.0
    assert calls[1][4] == pytest.approx(-105.0)
    assert result["offset_delta_degrees"] == pytest.approx(-105.0)
    assert result["fixed_point_delta_degrees"] == 0.0


def test_wrist_roll_zero_rejects_non_fixed_point(monkeypatch):
    def solve(
        _frames,
        _directions,
        offsets,
        _basis,
        _origin,
        _locked_direction=None,
    ):
        return {
            "fitted_direction": -1.0,
            "fitted_offset_degrees": offsets[4] + 7.0,
            "offset_delta_degrees": 7.0,
        }

    monkeypatch.setattr(
        staged,
        "solve_wrist_roll_zero_from_stock_gripper",
        solve,
    )
    with pytest.raises(ValueError, match="did not reach a fixed point"):
        staged.solve_wrist_roll_zero_to_fixed_point(
            [],
            [1.0] * 6,
            [0.0] * 6,
            np.eye(3),
            np.zeros(3),
        )


def test_wrist_roll_evidence_accepts_strong_full_surface():
    assert staged.wrist_roll_evidence_class(
        mesh_loss_m=0.007,
        strong_support_fraction=0.72,
        partial_support_fraction=0.91,
        peak_ratio=1.21,
        basin_width_degrees=13.0,
    ) == "strong_full_surface"


def test_wrist_roll_evidence_accepts_unique_partial_opposite_view():
    assert staged.wrist_roll_evidence_class(
        mesh_loss_m=0.020,
        strong_support_fraction=0.18,
        partial_support_fraction=0.52,
        peak_ratio=1.08,
        basin_width_degrees=23.0,
    ) == "unique_partial_surface"


@pytest.mark.parametrize(
    "mesh_loss, partial_support, peak_ratio, basin_width",
    (
        (0.026, 0.52, 1.08, 23.0),
        (0.020, 0.39, 1.08, 23.0),
        (0.020, 0.52, 1.05, 23.0),
        (0.020, 0.52, 1.08, 25.5),
    ),
)
def test_wrist_roll_evidence_rejects_weak_or_ambiguous_partial_view(
    mesh_loss,
    partial_support,
    peak_ratio,
    basin_width,
):
    assert staged.wrist_roll_evidence_class(
        mesh_loss_m=mesh_loss,
        strong_support_fraction=0.18,
        partial_support_fraction=partial_support,
        peak_ratio=peak_ratio,
        basin_width_degrees=basin_width,
    ) == "invisible_or_ambiguous"


def valid_d455_prior_wrist_zero():
    return {
        "method": "multiangle_distal_stock_gripper_sign_and_zero",
        "fixed_point_delta_degrees": 0.0,
        "moving_jaw_orientation_check": True,
        "model_convention_orientation_check": True,
        "coupling_converged": True,
        "reference_camera_validation_required": True,
        "reference_camera_serial": staged.REFERENCE_CAMERA_SERIAL,
        "validating_camera_count": 1,
        "multiangle_pose_count": 10,
        "multiangle_viewpoint_count": 2,
        "camera_results": [
            {
                "camera": (
                    f"RealSense D455 {staged.REFERENCE_CAMERA_SERIAL}"
                ),
                "evidence_class": "strong_full_surface",
                "mesh_loss_m": 0.012,
                "mapped_surface_support_fraction": 0.70,
                "visible_stock_gripper": True,
                "moving_jaw_visible_pose_count": 3,
                "mesh_peak_ratio": 1.15,
                "local_orientation_basin_width_degrees": 14.0,
            }
        ],
    }


def test_validated_prior_wrist_zero_accepts_strong_d455_intrinsic_zero():
    assert staged.validated_prior_wrist_zero_evidence(
        valid_d455_prior_wrist_zero()
    )


def test_validated_prior_wrist_zero_survives_guarded_preservation_chain():
    prior = {
        "method": (
            "preserved_validated_prior_wrist_zero_after_d455_motion_sign_check"
        ),
        "fixed_point_delta_degrees": 0.0,
        "moving_jaw_orientation_check": True,
        "orientation_preserved_not_reestimated": True,
        "prior_validation": valid_d455_prior_wrist_zero(),
    }
    assert staged.validated_prior_wrist_zero_evidence(prior)


def test_validated_prior_wrist_zero_rejects_weak_d455_orientation():
    prior = valid_d455_prior_wrist_zero()
    prior["camera_results"][0]["mesh_peak_ratio"] = 1.01
    assert not staged.validated_prior_wrist_zero_evidence(prior)


def test_wrist_roll_selects_agreeing_pair_instead_of_incompatible_rank_ones():
    strong = "strong_full_surface"
    accepted, deltas, disagreement = staged.select_wrist_roll_cross_camera_pair(
        {
            "D455": [
                {
                    "offset_delta_degrees": 0.0,
                    "mesh_loss_m": 0.005,
                    "evidence_class": strong,
                },
                {
                    "offset_delta_degrees": 20.0,
                    "mesh_loss_m": 0.006,
                    "evidence_class": strong,
                },
            ],
            "D435": [
                {
                    "offset_delta_degrees": 40.0,
                    "mesh_loss_m": 0.004,
                    "evidence_class": strong,
                },
                {
                    "offset_delta_degrees": 5.0,
                    "mesh_loss_m": 0.006,
                    "evidence_class": strong,
                },
            ],
        }
    )

    assert [result["offset_delta_degrees"] for result in accepted] == [5.0, 0.0]
    assert deltas == pytest.approx([5.0, 0.0])
    assert disagreement == pytest.approx(5.0)


def test_wrist_roll_selects_two_agreeing_views_and_rejects_third_outlier():
    def viewpoint(group, offset, evidence, loss):
        camera = {
            "fitted_offset_degrees": offset,
            "evidence_class": evidence,
        }
        return {
            "group_index": group,
            "orientation_camera": camera,
            "camera_results": [camera],
            "combined_mesh_loss_m": loss,
            "visible_in_both_cameras": evidence != "invisible_or_ambiguous",
        }

    pair, offsets, disagreement = (
        staged.select_wrist_roll_agreeing_viewpoint_pair(
            [
                viewpoint(0, 147.0, "strong_full_surface", 0.008),
                viewpoint(1, 112.5, "unique_partial_surface", 0.009),
                viewpoint(2, 120.0, "strong_full_surface", 0.007),
            ]
        )
    )

    assert [item["group_index"] for item in pair] == [1, 2]
    assert offsets == pytest.approx([112.5, 120.0])
    assert disagreement == pytest.approx(7.5)


def test_wrist_moving_jaw_gate_keeps_live_edge_return_but_rejects_hidden_basin():
    assert 0.022124 < staged.MAXIMUM_WRIST_ROLL_MOVING_JAW_LOSS_M
    assert staged.MAXIMUM_WRIST_ROLL_MOVING_JAW_LOSS_M < 0.024


def test_two_ambiguous_wrist_views_cannot_validate_one_another():
    def ambiguous(group, offset):
        camera = {
            "fitted_offset_degrees": offset,
            "evidence_class": "invisible_or_ambiguous",
        }
        return {
            "group_index": group,
            "orientation_camera": camera,
            "camera_results": [camera],
            "combined_mesh_loss_m": 0.006,
            "visible_in_both_cameras": False,
        }

    pair, offsets, disagreement = (
        staged.select_wrist_roll_agreeing_viewpoint_pair(
            [ambiguous(0, 10.0), ambiguous(1, 11.0)]
        )
    )
    assert pair == []
    assert offsets == []
    assert disagreement == float("inf")


def test_outward_chain_repeats_when_wrist_direction_changes(monkeypatch):
    outward_calls = []
    wrist_calls = []

    def preliminary(*_args):
        return {
            "method": "cross_camera_repeated_motion_collapse_direction",
            "fitted_direction": 1.0,
            "camera_offset_disagreement_degrees": 4.0,
            "candidate_wrist_flex_delta_degrees": -9.0,
        }

    def outward(
        _frames,
        _through_joint,
        directions,
        offsets,
        _basis,
        _origin,
    ):
        outward_calls.append((directions[4], offsets[4]))
        fitted = list(offsets)
        fitted[3] = -9.0 if directions[4] > 0 else -15.0
        return fitted, [{"joint_index": 3}], {"converged": True}

    def wrist(
        _frames,
        directions,
        offsets,
        _basis,
        _origin,
        locked_direction=None,
    ):
        assert locked_direction == directions[4]
        wrist_calls.append((directions[4], offsets[3], offsets[4]))
        fitted_directions = list(directions)
        fitted_offsets = list(offsets)
        fitted_directions[4] = -1.0
        fitted_offsets[4] = -70.0
        return fitted_directions, fitted_offsets, {
            "fitted_direction": -1.0,
        }

    monkeypatch.setattr(
        staged,
        "solve_wrist_roll_direction_from_motion",
        preliminary,
    )
    monkeypatch.setattr(
        staged,
        "solve_outward_chain_to_fixed_point",
        outward,
    )
    monkeypatch.setattr(
        staged,
        "solve_wrist_roll_zero_to_fixed_point",
        wrist,
    )
    directions, offsets, _results, _convergence, roll = (
        staged.solve_outward_chain_with_coupled_wrist_sign(
            [],
            3,
            [1.0] * 6,
            [0.0, 0.0, 0.0, 0.0, -77.0, 0.0],
            np.eye(3),
            np.zeros(3),
        )
    )

    assert outward_calls == [(1.0, -77.0), (-1.0, -70.0)]
    assert wrist_calls == [
        (1.0, -9.0, -77.0),
        (-1.0, -15.0, -70.0),
    ]
    assert directions[4] == -1.0
    assert offsets[3] == -15.0
    assert offsets[4] == -70.0
    assert roll["coupling_converged"] is True
    assert len(roll["coupling_history"]) == 2


def test_wrist_roll_fine_screening_reserves_dedicated_motion_views():
    candidates = [
        {
            "screening_mesh_loss_m": 0.001 + index * 0.0001,
            "frame": {"calibration_joint_index": 1},
            "id": f"other-{index}",
        }
        for index in range(8)
    ]
    candidates.extend(
        {
            "screening_mesh_loss_m": 0.020 + index * 0.001,
            "frame": {"calibration_joint_index": 4},
            "id": f"roll-{index}",
        }
        for index in range(5)
    )

    selected = staged.select_wrist_roll_fine_views(candidates)

    assert sum(
        item["frame"]["calibration_joint_index"] == 4 for item in selected
    ) == staged.MAXIMUM_WRIST_ROLL_FINE_VIEWS_PER_CAMERA


def test_wrist_roll_motion_sign_selects_lower_two_camera_collapse(monkeypatch):
    def solve(
        _frames,
        directions,
        _offsets,
        _basis,
        _origin,
    ):
        direction = directions[4]
        per_camera_loss = 0.005 if direction == -1.0 else 0.012
        return {
            "offset_delta_degrees": -9.0,
            "camera_offset_disagreement_degrees": 3.0,
            "camera_results": [
                {
                    "camera": "D455",
                    "collapse_loss_m": per_camera_loss,
                },
                {
                    "camera": "D435",
                    "collapse_loss_m": per_camera_loss,
                },
            ],
        }

    monkeypatch.setattr(
        staged,
        "solve_wrist_flex_from_direct_axis_collapse",
        solve,
    )
    result = staged.solve_wrist_roll_direction_from_motion(
        [],
        [1.0] * 6,
        [0.0] * 6,
        np.eye(3),
        np.zeros(3),
    )

    assert result["fitted_direction"] == -1.0
    assert result["validating_camera_count"] == 1
    assert result["reference_camera_validation_required"]
    assert [
        item["fitted_direction"] for item in result["candidate_results"]
    ] == [-1.0, 1.0]


def test_wrist_roll_motion_sign_rejects_ambiguous_collapse(monkeypatch):
    def solve(
        _frames,
        directions,
        _offsets,
        _basis,
        _origin,
    ):
        per_camera_loss = 0.005 if directions[4] == -1.0 else 0.0052
        return {
            "offset_delta_degrees": -9.0,
            "camera_offset_disagreement_degrees": 3.0,
            "camera_results": [
                {
                    "camera": "D455",
                    "collapse_loss_m": per_camera_loss,
                },
                {
                    "camera": "D435",
                    "collapse_loss_m": per_camera_loss,
                },
            ],
        }

    monkeypatch.setattr(
        staged,
        "solve_wrist_flex_from_direct_axis_collapse",
        solve,
    )
    with pytest.raises(
        ValueError,
        match="direction remains ambiguous",
    ):
        staged.solve_wrist_roll_direction_from_motion(
            [],
            [1.0] * 6,
            [0.0] * 6,
            np.eye(3),
            np.zeros(3),
        )
