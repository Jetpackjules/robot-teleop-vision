"""Exercise capture loss and solver/renderer contracts before physical retry."""

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_modules/so101/tools"))
import solve_so101_claw_rgb_tips as claw


def capture(tmp_path, openings=(5, 25, 50, 75, 90)):
    frames = []
    for roll in (-20, 0, 20):
        for opening in openings:
            path = tmp_path / f"roll{roll}_open{opening}.png"
            ok, encoded = cv2.imencode(".png", np.zeros((16, 16, 3), np.uint8))
            assert ok
            path.write_bytes(encoded.tobytes())
            frames.append(
                {
                    "calibration_joint_index": 5,
                    "pose": [0, 0, 0, 0, roll, opening],
                    "claw_tip_positions_local": [[1, 1, 1], [2, 2, 1]],
                    "joint_frames": [{"pivot": [0, 0, 1], "axis": [0, 0, 1]}] * 6,
                    "claw_metadata": {
                        "validated_angle_samples_normalized": [5, 25, 50, 75, 90],
                        "validated_angle_samples_degrees": [-10, 15, 40, 65, 80],
                    },
                    "rgb_snapshots": [{"name": "D435 B", "path": str(path)}],
                }
            )
    return {"type": "so101_motion_capture", "reference_camera": "D435 B", "frames": frames}


@pytest.mark.parametrize("openings", [(5, 25, 50, 65, 70, 90), (5, 10, 15, 20, 25, 30, 90)])
def test_selected_five_states_keep_the_measured_opening_span(tmp_path, openings):
    groups = claw.group_frames(capture(tmp_path, openings))
    for group in groups:
        assert len(group) == 5
        assert group[-1]["pose"][5] - group[0]["pose"][5] >= 70


def stub_tip_detection(monkeypatch):
    monkeypatch.setattr(
        claw,
        "terminal_tip_positions_local",
        lambda frame: np.array(frame["claw_tip_positions_local"], dtype=float),
    )
    monkeypatch.setattr(claw, "project", lambda point, *_: np.array(point[:2], dtype=float))
    monkeypatch.setattr(
        claw,
        "terminal_edge_tip",
        lambda _image, _hinge, prediction, _changed: (prediction, 1.0, None),
    )
    monkeypatch.setattr(
        claw, "temporal_dark_moving_tip", lambda *_: (np.array([2.0, 2.0]), 1.0, None)
    )


def test_corrupt_rgb_view_does_not_abort_two_decodable_views(tmp_path, monkeypatch):
    stub_tip_detection(monkeypatch)
    data = capture(tmp_path)
    Path(data["frames"][0]["rgb_snapshots"][0]["path"]).write_bytes(b"broken PNG")
    rejected = []
    observations = claw.extract_observations(
        claw.group_frames(data), tmp_path / "debug", rejected_views=rejected
    )
    assert len(observations) == 20
    assert {item.view_key for item in observations} == {1, 2}
    assert len(rejected) == 1 and "decode" in rejected[0]["reason"]
    Path(data["frames"][5]["rgb_snapshots"][0]["path"]).write_bytes(b"broken PNG")
    with pytest.raises(ValueError, match="two .*wrist views"):
        claw.extract_observations(claw.group_frames(data), tmp_path / "debug")


def test_rgb_capture_loads_from_unicode_windows_path(tmp_path, monkeypatch):
    stub_tip_detection(monkeypatch)
    directory = tmp_path / "\u673a\u5668\u4eba \u56fe\u50cf"
    directory.mkdir()
    observations = claw.extract_observations(
        claw.group_frames(capture(directory)), directory / "debug"
    )
    assert len(observations) == 30
    assert len(list((directory / "debug").glob("*.png"))) == 15


def test_result_reports_only_views_and_states_that_validated(tmp_path, monkeypatch):
    data = capture(tmp_path)

    def extract(_groups, _debug, direction, rejected_views):
        rejected_views.append({"view": 1, "reason": "could not decode RGB image"})
        return [direction]

    monkeypatch.setattr(claw, "extract_observations", extract)

    def fit(_observations, direction):
        return np.eye(4), {
            "all_views_improved": direction == 1,
            "baseline_median_px": 5.0,
            "candidate_median_px": 1.0,
            "maximum_candidate_px": 2.0,
            "angle_deltas_degrees": [0.0] * 5,
            "observation_count": 20,
            "view_metrics": [{"view": 2}, {"view": 3}],
            "observation_metrics": [
                {"view": view, "opening": opening}
                for view in [2, 3]
                for opening in [5, 25, 50, 75, 90]
            ],
            "gripper_hinge_direction": direction,
        }

    monkeypatch.setattr(claw, "fit_assembly", fit)
    result = claw.solve(data, tmp_path / "debug")
    assert result["validation_view_count"] == 2
    assert result["validation_pose_count"] == 10
    assert len(result["rejected_capture_views"]) == 1


def test_solver_does_not_publish_a_hinge_direction_the_overlay_cannot_apply(tmp_path, monkeypatch):
    data = capture(tmp_path)
    monkeypatch.setattr(
        claw, "extract_observations", lambda _groups, _debug, direction, **_: [direction]
    )

    def fit(observations, direction):
        return np.eye(4), {
            "all_views_improved": direction == -1,
            "baseline_median_px": 5.0,
            "candidate_median_px": 1.0,
            "maximum_candidate_px": 2.0,
            "angle_deltas_degrees": [0.0] * 5,
            "observation_count": 20,
            "view_metrics": [{"view": 1}, {"view": 2}],
            "gripper_hinge_direction": direction,
        }

    monkeypatch.setattr(claw, "fit_assembly", fit)
    with pytest.raises(ValueError, match="reversed.*hinge.*overlay"):
        claw.solve(data, tmp_path / "debug")
