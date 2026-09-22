"""Camera choice must follow usable motion evidence, including on dual-D435 sites."""
import copy
import sys
from pathlib import Path

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parents[1] / "robot_modules/so101/tools"
sys.path.insert(0, str(TOOLS))
import solve_so101_base_axis as base
import solve_so101_claw_rgb_tips as claw


def capture():
    return {"reference_camera": "D435 A", "frames": [
        {"full_camera_points": [{"name": "D435 A"}, {"name": "D435 B"}]}
        for _ in range(5)
    ]}


def fits(monkeypatch, values):
    calls = []
    monkeypatch.setattr(base, "plane_consensus", lambda _frames, indices:
                        (np.array([0., 1., 0.]), float(indices[0]), []))
    monkeypatch.setattr(base, "dynamic_clouds", lambda _frames, _normal, _offset, index:
                        [np.full((30, 3), index)] * 5)

    def fit(_frames, clouds, _normal, _offset, _seed):
        index = int(clouds[0][0, 0])
        calls.append(index)
        sign, loss = values[index]
        return {"sign": sign, "loss_m": loss}

    monkeypatch.setattr(base, "fit_axis", fit)
    return calls


@pytest.mark.parametrize("bad_fit", [(-1, .0242), (1, .003), (-1, float("nan"))])
def test_rejected_preferred_camera_does_not_veto_valid_alternative(monkeypatch, bad_fit):
    calls = fits(monkeypatch, [bad_fit, (-1, .0034)])
    selected = base.select_axis_evidence(capture())
    assert calls == [0, 1]
    assert selected["reference_camera"] == "D435 B"
    assert selected["axis_fit"]["loss_m"] == .0034
    assert not selected["camera_selection"]["attempts"][0]["accepted"]


def test_valid_preferred_camera_is_retained(monkeypatch):
    calls = fits(monkeypatch, [(-1, .004), (-1, .002)])
    assert base.select_axis_evidence(capture())["reference_camera"] == "D435 A"
    assert calls == [0]


def test_all_bad_cameras_still_reject_without_relaxing_gate(monkeypatch):
    fits(monkeypatch, [(-1, .0242), (1, .002)])
    with pytest.raises(ValueError, match="rejected in every captured camera"):
        base.select_axis_evidence(capture())


def test_inconsistent_camera_identity_is_not_used(monkeypatch):
    calls = fits(monkeypatch, [(-1, .001), (-1, .003)])
    data = capture()
    data["frames"][2]["full_camera_points"][0]["name"] = "wrong camera"
    assert base.select_axis_evidence(data)["reference_camera"] == "D435 B"
    assert calls == [1]


def test_rgb_groups_use_selected_d435_not_another_available_camera(tmp_path):
    images = []
    for name in ["D435 A", "D435 B", "D455"]:
        path = tmp_path / (name + ".png")
        path.write_bytes(b"snapshot selection only")
        images.append({"name": name, "path": str(path)})
    frames = [{"calibration_joint_index": 5, "pose": [0, 0, 0, 0, roll, opening],
               "claw_tip_positions_local": [[0, 0, 0], [1, 0, 0]], "rgb_snapshots": images}
              for roll in [-20, 0] for opening in [5, 25, 50, 75, 90]]
    groups = claw.group_frames({"frames": frames, "reference_camera": "D435 B"})
    assert len(groups) == 2
    assert all(frame["_rgb"]["name"] == "D435 B" for group in groups for frame in group)
    assert claw.d455_snapshot(frames[0])["name"] == "D455"
    with pytest.raises(ValueError, match="no saved RGB image"):
        claw.group_frames({"frames": frames, "reference_camera": "missing"})


def test_missing_rgb_in_one_sweep_does_not_veto_two_complete_sweeps(tmp_path):
    path = tmp_path / "reference.png"
    path.write_bytes(b"snapshot selection only")
    frames = [{"calibration_joint_index": 5, "pose": [0, 0, 0, 0, roll, opening],
               "claw_tip_positions_local": [[0, 0, 0], [1, 0, 0]],
               "rgb_snapshots": [{"name": "D435 B", "path": str(path)}]}
              for roll in [-20, 0, 20] for opening in [5, 25, 50, 75, 90]]
    frames[0]["rgb_snapshots"][0]["path"] = str(tmp_path / "missing.png")
    capture = {"frames": frames, "reference_camera": "D435 B"}
    groups = claw.group_frames(capture)
    assert len(groups) == 2
    assert [group[0]["pose"][4] for group in groups] == [0, 20]

    # Five frames at four opening states still cannot form the second view.
    duplicate = copy.deepcopy(frames[6])
    frames[5]["rgb_snapshots"] = []
    frames.append(duplicate)
    with pytest.raises(ValueError, match="fewer than two complete"):
        claw.group_frames(capture)
