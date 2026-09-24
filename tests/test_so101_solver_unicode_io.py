"""Exercise CLI JSON boundaries with a legacy Windows text encoding."""

import importlib
import json
import sys
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1] / "robot_modules/so101/tools"
sys.path.insert(0, str(TOOLS))


@pytest.mark.parametrize("name", ["base_axis", "claw_visual", "claw_rgb_tips", "staged_joints"])
@pytest.mark.parametrize("bom", [False, True])
def test_solver_cli_preserves_utf8_capture_paths(tmp_path, monkeypatch, name, bom):
    module = importlib.import_module(f"solve_so101_{name}")
    image = tmp_path / "机器人 相机.png"
    image.write_bytes(b"image bytes")
    capture = tmp_path / "capture.json"
    output = tmp_path / "result.json"
    payload = {
        "reference_camera": "相机 D435 B", "calibration_mode": "joints",
        "frames": [{"snapshot": str(image)}],
    }
    encoding = "utf-8-sig" if bom else "utf-8"
    capture.write_text(json.dumps(payload, ensure_ascii=False), encoding=encoding)
    args = [module.__file__, str(capture), "--output", str(output)]
    expected = {"label": "已校准", "median_residual_m": 0.001, "confidence": 0.9,
                "baseline_tip_residual_px": 4, "median_tip_residual_px": 1,
                "validation_view_count": 2}

    def check_capture(data):
        assert data["reference_camera"] == payload["reference_camera"]
        assert Path(data["frames"][0]["snapshot"]).read_bytes() == b"image bytes"

    def fake_solve(data, *_args):
        check_capture(data)
        return expected

    if name == "staged_joints":
        registration = tmp_path / "registration.json"
        registration.write_text(json.dumps({
            "basis_x": [1, 0, 0], "basis_y": [0, 1, 0], "basis_z": [0, 0, 1],
            "origin": [0, 0, 0], "joint_angle_directions": [1] * 6,
            "joint_angle_offsets_degrees": [0] * 6, "registration_source": "已校准",
        }, ensure_ascii=False), encoding=encoding)
        args.insert(2, str(registration))

        def fake_joint_solve(frames, _through, directions, offsets, *_args):
            check_capture({"reference_camera": module.REFERENCE_CAMERA_SERIAL, "frames": frames})
            return directions, offsets, [], {"converged": True}, {}

        monkeypatch.setattr(module, "solve_outward_chain_with_coupled_wrist_sign", fake_joint_solve)
        monkeypatch.setattr(module, "REFERENCE_CAMERA_SERIAL", "D455")
    else:
        monkeypatch.setattr(module, "solve", fake_solve)

    original_read = Path.read_text

    def legacy_windows_read(path, encoding=None, errors=None):
        return original_read(path, encoding=encoding or "cp1252", errors=errors)

    monkeypatch.setattr(Path, "read_text", legacy_windows_read)
    monkeypatch.setattr(sys, "argv", args)
    assert module.main() == 0
    result = json.loads(original_read(output, encoding="utf-8"))
    assert result.get("label", result.get("registration_source")) == "已校准"


def test_operator_profile_and_saved_rest_accept_windows_utf8_bom(tmp_path):
    from so101_arm_common import ArmPairProfile
    from so101_rest_pose import calibration_fingerprint, load_rest_pose

    data = json.loads((TOOLS.parents[2] / "tests/fixtures/so101_arm_pair.json").read_bytes())
    data["follower"]["serial"] = "实验室机械臂"
    profile_path = tmp_path / "arm_pair.json"
    profile_path.write_text(json.dumps(data, ensure_ascii=False), encoding="utf-8-sig")
    profile = ArmPairProfile.load(profile_path)
    raw = [2000] * 6
    saved = {
        "version": 2, "follower_serial": profile.follower_serial,
        "calibration_fingerprint": calibration_fingerprint(profile),
        "raw_positions": raw,
        "normalized_positions": profile.follower_calibration.raw_to_normalized(raw),
    }
    rest_path = tmp_path / "so101_rest_pose.json"
    rest_path.write_text(json.dumps(saved, ensure_ascii=False), encoding="utf-8-sig")
    assert load_rest_pose(profile, rest_path)["raw_positions"] == raw
