"""Run the production camera-rig transaction without opening any device."""
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from robot_teleop.doctor import find_godot, godot_console_executable

ROOT = Path(__file__).resolve().parents[1]


def test_camera_rig_transactions_and_coordinate_frames(tmp_path):
    godot = find_godot(os.environ.get("ROBOT_TELEOP_TEST_GODOT", ""))
    if godot is None:
        pytest.skip("Set ROBOT_TELEOP_TEST_GODOT for executable camera-rig tests")
    scripts = tmp_path / "godot/runtime/point_cloud"
    shutil.copytree(ROOT / "godot/runtime/point_cloud", scripts)
    shutil.copy2(ROOT / "tests/godot/realsense_rig_probe.gd", tmp_path / "probe.gd")
    (tmp_path / "project.godot").write_text(
        'config_version=5\n[application]\nconfig/name="Camera Rig Test"\n', encoding="utf-8"
    )
    result = subprocess.run(
        [str(godot_console_executable(godot)), "--headless", "--path", str(tmp_path),
         "--script", "res://probe.gd"],
        env={**os.environ, "APPDATA": str(tmp_path / "appdata"), "XDG_DATA_HOME": str(tmp_path / "data")},
        capture_output=True, text=True, timeout=30, check=False,
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0 and "SCRIPT ERROR" not in output, output
    assert "REALSENSE_RIG_PROBE checks=" in output and "failures=0" in output, output


@pytest.mark.skipif(os.environ.get("ROBOT_TELEOP_TEST_GPU") != "1", reason="Opt in to off-screen native GPU tests")
@pytest.mark.parametrize("renderer", ["gl_compatibility", "forward_plus"])
def test_native_multi_camera_mesh_rendering(tmp_path, renderer):
    godot = find_godot(os.environ.get("ROBOT_TELEOP_TEST_GODOT", ""))
    assert godot is not None
    scripts = tmp_path / "godot/runtime/point_cloud"
    scripts.mkdir(parents=True)
    shutil.copy2(ROOT / "godot/runtime/point_cloud/realsense_mesh_fusion.gd", scripts)
    shutil.copy2(ROOT / "tests/godot/realsense_fusion_probe.gd", tmp_path / "probe.gd")
    (tmp_path / "project.godot").write_text(
        'config_version=5\n[application]\nconfig/name="Camera Fusion Test"\n'
        f'[rendering]\nrenderer/rendering_method="{renderer}"\n', encoding="utf-8"
    )
    native = ROOT / "native/realsense_shared_memory"
    # The user's Downloads directory can contain another OpenCV ABI. Keep the
    # test executable away from it; do not move/delete any user-installed DLL.
    executable = tmp_path / "Godot.exe"
    graphical = Path(str(godot).replace("_console.exe", ".exe"))
    shutil.copy2(graphical, executable)
    descriptor = (native / "realsense_shared_memory.gdextension").read_text(encoding="utf-8")
    descriptor = descriptor.replace("res://native/realsense_shared_memory/", native.as_posix() + "/")
    (tmp_path / "native.gdextension").write_text(descriptor, encoding="utf-8")
    (tmp_path / ".godot").mkdir()
    (tmp_path / ".godot/extension_list.cfg").write_text("res://native.gdextension\n", encoding="utf-8")
    result = subprocess.run(
        [str(executable), "--path", str(tmp_path),
         "--rendering-method", renderer, "--position", "-32000,-32000",
         "--resolution", "128x128", "--script", "res://probe.gd"],
        env={**os.environ, "APPDATA": str(tmp_path / "appdata"), "XDG_DATA_HOME": str(tmp_path / "data"),
             "PATH": str(native / "bin") + os.pathsep + os.environ.get("PATH", "")},
        capture_output=True, text=True, timeout=90, check=False,
    )
    output = result.stdout + result.stderr
    (tmp_path / "render.log").write_text(output, encoding="utf-8")
    assert result.returncode == 0 and "SCRIPT ERROR" not in output and "SHADER ERROR" not in output, output
    assert "REALSENSE_FUSION_PROBE checks=" in output and "failures=0" in output, output


def test_benchmark_reads_world_rig_and_legacy_pair_files(monkeypatch):
    import importlib.util
    import json
    import sys
    import types

    import numpy as np

    if importlib.util.find_spec("pyrealsense2") is None:
        monkeypatch.setitem(sys.modules, "pyrealsense2", types.ModuleType("pyrealsense2"))
    from tools.realsense_cloud_align import benchmark_pair_transform

    a = {"R": [[0, -1, 0], [1, 0, 0], [0, 0, 1]], "T": [1, 2, 3], "coordinate_space": "world"}
    b = {**a, "T": [1, 2.5, 3]}
    rig = {"transforms": {"realsense:A": a, "realsense:B": b}}
    relative = benchmark_pair_transform(json.dumps(rig), "realsense:A", "realsense:B")
    np.testing.assert_allclose(relative[:3, 3], [0.5, 0, 0])
    np.testing.assert_allclose(relative[:3, :3], np.eye(3))
    pair = {"R": np.eye(3).tolist(), "T": [0.5, 0, 0], "reference_camera": "realsense:A", "target_camera": "realsense:B"}
    np.testing.assert_allclose(benchmark_pair_transform(json.dumps(pair), "realsense:A", "realsense:B"), relative)
    with pytest.raises(ValueError, match="different camera pair"):
        benchmark_pair_transform(json.dumps(pair), "realsense:A", "realsense:C")
    with pytest.raises(ValueError, match="no transform"):
        benchmark_pair_transform(json.dumps(rig), "realsense:A", "realsense:C")
