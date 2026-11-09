"""Check exposure freshness and camera-local recovery without camera hardware."""
import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_rgb_exposure_identity_does_not_follow_depth_publication(tmp_path):
    compiler = shutil.which("c++")
    if not compiler:
        pytest.skip("A C++ compiler is needed")
    binary = tmp_path / "rgb_freshness"
    subprocess.run([
        compiler, "-std=c++17", "-I", str(ROOT / "native/realsense_shared_memory/src"),
        str(ROOT / "tests/native/rgb_frame_freshness_test.cpp"), "-o", str(binary),
    ], check=True, capture_output=True, text=True)
    subprocess.run([str(binary)], check=True, timeout=10)


def test_rgb_stall_recovers_only_the_affected_camera(tmp_path):
    godot = os.environ.get("ROBOT_TELEOP_TEST_GODOT")
    if not godot:
        pytest.skip("Godot is needed")
    result = subprocess.run([
        godot, "--headless", "--path", str(ROOT), "--script",
        "res://tests/godot/rgb_recovery_probe.gd",
    ], env={**os.environ, "XDG_DATA_HOME": str(tmp_path), "ROBOT_TELEOP_MODULE": "disabled"},
        capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "RGB_RECOVERY passed" in result.stdout
