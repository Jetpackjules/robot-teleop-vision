"""Exercise placement commits and the actual GDScript parser without hardware."""
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from robot_teleop.doctor import find_godot, godot_console_executable

ROOT = Path(__file__).resolve().parents[1]


def test_placement_commit_preserves_model_and_rejects_superseded_results(tmp_path):
    godot = find_godot(os.environ.get("ROBOT_TELEOP_TEST_GODOT", ""))
    if godot is None:
        pytest.skip("Set ROBOT_TELEOP_TEST_GODOT for executable GDScript tests")
    scripts = tmp_path / "robot_modules/so101/godot"
    scripts.mkdir(parents=True)
    for path in (ROOT / "robot_modules/so101/godot").glob("*.gd"):
        shutil.copy2(path, scripts / path.name)
    shutil.copy2(ROOT / "tests/godot/so101_registration_probe.gd", tmp_path / "probe.gd")
    (tmp_path / "project.godot").write_text(
        '[application]\nconfig/name="SO101 Placement Commit Test"\n', encoding="utf-8")
    result = subprocess.run(
        [str(godot_console_executable(godot)), "--headless", "--path", str(tmp_path),
         "--script", "res://probe.gd", "--quit-after", "60"],
        env={**os.environ, "APPDATA": str(tmp_path / "appdata"),
             "XDG_DATA_HOME": str(tmp_path / "data")},
        capture_output=True, text=True, timeout=20, check=False,
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "SCRIPT ERROR" not in output, output
    assert "SO101_RIGID_REGISTRATION_PASS" in output, output
