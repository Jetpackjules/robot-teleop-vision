"""Portable visual calibration, with no hardware or original user-data access."""

import os
import shutil
import subprocess
from pathlib import Path

import pytest

from robot_teleop.doctor import find_godot, godot_console_executable
from robot_teleop.operator import load_robot_operator

ROOT = Path(__file__).resolve().parents[1]


def test_manual_save_reload_import_and_native_panel(tmp_path):
    godot = find_godot(os.environ.get("ROBOT_TELEOP_TEST_GODOT", ""))
    if godot is None:
        pytest.skip("Set ROBOT_TELEOP_TEST_GODOT for executable GDScript tests")
    scripts = tmp_path / "robot_modules/so101/godot"
    scripts.mkdir(parents=True)
    for source in (ROOT / "robot_modules/so101/godot").glob("*.gd"):
        shutil.copy2(source, scripts / source.name)
    shutil.copy2(ROOT / "tests/godot/so101_manual_calibration_probe.gd", tmp_path / "probe.gd")
    (tmp_path / "project.godot").write_text(
        'config_version=5\n[application]\nconfig/name="Visual Calibration Test"\n',
        encoding="utf-8",
    )
    result = subprocess.run(
        [str(godot_console_executable(godot)), "--headless", "--path", str(tmp_path),
         "--script", "res://probe.gd", "--quit-after", "60"],
        env={**os.environ, "APPDATA": str(tmp_path / "appdata"),
             "XDG_DATA_HOME": str(tmp_path / "data")},
        capture_output=True, text=True, timeout=30, check=False,
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "SCRIPT ERROR" not in output, output
    assert "SO101_MANUAL_PROBE checks=" in output and "failures=0" in output, output


def test_structured_visual_actions_are_bounded_and_module_owned():
    operator = load_robot_operator("so101")
    key = "robot_visual_calibration_action"
    action = {"operation": "import", "request_id": "upload", "file": {"type": "test"},
              "restore_base": False, "executable": "ignored"}
    result = operator.structured_view_settings({key: action})[key]
    assert result == {k: action[k] for k in ("operation", "request_id", "file", "restore_base")}
    for invalid in [None, {}, {"operation": "write_motor", "request_id": "a"},
                    {**action, "request_id": "a" * 81}, {**action, "restore_base": "yes"},
                    {**action, "file": {"value": float("nan")}},
                    {**action, "file": {"huge": "x" * 16385}}]:
        with pytest.raises(ValueError):
            operator.structured_view_settings({key: invalid})
    assert load_robot_operator("disabled").structured_view_settings({key: action}) == {}
