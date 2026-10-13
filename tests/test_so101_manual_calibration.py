"""Portable visual calibration, with no hardware or original user-data access."""

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from robot_teleop.doctor import find_godot, godot_console_executable
from robot_teleop.operator import load_robot_operator

ROOT = Path(__file__).resolve().parents[1]


def test_manual_tuning_opens_from_editor_inspector(tmp_path):
    godot = find_godot(os.environ.get("ROBOT_TELEOP_TEST_GODOT", ""))
    if godot is None:
        pytest.skip("Set ROBOT_TELEOP_TEST_GODOT for executable GDScript tests")
    scripts = tmp_path / "robot_modules/so101/godot"
    scripts.mkdir(parents=True)
    for source in (ROOT / "robot_modules/so101/godot").glob("*.gd"):
        shutil.copy2(source, scripts / source.name)
    plugin = tmp_path / "addons/panel_probe"
    plugin.mkdir(parents=True)
    shutil.copy2(ROOT / "tests/godot/so101_manual_editor_probe.gd", plugin / "plugin.gd")
    (plugin / "plugin.cfg").write_text(
        '[plugin]\nname="Panel probe"\ndescription="Isolated editor test"\n'
        'author="Test"\nversion="1"\nscript="plugin.gd"\n', encoding="utf-8",
    )
    # Use the shipped Inspector button and callbacks, without starting the
    # point-cloud scene, native extension, stream server, or hardware drivers.
    view_source = (ROOT / "godot/runtime/point_cloud/unified_point_cloud_view.gd").read_text(
        encoding="utf-8"
    )
    button = next(line for line in view_source.splitlines() if "var manual_robot_calibration_action:" in line)
    callbacks = []
    for name in ("_active_robot_module", "_open_manual_robot_calibration"):
        start = view_source.index(f"\nfunc {name}(")
        end = view_source.find("\nfunc ", start + 1)
        callbacks.append(view_source[start:end if end != -1 else None])
    (tmp_path / "test_view.gd").write_text(
        '@tool\nextends Node3D\nvar robot_position_calibration_status := ""\n'
        + button + "\n" + "\n".join(callbacks), encoding="utf-8",
    )
    (tmp_path / "test_view.tscn").write_text(
        '[gd_scene load_steps=2 format=3]\n'
        '[ext_resource type="Script" path="res://test_view.gd" id="1"]\n'
        '[node name="View" type="Node3D"]\nscript = ExtResource("1")\n', encoding="utf-8",
    )
    (tmp_path / "project.godot").write_text(
        'config_version=5\n[application]\nconfig/name="Manual Tuning Editor Test"\n'
        '[editor_plugins]\nenabled=PackedStringArray("res://addons/panel_probe/plugin.cfg")\n'
        '[rendering]\nrenderer/rendering_method="gl_compatibility"\n', encoding="utf-8",
    )
    graphical = os.environ.get("ROBOT_TELEOP_TEST_EDITOR_DISPLAY") == "1"
    result = subprocess.run(
        [str(godot_console_executable(godot)), *([] if graphical else ["--headless"]),
         "--editor", "--path", str(tmp_path), "--quit-after", "1000"],
        env={**os.environ, "APPDATA": str(tmp_path / "appdata"),
             "XDG_DATA_HOME": str(tmp_path / "data"),
             "ROBOT_TELEOP_RUNTIME_CONFIG": json.dumps({
                 "schema_version": 1, "module": "so101", "enabled": False,
             })},
        capture_output=True, text=True, timeout=60, check=False,
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "SCRIPT ERROR" not in output, output
    assert "SO101_MANUAL_EDITOR_PROBE checks=" in output and "failures=0" in output, output


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
