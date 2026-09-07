"""Execute production GDScript in an isolated project, with no camera or robot."""

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from robot_teleop.doctor import find_godot, godot_console_executable

ROOT = Path(__file__).resolve().parents[1]


def test_module_configures_ports_before_nodes_enter_tree():
    source = (ROOT / "robot_modules/so101/godot/so101_module.gd").read_text(encoding="utf-8")
    assert source.index('_overlay.set("telemetry_port"') < source.index(
        "world_anchor.add_child(_overlay)"
    )
    assert source.index('_overlay.set("editor_telemetry_port"') < source.index(
        "world_anchor.add_child(_overlay)"
    )
    assert source.index('_calibrator.set("follower_command_port"') < source.index(
        "_view.add_child(_calibrator)"
    )
    assert source.index('_calibrator.set("calibration_status_port"') < source.index(
        "_view.add_child(_calibrator)"
    )


def test_godot_transport_and_calibration_transactions_without_hardware(tmp_path):
    godot = find_godot(os.environ.get("ROBOT_TELEOP_TEST_GODOT", ""))
    if godot is None:
        pytest.skip("Install Godot or set ROBOT_TELEOP_TEST_GODOT for executable GDScript tests")
    # Copy scripts only: no native extension, Main scene, physical profiles,
    # saved registration or application user-data is loaded by this project.
    scripts = tmp_path / "robot_modules/so101/godot"
    scripts.mkdir(parents=True)
    for source in (ROOT / "robot_modules/so101/godot").glob("*.gd"):
        shutil.copy2(source, scripts / source.name)
    shutil.copy2(ROOT / "tests/godot/so101_transport_probe.gd", tmp_path / "probe.gd")
    (tmp_path / "project.godot").write_text(
        'config_version=5\n[application]\nconfig/name="Teleop Transport Test"\n',
        encoding="utf-8",
    )
    (tmp_path / ".teleop").mkdir()
    (tmp_path / ".teleop/runtime_config.json").write_text(
        json.dumps(
            {
                "schema_version": 1,
                "module": "so101",
                "enabled": True,
                "calibration_status_port": 14251,
                "module_settings": {
                    "command_port": 14248,
                    "status_port": 14249,
                    "telemetry_port": 14250,
                    "editor_telemetry_port": 14252,
                },
            }
        ),
        encoding="utf-8",
    )
    # Keep Godot's test user:// and caches apart from the operator's real data.
    environment = {
        **os.environ,
        "APPDATA": str(tmp_path / "appdata"),
        "XDG_DATA_HOME": str(tmp_path / "data"),
    }
    environment.pop("ROBOT_TELEOP_RUNTIME_CONFIG", None)
    result = subprocess.run(
        [str(godot_console_executable(godot)), "--headless", "--path", str(tmp_path),
         "--script", "res://probe.gd"],
        env=environment,
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "SCRIPT ERROR" not in output, output
    assert "SO101_TRANSPORT_PROBE checks=" in output and "failures=0" in output, output
