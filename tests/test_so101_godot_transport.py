"""Execute production GDScript in an isolated project, with no camera or robot."""

import json
import os
import shutil
import subprocess
import sys
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
    # Check the actual solver and Godot transforms agree, including nonzero
    # wrist offsets and both encoder directions. No additional mesh-to-joint
    # quarter turn is needed when a fitted offset is applied to the overlay.
    import numpy as np

    sys.path.insert(0, str(ROOT / "robot_modules/so101/tools"))
    import solve_so101_staged_joints as staged

    raw_points = [[0.021, -0.014, -0.07], [-0.015, 0.026, -0.09], [0.003, 0.009, -0.04]]
    wrist_cases = []
    for direction in [-1.0, 1.0]:
        for offset in [0.0, -45.0]:
            for roll in [-100.0, 0.0, 45.0]:
                pose = [-40.0, 130.0, 38.0, 78.0, roll, 25.0]
                directions = [1.0, -1.0, 1.0, 1.0, direction, 1.0]
                offsets = [40.4296875, 80.0, 0.0, -70.0, offset, 0.0]
                wrist_cases.append({
                    "pose": pose, "directions": directions, "offsets": offsets,
                    "raw_points": raw_points,
                    "expected_points": staged.posed_mesh(
                        np.array(raw_points), {"pose": pose}, 4, 0, directions,
                        offsets, np.eye(3), np.zeros(3),
                    ).tolist(),
                })
    (tmp_path / "wrist_pose_cases.json").write_text(json.dumps(wrist_cases), encoding="utf-8")
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
    environment["ROBOT_TELEOP_SOLVER_PYTHON"] = sys.executable
    # A solver fixture with a spaced path exercises Windows process quoting.
    (tmp_path / "solver fixture.py").write_text(
        "import json, pathlib, sys\npathlib.Path(sys.argv[1]).write_text(json.dumps({'value': 42}))\n",
        encoding="utf-8",
    )
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
