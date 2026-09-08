"""Run the isolated Godot task against real engine physics, without devices."""

import json
import os
import subprocess
from pathlib import Path

import pytest

from robot_teleop.doctor import find_godot, godot_console_executable

ROOT = Path(__file__).resolve().parents[1]


def test_virtual_alignment_grasp_release_contact_and_input(tmp_path):
    godot = find_godot(os.environ.get("ROBOT_TELEOP_TEST_GODOT", ""))
    if godot is None:
        pytest.skip("Set ROBOT_TELEOP_TEST_GODOT for executable simulation physics checks")
    environment = {
        **os.environ,
        "APPDATA": str(tmp_path / "appdata"),
        "XDG_DATA_HOME": str(tmp_path / "data"),
    }
    environment.pop("ROBOT_TELEOP_RUNTIME_CONFIG", None)
    result = subprocess.run(
        [str(godot_console_executable(godot)), "--headless", "--path", str(ROOT),
         "--log-file", str(tmp_path / "physics-probe.log"), "--fixed-fps", "60",
         "--script", "res://tests/godot/alignment_demo_probe.gd"],
        cwd=ROOT, env=environment, capture_output=True, text=True, timeout=30,
        check=False,
    )
    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "SCRIPT ERROR" not in output, output
    assert "Unicode parsing error" not in output, output
    assert "Cannot load bundled virtual arm mesh" not in output, output
    marker = "ALIGNMENT_DEMO_PROBE "
    evidence = next(line[len(marker):] for line in result.stdout.splitlines() if line.startswith(marker))
    state = json.loads(evidence)
    assert state["passed"], state
    assert state["final"]["completed"] is True
    assert state["final"]["held"] is False
    assert state["final"]["input_bound"] is False
