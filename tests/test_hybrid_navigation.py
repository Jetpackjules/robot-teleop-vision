"""Check browser navigation and webcam startup without hardware."""
from pathlib import Path
import shutil
import subprocess

import pytest


def _run_node_probe(name):
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js is needed for the frontend geometry checks")
    script = Path(__file__).with_name(name)
    result = subprocess.run(
        [node, str(script)], capture_output=True, text=True, timeout=15, check=False
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_mouse_orbit_keeps_its_pivot_and_drag_direction():
    _run_node_probe("hybrid_navigation.cjs")


def test_webcam_startup_closes_abandoned_streams():
    _run_node_probe("webcam_startup.cjs")
