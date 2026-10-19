"""Check mouse orbit geometry without a camera, GPU, or browser."""
from pathlib import Path
import shutil
import subprocess

import pytest


def test_mouse_orbit_keeps_its_pivot_and_drag_direction():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js is needed for the frontend geometry checks")
    script = Path(__file__).with_name("hybrid_navigation.cjs")
    result = subprocess.run(
        [node, str(script)], capture_output=True, text=True, timeout=15, check=False
    )
    assert result.returncode == 0, result.stdout + result.stderr
