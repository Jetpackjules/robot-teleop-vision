"""Exercise original-RGB request ordering and mode-exit cancellation."""
from pathlib import Path
import shutil
import subprocess

import pytest


def test_original_rgb_request_lifecycle():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js is needed for the RGB viewer checks")
    result = subprocess.run([node, str(Path(__file__).with_name("camera_rgb_viewer.cjs"))],
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
