"""Run exact native-depth, coalescing and worker-lifecycle checks in Node."""
from pathlib import Path
import shutil
import subprocess

import pytest


def test_native_decode_worker_and_camera_batching():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node is required for the native decoder checks")
    result = subprocess.run([node, str(Path(__file__).with_name("rgbd_decode_worker.cjs"))],
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
