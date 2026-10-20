"""Exercise the production stall guard without opening any camera."""
from pathlib import Path
import os
import subprocess

import pytest

from robot_teleop.doctor import find_godot, godot_console_executable


def test_stalled_capture_recovery_preserves_calibration_pause(tmp_path):
    godot = find_godot(os.environ.get("ROBOT_TELEOP_TEST_GODOT", ""))
    if godot is None:
        pytest.skip("Godot is required for the capture guard probe")
    root = Path(__file__).resolve().parents[1]
    source = (root / "godot/runtime/point_cloud/unified_point_cloud_view.gd").read_text()
    function = source[source.index("func _capture_active_after_recovery("):
                      source.index("func _apply_camera_renderer_settings(")].expandtabs(1)
    script = '''extends SceneTree
var _capture_paused := false
var _capture_recovery_at: Dictionary = {}
class Camera extends MeshInstance3D:
 var age := 0.0
 var closed := 0
 func get_display_frame_age_ms() -> float: return age
 func set_direct_realsense_enabled(enabled: bool) -> void:
  if not enabled: closed+=1
''' + function + '''
func _initialize() -> void:
 var camera := Camera.new()
 assert(_capture_active_after_recovery("C",camera,true))
 camera.age=6000.0
 _capture_paused=true
 assert(_capture_active_after_recovery("C",camera,true))
 assert(camera.closed==0)
 _capture_paused=false
 assert(not _capture_active_after_recovery("C",camera,true))
 assert(camera.closed==1)
 assert(_capture_active_after_recovery("C",camera,true))
 assert(camera.closed==1)
 assert(not _capture_active_after_recovery("C",camera,false))
 camera.free()
 print("CAPTURE_RECOVERY failures=0")
 quit()
'''
    (tmp_path / "project.godot").write_text('config_version=5\n[application]\nconfig/name="Capture guard test"\n')
    (tmp_path / "probe.gd").write_text(script)
    result = subprocess.run([str(godot_console_executable(godot)), "--headless", "--path", str(tmp_path),
                             "--script", "res://probe.gd"], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "ERROR:" not in result.stdout + result.stderr
    assert "CAPTURE_RECOVERY failures=0" in result.stdout
