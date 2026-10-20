"""Check native mode choice and the real browser capture-control boundary."""
from pathlib import Path
import os
import shutil
import subprocess

import pytest

from robot_teleop.doctor import find_godot, godot_console_executable

ROOT = Path(__file__).resolve().parents[1]


def test_capture_preset_transport():
    node = shutil.which("node")
    if node is None:
        pytest.skip("Node.js is required")
    result = subprocess.run([node, str(Path(__file__).with_name("capture_presets.cjs"))],
                            capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr


def test_native_mode_ranking(tmp_path):
    compiler = shutil.which("g++") or shutil.which("clang++")
    if compiler is None:
        pytest.skip("A C++ compiler is required; also exercised with the lab native build")
    harness = tmp_path / "modes.cpp"
    harness.write_text(r'''
#include "realsense_capture_profiles.h"
#include <cassert>
using namespace realsense_capture;
int main() {
 std::vector<Mode> depth={{848,480,60},{1280,720,30},{848,480,30},{1280,720,30},{0,0,60},{1280,720,90}};
 std::vector<Mode> color={{960,540,60},{848,480,60},{1920,1080,30},{1280,720,30},{9999,9999,90},{9999,9999,15}};
 auto fast=ranked_modes(depth,color,60,true),quality=ranked_modes(depth,color,30,true);
 assert((fast.front().depth==Mode{848,480,60}));
 assert((fast.front().color==Mode{960,540,60}));
 assert((quality.front().depth==Mode{1280,720,30}));
 assert((quality.front().color==Mode{1920,1080,30}));
 assert(fast.size()==6 && quality.size()==4);
 assert(fast[1].depth.fps==60 && fast[2].depth.fps==30);
 assert((ranked_modes({{640,480,30}},color,60,true).front().color==Mode{1920,1080,30}));
 assert(ranked_modes({{640,480,60}},{{640,480,30}},60,true).empty());
 assert(ranked_modes(depth,{},60,true).empty());
 assert((ranked_modes(depth,{},60,false).front().depth==Mode{848,480,60}));
 assert(ranked_modes(depth,color,0,true).empty());
 std::reverse(depth.begin(),depth.end());std::reverse(color.begin(),color.end());
 auto reversed=ranked_modes(depth,color,60,true);
 for(size_t i=0;i<fast.size();i++) assert(fast[i].depth==reversed[i].depth && fast[i].color==reversed[i].color);
}
''')
    executable = tmp_path / ("modes.exe" if os.name == "nt" else "modes")
    subprocess.run([compiler, "-std=c++17", "-I", str(ROOT / "native/realsense_shared_memory/src"),
                    str(harness), "-o", str(executable)], check=True, capture_output=True, timeout=30)
    subprocess.run([str(executable)], check=True, timeout=10)


def test_camera_preset_waits_for_calibration_and_routes_without_robot(tmp_path):
    godot = find_godot(os.environ.get("ROBOT_TELEOP_TEST_GODOT", ""))
    if godot is None:
        pytest.skip("Godot is required")
    source = (ROOT / "godot/runtime/point_cloud/unified_point_cloud_view.gd").read_text()
    methods = source[source.index("func apply_remote_stream_preset("):
                     source.index("func _shared_capture_preview(")].expandtabs(1)
    gateway = (ROOT / "godot/runtime/remote_control_gateway.gd").read_text()
    route = gateway[gateway.index("func _apply_view_settings("):
                    gateway.index("func _record_tracking(")].expandtabs(1)
    script = '''extends SceneTree
class View extends Node:
 var _requested_capture_profile := ""
 var _remote_capture_profile := ""
 var _capture_paused := false
 var aligning := false
 func _realsense_cloud_alignment_running() -> bool: return aligning
''' + "\n".join(" " + line for line in methods.splitlines()) + '''
class Gateway extends Node:
''' + "\n".join(" " + line for line in route.splitlines()) + '''
func _initialize() -> void:
 call_deferred("_run")

func _run() -> void:
 var view := View.new()
 root.add_child(view)
 view.add_to_group("unified_point_cloud_view")
 var gateway := Gateway.new()
 root.add_child(gateway)
 gateway._apply_view_settings({"stream_preset":"quality"})
 view._apply_pending_capture_profile()
 assert(view.get_remote_stream_preset()=="quality")
 view.aligning=true
 gateway._apply_view_settings({"stream_preset":"latency"})
 view._apply_pending_capture_profile()
 assert(view.get_remote_stream_preset()=="quality")
 view.aligning=false
 view._capture_paused=true
 view._apply_pending_capture_profile()
 assert(view.get_remote_stream_preset()=="quality")
 view._capture_paused=false
 view._apply_pending_capture_profile()
 assert(view.get_remote_stream_preset()=="latency")
 gateway._apply_view_settings({"stream_preset":"invalid"})
 view._apply_pending_capture_profile()
 assert(view.get_remote_stream_preset()=="latency")
 print("CAPTURE_PRESETS failures=0")
 quit()
'''
    (tmp_path / "project.godot").write_text('config_version=5\n[application]\nconfig/name="Capture preset test"\n')
    (tmp_path / "probe.gd").write_text(script)
    result = subprocess.run([str(godot_console_executable(godot)), "--headless", "--path", str(tmp_path),
                             "--script", "res://probe.gd"], capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "ERROR:" not in result.stdout + result.stderr
    assert "CAPTURE_PRESETS failures=0" in result.stdout
