"""Execute Godot mode migration and optional actual GPU composition without devices."""
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from robot_teleop.doctor import find_godot, godot_console_executable

ROOT = Path(__file__).resolve().parents[1]


def run_probe(tmp_path, probe, rendering_method=None):
    godot = find_godot(os.environ.get("ROBOT_TELEOP_TEST_GODOT", ""))
    if godot is None:
        pytest.skip("Set ROBOT_TELEOP_TEST_GODOT to run Godot probes")
    shutil.copytree(ROOT / "godot/runtime/point_cloud", tmp_path / "godot/runtime/point_cloud")
    shutil.copy2(ROOT / "tests/godot" / probe, tmp_path / "probe.gd")
    (tmp_path / "project.godot").write_text(
        'config_version=5\n[application]\nconfig/name="BlendPCR Test"\n', encoding="utf-8"
    )
    # Assemble exactly the static native shader selected by ensure_material(),
    # without needing camera SDKs or a platform-specific compiled extension.
    cpp = (ROOT / "native/realsense_shared_memory/src/realsense_shared_memory_point_cloud.cpp").read_text()
    shader = cpp.split('if (want_static_shader_mesh) {', 1)[1].split('R"(', 1)[1].split(')";', 1)[0]
    fusion = (ROOT / "native/realsense_shared_memory/src/multi_camera_fusion_shader.h").read_text().split('R"(', 1)[1].split(')";', 1)[0]
    shader = shader.replace("__COLOR_TEX_UNIFORM__", "uniform sampler2D color_tex : source_color, filter_nearest;")
    shader = shader.replace("void vertex() {", fusion + "\nvoid vertex() {")
    shader = shader.replace("point_position_local = VERTEX;", "point_position_local = VERTEX;\n    rig_world_position = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;")
    shader = shader.replace("void fragment() {", "void fragment() {\n    if (rig_overlap()) discard;")
    (tmp_path / "native_template.gdshader").write_text(shader, encoding="utf-8")
    command = [str(godot_console_executable(godot)), "--path", str(tmp_path), "--audio-driver", "Dummy"]
    if rendering_method:
        command += ["--rendering-method", rendering_method, "--resolution", "160x128"]
    else:
        command += ["--headless"]
    command += ["--script", "res://probe.gd"]
    result = subprocess.run(
        command, capture_output=True, text=True, timeout=90, check=False,
        env={**os.environ, "APPDATA": str(tmp_path / "appdata"), "XDG_DATA_HOME": str(tmp_path / "data")},
    )
    output = result.stdout + result.stderr
    (tmp_path / "probe.log").write_text(output, encoding="utf-8")
    assert result.returncode == 0, output
    assert "ERROR:" not in output, output
    assert "failures=0" in output, output
    return output


def test_render_mode_migration_and_diagnostic_fallback(tmp_path):
    run_probe(tmp_path, "blendpcr_modes_probe.gd")


@pytest.mark.skipif(os.environ.get("ROBOT_TELEOP_TEST_BLENDPCR_GPU") != "1", reason="Opt in to Godot GPU composition")
@pytest.mark.parametrize("rendering_method", ["gl_compatibility", "forward_plus"])
def test_blendpcr_multi_camera_rendering(tmp_path, rendering_method):
    output = run_probe(tmp_path, "blendpcr_probe.gd", rendering_method)
    assert "BLENDPCR_PROBE" in output
    assert f"BLENDPCR_RENDERER={rendering_method}" in output, output
