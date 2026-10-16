"""Render the production native fusion predicate without a camera or native DLL.

Optional Linux/EGL check: pip install pytest moderngl; python -m pytest -q
tests/test_realsense_fusion_shader.py. The existing Godot probe additionally tests
the engine, native material wiring, and age-to-readiness metadata.
"""
from pathlib import Path
import re

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def fusion_gpu():
    gl = pytest.importorskip("moderngl")
    try:
        context = gl.create_standalone_context(backend="egl", require=330)
    except Exception as error:
        pytest.skip(f"EGL rendering unavailable: {error}")
    # Godot's shader dialect adds sampler hints and calls fragment inputs
    # 'varying'. Keep the actual fusion functions unchanged.
    source = (ROOT / "native/realsense_shared_memory/src/multi_camera_fusion_shader.h").read_text()
    source = source.split('R"(', 1)[1].rsplit(')";', 1)[0]
    source = re.sub(r" : filter_nearest, repeat_disable", "", source)
    source = source.replace("varying vec3", "in vec3")
    program = context.program(
        vertex_shader="""#version 330
uniform float depth_m;
out vec3 rig_world_position;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2) * 2.0 - 1.0;
    gl_Position = vec4(p, depth_m * 0.5, 1.0);
    rig_world_position = vec3(p * 0.1, -depth_m);
}
""",
        fragment_shader="#version 330\nuniform float fusion_depth_tolerance = 0.035;\n"
        + source + """
uniform vec3 camera_color;
out vec4 result;
void main() {
    if (rig_overlap()) discard;
    result = vec4(camera_color, 1.0);
}
""",
    )
    vao = context.vertex_array(program, [])
    framebuffer = context.simple_framebuffer((32, 32))
    palette = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]

    def render(depths, ready, enabled=True):
        metadata = np.zeros((3, 8, 4), dtype="f4")
        for i in range(3):
            metadata[i, :3] = np.eye(4, dtype="f4")[:3]
            metadata[i, 3] = [16, 16, 7.5, 7.5]
            metadata[i, 4] = [16, 16, 1, 0]
            metadata[i, 5] = [0.1, 4, 0.05, 1]
            metadata[i, 6] = [16, 16, 1, ready[i]]
        depth = np.broadcast_to(np.array(depths, dtype="f4")[:, None, None], (3, 16, 16)).copy()
        textures = [
            context.texture_array((16, 16, 3), 1, depth.tobytes(), dtype="f4"),
            context.texture_array((16, 16, 3), 4, bytes([255] * 16 * 16 * 3 * 4)),
            context.texture((8, 3), 4, metadata.tobytes(), dtype="f4"),
        ]
        try:
            for unit, (name, texture) in enumerate(zip(["rig_depth", "rig_color", "rig_cameras"], textures)):
                texture.filter = (gl.NEAREST, gl.NEAREST)
                texture.use(unit)
                program[name] = unit
            framebuffer.use()
            framebuffer.clear(0, 0, 0, 1)
            context.enable(gl.DEPTH_TEST)
            program["rig_fusion_enabled"] = enabled
            for i, z in enumerate(depths):
                if z <= 0:
                    continue  # Mirrors the mesh's own invalid-depth rejection.
                program["depth_m"] = z
                program["rig_camera_index"] = i
                program["camera_color"] = palette[i]
                vao.render(vertices=3)
            return tuple(framebuffer.read(viewport=(16, 16, 1, 1), components=3))
        finally:
            for texture in textures:
                texture.release()
    yield render
    framebuffer.release()
    vao.release()
    program.release()
    context.release()


@pytest.mark.parametrize("stale_index", [0, 1, 2])
def test_stale_foreground_cannot_occlude_live_camera(fusion_gpu, stale_index):
    depths = [0, 0, 1.0]
    live_index = 2 if stale_index != 2 else 1
    depths[live_index] = 1.0
    depths[stale_index] = 0.7  # Outside tolerance, so no peer can discard it.
    ready = [1, 1, 1]
    ready[stale_index] = 0
    expected = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
    assert fusion_gpu(depths, ready) == expected[live_index]
    # The guard is confined to unified mode, and a new frame restores drawing.
    assert fusion_gpu(depths, ready, enabled=False) == expected[stale_index]
    ready[stale_index] = 1
    assert fusion_gpu(depths, ready) == expected[stale_index]


def test_live_overlap_and_all_stale(fusion_gpu):
    assert fusion_gpu([1.008, 1.006, 1.004], [1, 1, 1]) == (255, 0, 0)
    assert fusion_gpu([1.008, 1.006, 1.004], [0, 0, 0]) == (0, 0, 0)
