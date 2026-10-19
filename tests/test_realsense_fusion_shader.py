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
    best_view = (ROOT / "godot/runtime/point_cloud/best_view_fusion.gdshaderinc").read_text()
    best_view = re.sub(r"\s*:\s*filter_nearest\s*,\s*repeat_disable", "", best_view)
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
        + source + best_view + """
uniform vec3 camera_color;
out vec4 result;
void main() {
    if (rig_best_view ? rig_best_overlap() : rig_overlap()) discard;
    result = vec4(camera_color, 1.0);
}
""",
    )
    vao = context.vertex_array(program, [])
    framebuffer = context.simple_framebuffer((32, 32))
    palette = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]

    def render(depths, ready, enabled=True, *, best_view=False, focal=None,
               camera_x=None, maps=None, draw_indices=None, pixel=(16,16), select_owner=True,cached=False):
        metadata = np.zeros((3, 8, 4), dtype="f4")
        for i in range(3):
            metadata[i, :3] = np.eye(4, dtype="f4")[:3]
            metadata[i, 0, 3] = camera_x[i] if camera_x is not None else 0
            f = focal[i] if focal is not None else 16
            metadata[i, 3] = [f, f, 7.5, 7.5]
            metadata[i, 4] = [16, 16, 1, 0]
            metadata[i, 5] = [0.1, 4, 0.05, 1]
            metadata[i, 6] = [16, 16, 1, ready[i]]
        depth = np.broadcast_to(np.array(depths, dtype="f4")[:, None, None], (3, 16, 16)).copy()
        if maps is not None:
            for i, image in maps.items(): depth[i] = image
        textures = [
            context.texture_array((16, 16, 3), 1, depth.tobytes(), dtype="f4"),
            context.texture_array((16, 16, 3), 4, bytes([255] * 16 * 16 * 3 * 4)),
            context.texture((8, 3), 4, metadata.tobytes(), dtype="f4"),
            context.texture((48,16),4,bytes([255]*48*16*4)),
        ]
        try:
            for unit, (name, texture) in enumerate(zip(["rig_depth", "rig_color", "rig_cameras","rig_quality"], textures)):
                texture.filter = (gl.NEAREST, gl.NEAREST)
                texture.use(unit)
                program[name] = unit
            framebuffer.use()
            framebuffer.clear(0, 0, 0, 1)
            context.enable(gl.DEPTH_TEST)
            program["rig_fusion_enabled"] = enabled
            program["rig_best_view"] = best_view
            program["rig_select_owner"] = select_owner
            program["rig_quality_ready"] = cached
            program["rig_quality_grid"] = (3,1,16,16)
            for i, z in enumerate(depths):
                if draw_indices is not None and i not in draw_indices: continue
                if z <= 0:
                    continue  # Mirrors the mesh's own invalid-depth rejection.
                program["depth_m"] = z
                program["rig_camera_index"] = i
                program["camera_color"] = palette[i]
                vao.render(vertices=3)
            return tuple(framebuffer.read(viewport=(*pixel, 1, 1), components=3))
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


def test_best_view_uses_sampling_density_and_retains_legacy_priority(fusion_gpu):
    assert fusion_gpu([1,1,1], [1,1,1], focal=[10,16,24], best_view=True) == (0,0,255)
    assert fusion_gpu([1,1,1], [1,1,1], focal=[10,16,24]) == (255,0,0)
    assert fusion_gpu([1,1,1], [1,1,1], best_view=True) == (255,0,0)
    assert fusion_gpu([1,1,1], [1,1,1], focal=[10,16,24], best_view=True,cached=True) == (0,0,255)


def test_best_view_is_local_instead_of_one_global_camera(fusion_gpu):
    options = dict(best_view=True, focal=[10,10,10], camera_x=[0.4,-0.4,0])
    assert fusion_gpu([1,1,0], [1,1,0], pixel=(4,16), **options) == (255,0,0)
    assert fusion_gpu([1,1,0], [1,1,0], pixel=(28,16), **options) == (0,255,0)


def test_close_quality_chain_cannot_discard_every_camera(fusion_gpu):
    assert fusion_gpu([1,1,1],[1,1,1],best_view=True,focal=[10,10.075,10.149]) in [(255,0,0),(0,255,0),(0,0,255)]


def test_free_space_requires_better_coherent_peer_and_preserves_occlusions(fusion_gpu):
    edge=np.full((16,16),0.98,dtype='f4')
    edge[7,9]=edge[9,7]=0.965  # Valid triangle, uncertain support just outside it.
    options=dict(best_view=True,select_owner=False,focal=[10,24,16],maps={0:edge},draw_indices=[0])
    assert fusion_gpu([0.98,1.05,0],[1,1,0],**options) == (0,0,0)
    assert fusion_gpu([0.98,1.00,0],[1,1,0],**options) == (0,0,0)
    assert fusion_gpu([0.98,0.90,0],[1,1,0],**options) == (255,0,0)
    assert fusion_gpu([0.98,1.05,0],[1,0,0],**options) == (255,0,0)
    options['maps']={}
    assert fusion_gpu([0.98,1.05,0],[1,1,0],**options) == (255,0,0)
