# Experimental Godot BlendPCR rendering

Select **Rendering Mode → BlendPCR (experimental)** on the Unified Point Clouds
node in Godot. Capture, LightGlue/ArUco alignment, saved camera transforms, robot
masks, and RGB-D streaming use their existing paths. The browser point-cloud
renderer does not gain BlendPCR in this change; its existing `shader_mesh` value
is simply labelled **Shared-color mesh**.

This is a Godot/RealSense adaptation of the core rendering approach from
[BlendPCR](https://github.com/muehlenb/BlendPCR), Mühlenbrock, Weller and Zachmann,
ICAT-EGVE 2024 ([paper DOI](https://doi.org/10.2312/egve.20241366)). Adapted against
upstream commit `8679e426dd76f06cb5ad7563d24e8c87b75c03db`. The upstream Azure Kinect
capture/playback application is not included or required.

## Adaptation and limits

1. Each visible RealSense source mesh is drawn into private color and metadata
   viewports from the operator camera. The metadata stores 16-bit eye-space
   depth and sampling quality (projected focal density, range and incidence).
   Source RGB/depth textures, intrinsics, sampling stride and global transforms
   come from the existing native renderer. No sensor resolution is hardcoded.
2. At quarter output resolution, the dominance pass finds the nearest surface
   and chooses the highest-quality compatible camera. All candidates are gated
   against the same nearest depth, so camera ordering cannot extend tolerance
   through a chain of pairwise matches.
3. A spatial neighborhood of dominance labels produces per-camera weights.
4. The final spatial shader blends compatible color/depth samples using those
   weights and writes scene depth, so robot overlays retain proper occlusion.
   A local valid-camera fallback preserves pixels where smoothed weights vanish.

The upstream dominance/weight/blending passes are adapted to Godot's viewport
textures and depth conventions. This is **not** a byte-for-byte implementation
of the complete upstream benchmark pipeline: quality weighting is adapted for
RealSense sampling density; the upstream MLS, erosion, hole-filling, and
high-resolution UV-index encoding stages are not added. Existing source triangle
validation, cleanup, color correction and robot masks are retained. No temporal
accumulation or motion-triggered compression changes are introduced.

The first implementation supports **one to four cameras** with at most nine
fragment samplers, below OpenGL 3.3's minimum 16-sampler limit. It does not
truncate larger rigs: those keep independent meshes with an explicit status
message. Cameras with displayed frames at least 1000 ms old are excluded from
both dominance and composition. New/removed cameras and renderer mode switches
restore the original visibility layers and release the viewport resources.

Balanced quality renders at 75% of the active viewport's dimensions; Full
resolution uses 100%, Performance uses 50%. Surface tolerance and seam radius
are exposed only in this mode. Depth tolerance is a compatibility heuristic, not
proof that nearby samples describe the same physical surface: check thin objects
and opposite sides before using a large tolerance. The 16-bit metadata precision
is relative to the viewer's far plane; avoid an unnecessarily distant far plane.
The default 100 m far plane gives approximately 1.5 mm depth quantization.
Metadata emission compensates for Godot 4.6 Compatibility's approximate sRGB
conversion, so packed depth bytes are not treated as ordinary image colors.
Both backend tests assert metric depth as well as image color.

The runtime uses the viewport's active camera; editor preview follows the first
3D editor viewport. Additional simultaneous editor views are not independently
composited. This path uses GPU-to-GPU viewport dependencies, without texture
readback. The test harness reads output images only for assertions. It creates
several render targets per camera and is not yet benchmarked on a physical rig
or the operator laptop. Upstream performance numbers do not apply to this port.

## Mode compatibility

- Independent mesh remains the default.
- `shader_mesh` retains its behavior, displayed as Shared-color mesh.
- Legacy `prefer_d455` / `prefer_d435` scene values load as Unified mesh with the
  corresponding Unified Camera Priority. Serial order is also available.
- `cpu_mesh` remains supported through Rendering Diagnostics → Diagnostic CPU
  Mesh. Selecting a normal rendering mode leaves that fallback.
- Existing serialized `render_mode` strings remain the source of truth; the
  friendly mode selector and diagnostic checkbox are editor-only proxies.

## Validation

With Godot 4.6.3 available:

```sh
ROBOT_TELEOP_TEST_GODOT=/path/to/godot python -m pytest -q tests/test_blendpcr.py
```

For actual software-GPU rendering (Linux requires a display or `xvfb-run`):

```sh
ROBOT_TELEOP_TEST_GODOT=/path/to/godot \
ROBOT_TELEOP_TEST_BLENDPCR_GPU=1 LIBGL_ALWAYS_SOFTWARE=1 \
xvfb-run -a python -m pytest -q tests/test_blendpcr.py
```

The GPU probe runs on Compatibility/OpenGL and Forward+/Vulkan. It assembles the
production static native shader from its C++ sources without loading a camera
SDK or compiled extension, and exercises mixed RGB-D grids, seam blending, stale
foreground B with live C, missing depth, color space, overlay depth, camera
transforms, resize, removal, teardown/re-entry and explicit capacity fallback.
The mode probe verifies legacy settings and the CPU fallback. This does not
validate device capture, real calibration accuracy, performance or visual
quality on physical cameras.

## Licence and attribution

The files in this directory are provided under **EUPL-1.2**, including the adapted
shader passes and their Godot adapter. The full upstream licence is in `LICENSE`.
Original shader implementation copyright 2024 Andre Mühlenbrock, CGVR,
University of Bremen. Godot/RealSense adaptation: 2026. This directory's licence
notice does not change the licence headers of unrelated repository files.
