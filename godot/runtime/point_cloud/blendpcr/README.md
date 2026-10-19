# Experimental Godot BlendPCR rendering

Select **Rendering Mode → BlendPCR (experimental)** on the Unified Point Clouds
node in Godot. Capture, LightGlue/ArUco alignment, saved camera transforms, robot
masks, and RGB-D streaming use their existing paths. The browser point-cloud
renderer does not gain BlendPCR in this change; its existing `shader_mesh` value
is simply labelled **Shared-color mesh**.

For a rig overview, select **View → Workflow → Camera Guides**. **Positions**
shows camera optical origins with repeating red, green and blue markers;
**Positions and view cones** also shows the depth fields of view between the
configured near and far limits. The cones use each live depth grid's intrinsics
and stride. Guides follow the calibrated camera transforms, are transient
children of the camera renderers, and do not alter or save calibration. They are
hidden by default and also work while BlendPCR hides the original mesh layers.

This is a Godot/RealSense adaptation of the core rendering approach from
[BlendPCR](https://github.com/muehlenb/BlendPCR), Mühlenbrock, Weller and Zachmann,
ICAT-EGVE 2024 ([paper DOI](https://doi.org/10.2312/egve.20241366)). Adapted against
upstream commit `8679e426dd76f06cb5ad7563d24e8c87b75c03db`. The upstream Azure Kinect
capture/playback application is not included or required.

## Adaptation and limits

1. Each native RealSense mesh is drawn **once**, into a private metadata
   viewport. A 24-bit primitive ID identifies the exact original depth triangle;
   ray/triangle intersection and barycentric UVs recover its measured depth, RGB
   interpolation, color correction and edge shading. Other/custom shaders keep
   the compatible color-plus-metadata path with 16-bit depth. A per-capture GPU
   map evaluates each native triangle once. Unified mesh reuses that validation;
   peer-camera checks and BlendPCR reuse its confidence. BlendPCR keeps native
   vertex/fragment validation in its private worlds, so a cached mask cannot
   introduce invalid geometry. The map
   invalidates on in-place depth sequence updates and geometry-setting changes.
   Capture producers run before the view's cache and peer-layer consumers, so
   intermittent frames cannot combine fresh depth with an older triangle mask.
   Source RGB/depth textures, intrinsics, sampling stride and global transforms
   come from the existing native renderer. The primitive ID removes both depth
   quantization and a neighboring-triangle search. Lower-resolution passes use
   the sampled metadata texel's ray, preserving coverage at triangle boundaries.
   A shared GPU atlas lets peer cameras
   reuse validation and confidence too. No sensor resolution is hardcoded.
2. At quarter output resolution, the dominance pass finds the nearest surface
   and chooses the highest-quality compatible camera. All candidates are gated
   against the same nearest depth, so camera ordering cannot extend tolerance
   through a chain of pairwise matches.
3. A horizontal then vertical filter produces the same square-neighborhood
   weights. Intermediate integer counts are stored exactly: radius 3 takes
   14 texture reads instead of 49 without changing the kernel or output size.
4. The final spatial shader blends compatible **colors** using those weights.
   Geometry uses the highest-confidence camera's depth, avoiding invented
   intermediate surfaces at cube/table edges. Agreement narrows with sensor
   range and is capped by the user's surface tolerance. Robot overlays retain
   proper occlusion.
   A local valid-camera fallback preserves pixels where smoothed weights vanish.

The upstream dominance/weight/blending passes are adapted to Godot's viewport
textures and depth conventions. This is **not** a byte-for-byte implementation
of the complete upstream benchmark pipeline: quality weighting is adapted for
RealSense sampling density; the upstream MLS, erosion, and hole-filling stages
are not added. The primitive-ID reconstruction is specific to this port. Existing source triangle
validation, cleanup, color correction and robot masks are retained. No temporal
accumulation or motion-triggered compression changes are introduced.

BlendPCR and **Unified mesh → Unified Camera Priority → Best local view** use
the same per-surface camera confidence. Unified mesh chooses the local owner;
BlendPCR retains overlapping valid samples for seam smoothing. A coherent,
better-resolved peer can veto an uncertain edge triangle that lies in its
observed free space. A closer peer measurement means occlusion and is never
treated as a free-space contradiction. No match alone never removes a surface.
This is a conservative artifact check, not a substitute for good calibration.
Best local view is the default for newly created unified views; saved D455,
D435 and serial priorities retain their original numeric values. Ownership uses
a transitive quality ordering so three near-equal cameras cannot all discard
one another through a chain of pairwise ties.

The implementation supports **one to four cameras** with at most fourteen
fragment samplers, below OpenGL 3.3's minimum 16-sampler limit. It does not
truncate larger rigs: those keep independent meshes with an explicit status
message. Cameras with displayed frames at least 1000 ms old are excluded from
both dominance and composition. New/removed cameras and renderer mode switches
restore the original visibility layers and release the viewport resources.

Balanced quality renders at 75% of the active viewport's dimensions; Full
resolution uses 100%, Performance uses 50%. Surface tolerance and seam radius
are exposed only in this mode. Depth tolerance is a compatibility heuristic, not
proof that nearby samples describe the same physical surface: check thin objects
and opposite sides before using a large tolerance. Generic shaders retain
16-bit depth metadata relative to the viewer's far plane; avoid an unnecessarily
distant far plane.
For native sources, the depth span safely bounds all transformed capture frusta
instead of wasting precision on the editor camera's distant far plane. This
does not clip valid source geometry. Native metadata uses an exact 24-bit
primitive ID and reconstructs depth from the original floating-point samples.
Grids exceeding the ID capacity keep the compatible two-pass rendering path.
Metadata emission compensates for Godot 4.6 Compatibility's approximate sRGB
conversion, so packed depth bytes are not treated as ordinary image colors.
Both backend tests assert reconstructed metric depth as well as image color.
The inverse conversion is precomputed for all 256 possible byte values in a
small floating-point lookup texture. Shader uniforms, camera state and mesh
bindings are updated only when their inputs change. These optimizations do not
reduce capture or rendering resolution, sampling density or seam quality.

The runtime uses the viewport's active camera; editor preview follows the first
3D editor viewport, even when the edited scene has its own current camera in
Godot's hidden 2x2 scene viewport. Additional simultaneous editor views are not independently
composited. This path uses GPU-to-GPU viewport dependencies, without texture
readback. The test harness reads output images only for assertions. It creates
several render targets per camera. Physical-rig timings depend on viewport size,
capture settings, cameras, GPU, and desktop streaming. Upstream performance
numbers do not apply to this port. Private `/diagnostics` stream counters report
actual scene/capture FPS and dimensions without encoding images or GPU readback.

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
Additional tests compare every separable-filter output pixel against the original
square filter and verify in-place capture invalidation, camera-local ownership,
free-space/occlusion behavior, and stable ownership for three similar cameras.
Native RGB reconstruction is compared with original rendering on a transformed,
sloping checker/gradient surface, different depth/color sizes, active correction,
and perspective and orthographic cameras. Rectangular, non-power-of-two sensor
grids cross the 16-bit ID boundary. Coverage is unchanged in both backends.
An actual editor probe also checks that a current camera in a hidden 2x2 scene
viewport cannot replace the visible editor camera or collapse render resolution.
The mode probe verifies legacy settings and the CPU fallback. This does not
validate device capture, real calibration accuracy, performance or visual
quality on physical cameras.

## Licence and attribution

The files in this directory are provided under **EUPL-1.2**, including the adapted
shader passes and their Godot adapter. The full upstream licence is in `LICENSE`.
Original shader implementation copyright 2024 Andre Mühlenbrock, CGVR,
University of Bremen. Godot/RealSense adaptation: 2026. This directory's licence
notice does not change the licence headers of unrelated repository files.
