# Camera stream performance

Measure fresh acquisitions, encoded frames and browser redraws separately.
A 60 Hz redraw can reuse an older camera frame; the Controller's camera FPS
counter counts newly decoded captures, not animation frames.

On the three lab D435s, the SDK exposes 848×480 depth and 960×540 RGB at
60 Hz. The `fast60` profile selects the largest available 60 Hz RGB mode.
`highres30` uses 1280×720 at 30 Hz. These are sensor modes, not a guarantee
that encoding, network delivery and rendering sustain their nominal rate.

## Independent capture for a private Godot preview

The normal single-process startup remains supported. For a preview that is
slower than camera capture, run a separate headless sensor owner:

- Give both processes the same unique ASCII
  `ROBOT_TELEOP_CAPTURE_SHM_PREFIX`, containing only letters, digits and
  underscores. Generate a new value for a new session.
- Start Godot with `--headless --path <project> --script
  res://godot/runtime/streaming/camera_capture_service.gd`.
  `ROBOT_TELEOP_CAPTURE_PORT` defaults to loopback port 8781.
- Start the usual editor with `ROBOT_TELEOP_EDITOR_STREAM=1` and the same
  prefix. Its local streaming/diagnostics port remains 8780.
- Point the authenticated web gateway at the sensor service with
  `--capture-port 8781`, retaining `--stream-port 8780` for the Godot viewport.
  Existing authentication and desktop settings are unchanged.

The sensor owner loads the saved camera configuration from `godot/Main.tscn`
without modifying it. It omits the robot module host and UDP control gateway;
the existing interactive process keeps those services. This is a camera-only
source and does not publish robot overlay metadata of its own.

The rebuilt native extension is required. Capture nodes initialize the SDK on
the main thread, then use separate Godot
process groups, completing before configuration and HTTP processing. No GPU
meshes or fusion passes run in the sensor process. Immutable depth/RGBA
snapshots go to private, fixed-capacity shared mappings with a sequence lock;
the editor reads the latest complete snapshot at its own rate. Source cadence
and acquisition age remain separate from preview cadence.

The interactive preview forwards camera settings/transforms through a small
session-specific control file. Calibration temporarily releases the physical
pipelines using a heartbeat lease; an expired lease resumes capture. No saved
calibration or scene needs to be replaced. Stop both processes before removing
their session mappings. An ordinary publisher shutdown removes its mappings;
after a forced termination the launcher must clean up only its recorded prefix.

## Encoding and uncertain geometry

Quality requests up to 60 Hz, native depth samples and JPEG quality 95.
Full-resolution encoding copies measured holes as zero. Neighbour halo repair
is restricted to genuinely downsampled footprints. Direct JPEG scanlines avoid
resampling unchanged dimensions; identity-resampler equivalence is tested.

Mesh validation bounds a triangle's depth jump by both the existing user limit
and a range/pixel-footprint limit. The browser, native shader, GPU index builder
and CPU mesh use the same rule. A cube-sized step is rejected while a continuous
oblique plane remains valid. Blind regions stay empty; blending does not infer
surfaces outside the cameras' measurements. Incorrect but nonzero stereo depth
can still require a better camera angle or sensor tuning.

Original RGB camera mode uses a persistent same-origin WebSocket and a bounded
latest-frame decode queue. It pauses hidden RGB-D reconstruction unless its
manual calibration preview is visible, while leaving robot/control connections
intact. Persistent depth compression pipelines one successor during network
transmission without dropping the reference chain.

Encoding follows unique acquisition sequences with a bounded pacing allowance
for jitter. Completed cameras coalesce at the output cadence without waiting
for the slowest camera. Decoder backpressure retains named keyframes and skips
obsolete independent deltas. BlendPCR reuses unchanged camera raster layers;
view, transform, intrinsics, freshness and masking changes invalidate them.

`/diagnostics` on each private Godot port reports acquisition sizes/rates,
preview age, frame CPU time, view CPU time and stream-stage timings. Use these
alongside Advanced Setup's decode, render and per-camera FPS counters before
lowering resolution or changing capture settings.

## Measured boundary, 2026-10-20

The three-D435 lab check used a U300 workstation and the authenticated public
tunnel. Fresh per-camera rates were measured separately from cached redraws:

| Stage | Resolution | Fresh FPS per camera |
| --- | --- | --- |
| Camera-only acquisition, eight seconds | 848×480 depth, 960×540 original RGB | 59.3–59.7 |
| Local native RGB-D encoding/delivery, eight seconds | 848×480, JPEG 95, exact depth | 34.8–36.7 |
| Public browser, Quality BlendPCR | 848×480, full RGB updates | About 10 |
| Public browser, Low latency | 384×217 | About 22–27 |

The local native-quality stream used about 197 Mb/s; the public browser
received around 49–53 Mb/s. At the same payload size, 60 fresh FPS for all
three cameras would require roughly 330 Mb/s. These are observations of this
run, not hard bandwidth limits. Persistent exact-depth compression did not
materially improve the public native-quality result. A 60 Hz cached browser
redraw does not establish a 60 FPS camera stream.

Further work must reduce encoding and transport cost while retaining depth
precision; merely increasing the requested frame rate is insufficient. The
current changes do not claim sustained 60 FPS end to end.
