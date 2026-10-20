# Camera stream performance

Measure fresh acquisitions, encoded frames and browser redraws separately.
A 60 Hz redraw can reuse an older camera frame; the Controller's camera FPS
counter counts newly decoded captures, not animation frames.

The existing browser Stream dropdown now switches the actual capture profile:

| Preset | Native depth on the three lab D435s | Original RGB | Capture target |
| --- | --- | --- | --- |
| Low latency / `fast60` | 848×480 | 960×540 | 60 Hz |
| Quality / `highres30` | 1280×720 | 1920×1080 | 30 Hz |

These dimensions come from each device's advertised modes, with SDK validation
of the depth/RGB pair. Other devices use their largest supported pair at the
requested cadence, falling back to a lower advertised common cadence when
necessary. Both presets preserve native depth samples, JPEG 95 and full RGB
updates. A 3840-pixel transport ceiling does not upscale a smaller sensor image.
Aligned mesh color follows the depth grid; RGB camera mode shows the original
RGB image at its separate sensor resolution.

Switching briefly restarts the cameras and affects every viewer sharing that
capture owner. The request uses the existing authenticated view-settings
WebSocket even with head tracking disabled. It waits for active calibration
to finish and never changes saved transforms or the scene. Acquisition sizes,
profile and actual capture rates appear in the private `/diagnostics` response.
These are capture targets, not a guarantee that USB, encoding, network delivery,
decoding or rendering sustain their nominal rate.

The live three-camera dropdown check confirmed both native mode pairs, with
stride 1 and fresh frames from every sensor. The public Low latency viewer
reported 34–35 fresh FPS/camera in one boundary sample; Quality BlendPCR at the
larger depth grid reported 17–19 FPS/camera (about 37 ms codec plus 5 ms upload
submission). These are short diagnostic samples, not sustained guarantees or
an identical-load comparison. Quality capture itself varied around 15–25 FPS
in the sampled host window. The larger native mode therefore still needs
capture/encoding and client decode profiling to sustain 30 fresh FPS. Saved
scene and calibration hashes were unchanged after both switches.

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

Quality requests up to 30 Hz; Low latency requests up to 60 Hz. Both preserve
native depth samples and JPEG quality 95.
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

Advanced Setup separates complete RGB-D packet arrivals from completed decodes.
Its cumulative received/decoded totals can be differenced over the same time
window. Arrival counts packets, which may contain one or several cameras;
decoded counts batches, which can combine the newest independent capture of
each camera from several queued packets. Coalesced packets are reported
separately. Per-camera cumulative counters count actual decoded camera updates
and should be used for sustained fresh-camera FPS. Arrival time
is measured when JavaScript receives a complete WebSocket message or HTTPS
packet, rather than at the network interface.

Codec time measures asynchronous JPEG/depth reconstruction; upload submission
measures the subsequent WebGL texture submission. The previous Decode timing
included both stages and is now labeled Decode+upload. These are elapsed times,
including scheduling delays, not isolated CPU/GPU execution benchmarks. Queue
depth and wait time show whether decoding is accumulating a backlog. Cached
render FPS remains separate from both packet rates.

## Streaming optimizations, 2026-10-20

Normal Quality passes the native JPEG and losslessly compressed depth through
the gateway unchanged. Rebuilding exact temporal depth tiles cost about 69 ms
per packet in this lab run, with no size saving on noisy live depth. Persistent
reference compression is disabled in the web viewer, including for old saved
settings that enabled it. Its checkbox was removed; the backend codec remains
available for controlled bandwidth experiments. Both live presets bypass it.

Up to three decoder workers run depth inflation and JPEG decoding in parallel.
Native stateless frames transfer their buffers without cloning them; each
worker returns exact depth samples and RGBA pixels. A bounded successor batch
decodes while the current batch uploads. Queue coalescing retains the latest
capture of every camera, rather than letting an A-only packet erase B/C updates.
Temporal reference chains remain serial. Restarting cancels pending work and
releases decoded resources before a different renderer generation can use them.

RGBA8 texture uploads avoid RGB format conversion. GPU capacity checks and
framebuffer validation run on allocation, while the renderer supplies its
known framebuffer/VAO/viewport state during normal frames. BlendPCR uses shared
grid vertices plus a unique provoking vertex for each exact triangle ID.
Invalid triangles degenerate before rasterization. Fixed rig loop bounds and
float32 uniform camera data reduce repeated work; rigs exceeding the uniform
budget retain the texture metadata path. Output sampling and resolution stay
unchanged. Browser GPU checks compare final pixels, triangle IDs beyond 16 bits,
and overlay depth occlusion with the original path, including robot masking.

Linux native builds use optional `libdeflate` when `pkg-config` finds it
(`libdeflate-dev` on Ubuntu); other builds retain zlib. Both produce standard
zlib streams of the same exact predictor values. Production-encoder tests
inflate and compare every uint16 sample, including zeros and wraparound.
The optional compressor was built and exercised on the lab Linux workstation;
the Windows fallback does not require a new dependency.

The capture-only owner also detects an SDK source that still reports open but
has stopped producing frames for five seconds. Recovery runs after capture
workers complete, serializes reopening on the main thread, respects calibration
pauses, and backs off between attempts.

## Verified after optimization

The same public viewer on the Iris Xe laptop sustained the following fresh
camera rates, calculated from cumulative frame counters over 65.163 seconds.
The test used the previous Quality BlendPCR preset (before the native 30/60 Hz
preset split), all three cameras, 848×480 depth/aligned RGB,
JPEG 95 and exact uint16 depth. The canvas remained 2240×1260 physical pixels
(1280×720 CSS pixels at Windows display scaling); it was not downscaled.

| Camera suffix | Fresh decoded FPS |
| --- | ---: |
| A / 3665 | 32.55 |
| B / 3681 | 32.58 |
| C / 5787 | 33.79 |

Public throughput was around 187–197 Mb/s in the boundary snapshots. A separate
eight-second local capture/encoding check reached 48.1–48.9 FPS per camera with
median encode times around 10 ms. These stages were measured separately, not
treated as simultaneous samples of the same frames. Cached render rates and
packet/batch rates are not used as evidence of fresh camera FPS.

The target of 30 fresh FPS was met in this sustained run; 60 FPS end to end
was not. Native encoding and browser decode/GPU work still run below the
camera-only 60 Hz acquisition ceiling on these machines. This result does not
guarantee the same throughput on other network routes, browser loads or GPUs.

## Baseline before these optimizations, 2026-10-20

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

These baseline rates precede the gateway bypass, decoder workers, prefetch,
native compression and GPU changes above. Merely increasing the requested frame
rate does not establish sustained 60 FPS end to end.
