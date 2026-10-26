# Camera stream performance

## Camera preview and input latency

The fixed RGB view prefers a native-resolution H.264 preview through the authenticated
`/camera-preview-stream` WebSocket. WebCodecs decodes and paints the selected camera.
The encoder uses full-range JPEG color, CRF 10, no B frames, no lookahead, and an
ultrafast preset. This is high-quality preview recompression, not a lossless RGB
transport. Original host recordings, capture profiles, depth samples, and the 3D
RGB-D textures are unchanged. `?rgb_preview=jpeg` selects the original JPEG preview
for comparisons; browsers without a working VideoDecoder fall back automatically.

Only RGB mode opens this preview. Returning to 3D closes it; RGB mode pauses hidden
RGB-D processing unless a visible calibration panel needs that data. Leaving the
page or hiding the tab closes the camera decoder and connection. Camera changes
retain the last painted image until the new camera decodes. The server retains one
newest source image and allows at most sixteen unacknowledged encoded frames;
backpressure drops source images before encoding, preserving the H.264 reference
chain. A lagging decoder can request an independent keyframe.

Independent native RGB-D WebSocket viewers also negotiate `flow_control=1`.
Small sequence messages accompany the unchanged binary RGB-D packets, and the
browser acknowledges their receipt. The sender bounds outstanding frames using
the smallest observed acknowledgement round trip and requested cadence (2–16
frames). Congestion cannot inflate that window indefinitely. When credit runs
out, source draining continues in its newest-frame slot and expensive packet
processing waits. The next available credit selects current captures rather
than replaying every missed frame. Legacy clients and temporal-reference streams
retain their existing wire protocol.

The browser also bounds GPU work to two submitted views using nonblocking WebGL
fences. While the GPU is busy, camera uploads continue coalescing to the newest
complete frame per camera; the main thread remains available for input and
decoder completion. No sensor samples, RGB texture dimensions, or shader detail
are reduced. Submitted-render FPS is not a measurement of display presentation.

Keyboard press/release transitions send immediately, with the existing 30 Hz
heartbeat retained. OS repeats do not add commands. The follower loop frequency,
physical speed/step limits, watchdog, Pause, and explicit Enable behavior are unchanged.

Fresh-frame FPS, camera-frame age, and command-to-visible-motion latency are different
measurements. Public-route request round trips alone were 214–261 ms in the
2026-10-26 Singapore-host probe; direct STUN-only WebRTC did not connect during its
twenty-second observation window. A sub-100 ms end-to-end guarantee is not established
for that route. Measure actual physical movement in captured images and browser
presentation separately; do not treat an animated robot overlay as physical feedback.

Measure fresh acquisitions, encoded frames and browser redraws separately.
A 60 Hz redraw can reuse an older camera frame; the Controller's camera FPS
counter counts newly decoded captures, not animation frames.

The existing browser Stream dropdown now switches the actual capture profile:

| Preset | Native depth on the three lab D435s | Original RGB | Capture target |
| --- | --- | --- | --- |
| Low latency / `fast60` | 848×480 | 960×540 | 60 Hz |
| Quality / `highres30` | 1280×720 | 1920×1080 | 30 Hz |

These dimensions come from each device's advertised modes, with SDK validation
of the depth/RGB pair. Quality maximizes both grids at 30 Hz; Low latency does
the same at 60 Hz. The D435's recommended 848×480 depth mode is a separate
accuracy recommendation, not its maximum resolution; see
[RealSense depth post-processing guidance](https://dev.realsenseai.com/docs/depth-post-processing-for-intel-realsense-depth-camera-d400-series/).
All devices use their largest supported pair at the requested cadence,
falling back to a lower advertised common cadence when
necessary. Both presets preserve native depth samples, JPEG 95 and full RGB
updates. A 3840-pixel transport ceiling does not upscale a smaller sensor image.
Aligned mesh color follows the depth grid; RGB camera mode shows the original
RGB image at its separate sensor resolution.

All three lab D435s advertise and resolve the 1280×720 depth + 1920×1080 RGB
pair at 30 Hz. Their largest depth output at any frame rate is also 1280×720;
reducing the target below 30 Hz does not expose an additional depth resolution.

Switching briefly restarts the cameras and affects every viewer sharing that
capture owner. The request uses the existing authenticated view-settings
WebSocket even with head tracking disabled. It waits for active calibration
to finish and never changes saved transforms or the scene. Acquisition sizes,
profile and actual capture rates appear in the private `/diagnostics` response.
These are capture targets, not a guarantee that USB, encoding, network delivery,
decoding or rendering sustain their nominal rate.

The earlier live three-camera dropdown check confirmed both maximum native mode pairs, with
stride 1 and fresh frames from every sensor. The public Low latency viewer
reported 34–35 fresh FPS/camera in one boundary sample; Quality BlendPCR at the
larger depth grid reported 17–19 FPS/camera (about 37 ms codec plus 5 ms upload
submission). These are short diagnostic samples, not sustained guarantees or
an identical-load comparison. Quality capture itself varied around 15–25 FPS
in the sampled host window. The larger native mode therefore still needs
capture/encoding and client decode profiling to sustain 30 fresh FPS. Saved
scene and calibration hashes were unchanged after both switches. Those figures
describe the maximum 1280×720 Quality depth grid.

During a separate test with 848×480 recommended depth, a 10-second loopback measurement
delivered about 17–19 fresh encoded FPS per camera; Low latency delivered
44–46 FPS. These are host-to-loopback measurements, not public-browser FPS.
A separate six-second raw SDK check reached 29.5–29.7 and 59.5–59.7 FPS on
all three cameras, respectively. Capture, alignment and encoding therefore
still fall short of the sensor targets under the complete application load.
Changing presets does not by itself establish sustained 30/60 FPS delivery.
That recommended-depth measurement is historical and does not describe the
current maximum-resolution Quality preset.

Native BGR/RGBA, depth packing and lossless depth prediction use SIMD with scalar fallbacks. Equivalence
tests cover every native 16-bit value, float rounding boundaries, non-finite
inputs and tight row buffers. Original RGB bytes use reusable owned storage;
the full-resolution RGBA image is materialized only when RGB view/calibration
requests it. An SDK frame is not retained between captures. The private
diagnostics report sensor cadence, exposure availability and capture-stage
timings without materializing that image.

The vendor GPU alignment path was evaluated separately on identical captured
frames. It took about 16–17 ms including CPU readback, versus 4–6 ms for CPU
alignment, and changed color samples. It was not enabled. A range-pruning
experiment also failed to improve delivered throughput and was removed.

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

A smooth false slope can have perfect local neighbour support. Unified mesh
and BlendPCR therefore also check coherent surfaces against a substantially
better-resolved camera's measured free space. Wider confidence and calibration
margins protect valid detail; a nearer occluder, blind camera, stale camera or
coarser view cannot erase the surface. No replacement geometry is invented.

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

## RGB coverage and viewer navigation

SDK color aligned to depth has black padding outside the RGB sensor's field of
view. JPEG does not retain its alpha channel. The native stream now includes
the undistorted RGB intrinsics and depth-to-RGB extrinsics when available; the
GPU derives color coverage from this calibration instead of pixel brightness.
All measured depth remains available. Shared color and BlendPCR exclude absent
RGB observations, use an agreeing camera's color where available, and retain
neutral geometry otherwise. Real black pixels inside the RGB field remain
valid. Providers without supported projection metadata retain their existing
behavior. The standalone RGB view shows the original sensor image and exposes
a camera selector only while that view is active and multiple cameras exist.

Capture-profile reconnects retain the picked world pivot, camera frame, viewing
angles, dolly and head-tracking baseline. Pan moves the pivot with the same
camera motion as before; a new double-click or middle-click pivot replaces the
previous pan translation. Zoom distance and height behavior are unchanged.
