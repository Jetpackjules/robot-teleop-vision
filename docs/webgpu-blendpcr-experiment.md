# Unified WebGPU Quality BlendPCR experiment

This prototype keeps native quality flags, robot masks, self/peer visibility
history, primitive metadata and original-RGB BlendPCR composition on the same
WebGPU device. It retains the measured samples, accepted native triangles,
RGB resolution and output resolution. There is no adaptive geometry or lossy
compression. Full lossless acceptance is still blocked by the near-plane
depth case below.

**It is not imported or enabled by the production renderer.** The existing
WebGL renderer remains the default. `experimental_webgpu_blendpcr_adapter.js`
requires explicit installation and a decoded-color upload hook before the
normal camera uploader closes or deletes that data. Tests inject those hooks
only into a separate browser context. There is no persistent host setting.

## Arithmetic and presentation

WGSL's ordinary arithmetic does not reproduce this machine's ANGLE/D3D shader
rounding. Matching the existing occlusion edges requires compensated binary32
multiply-add operations, identical projection/depth arithmetic and independent
current/previous observations. Exact color also requires matching the compiler's
normal-vector translation cancellation, gamma interpolation and fused weight
accumulation. A one-byte RGB difference fails the gate.

Frame-wide clip matrices and camera-local viewer origins are computed once on
the GPU with the same operations, rather than once per output fragment. Camera
metadata and projection constants upload only when their values change.

The existing WebGL CAD models, guides and workspace editor remain in place.
Only finished color and depth cross APIs. They share one opaque RGBA8 surface:
RGB bytes in its left half and exact 24-bit depth bytes in its right half.
This is lossless presentation packing, not a change to the RGB-D wire format.
The full-size presentation comparison checks both color bytes and sampled
depth bits. Three destination textures avoid overwriting the previous draw.

This final cross-API handoff and duplicate camera uploads remain potential
costs. A faster isolated WebGPU stage is not evidence of a faster UI.

## Gates and fallback

The adapter first runs WebGL and native composition together. It waits for
decoded RGB and two observed captures from every source, then compares the
same complete color/depth frame with zero tolerance. The depth-readback shader
is prepared beforehand so asynchronous compilation cannot substitute a newer
capture. A failed gate never activates native presentation.

Disabling the experiment, leaving the supported view, clearing targets or
disposing the adapter invalidates pending verification and rebuilds WebGL
history/uploads. Camera-layout changes, GPU errors and device loss fail back
to WebGL. Larger canvases that exceed packed-texture limits are rejected.

## Verification

`tests/webgpu_blendpcr_browser.cjs` runs the actual production WebGL shaders
against the native implementation. It compares quality bytes, primitive IDs,
all twelve canonical visibility/history maps, composed color/depth and the
final presentation. `TEST_SEQUENCE=1` adds viewpoint changes, near-plane
movement, independent source updates, rotated crops, table bounds, masks,
gray/RGB transitions, stale/empty sources and resizing.

The strict extended gate currently **fails the near-plane depth case**. Keep
that failure visible; do not increase the tolerance or enable this backend.

```powershell
$env:TEST_BROWSER_CHANNEL='msedge'
$env:TEST_SEQUENCE='1'
node tests/webgpu_blendpcr_browser.cjs
```

Set `PLAYWRIGHT_MODULE` when using a bundled Playwright runtime. To replay a
saved independent RGB-D packet, set `CAPTURE_PACKET` to its absolute path.
`VIEW_WIDTH=1972` and `VIEW_HEIGHT=1290` reproduce the tested full UI resolution.
`RESULT_PATH` saves the complete comparison. `GPU_NATIVE_MATH=1` is a diagnostic
negative control, not a supported lossless mode.

Live tests use a separate visible Edge window and the full controller, with
three Quality cameras, seven CAD models, the active table editor and a
1972x1290 canvas. Robot-control sockets are intercepted and settings writes
blocked. GeForce NOW is left untouched; a running container alone does not
establish active gaming load. Reported UI-counter samples are short windows,
not a guaranteed per-frame floor in the operator's existing tab.

Local raw evidence and scripts are retained outside OneDrive in
`C:/Users/jetpa/AppData/Local/Temp/teleop-webgpu-unified-20261027/`.

## 2026-10-27 results and decision

The current three-camera capture at 1972x1290 matched all quality bytes,
primitive IDs and twelve visibility/history maps throughout the fourteen-case
sequence. Color matched in every case. Composed depth matched in thirteen
cases; the newly added near-plane viewpoint exposed 11,162 differing sampled
depth values. Ordinary-view finished presentation matched color and depth
exactly. Near-plane final presentation has not been established. The live
activation gate also passed with zero color/depth differences.

The final paired visible-UI comparison held the table editor at about 4.59%
retained measured points and moved the viewpoint in each phase:

| UI counter | WebGL before | Unified WebGPU | WebGL after |
| --- | ---: | ---: | ---: |
| Median FPS | 43.15 | 17.31 | 43.85 |
| Lowest sampled FPS | 35.35 | 13.45 | 30.53 |
| Readings below 30 FPS | 0/32 | 29/32 | 0/32 |

All native samples confirmed active native rendering, with no GPU or browser
errors. No robot commands were sent. The test window was closed afterward.
GeForce NOW was not stopped. These results apply to this workload and load,
not every operator session or sustained gaming workload.

Rolling CPU measurements near the end of the native phase were about 2.81 ms
for native update, 3.48 ms for native render including that update, 1.04 ms per
decoded RGB upload and 0.16 ms for presentation submission. These overlap and
are not GPU timestamps or complete frame latency. They do not support blaming
the remaining regression on CPU time spent in the final image handoff alone.
Native visibility still draws full RGB visibility maps; deployed WebGL can
restrict work to crop sampling regions while replaying history on expansion.

**Do not enable this prototype.** It fails both the full fidelity gate and the
steady-30-FPS performance gate. Keep the working WebGL path. Further work must
resolve near-plane depth semantics and profile/port conservative visibility
region culling with occluders and history intact. The live benchmark rejects
the current rewrite; it is not evidence for a promised speedup.
