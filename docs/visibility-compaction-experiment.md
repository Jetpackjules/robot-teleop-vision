# Native visibility compaction experiment — 2026-10-27

This is an isolated prototype, not an enabled renderer change. The new exact
arithmetic mode reproduces the tested production WebGL visibility maps on
this laptop. Live UI performance remains a separate gate.
This prototype does not change the live controller, capture service,
calibration or stream. The separate WebGL region optimization described in
`camera-stream-performance.md` passed its lossless gate.

## What was implemented

`web/experimental_gpu_visibility.js` consumes the existing native triangle
validity flags. A WebGPU compute pass builds one compact index buffer per
depth camera. A workgroup prefix scan reserves one contiguous output range
per group, avoiding an atomic increment for every triangle. The four-column,
eight-row traversal retains locality in the vertex cache.

Each accepted triangle keeps its exact three native grid vertices. The
resulting buffer is reused across RGB cameras using indexed indirect draws.
The compact rendering path needs one triangle pass instead of two passes
that carry separate validity flags for the two halves of each native cell.

For this benchmark, the production GPU quality flags are read back once to
establish identical inputs. No index readback occurs in the timed path.
Readbacks of indices and depth maps are used separately for validation.
A production port would need equivalent quality generation and compatible
downstream rendering without per-frame readbacks between WebGL and WebGPU.

## Findings on the laptop

The test used the saved three-camera Quality capture, with 1280×720 depth and
1920×1080 RGB per camera. This is a frozen input for GPU benchmarking, not a
live camera or UI FPS measurement. GeForce NOW remained running. The adapter
reported Intel `gen-12lp`, with `isFallbackAdapter: false`.

The capture contains 5,517,606 possible native triangles, of which 4,478,212
pass the existing validity checks. Depending on the camera, about 18–19% are
rejected. Each accepted triangle appeared exactly once in the compact output;
there were no invalid indices, duplicate triangles or missing triangles.

Disabling rasterization left the production visibility pass at roughly
37–39 ms in an initial diagnostic. That points to vertex processing as the
dominant cost for this input.

The earlier, non-lossless experiment in the same browser measured:

| GPU visibility stage | Median time |
| --- | ---: |
| Production WebGL, including its current culling and caching | 36.40 ms |
| Compacted WebGPU, including compute and all RGB projections | 24.94 ms |

That is a 31.5% reduction for this GPU stage. It excludes camera encoding,
network transport, decoding, uploads, BlendPCR composition and the arm overlay.
It does not establish a 31.5% UI FPS gain or a sustained 30 FPS result.
The unoptimized WebGPU reference is substantially slower than production
WebGL and must not be used as the headline speedup baseline.

## Lossless gate

Compacted and uncompacted WebGPU produced bit-identical depth across all
12,441,600 tested depth texels. The synthetic partial-tile case also matched
production WebGL exactly.

Before the arithmetic fix, the full captured scene did **not** match WebGL:
3,075,407 depth texels differ, mostly by small floating-point rounding amounts.
Five texels differ in covered/uncovered status, and some overlapping-surface
edge texels select different depths. These cannot all be dismissed as harmless
rounding. Changing to a packed depth/stencil format, invariant positions, or a
different matrix summation order did not eliminate the discrepancy.

Feeding WebGPU the exact vertex positions captured from WebGL transform
feedback eliminated every difference. The cause was vertex matrix arithmetic,
rather than triangle membership or raster rules. On this laptop's backend,
production uses two fused, single-rounding multiply-add steps and a translation
addition. WGSL [`fma`](https://www.w3.org/TR/WGSL/#fma-builtin) can expand to
separately rounded multiply/add operations,
so merely spelling out that order still failed.

The new default `webgl_exact` mode uses an error-free product, compensated
sums and explicit rounding/tie handling for finite normal-range projection
arithmetic. It now has **zero differing depth texels across all 12,441,600
texels**, including the formerly different coverage edges. A BigInt oracle
also validates 60,252 binary32 cases, including cancellation, boundaries and
signed zeros. This is not a general implementation of every IEEE exceptional
or subnormal case, nor a portability guarantee for other drivers.

The browser harness exposes `productionParityPassed` and can enforce that
gate with `REQUIRE_PRODUCTION_PARITY=1`. That gate passes for this capture in
`webgl_exact` mode. Other target backends still require their own comparison.

A separate one-pass WebGL experiment preserved every tested depth and final
image byte, but measured roughly 27 ms with either path and was rejected.
Its source and results are retained with the local experiment artifacts.

## Exact-mode integration follow-up

A new alternating comparison measured **25.49 ms production WebGL versus
22.09 ms compacted WebGPU**, including compute and the exact arithmetic:
about a 13.3% stage reduction. Contention differs from the earlier timing, so
the two experiments cannot be compared as an additional gain. Neither stage
comparison establishes a live UI FPS improvement.

`web/experimental_gpu_visibility_bridge.js` is an opt-in BlendPCR integration
harness, outside the production import graph. It copies the original WebGL
quality flags to WebGPU and restores exact 24-bit depth through opaque canvas
textures into the existing self/peer maps. History rotation stays in WebGL.
The render path performs no JS GPU readback. It imports only dirty maps,
excludes stale sources, and retains WebGL on a small/unsupported viewport.

The bridge regression compares every rendered byte and every current/previous
depth texel: 316 checks passed on synthetic input and 316 on the full capture.
It covers asynchronous camera updates, crop changes, modeled
tables/dragging, gray transitions, empty sources, calibration changes,
freshness and fallback. It forbids `readPixels`, `getBufferSubData` and
`GPUBuffer.mapAsync` during candidate rendering. The exact normalization of
24-bit depth was checked against all 16,777,216 possible integers; approximate
shader reciprocal division had changed restored depth and was rejected.

The bridge currently uses full maps. Canvas interchange can impose synchronous
waits: a captured-input diagnostic measured roughly **40 ms CPU call time per
bridge rebuild versus 8 ms for WebGL**. WebGL GPU timers exclude the separate
WebGPU work and must not be presented as total bridge timing. This transfer
cost can erase the stage gain. Compare the complete UI against the deployed
crop-aware WebGL path before enabling the rewrite.

After the lab PC rebooted, its preview services were restored independently.
No capture or gateway restart was performed by this experiment. The paired
live tests used the restored three-camera session in an isolated production
controller: 1280×720 native depth, 1920×1080 RGB, seven arm models, active table
editor, approximately 4.8% retained points and a 1972×1290 canvas. Robot control
WebSockets were intercepted locally, and settings writes were blocked.

The first A–B–B–A sequence in headless Edge measured 41.78 FPS median and a
38.86 lowest sampled reading for deployed WebGL, versus 34.57 and 27.76 for
the exact GPU bridge (64 sampled UI counters per method).

A second A–B–A sequence in a separate **visible Edge window** measured:

| Viewer UI counter | Deployed WebGL | Exact GPU bridge |
| --- | ---: | ---: |
| Median FPS | 41.16 | 34.33 |
| 10th percentile FPS | 35.66 | 32.63 |
| Lowest sampled FPS | 33.41 | 27.35 |
| Readings below 30 FPS | 0 of 64 | 1 of 32 |
| Median decoded camera batches/s | 15.56 | 13.83 |
| Median reported codec time | 26.93 ms | 45.01 ms |

Both runs had no browser/submission errors and sent no robot commands to the
host. GeForce NOW was left untouched; active gaming load was not established.
These are short sampled UI-counter windows, not per-frame presentation times
or a guaranteed floor in the operator's existing tab, which was left untouched.
The temporary visible window was closed after the test.

**The bridge is a UI performance regression and remains disabled.** Exact
projection fixes the edge problem, but cross-API transfers/synchronization
erase the visibility-stage gain and reduce decode throughput. Keeping
visibility and BlendPCR composition in one WebGPU pipeline is the next
integration direction to investigate. The subsequent native implementation
and its separate live acceptance gates are documented in
[the unified WebGPU BlendPCR experiment](webgpu-blendpcr-experiment.md).

Two further WebGL variants were removed: GPU-written element buffers are
disallowed by WebGL, and a projected-vertex cache failed exact edge parity.

## Reproducing

Set `PLAYWRIGHT_MODULE` if Playwright is supplied by a bundled runtime rather
than the local Node module search path. Then run the generated synthetic case:

```powershell
$env:TEST_BROWSER_CHANNEL='msedge'
$env:SKIP_BENCHMARK='1'
$env:REQUIRE_PRODUCTION_PARITY='1'
node tests/gpu_visibility_compaction_browser.cjs
node tests/gpu_visibility_arithmetic_browser.cjs
node tests/experimental_gpu_visibility_bridge_browser.cjs
```

`TEST_GEOMETRY` selects `holes` (default), `dense`, or `empty`. For a saved
Quality packet, set `CAPTURE_PACKET` to its absolute path. Set `RESULT_PATH`
to save the complete results JSON. Set `SKIP_BENCHMARK=0` for timings.
The full-capture production parity gate now passes with the default
`MATRIX_MATH=webgl_exact` on the tested backend. The older
`native|pairwise|sequential|webgl` modes are diagnostic options and still fail
that gate. `PROFILE_REGIONS=1` enables bridge diagnostic timing.

Evidence is retained outside OneDrive at
`C:/Users/jetpa/AppData/Local/Temp/teleop-compaction-20261027/`, including
`precision.json`, `matrix-math.json`, `arithmetic-proof.json`,
`captured-compensated-timing.json`, `bridge-captured-final.json`, and the
earlier `production-comparison.json`. Live/visible results and screenshots
are in the adjacent `teleop-compaction-20261027-pass2` directory. No robot
commands were sent and the production renderer was not changed.
