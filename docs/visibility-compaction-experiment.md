# Native visibility compaction experiment — 2026-10-27

This is an isolated prototype, not an enabled renderer change. It does not yet
meet the requirement to reproduce the production WebGL visibility maps.
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

An alternating comparison in the same browser measured:

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

The full captured scene does **not** match production WebGL bit for bit:
3,075,407 depth texels differ, mostly by small floating-point rounding amounts.
Five texels differ in covered/uncovered status, and some overlapping-surface
edge texels select different depths. These cannot all be dismissed as harmless
rounding. Changing to a packed depth/stencil format, invariant positions, or a
different matrix summation order did not eliminate the discrepancy.

The browser harness exposes `productionParityPassed` and can enforce that
gate with `REQUIRE_PRODUCTION_PARITY=1`. Until it passes, this prototype must
remain outside the production import graph.

A separate one-pass WebGL experiment preserved every tested depth and final
image byte, but measured roughly 27 ms with either path and was rejected.
Its source and results are retained with the local experiment artifacts.

## Reproducing

Set `PLAYWRIGHT_MODULE` if Playwright is supplied by a bundled runtime rather
than the local Node module search path. Then run the generated synthetic case:

```powershell
$env:TEST_BROWSER_CHANNEL='msedge'
$env:SKIP_BENCHMARK='1'
$env:REQUIRE_PRODUCTION_PARITY='1'
node tests/gpu_visibility_compaction_browser.cjs
```

`TEST_GEOMETRY` selects `holes` (default), `dense`, or `empty`. For a saved
Quality packet, set `CAPTURE_PACKET` to its absolute path. Set `RESULT_PATH`
to save the complete results JSON. Set `SKIP_BENCHMARK=0` for timings.
The full-capture production parity gate currently fails as described above;
omitting `REQUIRE_PRODUCTION_PARITY` permits diagnostic measurements while
still enforcing exact index membership and within-WebGPU depth parity.

Evidence is retained outside OneDrive at
`C:/Users/jetpa/AppData/Local/Temp/teleop-compaction-20261027/`, including
`visibility-stats.json`, `production-comparison.json`, and `single-pass.json`.
No robot commands were sent and no shared services were restarted.
