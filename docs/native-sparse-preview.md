# Native sparse camera preview

The 20k/40k options read owned depth and original RGB snapshots from the existing
capture process before JPEG or full-depth compression. They do not open another
RealSense device or alter sensor settings. Each camera has an independent worker
and a single replaceable pending packet; a slow camera does not form a batch
barrier for the other cameras.

The capture publisher adds a separate `RSNP01` shared-memory mapping with a
256-byte header, native float depth and original BGR/BGRA bytes. Its sequence
counter brackets each write. The reader copies to owned storage and verifies
the sequence before using any pixels. Camera intrinsics, RGB projection and
world transforms travel with that acquisition. Existing `RSPG01` mappings and
the full-quality recording endpoints remain intact.

Only retained workspace samples enter the preview budget. The wire payload is
the existing losslessly compressed nine-byte native-sample representation:
pixel index, millimetre depth and RGB8. Native kernels avoid large intermediate
NumPy arrays; the Python reference remains available as a fallback. Build the
optional Linux kernels with:

```sh
python tools/build_native_sparse_kernels.py
```

The browser badge says **camera buffers** for this path and **host sampled** for
the older compressed-image fallback. Full detail restores the original RGB-D
stream. Robot control is separate. Measured joint history is attached without
waiting for a future joint sample; calibration is neither fitted nor rewritten.

## Validation boundary

The October 27, 2026 deployment passed five native-buffer/kernel tests, six
legacy sparse/control tests, and 29 browser rendering checks. It also verified
the separate full-resolution source: 1280x720 depth and 1920x1080 original RGB
for all three cameras. Nine protected saved calibration/configuration files
matched their pre-deployment hashes. The editor and follower were not restarted.

An isolated read-only live Cloudflare browser test verified 20k, 40k, Full detail
and 20k again, with no browser errors or robot commands. Its short sample windows
measured 8.5–9 updates/camera/s in the initial 20k phase, 10–10.5 at 40k, and
20.5–21.7 in the final warmed 20k phase. This was a separate test browser, not the
user's existing tab. These results do **not** establish steady 30 updates/camera/s.
The capture pipeline itself was publishing roughly 22–24 acquisitions/s during
the investigation, despite the sensors' nominal 30 FPS profile.

Detailed deployment, calibration hashes, functional test results and timing
samples are retained in `.teleop/native-sparse-20261027/`.


## Follow-up: live preset and sparse transport throughput

The capture owner's startup profile previously overrode the editor's live
capture-control settings. A Low latency viewer could therefore request 60 FPS
while the cameras remained in highres30. Live camera settings now take precedence,
and diagnostics report the controlled profile rather than the startup fallback.
Quality and Low latency were both verified against actual camera dimensions.
No extra resolution reduction was introduced; these remain the existing presets.

Sparse depth conversion uses an optional AVX2 path with double-precision
arithmetic and exact scalar parity, retaining the portable fallback. The native
snapshot decoder and GPU now retain the compact 9-byte point layout. New viewers
negotiate `point_predictor=delta_v1`: independent within-frame integer differences
compress to roughly half the former packet size and reconstruct the identical
indices, millimetre depths and RGB bytes. Older viewers retain the original wire
format. Full-detail and recording endpoints are unchanged.

The old native acknowledgment window allowed little headroom above the minimum
observed round trip. Its repeated exhaustion paused transmission despite fresh
camera buffers. Negotiated delta streams now allow two minimum-RTT windows,
bounded to 96 outstanding packets, while retaining only the newest unsent packet
per camera. This does not queue an unbounded frame history.

A separate live Edge UI over Cloudflare, using Low latency and the same laptop,
measured 30 seconds per option in twelve 2.5-second windows:

| Points/camera | Mean fresh updates/camera/s | Lowest window | Viewer drawing |
| --- | ---: | ---: | ---: |
|20k|41.0|31.6|55.9+FPS|
|40k|42.2|33.1|56.2+FPS|

These are actual UI camera-upload counts in a separate read-only test browser,
not a guarantee for every network condition or the user's existing tab.
Sensor rates were about 59.5 FPS; capture exported about 52�58 FPS. Quality retains
its 30 FPS sensor profile and its separate capture/export limit.

Validation: 41 browser checks, 7 native stream/kernel tests, and 7 Godot preset/
diagnostic assertions passed. Integer predictor round trips and depth conversion
preserved exact values. Nine protected saved configuration/calibration files
were checked; canonical arm registration remained unchanged. The editor and
follower were not restarted; recording was idle during managed service reloads.
Evidence and rollback files: `.teleop/sparse-throughput-20261027/`.
