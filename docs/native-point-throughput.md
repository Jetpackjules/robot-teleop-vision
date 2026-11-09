# Native point update throughput

The Low latency 20k/40k preview uses independent per-camera encoders and two
delivery connections per camera. New clients negotiate the reversible
`shuffle_delta_v2` point predictor: index differences, signed depth differences,
green differences and byte planes are compressed independently within each
capture. Decoding restores the original indices, millimetres and RGB8 bytes.
There is no extra point reduction, quantization or inter-frame reference.
Older clients can continue using `delta_v1`; All native points retains that
predictor. Capture profiles, recording settings and calibration are separate.

The workspace crop combines the existing six clipping planes, quad retention,
8x16 tiles and two-pixel halo in a native pass. Boundary fixtures and saved-camera
payloads verify equality with the prior path, including 20k, 40k and All points.

Fast native point connections share publication claims. Available connections
prefer the lane with the lower acknowledgement delay, also considering the age
of outstanding data before a delayed acknowledgement returns. Claims are
bounded to current publications, and late frames still cannot replace newer
captures. Quality mode and Balanced mesh retain their previous lane selection.

The browser keeps short native-point bursts for successive actual draws rather
than immediately replacing every earlier capture. Decode and upload queues keep
at most three captures per camera, discard stale backlog after 50 ms, and retain
the latest capture when a source stops. Named temporal references retain their
original serial path. The timing probe counts distinct captures submitted in
real render frames, separately from received and decoded packets.

Read-only live tests used the laptop's Intel Iris Xe GPU, a 2926x1640 drawing
buffer, three live cameras and unchanged Low latency capture. The original
latest-only path drew about 43-46 new captures/s per camera in a 20-second sample.
The final HTTP/2 candidate averaged about 54-56 over longer samples, with short
repeats around 56-57. A separate 90-second QUIC host-tunnel test averaged
55.2-55.8 on all three cameras. Drawing remained about 60 FPS. These samples do
not establish a 55 FPS minimum: network stalls still produced occasional gaps.
QUIC did not establish a sufficient improvement to change the production URL.

The lossless candidate is now active on the existing production URL. A
90-second production UI check measured 55.66, 55.61 and 50.84 distinct draws/s
for cameras 4294, 5787 and 6048, with 60.09 drawing FPS. The slowest camera still
received about 57 packets/s, but late publications were superseded. Its longest
draw gap was 517 ms; the other cameras reached 367 and 233 ms. A short return
to 20k measured 56.83, 56.08 and 53.09 draws/s. A steady 55-60 on every camera
has therefore not been established. All points and 40k switches, native point
counts and the remembered point setting also passed the deployed UI check.

An isolated unordered WebRTC trial over the existing free relay connected after
extending its initial no-data timeout and bounding SCTP's unsent queue. It
delivered only 25-26 fresh draws/s per camera in a 20-second sample. That route,
the timeout change and the experimental sender are not enabled in production.
One-connection delivery and aggressive late-connection recovery were also
slower and were rejected. The production route retains two connections per
camera. Final checks preserve all nine protected calibration/configuration
hashes, full-quality recording settings and the existing capture and follower
processes.

The 90-second queue sample measured median local receive-to-upload time of
20-23 ms and 95th-percentile time of 36-40 ms. Short burst preservation trades
some client queue delay for more distinct draws; it does not replay a long
backlog. Capture and Internet transit latency are additional to those figures.

Native crop checks, point predictor equality checks and worker lifecycle tests
pass. Five older workspace-crop tests also fail on the unchanged production
baseline (removed volume settings and an outdated gateway fixture); their
failure set is unchanged. Evidence, guarded staging and rollback helpers are
in `.teleop/near60-native-20261028/`.
