# Compressed mesh preview

View → Point detail → **Balanced mesh (compressed)** retains the selected 3D
renderer, including BlendPCR, with a regular depth grid sampled every two pixels
in each dimension. Retained millimetre depths and calibrated rays are unchanged;
three quarters of source depth samples are omitted. Camera RGB uses JPEG quality
65 with 4:2:0 chroma. This is a lossy preview option.

The option is remembered in this browser along with the existing point-detail
choice. Full detail and the native 20k/40k paths remain available. Recording,
capture profiles and calibration files are independent of this preview encoder.

Per-camera workers generate packets directly from owned camera-buffer snapshots.
Independent two-lane camera sockets carry bounded, latest-frame packets. Depth
dimensions remain stable as workspace edge samples change, avoiding repeated
GPU rig allocation and shader compilation. Linux uses persistent processes;
the threaded implementation uses the same encoder as its fallback.

In a 45-second Low latency test on the laptop's actual GPU and the live UI,
fresh applied rates were 36–44 updates/s per camera with about 60 viewer FPS.
Every five-second window exceeded 34 updates/s; maximum individual camera gaps
were 85–87 ms. This is a short controlled preview test, not a guarantee of 60 new
camera frames/s or a Quality-preset benchmark. A subsequent 30-second integrated
UI sample reached 38–45 fresh updates/s; after several mode switches, a shorter
repeat dipped to 25 on one camera and had a 430 ms maximum gap. The option
improves throughput, but does not eliminate all delivery stalls or guarantee
steady 30 on every camera. The Quality capture profile still
limits how many unique frames can be produced.

Worker decoding now retries transient failures with bounded backoff. An explicit
renderer shutdown cannot revive the decoder; repeated failures retain the
compatibility path. No depth or color values change during that recovery.

Queued view settings also retain workspace bounds across partial point-detail
updates. This prevents a fresh reload from accidentally sampling the entire
camera scene while the viewer itself displays only the workspace. The queue
retains whitelisted viewing fields only and isolates different client IDs.
