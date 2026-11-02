# Linux RGB capture recovery

The host uses librealsense 2.59.1. Its V4L2 video/metadata synchronizer can return
a matched frame from an earlier polling iteration. The original upload code
reuses the current poll's `buffers_mgr`, whose descriptors may identify different
buffers. The release callback then fails to return the frame's actual buffers
to the kernel. Under an encoding workload, RGB delivery can stop while the
pipeline continues publishing depth with its last RGB exposure.

`librealsense-2.59.1-v4l2-buffer-owner.patch` gives the matched frame a separate
buffer manager. The current poll keeps its own queued/rejected buffer guards.
Camera profiles, pixel processing, alignment, and compression are unchanged.

Build a private SDK library on the Linux host:

```sh
python tools/build_realsense_v4l2_runtime.py
```

This pins upstream revision `625279aa1fe903528a818f48746ee47c1b5fefd3`, applies
the patch, and writes the library plus a hash manifest under
`.local_deps/realsense-v4l2-fixed/lib`. CMake, a compiler, and the normal SDK
development dependencies are needed. `--cmake-arg=-DNAME=VALUE` can supply
private dependency paths. No system library is replaced.

Rebuild the native extension after installing the SDK correction. Its Linux
runtime search path prefers that private library directory; a normal launch
then uses it without environment overrides. Existing extension builds can
instead prepend the absolute directory to `LD_LIBRARY_PATH` for the camera
owner and its Godot consumer. Existing processes must be reloaded to use a new
SDK. Preserve camera settings and recording/calibration
state. A stopped publisher can leave POSIX shared-memory mappings behind;
recreate its exact mappings, or use a fresh prefix for both publisher and reader.

The native camera diagnostics separately report `rgb_frame_number`,
`rgb_sensor_timestamp_ms`, and `rgb_frame_age_ms`. Depth publication sequence
numbers do not prove that RGB is fresh. A five-second RGB stall now triggers
camera-local source recreation, except while acquisition is paused for another
camera user. The SDK correction prevents the buffer-loss cause; the watchdog
also handles subsequent independent capture failures.
