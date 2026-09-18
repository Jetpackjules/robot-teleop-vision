# Windows calibration runtime

The Windows extension is built with MSVC x64, native RealSense, OpenCV 4.13,
and the ONNX Runtime C API. Runtime dependencies and the model are committed so
an existing source installation needs only a pull with Godot closed, then restart.
The markerless solver uses the CPU execution provider; CUDA is not required.

- `bin/realsense_shared_memory.dll`: this project's rebuilt extension.
- `bin/realsense2.dll`: Intel RealSense runtime, matching the build SDK.
- `bin/opencv_world4130.dll`: official OpenCV 4.13.0 Windows MSVC runtime.
  Source: https://github.com/opencv/opencv/releases/tag/4.13.0
- `bin/onnxruntime.dll`, `bin/onnxruntime_providers_shared.dll`: ONNX Runtime CPU
  runtime matching C API version 28. Source: https://github.com/microsoft/onnxruntime
- `bin/{concrt140,msvcp140,msvcp140_1,vcruntime140,vcruntime140_1}.dll`:
  app-local Microsoft Visual C++ 2022 redistributable runtime from the installed
  Visual Studio 2022 redistributable directory. Redistribution terms:
  https://learn.microsoft.com/visualstudio/releases/2022/redistribution
- `models/superpoint_lightglue_pipeline.onnx`: unmodified official LightGlue-ONNX
  v2.0 release, https://github.com/fabio-sim/LightGlue-ONNX/releases/tag/v2.0
  SHA-256: `228994cea8c010146fa2aef933baa3ffaa4bcdc522bc8aa560087fcff8134526`.
  Inputs: `images` (two grayscale images); outputs: `keypoints`, `matches`,
  `mscores`. The solver supplies 544 by 960 images and expects 1024 keypoints.

Licenses and notices are in `licenses/` and `models/LIGHTGLUE_ONNX_LICENSE`.
Run `tests/godot/markerless_runtime_probe.gd` after any native dependency update;
it runs inference on synthetic images and does not connect to hardware.
