# Windows first-run and validation

Use 64-bit Windows 10 or 11. A source checkout is the preferred onboarding path; it exercises the same repository a new robot integrator will receive.

## Install prerequisites

Install these before cloning:

- Git for Windows.
- Python 3.11 x64, including the `py` launcher and **Add Python to PATH** option.
- Godot 4.6.3 or newer x64, available as `godot` or `godot4` on `PATH`. If you use a custom location, run setup once to create `config/local.toml`, set `godot.executable` there, and rerun setup.
- Intel RealSense SDK 2.0/runtime and the current camera firmware/driver.
- `cloudflared` on `PATH` for the default free temporary public URL.

The repository already contains the Windows x86-64 GDExtension, `realsense2.dll`, and `realsense_shared_memory.dll`. Building native code is not part of first-run setup.

## Clean-clone setup

Open PowerShell. Do not copy `config/local.toml` from another computer: it contains site identities and hardware-specific settings.

```powershell
git clone https://github.com/Jetpackjules/robot-teleop-vision.git
cd robot-teleop-vision
Set-ExecutionPolicy -Scope Process Bypass
.\scripts\windows_setup.ps1
.\.venv\Scripts\robot-teleop.exe doctor
.\.venv\Scripts\robot-teleop.exe devices --json
.\.venv\Scripts\robot-teleop.exe modules --json
```

The setup script creates ignored `config/local.toml` in safe vision-only mode, assigns a unique local operator password, and performs the one-time headless Godot editor import required to register the bundled GDExtension. It stops before runtime startup if `.godot/extension_list.cfg` does not list the RealSense extension. A clean clone cannot command a robot.

Start the stack in that PowerShell window:

```powershell
.\.venv\Scripts\robot-teleop.exe start
```

Wait for both a local URL and a verified `https://...trycloudflare.com/controller.html` URL. In another PowerShell window:

```powershell
.\.venv\Scripts\robot-teleop.exe status
.\.venv\Scripts\robot-teleop.exe open
```

The newest browser page becomes the sole stream/control owner. Opening another page should immediately supersede the older page.

## Godot editor path

Import this checkout's `project.godot`. The project title is **Robot Teleop Vision**. The **Teleop Setup** dock can create config, run Doctor, start the same supervised runtime, open the website, and safely stop it. Calibration, motion, and view controls intentionally remain in the website.

Use Forward+ unless that PC cannot support it. Compatibility is supported, but Forward+ is the normal desktop renderer. The native RealSense nodes appear only when a camera and the Intel runtime are available.

## Native RealSense extension verification

The following warning is not an expected camera or USB warning:

```text
RealSenseSharedMemoryPointCloud native extension is unavailable
```

It means Godot did not load the point-cloud renderer, so the browser can remain connected while showing no point cloud. From the repository root, rerun setup and Doctor before investigating camera profiles:

```powershell
.\scripts\windows_setup.ps1
Get-Content .godot\extension_list.cfg
.\.venv\Scripts\robot-teleop.exe doctor
```

The extension list must contain:

```text
res://native/realsense_shared_memory/realsense_shared_memory.gdextension
```

If setup cannot create that entry, preserve its complete Godot output. Do not treat a successful tunnel, web page, or camera USB negotiation as evidence that the native renderer loaded.

## Browser acceptance checklist

Validate this on the temporary Cloudflare URL, not only on `localhost`:

1. The page connects and shows a live 3D point cloud.
2. Left-drag orbits, right-drag pans, and the wheel zooms without head tracking.
3. In **Advanced Setup**, turn **Head tracking** off and confirm the webcam is released while mouse navigation still works.
4. Switch **View** between **3D point cloud** and **RGB camera** and confirm both remain live.
5. Turn **Full RGB frame updates** on and off; both modes must keep moving RGB content current.
6. Confirm the flat support surface is level and adjustable. A robot envelope appears only for a module that declares one; it is a visual guide, not a core motor stop.
7. Move to a useful point-cloud camera pose, choose **Save Current Setup as Default**, then open a new browser. It should start with that view and the saved stable settings.
8. Confirm the simple **Controller** panel contains daily controls only. Calibration, diagnostics, camera defaults, and tuning live behind **Advanced Setup**.
9. Leave robot hardware disabled for this onboarding test unless a separately reviewed module/site procedure explicitly enables it.

## Codex issue-report checklist

Ask Codex to read this file and run the following read-only verification. It should not edit code first, enable motion, calibrate, or copy configuration from another machine.

```powershell
git status --short
git log -1 --oneline
.\.venv\Scripts\python.exe -m compileall -q robot_teleop robot_modules tools tests
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\robot-teleop.exe doctor
.\.venv\Scripts\robot-teleop.exe devices --json
.\.venv\Scripts\robot-teleop.exe modules --json
.\.venv\Scripts\robot-teleop.exe status
```

Its report should include:

- Windows version, Python version, Godot version, RealSense model/serial, and whether `cloudflared` is found.
- The checked-out commit and whether the worktree was clean before testing.
- Each failing command with its complete error text.
- Local and public connection result, point-cloud/RGB result, observed FPS/latency, and the browser used.
- Which numbered browser acceptance checks passed or failed.
- Relevant supervisor output around a failure, with passwords/tokens redacted.

It can also run `robot-teleop support-bundle` after reproducing the problem and attach the resulting sanitized zip. Windows has no `journald`, so the bundle records that journal history was unavailable while retaining the other checks. That is enough evidence to reproduce an onboarding problem without exposing local credentials.

## Packaged runtime

A release or manually dispatched CI run may publish `robot-teleop-vision-windows`. Extract it, run the same `scripts/windows_setup.ps1`, then use the same `doctor`, `start`, `status`, and browser checks. When `RobotTeleopVision.exe` is present, the setup script automatically selects packaged-runtime mode; a source clone remains in Godot-project mode.

## Native rebuild (maintainers only)

The committed Windows binaries include camera calibration and the markerless
model. For an existing source checkout, close the Godot editor and stop the
launcher, run `git pull --ff-only`, then reopen Godot/restart the launcher. Do
not leave Godot open during the update: Windows can lock the old DLL. No native
build, extra model download, or Python package installation is needed for this
calibration update. Site profiles and saved calibrations remain local.

To rebuild, use MSVC x64 and Godot 4.5-compatible `godot-cpp` sources in
`native/realsense_shared_memory/godot-cpp`. Install SCons and set
`REALSENSE2_SDK_DIR` to the RealSense C++ SDK, `OPENCV_SDK_DIR` to the OpenCV
4.13 SDK (headers plus the MSVC `opencv_world4130.lib`), and
`ONNXRUNTIME_INCLUDE_DIR` to the matching ONNX Runtime C headers. Then run:

```powershell
cd native\realsense_shared_memory
scons platform=windows target=template_release arch=x86_64 build_profile=build_profile.json require_calibration=yes -j8
```

Windows builds now reject missing calibration dependencies instead of silently
shipping a stub. `require_calibration=no` is an explicit development-only opt-out.
When updating dependencies, replace the matching runtime DLLs in `bin/` as well;
do not mix the MSVC and MinGW OpenCV ABIs. See the native `DISTRIBUTION.md` for
the bundled files and model provenance.

Validate the actual native model execution without cameras or robot movement:

```powershell
godot --headless --path . --script tests/godot/markerless_runtime_probe.gd
```

The Windows CI job runs this same test, including rejection of a missing model.
Doctor also executes the markerless runtime check. Physical camera coverage and
the quality of an alignment still need checking on the robot PC.
