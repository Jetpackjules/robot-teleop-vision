# Windows first-run and validation

Use 64-bit Windows 10 or 11. A source checkout is the preferred onboarding path; it exercises the same repository a new robot integrator will receive.

## Install prerequisites

Install these before cloning:

- Git for Windows.
- Python 3.11 x64, including the `py` launcher and **Add Python to PATH** option.
- Godot 4.6.3 or newer x64.
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

The setup script creates ignored `config/local.toml` in safe vision-only mode and assigns a unique local operator password. A clean clone cannot command a robot.

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

To rebuild the extension, clone `godot-cpp` into `native/realsense_shared_memory/godot-cpp`, install SCons, set `REALSENSE2_SDK_DIR`, and run:

```powershell
cd native\realsense_shared_memory
scons platform=windows target=template_release
```
