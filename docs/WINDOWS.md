# Windows

The project includes a Windows x86-64 GDExtension and `realsense2.dll`. Install the Intel RealSense runtime/driver and Python 3.10–3.12.

For source development, open the repository in Godot 4.6+ and use the same setup dock as Linux. A CI artifact named `robot-teleop-vision-windows` contains an exported Godot runtime plus the Python/server files. In its root:

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\scripts\windows_setup.ps1
.\.venv\Scripts\robot-teleop.exe doctor
.\.venv\Scripts\robot-teleop.exe start
```

The packaged configuration sets `godot.packaged_runtime = true`, so the supervisor launches the exported app without `--path`. Windows CI validates Python/protocol/calibration logic and creates the package without requiring USB hardware. RealSense/robot hardware validation remains a local site responsibility.

To rebuild the native extension, clone `godot-cpp` into `native/realsense_shared_memory/godot-cpp`, install SCons, set `REALSENSE2_SDK_DIR`, and run:

```powershell
cd native\realsense_shared_memory
scons platform=windows target=template_release
```
