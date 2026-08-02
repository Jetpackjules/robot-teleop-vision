# Robot Teleop Vision

A reusable RealSense-to-browser teleoperation stack with a Godot 3D runtime. It provides a live RGB-D point cloud, multiple dynamically discovered RealSense cameras, optional browser head tracking, robot overlays, and guarded SO-101 control/calibration.

The browser is the single operator interface. The Godot dock is intentionally limited to setup, diagnostics, launch, and safe shutdown.

## First run

Requirements:

- Python 3.10–3.12
- Godot 4.6 or newer
- Intel RealSense SDK/runtime for local RealSense capture
- `cloudflared` only when a free temporary public URL is wanted

```bash
git clone https://github.com/Jetpackjules/robot-teleop-vision.git
cd robot-teleop-vision
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[realsense,calibration,test]"
robot-teleop init --example vision_only
robot-teleop doctor
robot-teleop start
```

In another terminal:

```bash
robot-teleop open
```

You can instead open this repository directly in Godot and use the **Teleop Setup** dock. It invokes the same commands and does not create a second operator UI.

### Opening the correct Godot project

The repository root is the Godot project root. From Godot's Project Manager, choose **Import**, select this repository's `project.godot`, then choose **Import & Edit**. From a terminal:

```bash
godot --editor --path /path/to/robot-teleop-vision
```

The project should appear as **Robot Teleop Vision** with the cyan-and-gold claw icon. If Godot still says **2.5D window V3**, that is the original research workspace, not this repository. The standalone project does not load the legacy views or old Windows experiments.

`start` stays in the foreground and supervises all child processes. Stop with `Ctrl+C` or `robot-teleop stop`; the launcher sends Robot Hold before terminating processes.

## Add an SO-101

Start with the robot powered off or physically supported. Create the local configuration:

```bash
robot-teleop init --example so101_realsense --force
```

Place the arm's own LeRobot-generated pair profile at `tools/so101_arm_pair.json`, set `robot.enabled = true` in `config/local.toml`, and point `robot.python` at a compatible LeRobot environment if it is not the active Python. Both files and all Godot calibration state are local-only and ignored by Git.

Run `robot-teleop doctor` before enabling motion. The browser's **Calibrate Full Arm** button performs fresh movement-derived base registration and then works outward through the joints. It commits only a complete validated result; a failed stage holds the arm and retains the last known-good registration.

## Public access without a paid service

Set `stack.public_mode = "quick"` in `config/local.toml`, install `cloudflared`, and provide a strong password:

```bash
export GODOT_REMOTE_PASSWORD='use-a-long-unique-password'
robot-teleop start
```

The verified `trycloudflare.com/controller.html` URL appears in `robot-teleop status`. Quick tunnels are transient and free. Do not expose arm control with the example password.

Only the newest browser page owns both streaming and control. Opening a newer page immediately Holds and disconnects an older controller, even if older tabs remain open.

## What is preserved from the research prototype

- Dynamic per-serial RealSense discovery; no apartment camera serials are embedded.
- Multiple RealSense renderers and serial-keyed local alignment state.
- Independent meshes by default; the poorer shader-merge mode is not the default.
- Recoverable temporal depth and RGB tile updates, persistent still-scene references, latest-only capture, parallel RGB/depth encoding, and a full-RGB-frame fallback toggle.
- Mouse orbit, pan, zoom, optional MediaPipe head tracking, white background, overlay style/occlusion controls, latency graphing, wrist feed, and full-arm calibration progress.
- Measured servo feedback, target ghosting, following-error/contact Hold, stale-telemetry freezing, and optional wrist visual correction.
- The incorrect custom wrist-camera attachment model is deliberately omitted from visualization and stock-mesh calibration.

## Repository map

| Path | Purpose |
| --- | --- |
| `robot_teleop/` | Launcher, configuration, discovery, adapters, health checks |
| `godot/` | Runtime scene, point-cloud rendering, overlay, calibration gateway |
| `web/` | Canonical browser operator UI and RGB-D renderer |
| `native/realsense_shared_memory/` | Cross-platform GDExtension source and runtime binaries |
| `tools/` | Local server, guarded SO-101 follower, calibration solvers |
| `config/examples/` | Safe hardware-disabled starting configurations |
| `tests/` | Hardware-free protocol, safety, calibration, and codec regression tests |

Read [Architecture](docs/ARCHITECTURE.md), [Safety](docs/SAFETY.md), [Calibration](docs/CALIBRATION.md), and [Adapters](docs/ADAPTERS.md) before deploying a new robot/site.

## Verify a change

```bash
python -m compileall -q robot_teleop tools tests
python -m pytest -q
godot --headless --editor --path . --quit
```

Linux has the local hardware verification gate. GitHub Actions runs hardware-free Python tests and a Godot script/import scan on every pull request, plus a Windows test job and Windows packaged-runtime artifact. Windows hardware is supported by the included GDExtension path but is not claimed as a CI hardware gate.

## Migrating this laptop's existing calibration

This is needed only for the original large Godot workspace:

```bash
robot-teleop migrate-state --from-project "2.5D window V3"
```

It copies only durable current registration/alignment files into Robot Teleop Vision's Godot user-data directory. Captures, debug RGB, logs, backups, and physical profiles are never copied into the repository.
