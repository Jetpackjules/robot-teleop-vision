# Robot Teleop Vision

A reusable RealSense-to-browser teleoperation stack with a Godot 3D runtime. The shared application discovers one or more RealSense cameras, streams an optimized RGB-D point cloud, arbitrates a single operator, and loads an optional robot module. Robot geometry, controls, hardware transport, calibration, and extra camera views live outside the core.

The browser is the single operator interface. Its compact **Controller** panel contains the selected view and daily robot controls; **Advanced Setup** contains installation defaults, calibration, diagnostics, and tuning. The Godot dock is intentionally limited to setup, diagnostics, launch, and safe shutdown. A clean clone starts in vision-only mode, with physical motion disabled.

## First run

Requirements:

- Python 3.10–3.12
- Godot 4.6 or newer
- Intel RealSense SDK/runtime for local RealSense capture
- `cloudflared` for the default free temporary public URL (or explicitly select local-only mode)

```bash
git clone https://github.com/Jetpackjules/robot-teleop-vision.git
cd robot-teleop-vision
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[realsense,calibration,test]"
robot-teleop init --example vision_only
robot-teleop doctor
robot-teleop modules
robot-teleop start
```

In another terminal:

```bash
robot-teleop open
```

You can instead import this repository's `project.godot` in Godot and use the **Teleop Setup** dock. The project should appear as **Robot Teleop Vision** with the cyan-and-gold icon. If Godot says **2.5D window V3**, that is the old research workspace.

`start` remains in the foreground and supervises every child process. Stop it with `Ctrl+C` or `robot-teleop stop`; the selected robot module's idempotent Hold action runs before shutdown.

For an unattended Linux host reached over SSH, follow [Headless Linux and remote debugging](docs/HEADLESS_LINUX.md). On systemd-based Linux, supervisor and child output is automatically mirrored to `journald`. After reproducing a problem, `robot-teleop support-bundle` creates a redacted diagnostic zip without local credentials, profiles, calibration payloads, or camera imagery.

## Robot modules

The base application contains no SO-101 commands, meshes, calibration solvers, or UI. Each integration is a self-contained directory:

```text
robot_modules/<robot>/
├── robot.json          capability manifest
├── python/             launcher adapter and protocol operator
├── godot/              optional overlay and calibration scene
├── web/                optional capability-specific browser controls
├── assets/             optional meshes and licenses
└── tools/              hardware service and robot-specific solvers
```

The versioned `robot-teleop/v1` envelope carries semantic commands. Keyboard, gamepad, a leader robot, or another controller can all produce the same negotiated joint, Cartesian, gripper, or mobile-base command spaces. A module translates those semantics into its hardware protocol and owns watchdogs and limits.

The included SO-101 module is a complete working example, not a core dependency. See [Robot modules](docs/ROBOT_MODULES.md) to add a different robot without editing shared runtime files.

## Enable the SO-101 example

Start with the robot powered off or physically supported:

```bash
robot-teleop init --example so101_realsense --force
```

Place that arm's local LeRobot pair profile at `robot_modules/so101/local/arm_pair.json`, then set `robot.enabled = true` in ignored `config/local.toml`. Run `robot-teleop doctor` before enabling motion. The SO-101 browser module supplies its leader/keyboard controls, wrist view, safety feedback, overlay, and transactional full-arm calibration workflow.

## Free temporary public access

Cloudflare Quick Tunnel access is enabled by default. `robot-teleop init` writes a unique,
strong operator password into ignored `config/local.toml`; install `cloudflared`, then start:

```bash
robot-teleop start
```

The supervisor verifies the transient `trycloudflare.com/controller.html` URL before publishing it in `robot-teleop status`. Only the newest browser owns streaming and control; a newer page immediately Holds and supersedes the old controller.

For an explicitly local-only deployment, set `stack.public_mode = "off"`.

## Streaming features

- Dynamic, serial-keyed discovery for zero, one, or multiple RealSense cameras.
- Independent meshes by default, with optional multi-camera alignment and color matching.
- Latest-only capture and cancellable latest-frame delivery; slow clients do not build a stale queue.
- Parallel RGB/depth encoding, persistent references, absolute temporal tile updates, plane-aware depth stabilization, and recoverable keyframes.
- A selectable live 3D point cloud or regular 2D RGB camera view, with full-frame RGB compatibility fallback.
- Mouse orbit/pan/zoom that works independently of optional browser head tracking; disabling tracking releases the webcam.
- Installation-wide camera-start defaults and stable view settings shared by new browsers, without persisting mutating actions.
- A flat support-surface guide plus an optional module-declared workspace envelope. Motor stopping remains robot-specific and hardware-authoritative.
- Temporal-update toggles, white background, and latency graphing.
- Dynamically loaded robot overlay, occlusion, auxiliary views, controls, telemetry, and calibration UI.

For camera setup, select an auto-discovered RealSense camera node in Godot. Routine capture, crop, projector, color, and far-depth controls stay at the top; the collapsed **Expert Depth Tuning** group contains the real per-camera Intel SDK filter chain. Its master filter switch is off by default because post-processing can trade motion latency and capture FPS for smoother depth.

## Repository map

| Path | Purpose |
| --- | --- |
| `robot_teleop/` | Robot-neutral launcher, configuration, module discovery, contracts, and health checks |
| `robot_modules/` | Optional self-contained robot integrations; SO-101 is the reference example |
| `godot/` | Shared camera, point-cloud, streaming, tracking, and module-host runtime |
| `web/` | Shared browser shell, module host, and descriptor-driven RGB-D renderer |
| `native/realsense_shared_memory/` | Cross-platform RealSense GDExtension source and runtime binaries |
| `tools/` | Robot-neutral local operator/stream server and camera utilities |
| `config/examples/` | Safe hardware-disabled starting configurations |
| `tests/` | Hardware-free protocol, module-boundary, safety, calibration, and codec tests |

Read [Windows first-run](docs/WINDOWS.md), [Headless Linux](docs/HEADLESS_LINUX.md), [Architecture](docs/ARCHITECTURE.md), [Robot modules](docs/ROBOT_MODULES.md), [Safety](docs/SAFETY.md), [Calibration](docs/CALIBRATION.md), and [Adapters](docs/ADAPTERS.md) before deploying a new robot or site.

## Verify a change

```bash
python -m compileall -q robot_teleop robot_modules tools tests
python -m pytest -q
godot --headless --editor --path . --quit
```

GitHub Actions runs the hardware-free Python suite and a Godot import/script scan on every pull request, plus a Windows test/package job. RealSense and robot hardware remain a local verification gate.

## Migrate this laptop's existing calibration

Only the original large Godot workspace needs this:

```bash
robot-teleop migrate-state --from-project "2.5D window V3"
```

Shared camera/world state and the installed module manifests' declared durable files are copied into Robot Teleop Vision's Godot user-data directory. Captures, logs, physical profiles, and debug images are never copied into the repository.

## License

Robot Teleop Vision is licensed under the [Apache License 2.0](LICENSE). Third-party models and robot assets retain the licenses documented alongside them.
