# Calibration

## Camera alignment (shared)

Every connected RealSense is discovered dynamically. The first enabled camera is the default reference unless a local serial-keyed registry selects another. Fixed multi-camera sites should save one trusted local transform per camera after setup and recreate it whenever hardware moves.

Use overlapping static geometry, a flat support surface, and adequate depth texture. More cameras improve coverage only when their extrinsics are trustworthy; a bad auxiliary camera must not veto a strong reference-camera solve.

## Robot calibration (module-owned)

Core exposes generic start, refine, cancel, status, save, restore, and clear hooks. The selected Godot module decides which evidence, movements, kinematics, and transactional gates those hooks use. A robot without automatic calibration can omit them and provide a module-specific manual workflow.

Calibration files live in Godot `user://`. A module lists only the durable filenames in `robot.json`, allowing explicit state migration without putting site transforms in Git.

## SO-101 reference workflow

The SO-101 module's **Calibrate Full Arm** button performs a transactional outward solve:

1. Preflight fresh D455 depth, telemetry, and a modeled-clearance-safe sweep.
2. Rediscover the base from shoulder-pan motion instead of trusting its old position.
3. Lock the validated base candidate.
4. Sweep and solve shoulder, elbow, wrist flex, wrist roll, and claw evidence progressively.
5. Check convergence/evidence against the selected reference camera.
6. Atomically save a complete candidate, or Hold and restore the prior registration.

Physically moving that robot invalidates registration but does not require source edits; a fresh run rediscovers the base. The custom wrist bracket is deliberately omitted because its exact mesh is unavailable. Distal solving uses mapped jaw evidence and excludes the attachment region.

The SO-101 manual wrist/claw workspace overlays exact projected geometry on aligned RGB without moving hardware. Save is the only operation that modifies current registration. Translation/rotation trims, wrist zero/direction, and opening offset/scale apply to the united physical claw hierarchy.

### Web calibration and local ports

Normally keep `[godot] launch_runtime = true`, start `robot-teleop`, and use
**Calibrate Full Arm** in the operator website. No Godot editor is required.
Keep the robot and camera fixed after a successful registration, and keep the
robot's movement area clear during the automatic sweep.

The follower profile is the source of truth for `command_port` (default 4248),
`status_port` (4249), `godot_status_port` (4250), and `editor_status_port` (4252).
The launcher gives the follower, web server, and Godot the same resolved ports;
`stack.calibration_status_port` (4251) carries calibration results back to the
website. These UDP ports must be distinct, in 1–65535, and not reserved for
head tracking. They are local
loopback connections, not ports to forward on the router. A busy telemetry
listener reports the actual port and retries after the conflicting process exits.
The physical USB/COM device is separately selected by `follower.port` in the
profile: no automatic search or motor activation on another serial device occurs.
Windows COM names are opened by the serial driver, not checked as filesystem
paths; this also permits valid higher-numbered ports such as `COM10`.

After updating the repo or changing ports, restart the whole launcher so both
Godot and the follower use the same version/settings. Existing default-port
profiles require no edits. Sweep requests now have IDs: an old rejection cannot
cancel a new attempt, duplicate requests cannot restart stopped motion, and Hold
still cancels immediately when processed. The acknowledgement wait is bounded;
there is no automatic retry of an unacknowledged movement request. New starts
also expire, and the follower checks expiry again after planning before enabling
motion. Errors report
the follower's rejection, fault, interruption, or missing acknowledgement rather
than incorrectly claiming that web calibration always needs editor UDP 4252.

For editor-only calibration, temporarily set `launch_runtime = false`, restart
the launcher (which still runs the follower), then reopen `godot/Main.tscn` and
use **View → Robot Module → Calibrate Robot** without pressing Play. The editor
reads the launcher's ignored `.teleop/runtime_config.json`; reopen the scene after
changing its settings. The file contains port settings, not credentials, and does
not prove the follower is alive: fresh encoder telemetry and depth are still
required. Close the editor, restore `launch_runtime = true`, and restart to return
to web video. With the runtime disabled, the website alone cannot stream cameras.

Port wiring does not guarantee a successful geometric fit. Camera coverage,
correct motor calibration, current hardware feedback, and clearance checks remain
required; do not bypass these checks to obtain a saved overlay.

## Typical rejection reasons

- **Not enough frames**: the reference camera did not provide enough settled, distinct poses. Keep the relevant links visible and rerun.
- **Geometrically indistinct path**: measured poses did not span enough angle to identify the intended axis.
- **Hardware process faulted**: remove power/support the robot, inspect power/USB/mechanics, and clear the underlying fault before requesting another sweep.
- **Stage rejected**: the last complete registration remains active by design.
