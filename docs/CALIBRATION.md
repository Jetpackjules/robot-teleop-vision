# Calibration

## Camera alignment (shared)

Every connected RealSense is discovered dynamically. The first enabled camera is the default reference unless a local serial-keyed registry selects another. Fixed multi-camera sites should save one trusted local transform per camera after setup and recreate it whenever hardware moves.

Use overlapping static geometry, a flat support surface, and adequate depth texture. More cameras improve coverage only when their extrinsics are trustworthy; a bad auxiliary camera must not veto a strong reference-camera solve.

Robot base fitting tries the capture-preferred camera first. If its motion fit
fails, it evaluates the other captured cameras independently, retaining the same
expected direction and 12 mm residual limit. The selected camera and rejected
attempts are recorded in the base-fit JSON. Outward joint and RGB claw stages
keep that validated reference, including on installations with two D435s.
This selection does not modify camera-to-camera alignment.

## Robot calibration (module-owned)

Core exposes generic start, refine, cancel, status, save, restore, and clear hooks. The selected Godot module decides which evidence, movements, kinematics, and transactional gates those hooks use. A robot without automatic calibration can omit them and provide a module-specific manual workflow.

Calibration files live in Godot `user://`. A module lists only the durable filenames in `robot.json`, allowing explicit state migration without putting site transforms in Git.

## SO-101 reference workflow

### Physical leader motion directions

Physical leader control currently tracks changes relative to each arm's pose at
Enable. A starting-pose difference is therefore retained. This is separate from
a joint moving in the opposite direction: matching initial poses cannot fix a
reversed response.

An arm-pair profile may specify `"leader_joint_directions": [1, -1, -1, 1, 1, 1]`
to reverse only shoulder lift (motor 2) and elbow flex (motor 3) during physical
leader control. Use this only when those joints have been observed to move
oppositely; it is not a universal SO-101 or LeRobot default. Missing settings
keep all six directions at `+1`.

With the launcher stopped, update the configured local profile from the repo root:

```powershell
.\.venv\Scripts\python.exe scripts/set_so101_leader_directions.py --invert 2 3 --apply
```

The command prints the selected profile, makes a backup, and changes only this
mapping. Omit `--apply` for a preview; use `--reset --apply` to restore all normal
directions. `--profile PATH` selects an explicit arm-pair JSON instead of the
active local configuration. Re-running the command is safe: it sets the signs,
never toggles them. It makes no motor connection or hardware-calibration writes.

Restart the complete Python launcher/follower after applying the setting. The
startup log prints the six active directions. Test a small movement of each
affected joint away from its limits before normal use. Direction changes apply
to motion after Enable, preserving the initial hold, motion rate limits, travel
limits, and stall detection. They do not change keyboard control, saved rest
poses, or the camera/overlay calibration; a reversed wrist overlay needs its own
visual-calibration diagnosis.

### Automatic visual calibration

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

- **Hardware LeRobot calibration copied into a legacy profile**: hardware
  `Homing_Offset` values are already applied inside Feetech motors. Copying them
  into the old software-offset formula produces incorrect joint angles and
  targets. Profiles can explicitly select `coordinate_system: "lerobot_urdf"`
  within `follower.calibration`. This uses [LeRobot's degree conversion](https://github.com/huggingface/lerobot/blob/main/src/lerobot/motors/motors_bus.py)
  around the recorded range midpoint, then maps URDF angles into the existing
  planner/model convention. Legacy profiles retain their existing conversion.
  With the launcher stopped, first run the read-only motor diagnostic, then
  `python scripts/repair_so101_profile_coordinates.py --apply`. This file-only
  repair requires matching motor identities, offsets, and ranges in the saved
  diagnostic, creates an exact backup, and changes only the follower's coordinate
  convention. It does not recalibrate motors or modify camera alignment. Restart
  the follower after applying it. Old saved robot poses/registration must be
  revalidated under the corrected convention before reuse; simulation tests do
  not prove the physical arm's calibration or workspace clearance.
- The follower reads encoders throughout automatic motion. Sustained following
  error cancels the sweep and rebases the hold to the measured pose, even if
  ordinary teleoperation feedback settings are disabled. Loss of encoder data
  for 0.6 seconds faults the sweep and attempts to disable torque. A sampling
  pose or completed sweep requires fresh encoder agreement, not just a sent goal.
- Calibration reads each motor's existing position limits before enabling
  motion. Observation ranges are translated inside those limits with a small
  margin, preserving sample spacing, then the adjusted route is checked for
  modeled clearance. The hardware limits are never widened. This handles arms
  whose valid encoder ranges differ from the original reference arm; it does
  not replace correct motor calibration or checks for physical obstructions.
- **Overlay moves but the physical arm does not**: stop the attempt and support
  the arm before stopping the launcher (shutdown releases torque). With the
  launcher stopped and motor power connected, run
  `.venv\Scripts\python.exe scripts/diagnose_so101_motors.py` on Windows. This
  reads the configured follower's registers without enabling torque, sending
  positions, or changing limits. It saves `.teleop/so101_motor_diagnostics.json`.
  Goal/position, operating mode, hardware limits, and fault flags help separate
  a rejected target from a stalled motor. Torque values are observed after
  shutdown; zero then does not prove torque was off during the failed sweep.
- **Could not reach the compact base-axis anchor pose**: the modeled approach
  does not meet the clearance threshold. Update and restart the full launcher
  if using an older follower: calibration now preserves profile angle turns
  (for example, an elbow at 388 degrees approaches 420 degrees, equivalent to
  60 degrees, instead of attempting a 328-degree movement). The overlay's
  unregistered world position does not affect this check. Encoder-range and
  modeled-clearance checks still apply; a physically familiar rest pose does
  not imply a matching saved rest-pose file exists.
- **Return to Rest** uses a separate per-arm encoder pose. Save six previously
  observed, operator-confirmed raw encoder readings with
  `.venv\Scripts\python.exe scripts/save_so101_rest_pose.py --raw R1 R2 R3 R4 R5 R6 --apply`.
  Replace each `R` value with that arm's reading; there is no universal raw pose.
  This file-only command validates the corrected profile, ranges and stock mesh
  floor clearance, then backs up and saves `so101_rest_pose.json` beside the
  configured arm profile. It does not connect to motors or change cameras or
  visual registration. The updated follower reloads the file on the next Return
  to Rest request, rejects a changed arm/calibration, and checks live hardware
  limits and the complete planned route before enabling motion. Restart the
  launcher once after updating its Python code; support the arm before shutdown
  releases torque. Godot can stay open for this Python-only update.
  Rest-return floor checks use unsimplified mesh support vertices at measured
  joint angles, avoiding empty bounding-box corners near the claw. Other
  calibration/teleoperation clearance checks remain unchanged. The check models
  a plane and stock links, not obstacles or self-collision; supervise the first
  physical return. Regenerate the asset after mesh changes with
  `scripts/build_so101_rest_hulls.py` (Open3D required only for regeneration).
- **Not enough frames**: the reference camera did not provide enough settled, distinct poses. Keep the relevant links visible and rerun.
- **Geometrically indistinct path**: measured poses did not span enough angle to identify the intended axis.
- **Hardware process faulted**: remove power/support the robot, inspect power/USB/mechanics, and clear the underlying fault before requesting another sweep.
- **Stage rejected**: the last complete registration remains active by design.
