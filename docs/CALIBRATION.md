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

## Typical rejection reasons

- **Not enough frames**: the reference camera did not provide enough settled, distinct poses. Keep the relevant links visible and rerun.
- **Geometrically indistinct path**: measured poses did not span enough angle to identify the intended axis.
- **Hardware process faulted**: remove power/support the robot, inspect power/USB/mechanics, and clear the underlying fault before requesting another sweep.
- **Stage rejected**: the last complete registration remains active by design.
