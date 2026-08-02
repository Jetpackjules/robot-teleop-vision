# Calibration

## Camera alignment

Every connected RealSense is discovered dynamically. The first enabled camera is the default alignment reference unless a local serial-keyed registry selects another. Fixed multi-camera installations should save one good local transform per camera after setup; those files remain in Godot user data and should be recreated at each physical site.

Use overlapping static geometry, a flat support surface, and adequate depth texture. More cameras improve coverage only when their extrinsics are trustworthy; a bad auxiliary camera must not veto a strong reference-camera solve.

## Full-arm button

**Calibrate Full Arm** is a transactional outward workflow:

1. Preflight fresh depth, telemetry, and a modeled-clearance-safe sweep.
2. Rediscover the base from shoulder-pan motion instead of trusting its old position.
3. Lock the validated base candidate.
4. Sweep and solve shoulder, elbow, wrist flex, wrist roll, and claw evidence progressively.
5. Run convergence/evidence checks against the selected reference camera.
6. Atomically save the complete candidate or Hold and restore the prior registration.

This means physically moving the robot invalidates local registration but does not require a source edit. Pressing the button performs fresh base discovery every time.

The custom wrist camera/bracket is intentionally not fitted as stock geometry because its exact mesh is unavailable. Distal calibration uses mapped fixed/moving jaw evidence and excludes the attachment region.

## Manual wrist/claw workspace

Use the manual workspace when distal visibility is ambiguous. It previews exact projected 3D geometry over aligned RGB without moving hardware. Save is the only action that changes current registration. Tool translation/rotation trims, wrist zero/direction, and opening offset/scale are applied to the united physical claw hierarchy—not detached halves.

## Failure messages

- **Not enough frames**: the selected reference camera did not provide enough settled, distinct poses. Keep the arm visible and rerun.
- **Geometrically indistinct path**: measured poses did not span enough angle to identify a motion axis.
- **Follower faulted**: remove power/support the arm, inspect hardware/power/USB, clear the underlying follower fault, and do not repeatedly request sweeps.
- **Stage rejected**: the old calibration remains active by design.
