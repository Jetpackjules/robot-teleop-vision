# Architecture

The stack has four local trust boundaries:

1. **Camera adapters** discover hardware. Godot's native extension owns capture and publishes the newest RGB-D packet.
2. **Godot runtime** reconstructs cameras, maintains local alignments, renders robot geometry, and runs calibration capture/validation.
3. **Operator server** authenticates browsers, compresses RGB-D, relays tracking/view commands, and arbitrates one newest-page owner.
4. **Robot adapter** runs in a separate follower process. Browser messages cannot write motors directly; the follower applies watchdogs, workspace limits, measured-feedback checks, and Hold.

The browser is the only operational UI. The Godot editor plugin calls the same repository CLI for first-run setup and diagnostics.

## Data flow

```text
RealSense(s) -> native Godot capture -> latest-only RGB-D socket
                                      -> temporal RGB/depth encoder
                                      -> authenticated newest browser

newest browser -> authenticated arm socket -> local UDP follower -> servos
newest browser -> view settings/tracking -> Godot runtime
follower telemetry -> Godot overlay + browser health panel
```

RGB-D capture and encoding are decoupled so a slow frame encoder cannot block the camera drain. Temporal RGB and depth work in parallel. Persistent-reference mode sends an initial complete frame followed by reliable ordered absolute tile updates; turning it off restores periodic independently recoverable keyframes. The **Full RGB frame updates** toggle disables temporal RGB while retaining the depth path.

## Local-only state

Device serials are discovery keys, not source constants. Camera alignments and robot registration use Godot `user://`; motor ports, identities, and calibration tables belong in ignored `config/local.toml` and `tools/so101_arm_pair.json`. A clean clone therefore starts with motion disabled.

## Extension points

The Python registry supports built-in and Python entry-point adapters in these groups:

- `robot_teleop.cameras`
- `robot_teleop.robots`
- `robot_teleop.trackings`

Godot robot geometry/calibration remains a robot-specific module under `godot/robots/<robot>/`. The shared stream and browser layers do not assume SO-101 geometry.
