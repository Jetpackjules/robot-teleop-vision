# Multi-camera instability: diagnostic handoff

## Established defect

In Godot unified mesh mode (also `prefer_d455` / `prefer_d435`), a camera
whose displayed frame reaches 1000 ms is marked not ready in
`realsense_mesh_fusion.gd`. The native shader already excludes that layer as
an overlap owner, but previously still drew its own frozen mesh. Stale B
in the foreground could therefore cover live C, even with A absent. The
original stale-first-camera test placed A behind the other cameras and
did not expose this failure.

The shader now rejects the current layer when its readiness flag is false.
It uses the existing timeout, retains layer ordering, and resumes drawing
on fresh metadata. Independent mesh behavior is unchanged. The native
extension must be rebuilt and Godot restarted to pick up this shader change;
the committed prebuilt binaries are not updated by this source patch.

## Rendering and calibration findings

- `godot/Main.tscn`, the exported script default, and clean-default actions
  select `independent_mesh`. That mode renders overlapping surfaces separately.
  `unified_mesh` enables depth-verified overlap suppression; `shader_mesh`
  primarily shares reference color. Do not infer the workstation's mode from
  the repository default.
- Godot fusion order is model preference followed by serial-keyed camera ID,
  not discovery/arrival order or the displayed A/B/C label. `prefer_d435`
  changes the model preference. Web rendering sorts by transmitted priority
  and camera ID and independently maps its display-mode setting.
- Godot fusion metadata uses each renderer's inverse global transform.
  Rig calibration composes target-to-reference with reference-to-world;
  saved relative transforms are resolved through their parent chain. Existing
  hardware-free transaction/coordinate tests pass; this does not verify saved
  extrinsics on the workstation or accuracy with physical cameras.
- The web unified path already skips stale cameras before drawing. The new
  B/C fixture protects that behavior alongside the native shader regression.
  Neither path's local freshness check establishes physical exposure sync.

## Workstation session

1. Record commit, native binary build, actual render mode, and the A/B/C to
   serial/model mapping. Record frame sequence/held age and capture FPS while
   reproducing; arrange remote workstation access during the agreed session.
2. With the scene stationary, compare B alone, C alone, then B+C with A off.
   Compare independent and unified mode explicitly. Pause/disconnect B and
   verify that it disappears in unified mode at the timeout, that C remains
   visible, and that B returns on a fresh frame.
3. Export the current calibration before changing it. Check B/C alignment on
   a stationary surface using the same world frame. If each isolated view is
   stable but the combination jitters, distinguish double surfaces/calibration
   disagreement from frame stalls before changing tolerances.
4. Repeat with motion and all cameras. Capture short clips, serial-keyed frame
   ages, and capture timing if available. If instability also occurs within
   isolated views, inspect capture/depth quality and timing independently of
   overlap ownership. No real-camera validation has been performed for this fix.

## Reproducing the hardware-free checks

`python -m pytest -q tests/test_realsense_fusion_shader.py` uses optional
ModernGL/EGL (included in the Linux test extra) to compile the production
native fusion functions with only Godot syntax adapted to GLSL. It checks stale
foreground A/B/C, recovery, independent mode, normal overlap and all-stale
frames. It does not load the native extension or validate Godot's GPU backend.

`tests/godot/realsense_fusion_probe.gd` additionally exercises actual native
shader/material wiring with synthetic frames, including the 999/1000 ms
boundary; its existing Windows GPU harness requires explicit opt-in and a
rebuilt extension. `tests/realsense_web_rendering.cjs` exercises actual WebGL2.
