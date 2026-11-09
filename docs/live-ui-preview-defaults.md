# Preview defaults, overlay recovery and input setup

The installation defaults are All native points (squares), Low latency and
BlendPCR. Point detail `-1` uses every retained native point and square point
rendering; it overrides the selected mesh renderer. A guided Fixed RGB trial
still switches to RGB for that condition. Capture and recording are independent
of point-preview reduction.

`point_preview_detail` is saved on the host with the other site settings.
A new tunnel origin therefore inherits the installation choice. An updated
host-default revision replaces a browser's older fallback once; subsequent
explicit browser choices remain last-used until defaults change again.
Reload Saved Defaults explicitly restores the host choice.

New episode trial metadata snapshots `point_preview_detail`, `stream_preset`,
`robot_overlay_enabled` and `control_source` at recording start. Older recordings
are unchanged. In particular, an older `display_mode: blendpcr` label alone does
not distinguish actual BlendPCR triangles from a square-point override, and
does not establish the preview point count or stream preset.

Set up session retains an already connected leader. With no connected or
partially opened leader, that explicit click enables keyboard input instead of
opening a USB chooser. A connected follower, fresh feedback and acknowledged
Enable remain required. A later trial never silently changes a disconnected
leader to keyboard. For leader-based studies, connect the physical leader
before setup; the chosen input is then included in the new recording metadata.

The normal Godot runtime publishes calibrated link poses for capture and the
native point streams. A gateway/capture-only startup lacks that publisher and
correctly retires stale overlay poses after two seconds; it does not imply lost
calibration. The optional headless `tools/robot_overlay_pose_service.gd` restores
this publishing role without starting cameras, opening control sockets or
sending servo commands. It uses the existing overlay implementation, saved
world-level transform and saved registration, and checks its base against the
previous publisher before writing the runtime pose mirror. Fresh measured
telemetry is required; an active normal publisher takes ownership back.

The pose service does not save, fit or modify calibration. It can be run with:

```sh
godot --headless --path . --script res://tools/robot_overlay_pose_service.gd
```
