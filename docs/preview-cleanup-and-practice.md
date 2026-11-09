# Preview cleanup and practice

Settings > Arm > Adjust Claw Translation opens the existing saved visual
calibration editor with local claw X/Y/Z inputs in millimetres. A 0.25 cm
correction is 2.5 mm. These inputs use the existing distal visual transform;
they do not move motors, translate the base or change joint angles. Preview
does not write calibration. Save Calibration commits it; Cancel restores it.
The normal Godot scene must be running for base/manual calibration. The
read-only overlay publisher alone cannot service calibration commands.

Settings > View > Point cleanup is disabled by default. When enabled, it runs
AFTER crop and 20k / 40k / all-native-point selection and before compression.
It counts neighbouring selected points within a 3D radius in millimetres.
Retained point index, depth and RGB values are unchanged. Larger support and
smaller radius remove more points and can remove real thin details. The UI
reports how many selected points were removed, using applied packet metadata.
Off bypasses neighbour calculations and compaction entirely. Changes remain
local to the current preview until Save Current Setup as Default is pressed;
that saves the toggle and sliders on the host for other viewers on reload and
across host restarts. Capture and recording are untouched. Trial metadata
records the enabled flag, support and radius. Balanced mesh and full-detail
BlendPCR are unaffected. The dedicated preview control socket carries all
three cleanup settings, even without head tracking or WebRTC.

New practice plans use Fixed RGB, Mouse 3D, then Head 3D, each with the host's
practice duration (default 60 seconds). After the explicit Start practice,
Done or timeout advances to the next practice view and starts when its view,
tracking and robot requirements are ready. Errors/offline stop automatic
cycling; reload never starts a recording automatically. The first measured
trial still requires an explicit Start. Existing saved trial plans are retained.

Recording feedback timestamps now use the follower's send timestamp minus
read/write age, mapped to the same host monotonic clock as camera timestamps.
An isolated alignment miss waits up to 100 ms for a genuinely matching pair.
Each saved camera/feedback pair must still meet the 100 ms alignment bound;
stale input and persistent mismatch still preserve an incomplete episode.

Keyboard practice handoffs acknowledge a newly sent zero-motion input by its
follower sequence and fresh encoder feedback before deciding whether Enable
is needed. Cached status from before a host timeout cannot decide that handoff.
Pending confirmation polls status only; it never retries movement or Enable.
No recording starts without the confirmed keyboard state. A timeout reports
the last state, control source and fault for diagnosis.

Wheel zoom clamps actual optical distance to the aimed-at target, including
the camera elevation and claw aim offset, instead of clamping the nominal
orbit radius. The closest distance is 0.125 m. The original neutral framing,
head orbit, straight dolly direction, target picking and saved views remain.
