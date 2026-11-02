# Preview cleanup and practice

Settings > Arm > Adjust Claw Translation opens the existing saved visual
calibration editor with local claw X/Y/Z inputs in millimetres. A 0.25 cm
correction is 2.5 mm. These inputs use the existing distal visual transform;
they do not move motors, translate the base or change joint angles. Preview
does not write calibration. Save Calibration commits it; Cancel restores it.
The normal Godot scene must be running for base/manual calibration. The
read-only overlay publisher alone cannot service calibration commands.

Settings > View > Point cleanup filters the native square-point preview before
crop and sampling. It counts the eight neighbouring source depth pixels within
the selected depth tolerance. Unsupported pixels are removed; retained XYZ/RGB
values are unchanged. This is a reversible preview filter that can remove real
thin features, not a lossless representation of all source pixels. Support 0
disables it exactly. Start with support 2 and tolerance 15 mm. Save cleanup for
everyone persists these values on the host across reloads/restarts. It leaves
capture profiles and original recording JPEGs unchanged. Trial metadata records
the cleanup settings. Balanced mesh and full-detail BlendPCR are unaffected.

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
