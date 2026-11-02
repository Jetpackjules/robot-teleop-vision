# All native points with square sprites

Select **Point detail → All native points (squares)** for the same fixed 5 mm
coloured square sprites used by the 20k/40k views. Every valid native depth
sample retained by the workspace crop is streamed, with calibrated per-sample
RGB. The renderer applies the existing workspace and robot-overlay masks.

This uses the compact point stream and point shader. It does not build BlendPCR
triangles or run the full-detail point-colour fusion passes. Selected depth
millimetres and RGB8 bytes are packed losslessly, with no point-count reduction.
It cannot recreate measurements missing from the source depth cameras.

Camera profile and displayed point count are independent: **Quality + 40k**
already selects up to 40,000 samples per camera from the Quality capture. **All**
selects all available samples from whichever profile is selected. Changing
capture profile can affect source resolution and cadence; changing point detail
alone leaves capture, recording and calibration unchanged.

A read-only live UI comparison in Low latency mode drew about 139,000 points
across the current three-camera workspace, versus about 105,000 at the 40k cap.
All produced 44–45 fresh camera updates/s and about 60 viewer drawing FPS during
the 30-second sample. Those figures describe that sample, not a guarantee for a
different scene, camera profile or network condition.

Budget zero denotes All on the point wire protocol; Full detail remains the
separate UI value zero, while All uses UI value minus one. Legacy 20k/40k packet
layouts and values are unchanged. The All decoder verifies that submitted and
source point counts agree. Eight-megabyte bounded worker slots cover the current
devices' full native grids. All uses the independent WebSocket camera transport;
the optional experimental WebRTC point transport remains limited to 20k/40k.

The last point-detail choice is remembered in this browser. More points increase
packing, bandwidth and drawing work, so performance needs a live measurement.
