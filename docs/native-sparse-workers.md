# Persistent sparse-preview workers

On Linux, the native 20k/40k WebSocket previews use one persistent encoder process
per camera and browser, shared by its transport lanes. Two bounded shared-memory
slots retain the latest encoded publication for each lane. Slow network sends
do not block camera encoding.

Crop and point-budget updates use a generation-tagged control slot. Encoders
stay running, and results made with older settings are rejected. The original
point-selection, calibrated RGB lookup, millimetre depths and lossless packing
functions are unchanged. Capture, recording and calibration files are untouched.

Workers stop when their final subscriber disconnects and exit if the gateway
disappears. Full-detail RGB-D retains its existing path. Non-Linux hosts retain
the threaded sparse path. Set `ROBOT_TELEOP_NATIVE_POINT_WORKERS=0` at gateway
startup to select that fallback explicitly.

Validation covers exact 20k/40k point bytes, shared lanes, hot settings changes,
viewer isolation, bounded pending data, worker failure and resource cleanup.
Run `python -m unittest tests.test_native_sparse_workers` on the Linux host.
