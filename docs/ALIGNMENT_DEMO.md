# Local alignment demo

This separate Godot scene illustrates viewpoint-dependent alignment with a
virtual SO-101. It does not start the production camera, follower, tunnel or
operator stack. Its scene and object interactions are synthetic, not a replay
of the Sweden–Singapore experiment or evidence of physical task success.

## Windows launcher

From the repository root, run:

```powershell
.\scripts\start_alignment_demo.ps1
```

The launcher opens the standalone Godot scene using the OpenGL compatibility
renderer and opens the controller page. It
uses the project `.venv` Python when available, otherwise `python` on PATH.
Godot is located on PATH or in Downloads; use `-GodotPath 'C:\Tools\Godot_console.exe'`
to select it explicitly. Add `-NoBrowser` to leave the browser closed, or
`-DryRun` to print the commands without starting anything. `-?` shows help.

Logs are written under `.teleop/alignment-demo`. The Python relay runs hidden
and Godot opens a visible window. An existing relay on port 14860 is reused
only if `/config` identifies this simulation protocol, version and UDP port
14861. Other occupied ports produce an error without stopping any process.
The relay remains available after the Godot window closes; its PID is printed
when the launcher starts it. Webcam and leader input stay off until explicitly
enabled in the controller.

Alternatively, open two terminals from the repository root:

```powershell
# Terminal 1: replace godot with your Godot executable if it is not on PATH.
godot --path . --rendering-method gl_compatibility res://godot/simulation/AlignmentDemo.tscn -- --simulation-input-port=14861

# Terminal 2: only Python's standard library is required by this relay.
.\.venv\Scripts\python.exe tools/serve_alignment_demo.py
```

Open [the local controller](http://127.0.0.1:14860/) in desktop Chrome or Edge.
The relay binds only `127.0.0.1:14860` and forwards a restricted simulation
envelope only to `127.0.0.1:14861`. Godot opens the receive-only input socket
only when the `--simulation-input-port` flag is present. If either port is
occupied, choose an unused HTTP port with `--port`; change the UDP port on
both launch commands together. Do not use production SO-101 ports.

1. With no hardware, use the Godot demonstration or press **Enable sliders**.
2. For actual head tracking, press **Start head tracking** and allow the
   chosen webcam. Face landmarks are mapped to x/y/z using the existing
   browser tracker. **Stop webcam** releases the camera. First use loads
   MediaPipe and its model from their existing public CDN endpoints, so an
   internet connection is needed. Head tracking does not send images to the
   relay: only x/y/z samples are forwarded locally.
3. For a physical leader, select its local arm-pair JSON profile (for example,
   the ignored `robot_modules/so101/local/arm_pair.json`) or a JSON containing
   its `calibration` object. Only `leader.calibration` is read; the file stays
   in the browser and follower settings are not used. Press **Connect leader**
   and select its USB serial port. Disconnect it from any existing controller
   first. The reader polls Feetech motor IDs 1–6 at 1,000,000 baud, using only
   sync-read instruction `0x82` for position register 56. It does not write
   positions, enable/disable torque, change calibration or auto-reconnect.
   The leader must already be freely movable. **Disconnect** closes its port.

Head and joint streams pause when samples are stale; closing the page releases
its inputs. Reset/recenter/replay affect only the synthetic scene. The relay
has no simulator acknowledgement, so “relay connected” confirms delivery to
the local UDP endpoint, not that Godot is running. Keep the physical follower
powered off while using this demonstration.

The six joint values use the existing SO-101 calibration convention: five
servo-normalized angles plus gripper percentage. They are inputs to a virtual
kinematic model, not measured follower state. A demonstration cannot validate
the real arm's calibration, collision safety, contact mechanics or remote
camera coverage.

Validation without any camera or robot:

```powershell
.\.venv\Scripts\python.exe -m pytest -q tests/test_alignment_demo_controller.py
```

The test drives a real local WebSocket/UDP relay with fake head and joint
packets, verifies origin/sequence/shape restrictions and disconnect behavior,
and exercises the JavaScript reader against a fake serial port. The only
serial writes in that test are checked against the fixed encoder-read packet.
It also verifies cancellation during the port chooser and during a pending
port-open operation: a late completion cannot start polling.

Set `ROBOT_TELEOP_TEST_GODOT` to your Godot console executable to include the
end-to-end scene check (otherwise that check skips when Godot is not found).
That probe sends fake browser input through the real relay into a running
headless `AlignmentDemo`, then checks head displacement, virtual joint/gripper
changes, stale-packet rejection and timeout behavior. Headless checks validate
scene state, not rendered appearance. Actual webcam and physical-leader
operation require the explicit user actions above and are separate from this
hardware-free validation.

Verified on September 8, 2026 with Godot 4.7.1 and Node available: **18 tests
passed**, including the full relay-to-running-scene check and both pending
serial-connection cancellation cases. Ruff and JavaScript syntax checks also
passed. These results do not certify physical hardware behavior.
