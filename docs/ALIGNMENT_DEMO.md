# Local alignment demo

This separate Godot scene illustrates viewpoint-dependent alignment with a
virtual SO-101. It does not start the production camera, follower, tunnel or
operator stack. Its scene and object interactions are synthetic, not a replay
of the Sweden–Singapore experiment or evidence of physical task success.
The new scene and controller live under `examples/alignment_demo`; existing
production pages, scenes and robot modules remain separate.

## Windows launcher

From the repository root, run:

```powershell
.\scripts\start_alignment_demo.ps1
```

The launcher opens the standalone Godot scene using the OpenGL compatibility
renderer and opens the controller page with a live view of that Godot render.
Solid geometry is selected initially. **Synthetic depth** switches to the
single virtual depth sensor's point cloud; **Solid geometry** restores the
complete geometry view. **Front view** and **Side view** change the viewpoint
without a webcam. The existing head tracking, leader and slider controls are
below the preview. The launcher uses the project `.venv` Python when available,
otherwise `python` on PATH.
Godot is located on PATH or in Downloads; use `-GodotPath 'C:\Tools\Godot_console.exe'`
to select it explicitly. Add `-NoBrowser` to leave the browser closed, or
`-DryRun` to print the commands without starting anything. `-?` shows help.

Logs are written under `.teleop/alignment-demo`. The Python relay runs hidden
and Godot opens a visible window. An existing relay on port 14860 is reused
only if `/config` identifies this simulation protocol version 1, controller
revision 2 and UDP port 14861. Older relays are refused because they cannot
provide the current controller ownership/preview protocol. Other occupied
ports produce an error without stopping any process.
The relay remains available after the Godot window closes; its PID is printed
when the launcher starts it. Webcam and leader input stay off until explicitly
enabled in the controller.

Alternatively, open two terminals from the repository root:

```powershell
# Terminal 1: replace godot with your Godot executable if it is not on PATH.
godot --path . --rendering-method gl_compatibility res://examples/alignment_demo/godot/AlignmentDemo.tscn -- --simulation-input-port=14861 --simulation-preview-dir=.teleop/alignment-demo

# Terminal 2: only Python's standard library is required by this relay.
.\.venv\Scripts\python.exe tools/serve_alignment_demo.py
```

Open [the local controller](http://127.0.0.1:14860/) in desktop Chrome or Edge.
The relay binds only `127.0.0.1:14860` and forwards a restricted simulation
envelope only to `127.0.0.1:14861`. Godot opens the receive-only input socket
only when the `--simulation-input-port` flag is present. If either port is
occupied, choose an unused HTTP port with `--port`; change the UDP port on
both launch commands together. Do not use production SO-101 ports.

The browser preview is the actual Godot viewport, exported locally as a
960-pixel JPEG up to ten times per second. Godot atomically replaces the fixed
`preview.jpg` and `preview-state.json` files in `.teleop/alignment-demo`; the
relay serves only those fixed preview paths. The page checks the frame time
and displays stopped/stale status when the renderer is no longer updating.
Relay connectivity alone is not a renderer handshake. Headless Godot reports
that no rendered preview is available and never supplies a substitute image.
The controls also require the scene to report a bound input socket on the
relay's intended UDP port. Use the native Godot window for full-refresh
recording; the browser panel is a ten-frame-per-second preview.

Opening a newer controller page takes ownership automatically. The old page
releases webcam, leader and slider input and does not reconnect. Temporary
relay loss retries automatically with devices off. Camera-start cancellation
uses an example-local tracker bridge; the production tracker is unchanged.

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
its inputs. Reset/recenter/replay affect only the synthetic scene. “Relay
connected” describes the input connection; the separate preview status uses
fresh state and decoded frames from Godot to verify that the renderer is
running. This is not a per-command acknowledgement. Keep the physical follower
powered off while using this demonstration.

The newest controller page takes ownership of input. A superseded page stops
its webcam and serial reader and does not automatically take control back.
After a temporary relay interruption, the page may reconnect its input
connection, but webcam, leader and manual input remain off until enabled again.
Other pages can continue viewing decoded native preview frames.

The six joint values use the existing SO-101 calibration convention: five
servo-normalized angles plus gripper percentage. They are inputs to a virtual
kinematic model, not measured follower state. A demonstration cannot validate
the real arm's calibration, collision safety, contact mechanics or remote
camera coverage.

In Godot, **P** starts replay and **R** resets the task. **W/A/S/D** move the
tool in X/Z, **Q/E** move down/up, and **Space** toggles the gripper. Hold the
right mouse button to orbit and use the wheel to zoom. **1/2** select front/side
views, **C** toggles cloud/solid rendering, and **H** recenters head input.
Launch options include `--simulation-replay`, `--simulation-mesh` (the default),
`--simulation-point-cloud`, `--simulation-preview-dir=.teleop/alignment-demo`,
`--simulation-clean-view` and `--simulation-fixed-view`, after the `--` separator.

Keyboard and replay use position-only inverse kinematics; leader joint values
drive forward kinematics directly. Grasp attachment uses a proximity rule.
Released objects use rigid-body gravity and collisions with the bowl/table;
the arm itself does not exert contact forces. The cloud is raycast at 8 Hz from
one fixed virtual depth sensor, so its occlusion gaps reflect synthetic sensor
coverage. The front reference view and depth sensor have different poses.
Rendered replay footage uses scripted head motion, not recorded webcam input.

Validation without any camera or robot:

```powershell
.\.venv\Scripts\python.exe -m pytest -q tests/test_alignment_demo_controller.py tests/test_alignment_demo_physics.py
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

The physics probe also verifies solid/depth and front/side actions. A separate
headless test requires an unavailable preview state and no fabricated image.
On September 8, 2026, native Godot 4.7.1 preview validation confirmed advancing
JPEG frames, solid/depth switching, fake head/joint input, stale-input holding
and a stopped state when Godot closed. These results do not certify physical
hardware behavior.

The final full suite passed **311 tests** on September 8, 2026. Additional
checks cover newest-tab handoff, idle connections, reconnect recovery,
pending camera/serial cancellation, source-history retirement and mismatched
scene input ports. Browser review confirmed live frames, slider input,
solid/depth switching, view controls and a completed placement replay. A real
relay stop/start also confirmed automatic browser recovery without a reload;
webcam and leader remained off.
