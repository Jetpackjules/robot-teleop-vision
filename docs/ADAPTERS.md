# Adding hardware

Camera and tracking sources use small shared adapters. A robot uses a module because its geometry, controls, status translation, safety process, and calibration often need to ship together.

## Camera adapter

Implement `CameraAdapter.discover()` and register it:

```python
from robot_teleop.registry import register

class MyCamera:
    name = "my_camera"

    def discover(self):
        return []

register("camera", MyCamera.name, MyCamera)
```

Add the built-in import to `robot_teleop/cameras/__init__.py`, or publish an entry point in `robot_teleop.cameras`. Device identifiers must be locally discovered keys; never embed a lab's serials in source.

## Robot module

Copy `robot_modules/so101` as a structural reference, then replace its implementation rather than adding conditions to core. At minimum a module provides:

- `robot.json`, including negotiated capabilities and the required `hold` action.
- A Python `RobotAdapter` with `launch_spec()`, idempotent `hold()`, `public_manifest()`, and optional operator environment.
- A hardware process that enforces command age, watchdog, limits, and Hold independently of the browser.

Godot geometry/calibration and web controls are optional. Without them the common point-cloud viewer and semantic robot transport still work. Full details and examples are in [Robot modules](ROBOT_MODULES.md).

## Tracking adapter

A tracking adapter describes the tracking source. Use `disabled` for direct mouse orbit/pan/zoom. Browser MediaPipe is optional and never required for robot control.

## Acceptance checklist

- Clean clone launches with physical motion disabled.
- `doctor` handles zero, one, and multiple cameras without source edits.
- Module has hardware-free tests or a fake transport.
- Hold is idempotent and works before, during, and after startup.
- Disconnect/reconnect, stale commands, and newest-browser ownership are tested.
- Stop/crash paths invoke Hold.
- No physical serials, ports, tokens, passwords, or calibration payloads are committed.
- Shared `robot_teleop`, `godot`, `web`, and `tools` code contains no module's wire commands or model names.
