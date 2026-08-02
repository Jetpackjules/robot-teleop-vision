# Adding adapters

## Camera

Implement `CameraAdapter.discover()` and register it:

```python
from robot_teleop.registry import register

class MyCamera:
    name = "my_camera"
    def discover(self):
        return []

register("camera", MyCamera.name, MyCamera)
```

Add the module import to `robot_teleop/cameras/__init__.py`, or publish an entry point named `my_camera` in group `robot_teleop.cameras`. Device identifiers must be local discovery keys; do not embed a lab's serials in source.

## Robot

Implement `launch_spec()` and `hold()`. `hold()` must be idempotent and safe to call before or during shutdown. Register under `robot`, add a hardware-disabled example configuration, and put robot-specific Godot geometry under `godot/robots/<name>/`.

The browser/operator server should send semantic commands. Hardware protocols and motor writes stay inside the isolated robot process, where watchdog and limit enforcement can be tested without a browser.

## Tracking

A tracking adapter describes the tracking source. Use `disabled` for mouse-only orbit/pan teleoperation. Browser MediaPipe is optional and never required for arm control.

## Acceptance checklist

- Clean clone launches with all physical motion disabled.
- `doctor` discovers zero, one, and multiple devices without source edits.
- Adapter has hardware-free tests or a fake transport.
- Disconnect/reconnect and duplicate-browser ownership are tested.
- Stop/crash paths call Hold.
- No physical serials, motor ports, tokens, passwords, or calibration payloads are committed.
