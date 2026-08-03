# Robot modules

Robot Teleop Vision treats a robot as an optional plugin-shaped directory. The shared camera, stream, tracking, and browser application remain useful with no robot selected; adding a robot must not require `if robot == ...` branches in core.

## Directory contract

```text
robot_modules/acme_bot/
├── __init__.py
├── robot.json
├── python/
│   ├── __init__.py
│   ├── adapter.py
│   └── operator.py
├── godot/
│   ├── RobotModule.tscn
│   └── acme_bot_module.gd
├── web/
│   └── module.js
├── assets/
└── tools/
```

Only `robot.json` and the Python adapter are mandatory. Omit Godot, web, assets, views, or calibration when the robot does not need them.

## Manifest

```json
{
  "schema_version": 1,
  "id": "acme_bot",
  "label": "Acme mobile manipulator",
  "version": "1.0.0",
  "description": "Example integration",
  "capabilities": {
    "control_spaces": [
      "joint_velocity",
      "cartesian_tool_velocity",
      "gripper_velocity",
      "base_velocity"
    ],
    "inputs": ["keyboard", "gamepad", "leader"],
    "actions": ["enable", "hold", "restart", "return_rest"],
    "views": ["tool_rgb"],
    "features": ["overlay", "automatic_calibration", "measured_feedback"]
  },
  "entrypoints": {
    "python": "robot_modules.acme_bot.python.adapter:AcmeRobotAdapter",
    "operator": "robot_modules.acme_bot.python.operator:AcmeRobotOperator",
    "godot": "res://robot_modules/acme_bot/godot/RobotModule.tscn",
    "web": "/robot-modules/acme_bot/web/module.js"
  },
  "state_files": ["acme_bot_registration.json"]
}
```

IDs are lowercase safe identifiers. Every physical module must advertise `hold`; Hold must be idempotent. `state_files` accepts plain filenames only—never paths. Python/operator entrypoints and state filenames are private; the browser receives only public capabilities plus Godot/web composition points.

Capabilities are negotiation, not decoration. A command outside the advertised control spaces or system actions is rejected before it reaches hardware.

## Configuration and discovery

Select the module and place its settings under the module-owned table:

```toml
[robot]
adapter = "acme_bot"
enabled = false
module_paths = ["../site-robot-modules"]

[robot.options]
profile = "local/acme.json"
dry_run = true
tool_camera_device = ""
```

Core preserves options without interpreting them. Relative paths are resolved by the module, which should make their policy explicit. `ROBOT_TELEOP_MODULE_PATH` and `robot.module_paths` add discovery roots for Python and browser modules. A module with Godot resources must be present (or linked) at `res://robot_modules/<id>` so Godot can import it.

Use `robot-teleop modules --json` to inspect public negotiation data and `robot-teleop doctor` to validate the selected module while motion remains disabled.

## Python adapter

The adapter composes and safely stops the isolated hardware process:

```python
from robot_teleop.interfaces import LaunchSpec

class AcmeRobotAdapter:
    name = "acme_bot"

    def __init__(self, config):
        self.config = config

    def launch_spec(self):
        if not self.config.enabled:
            return None
        return LaunchSpec(
            "Acme robot service",
            ("python", "-m", "robot_modules.acme_bot.tools.service"),
            {"ACME_PROFILE": str(self.config.option("profile"))},
        )

    def hold(self):
        # Safe before startup and safe to call repeatedly.
        send_local_hold_if_service_exists()

    def public_manifest(self):
        ...

    def operator_environment(self):
        return {}
```

The hardware service should be independently testable and should retain final authority over deadman, command age, watchdog, limits, feedback error, and Hold. A browser message must never be a direct motor write.

## Semantic protocol

`robot-teleop/v1` is JSON and transport-neutral:

```json
{
  "protocol": "robot-teleop/v1",
  "type": "teleop_command",
  "module": "acme_bot",
  "channel": "robot",
  "control_space": "cartesian_tool_velocity",
  "command": "move",
  "sequence": 418,
  "sent_unix_ms": 1785712345678,
  "payload": {
    "linear": [0.1, 0.0, 0.0],
    "angular": [0.0, 0.0, 0.0],
    "frame": "base",
    "deadman": true
  }
}
```

System operations use `control_space: "system"`, such as `command: "hold"`. Status uses the same version and module identity:

```json
{
  "protocol": "robot-teleop/v1",
  "type": "teleop_status",
  "module": "acme_bot",
  "connected": true,
  "enabled": false,
  "state": "hold",
  "fault": "",
  "sequence": 902
}
```

Recommended control-space names include `joint_position`, `joint_velocity`, `cartesian_tool_pose`, `cartesian_tool_velocity`, `gripper_position`, `gripper_velocity`, `base_velocity`, and `leader_raw_position`. A module may add a specific dotted name. Payload shape and command verbs belong to that module until a future protocol version standardizes them.

`GenericRobotOperator` works unchanged when the local hardware service already speaks this protocol. Extend it only to translate a legacy packet/status format, declare auxiliary camera discovery, or validate module-specific view settings. The SO-101 operator demonstrates a legacy translation without leaking those packets into the common server.

A module advertising `restart` should increment the optional integer `restart_marker` after a completed hardware reconnect. This lets the common operator distinguish success from the eight-second restart timeout.

## Leader/follower and other inputs

An input device is a producer, not a second hardware architecture:

```text
keyboard ─┐
gamepad  ─┼─> semantic command envelope -> module operator -> robot service
leader   ─┘
```

A calibrated leader should preferably produce joint or Cartesian semantics. If raw leader encoders are needed, advertise `leader_raw_position` and keep leader/follower calibration inside the robot module. Browser ownership, freshness, watchdog, and Hold remain identical for every producer.

## Browser module

The public `web` entrypoint exports one factory:

```javascript
export async function createRobotModule(manifest) {
  return {
    renderer: null,
    async mount({ settingsRoot, setupRoot, viewRoot, statusRoot, floatingRoot }) {},
    bind({ sendViewSettings, updateStatus }) {},
    extendViewSettings(payload) { return payload; },
    restoreViewSettings(saved) {},
    statusSummary() { return manifest.label; },
    renderStatus() {},
    start() {},
    stop() {},
  };
}
```

Mount only capability-specific controls into the supplied regions; do not create another settings sidebar. `settingsRoot` is for routine operator controls. Put infrequent connection/safety setup in `setupRoot` and calibration/manual registration in `viewRoot`; both are shown only in **Advanced Setup**. Shared stream quality, mouse navigation, head tracking, RGB behavior, background, and latency diagnostics remain in the common UI.

For an overlay, expose a renderer descriptor with `modelUrls`, colors, base/accent/calibration links, and optional mask-chain metadata. The common renderer loads the named links without knowing the robot model. A module may also declare a visual workspace envelope:

```javascript
workspaceBoundary: {
  shape: "cylinder",
  anchorLink: "base_link",
  innerRadius: 0.08,
  outerRadius: 0.5,
  minimumHeight: 0.02,
  maximumHeight: 0.6,
}
```

The common renderer can draw this descriptor beside its robot-neutral flat support surface. It is orientation feedback only. Deadman, joint/workspace limits, collision/contact policy, and actual stopping stay inside the robot module's hardware process. Robots may omit the descriptor or use a future module-specific renderer when a cylinder is inappropriate.

The operator may expose `persistent_view_settings()` for module-owned booleans/enums that are safe to save as site defaults. Never include momentary actions such as enable, restart, calibrate, save, clear, or return-to-rest: loading a browser setup must be non-mutating.

Auxiliary views are declared in `capabilities.views`, described by the operator, and served as `/robot-view/<view-id>`. Device discovery stays private and local.

## Godot module

`RobotModule.tscn` should add its implementation to the `robot_module` group. The common runtime calls methods only when they exist:

- `apply_remote_settings(payload)`
- `start_full_calibration(from_editor)`
- `refine_calibration(from_editor)`
- `return_to_rest()`
- `clear_calibration()`
- `save_calibration_checkpoint()`
- `restore_calibration_checkpoint()`
- `get_calibration_status()`

Geometry belongs below the common `WorldLevelAnchor`. Registration should remain module-owned and serial/site-specific. Missing methods mean missing capabilities, not a startup failure.

## Definition of done

- Vision-only mode still launches after the module directory is removed.
- No module name, link, motor, or wire command appears in shared folders.
- Manifest accurately lists every accepted control space/action and no unsupported ones.
- Disabled configuration launches no hardware process.
- Hold is idempotent and is exercised on stop/crash/preemption.
- Stale, wrong-module, wrong-channel, and unnegotiated commands are rejected.
- Hardware-free tests cover translation and status validation.
- Mesh/source licenses are included and local physical identities remain ignored.
