from __future__ import annotations

import json
import time
from pathlib import Path

import pytest

from robot_teleop.modules import (
    RobotModuleManifest,
    load_robot_modules,
    public_robot_module,
    robot_module_entrypoint,
)
from robot_teleop.operator import GenericRobotOperator, RobotCommandError, load_robot_operator
from robot_teleop.protocol import PROTOCOL, TeleopCommand, TeleopProtocolError


def manifest_data(**overrides):
    data = {
        "schema_version": 1,
        "id": "testbot",
        "label": "Test robot",
        "version": "1.0.0",
        "description": "Hardware-free test module",
        "capabilities": {
            "control_spaces": ["joint_velocity", "cartesian_tool_velocity"],
            "inputs": ["keyboard", "leader"],
            "actions": ["enable", "hold", "restart"],
            "views": ["tool_rgb"],
            "features": ["overlay"],
        },
        "entrypoints": {
            "python": "testbot.python.adapter:TestRobotAdapter",
            "operator": "testbot.python.operator:TestRobotOperator",
            "godot": "res://robot_modules/testbot/godot/RobotModule.tscn",
            "web": "/robot-modules/testbot/web/module.js",
        },
        "state_files": ["testbot_registration.json"],
    }
    data.update(overrides)
    return data


def write_manifest(path: Path, data: dict) -> RobotModuleManifest:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data), encoding="utf-8")
    return RobotModuleManifest.load(path)


def command(
    *,
    control_space: str = "joint_velocity",
    action: str = "move",
    module: str = "testbot",
    channel: str = "robot",
    sent_unix_ms: int | None = None,
) -> dict:
    return {
        "protocol": PROTOCOL,
        "type": "teleop_command",
        "module": module,
        "channel": channel,
        "control_space": control_space,
        "command": action,
        "sequence": 7,
        "sent_unix_ms": sent_unix_ms or int(time.time() * 1000),
        "payload": {"velocities": [0.1, 0.0]},
    }


def test_external_robot_module_is_discovered_and_private_entrypoints_stay_private(
    tmp_path: Path,
):
    root = tmp_path / "modules"
    module = root / "testbot"
    (module / "python").mkdir(parents=True)
    (module / "__init__.py").write_text("", encoding="utf-8")
    (module / "python" / "__init__.py").write_text("", encoding="utf-8")
    (module / "python" / "adapter.py").write_text(
        "class TestRobotAdapter:\n"
        "    name = 'testbot'\n"
        "    def __init__(self, config=None): self.config = config\n",
        encoding="utf-8",
    )
    (module / "python" / "operator.py").write_text(
        "from robot_teleop.operator import GenericRobotOperator\n"
        "class TestRobotOperator(GenericRobotOperator):\n"
        "    pass\n",
        encoding="utf-8",
    )
    write_manifest(module / "robot.json", manifest_data())

    discovered = load_robot_modules((str(root),))
    assert any(item.id == "testbot" for item in discovered)
    public = public_robot_module("testbot", (str(root),))
    assert public["entrypoints"] == {
        "godot": "res://robot_modules/testbot/godot/RobotModule.tscn",
        "web": "/robot-modules/testbot/web/module.js",
    }
    assert "state_files" not in public
    assert robot_module_entrypoint("testbot", "operator", (str(root),)).__name__ == (
        "TestRobotOperator"
    )


@pytest.mark.parametrize("unsafe", ["..", ".", "../secret.json", "nested/state.json"])
def test_manifest_rejects_unsafe_state_file_names(tmp_path: Path, unsafe: str):
    data = manifest_data(state_files=[unsafe])
    with pytest.raises(ValueError, match="safe file names"):
        case_name = unsafe.replace("/", "_").replace(".", "dot") or "empty"
        write_manifest(tmp_path / case_name / "robot.json", data)


def test_manifest_requires_an_idempotent_hold_action(tmp_path: Path):
    data = manifest_data()
    data["capabilities"]["actions"] = ["enable"]
    with pytest.raises(ValueError, match="safety action 'hold'"):
        write_manifest(tmp_path / "testbot" / "robot.json", data)


def test_generic_operator_accepts_only_negotiated_module_commands(tmp_path: Path):
    manifest = write_manifest(tmp_path / "testbot" / "robot.json", manifest_data())
    operator = GenericRobotOperator(manifest)

    assert operator.validate_command(command())["payload"]["velocities"] == [0.1, 0.0]
    assert operator.validate_command(command(control_space="system", action="hold"))[
        "command"
    ] == "hold"
    with pytest.raises(RobotCommandError, match="not negotiated"):
        operator.validate_command(command(control_space="base_velocity"))
    with pytest.raises(RobotCommandError, match="not negotiated"):
        operator.validate_command(command(control_space="system", action="self_destruct"))
    with pytest.raises(RobotCommandError, match="different robot module or channel"):
        operator.validate_command(command(module="other"))
    with pytest.raises(RobotCommandError, match="different robot module or channel"):
        operator.validate_command(command(channel="camera"))


def test_generic_operator_restart_state_completes_or_times_out(tmp_path: Path):
    manifest = write_manifest(tmp_path / "testbot" / "robot.json", manifest_data())
    operator = GenericRobotOperator(manifest)

    restarting, clear = operator.browser_status(
        {"state": "hold", "restart_marker": 3},
        10.0,
        1.0,
        3,
    )
    assert restarting["state"] == "restarting"
    assert not clear

    completed, clear = operator.browser_status(
        {"state": "hold", "restart_marker": 4},
        10.0,
        1.0,
        3,
    )
    assert completed["state"] == "hold"
    assert clear

    timed_out, clear = operator.browser_status(
        {"state": "offline", "restart_marker": 3},
        10.0,
        9.0,
        3,
    )
    assert timed_out["state"] == "fault"
    assert clear


def test_protocol_rejects_stale_and_future_commands():
    now = 10_000
    with pytest.raises(TeleopProtocolError, match="timestamp"):
        TeleopCommand.from_mapping(
            command(sent_unix_ms=9_000),
            now_unix_ms=now,
            max_age_ms=250,
        )
    with pytest.raises(TeleopProtocolError, match="timestamp"):
        TeleopCommand.from_mapping(
            command(sent_unix_ms=11_000),
            now_unix_ms=now,
            max_age_ms=250,
        )


def test_shared_runtime_contains_no_so101_wire_protocol_tokens():
    root = Path(__file__).resolve().parents[1]
    shared_roots = [root / name for name in ("robot_teleop", "web", "godot", "tools")]
    forbidden = ("so101", "so-101", "/arm-control", "arm_cartesian_velocity", "arm_hold")
    violations = []
    for shared_root in shared_roots:
        for path in shared_root.rglob("*"):
            if path.suffix not in {".py", ".js", ".gd", ".tscn", ".html"}:
                continue
            text = path.read_text(encoding="utf-8", errors="replace").lower()
            for token in forbidden:
                if token in text:
                    violations.append(f"{path.relative_to(root)}: {token}")
    assert violations == []


def test_so101_installation_defaults_never_include_mutating_actions():
    operator = load_robot_operator("so101")
    persistent = set(operator.persistent_view_settings())
    assert "arm_idle_return_enabled" in persistent
    assert "arm_following_error_safety_enabled" in persistent
    assert "arm_measured_feedback_enabled" not in persistent
    assert "arm_freeze_overlay_on_stale_enabled" not in persistent
    assert "calibrate_robot_position" not in persistent
    assert "manual_claw_calibration_save" not in persistent
