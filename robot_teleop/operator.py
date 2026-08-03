from __future__ import annotations

import time
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from robot_teleop.modules import RobotModuleManifest, robot_module_entrypoint, robot_module_manifest
from robot_teleop.protocol import PROTOCOL, TeleopCommand, TeleopProtocolError


class RobotCommandError(ValueError):
    """A command rejected at the module-independent operator boundary."""


@dataclass(frozen=True)
class AuxiliaryView:
    id: str
    label: str
    stream: str = "mjpeg"


class GenericRobotOperator:
    """Default transport contract for modules that speak robot-teleop/v1 directly."""

    command_port = 4248
    status_port = 4249
    status_timeout_ms = 500.0
    auxiliary_views: Mapping[str, AuxiliaryView] = {}

    def __init__(self, manifest: RobotModuleManifest | None = None) -> None:
        self.manifest = manifest
        self.module_id = manifest.id if manifest else "disabled"
        capabilities = manifest.capabilities if manifest else {}
        self.control_spaces = frozenset(capabilities.get("control_spaces", ()))
        self.actions = frozenset(capabilities.get("actions", ()))

    def validate_command(self, data: object) -> dict[str, Any]:
        if not isinstance(data, dict):
            raise RobotCommandError("robot command must be an object")
        try:
            command = TeleopCommand.from_mapping(
                data,
                allowed_control_spaces=self.control_spaces | {"system"},
            )
        except TeleopProtocolError as exc:
            raise RobotCommandError(str(exc)) from exc
        if command.module != self.module_id or command.channel != "robot":
            raise RobotCommandError("command targets a different robot module or channel")
        if (
            command.control_space == "system"
            and command.command not in self.actions | {"status"}
        ):
            raise RobotCommandError(f"system action {command.command!r} was not negotiated")
        return command.to_mapping()

    def validate_status(self, data: object) -> dict[str, Any] | None:
        if (
            not isinstance(data, dict)
            or data.get("protocol") != PROTOCOL
            or data.get("type") != "teleop_status"
            or data.get("module") != self.module_id
        ):
            return None
        return dict(data)

    def default_status(self) -> dict[str, Any]:
        return {
            "protocol": PROTOCOL,
            "type": "teleop_status",
            "module": self.module_id,
            "connected": False,
            "state": "offline",
            "enabled": False,
            "fault": "robot service unavailable",
        }

    def hold_message(self) -> dict[str, Any]:
        return self._system_command("hold")

    def _system_command(self, command: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        return {
            "protocol": PROTOCOL,
            "type": "teleop_command",
            "module": self.module_id,
            "channel": "robot",
            "control_space": "system",
            "command": command,
            "sequence": 0,
            "sent_unix_ms": int(time.time() * 1000),
            "payload": payload or {},
        }

    def is_motion(self, message: Mapping[str, Any]) -> bool:
        return message.get("type") == "teleop_command" and message.get("control_space") != "system"

    def is_restart(self, message: Mapping[str, Any]) -> bool:
        return (
            message.get("type") == "teleop_command"
            and message.get("control_space") == "system"
            and message.get("command") == "restart"
        )

    def is_status_request(self, message: Mapping[str, Any]) -> bool:
        return (
            message.get("type") == "teleop_command"
            and message.get("control_space") == "system"
            and message.get("command") == "status"
        )

    def restart_marker(self, status: Mapping[str, Any]) -> int:
        """Return a monotonic marker used to recognize a completed restart."""

        try:
            return int(status.get("restart_marker", 0))
        except (TypeError, ValueError):
            return 0

    def needs_transport_timing(self, message: Mapping[str, Any]) -> bool:
        return self.is_motion(message) and isinstance(message.get("sent_unix_ms"), int)

    def add_control_session(self, message: dict[str, Any], controller_id: str) -> None:
        payload = message.setdefault("payload", {})
        if isinstance(payload, dict):
            payload["control_session"] = controller_id

    def recovery_message(
        self,
        message: Mapping[str, Any],
        status: Mapping[str, Any],
        controller_id: str,
    ) -> dict[str, Any] | None:
        return None

    def browser_status(
        self,
        status: Mapping[str, Any],
        status_age_ms: float | None,
        restart_age_seconds: float | None,
        restart_baseline_count: int,
    ) -> tuple[dict[str, Any], bool]:
        packet = dict(status)
        packet["status_age_ms"] = status_age_ms
        responsive = status_age_ms is not None and status_age_ms <= self.status_timeout_ms
        packet["responsive"] = responsive
        if status_age_ms is not None and not responsive:
            packet.update(
                connected=False,
                enabled=False,
                state="unresponsive",
                fault=f"robot status stalled for {status_age_ms:.0f} ms",
            )
        clear_restart = False
        if restart_age_seconds is not None:
            completed = (
                self.restart_marker(packet) > restart_baseline_count
                and packet.get("state") != "restarting"
            )
            if completed:
                clear_restart = True
            elif restart_age_seconds <= 8.0:
                packet.update(
                    state="restarting",
                    enabled=False,
                    fault="",
                    message="Restarting robot connection...",
                )
            else:
                clear_restart = True
                packet.update(
                    state="fault",
                    enabled=False,
                    fault="robot restart timed out; inspect hardware and retry",
                )
        return packet, clear_restart

    def numeric_view_settings(self) -> Mapping[str, tuple[float, float]]:
        return {}

    def boolean_view_settings(self) -> tuple[str, ...]:
        return ()

    def enum_view_settings(self) -> Mapping[str, frozenset[str]]:
        return {}

    def persistent_view_settings(self) -> tuple[str, ...]:
        """Module settings that are safe to store as installation defaults.

        Action flags such as calibrate, restart, and save must never be
        persisted because applying a saved setup must remain non-mutating.
        """

        return ()

    def resolve_auxiliary_device(self, view_id: str, configured: str = "") -> str:
        return str(configured or "")


def load_robot_operator(
    module_id: str,
    extra_paths: tuple[str, ...] = (),
) -> GenericRobotOperator:
    if module_id == "disabled":
        return GenericRobotOperator()
    manifest = robot_module_manifest(module_id, extra_paths)
    factory = robot_module_entrypoint(module_id, "operator", extra_paths)
    if factory is None:
        return GenericRobotOperator(manifest)
    operator = factory(manifest)
    if not isinstance(operator, GenericRobotOperator):
        raise TypeError(
            f"robot module {module_id!r} operator must extend GenericRobotOperator"
        )
    return operator
