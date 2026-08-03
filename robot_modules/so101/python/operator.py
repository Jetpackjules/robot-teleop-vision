from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any, ClassVar

from robot_modules.so101.tools.so101_arm_common import (
    ArmProtocolError,
    validate_browser_arm_message,
)
from robot_teleop.modules import RobotModuleManifest
from robot_teleop.operator import AuxiliaryView, GenericRobotOperator, RobotCommandError
from robot_teleop.protocol import PROTOCOL, TeleopCommand, TeleopProtocolError


class So101Operator(GenericRobotOperator):
    command_port = 4248
    status_port = 4249
    auxiliary_views: ClassVar[Mapping[str, AuxiliaryView]] = {
        "wrist_rgb": AuxiliaryView("wrist_rgb", "Wrist camera"),
    }
    _motion_types = frozenset(
        {
            "arm_command",
            "arm_cartesian_velocity",
            "arm_joint_velocity",
            "arm_tool_velocity",
        }
    )

    def __init__(self, manifest: RobotModuleManifest) -> None:
        super().__init__(manifest)

    def validate_command(self, data: object) -> dict[str, Any]:
        if isinstance(data, dict) and data.get("protocol") == PROTOCOL:
            return self._translate_protocol_command(data)
        try:
            return validate_browser_arm_message(data)
        except ArmProtocolError as exc:
            raise RobotCommandError(str(exc)) from exc

    def _translate_protocol_command(self, data: dict[str, Any]) -> dict[str, Any]:
        try:
            command = TeleopCommand.from_mapping(
                data,
                allowed_control_spaces=self.control_spaces | {"system"},
            )
        except TeleopProtocolError as exc:
            raise RobotCommandError(str(exc)) from exc
        if command.module != self.module_id or command.channel != "robot":
            raise RobotCommandError("command targets a different robot module or channel")
        payload = dict(command.payload)
        if command.control_space == "system":
            if command.command not in self.actions | {"status"}:
                raise RobotCommandError(
                    f"SO-101 system action {command.command!r} was not negotiated"
                )
            native = {
                "hold": {"type": "arm_hold"},
                "enable": {"type": "arm_enable", "source": str(payload.get("source", "keyboard"))},
                "restart": {"type": "arm_restart"},
                "status": {"type": "arm_status_request"},
                "return_rest": {"type": "arm_return_to_rest"},
            }.get(command.command)
            if native is None:
                raise RobotCommandError(f"unsupported SO-101 system action {command.command!r}")
            return self.validate_command(native)
        if command.control_space == "gripper_velocity":
            native = {
                "type": "arm_cartesian_velocity",
                "seq": command.sequence,
                "sent_unix_ms": command.sent_unix_ms,
                "linear": [0.0, 0.0, 0.0],
                "angular": [0.0, 0.0, 0.0],
                "wrist": [0.0, 0.0],
                "gripper": payload.get("velocity", payload.get("gripper", 0.0)),
                "frame": "base",
                "angular_frame": "base",
                "precision": bool(payload.get("precision", False)),
                "deadman": payload.get("deadman", False),
            }
            return self.validate_command(native)
        native_type = {
            "cartesian_tool_velocity": "arm_cartesian_velocity",
            "joint_velocity": "arm_joint_velocity",
            "tool_velocity": "arm_tool_velocity",
            "leader_raw_position": "arm_command",
        }.get(command.control_space)
        if not native_type:
            raise RobotCommandError(f"unsupported SO-101 control space {command.control_space!r}")
        return self.validate_command(
            {
                "type": native_type,
                "seq": command.sequence,
                "sent_unix_ms": command.sent_unix_ms,
                **payload,
            }
        )

    def validate_status(self, data: object) -> dict[str, Any] | None:
        if not isinstance(data, dict) or data.get("type") != "arm_status":
            return None
        return dict(data)

    def default_status(self) -> dict[str, Any]:
        return {
            "type": "arm_status",
            "follower_connected": False,
            "state": "offline",
            "armed": False,
            "torque_enabled": False,
            "fault": "follower service unavailable",
        }

    def hold_message(self) -> dict[str, Any]:
        return {"type": "arm_hold"}

    def is_motion(self, message: Mapping[str, Any]) -> bool:
        return message.get("type") in self._motion_types

    def is_restart(self, message: Mapping[str, Any]) -> bool:
        return message.get("type") == "arm_restart"

    def is_status_request(self, message: Mapping[str, Any]) -> bool:
        return message.get("type") == "arm_status_request"

    def restart_marker(self, status: Mapping[str, Any]) -> int:
        return int(status.get("restart_count", 0))

    def needs_transport_timing(self, message: Mapping[str, Any]) -> bool:
        return self.is_motion(message)

    def add_control_session(self, message: dict[str, Any], controller_id: str) -> None:
        message["control_session"] = controller_id

    def recovery_message(
        self,
        message: Mapping[str, Any],
        status: Mapping[str, Any],
        controller_id: str,
    ) -> dict[str, Any] | None:
        recoverable = (
            "command watchdog expired",
            "received keyboard command is older than watchdog limit",
        )
        if (
            not self.is_motion(message)
            or message.get("deadman") is not True
            or status.get("state") != "hold"
            or status.get("control_session") != controller_id
            or not any(str(status.get("message", "")).startswith(reason) for reason in recoverable)
        ):
            return None
        return {
            "type": "arm_enable",
            "source": "keyboard",
            "control_session": controller_id,
        }

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
        packet["follower_responsive"] = responsive
        if status_age_ms is not None and not responsive:
            packet.update(
                follower_connected=False,
                armed=False,
                state="unresponsive",
                fault=f"follower status stalled for {status_age_ms:.0f} ms",
            )
        clear_restart = False
        if restart_age_seconds is not None:
            completed = (
                int(packet.get("restart_count", 0)) > restart_baseline_count
                and packet.get("state") != "restarting"
            )
            if completed:
                clear_restart = True
            elif restart_age_seconds <= 8.0:
                packet.update(
                    state="restarting",
                    armed=False,
                    fault="",
                    message="Restarting follower motor connection...",
                )
            else:
                clear_restart = True
                packet.update(
                    state="fault",
                    armed=False,
                    fault="follower restart timed out; check USB power/cable and retry",
                )
        return packet, clear_restart

    def numeric_view_settings(self) -> Mapping[str, tuple[float, float]]:
        return {
            "manual_wrist_flex_trim_degrees": (-90.0, 90.0),
            "manual_wrist_roll_trim_degrees": (-180.0, 180.0),
            "manual_wrist_roll_direction": (-1.0, 1.0),
            "manual_tool_x": (-0.05, 0.05),
            "manual_tool_y": (-0.05, 0.05),
            "manual_tool_z": (-0.05, 0.05),
            "manual_tool_roll": (-180.0, 180.0),
            "manual_tool_pitch": (-180.0, 180.0),
            "manual_tool_yaw": (-180.0, 180.0),
            "manual_opening_offset_degrees": (-35.0, 35.0),
            "manual_opening_scale": (0.5, 1.5),
            "manual_overlay_opacity": (0.05, 0.9),
        }

    def boolean_view_settings(self) -> tuple[str, ...]:
        return (
            "calibrate_robot_position",
            "refine_robot_joint_alignment",
            "cancel_robot_position_calibration",
            "arm_measured_feedback_enabled",
            "arm_target_ghost_enabled",
            "arm_following_error_safety_enabled",
            "arm_freeze_overlay_on_stale_enabled",
            "arm_d455_visual_correction_enabled",
            "arm_idle_return_enabled",
            "manual_claw_calibration_begin",
            "manual_claw_calibration_reset",
            "manual_claw_calibration_cancel",
            "manual_claw_calibration_save",
        )

    def persistent_view_settings(self) -> tuple[str, ...]:
        return (
            "arm_measured_feedback_enabled",
            "arm_target_ghost_enabled",
            "arm_following_error_safety_enabled",
            "arm_freeze_overlay_on_stale_enabled",
            "arm_d455_visual_correction_enabled",
            "arm_idle_return_enabled",
        )

    def resolve_auxiliary_device(self, view_id: str, configured: str = "") -> str:
        if view_id != "wrist_rgb":
            return ""
        requested = str(configured or "").strip()
        if requested.lower() in {"off", "none", "disabled"}:
            return ""
        if requested:
            path = Path(requested).expanduser()
            return str(path.resolve()) if path.exists() else str(path)
        by_id = Path("/dev/v4l/by-id")
        if by_id.is_dir():
            matches = sorted(by_id.glob("*USB2.0_CAM1*video-index0"))
            if matches:
                return str(matches[0])
        for name_path in sorted(Path("/sys/class/video4linux").glob("video*/name")):
            try:
                if "USB2.0_CAM1" in name_path.read_text(encoding="utf-8", errors="replace"):
                    return f"/dev/{name_path.parent.name}"
            except OSError:
                continue
        return ""
