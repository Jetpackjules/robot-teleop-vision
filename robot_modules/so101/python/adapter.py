from __future__ import annotations

import json
import socket
import sys
from pathlib import Path

from robot_teleop.config import REPO_ROOT, RobotConfig
from robot_teleop.interfaces import LaunchSpec
from robot_teleop.modules import RobotModuleManifest


class So101RobotAdapter:
    name = "so101"

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self._resolved_transport_ports: dict[str, int] | None = None

    def _profile_path(self) -> Path:
        profile_value = str(self.config.option("profile", ""))
        if not profile_value:
            raise ValueError("SO-101 robot option 'profile' is required when hardware is enabled")
        profile = Path(profile_value).expanduser()
        return profile if profile.is_absolute() else (REPO_ROOT / profile).resolve()

    def _transport_ports(self) -> dict[str, int]:
        if self._resolved_transport_ports is not None:
            return dict(self._resolved_transport_ports)
        data = json.loads(self._profile_path().read_text(encoding="utf-8"))
        ports: dict[str, int] = {}
        for name, default in (
            ("command_port", 4248),
            ("status_port", 4249),
            ("godot_status_port", 4250),
            ("editor_status_port", 4252),
        ):
            value = data.get(name, default)
            if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 65535:
                raise ValueError(
                    f"SO-101 profile {name} must be an integer from 1 to 65535 "
                    "for the web/editor stack; telemetry cannot be disabled here"
                )
            ports[name] = value
        if len(set(ports.values())) != len(ports):
            raise ValueError("SO-101 command/status/runtime/editor UDP ports must all be distinct")
        self._resolved_transport_ports = dict(ports)
        return ports

    def godot_configuration(
        self,
        *,
        calibration_status_port: int,
        reserved_udp_ports: dict[str, int] | None = None,
    ) -> dict[str, int]:
        """Export only module-owned local transport settings, never profile secrets."""
        if not self.config.enabled:
            return {}
        ports = self._transport_ports()
        if calibration_status_port in ports.values():
            raise ValueError(
                "SO-101 profile UDP ports must differ from stack.calibration_status_port "
                f"({calibration_status_port})"
            )
        for label, reserved_port in (reserved_udp_ports or {}).items():
            if (
                isinstance(reserved_port, bool)
                or not isinstance(reserved_port, int)
                or not 1 <= reserved_port <= 65535
            ):
                raise ValueError(f"{label} must be an integer UDP port from 1 to 65535")
            if reserved_port in ports.values():
                raise ValueError(
                    f"SO-101 profile UDP ports conflict with {label} ({reserved_port}); "
                    "choose distinct follower command/status/telemetry ports"
                )
        return {
            "command_port": ports["command_port"],
            "status_port": ports["status_port"],
            "telemetry_port": ports["godot_status_port"],
            "editor_telemetry_port": ports["editor_status_port"],
        }

    def launch_spec(self) -> LaunchSpec | None:
        if not self.config.enabled:
            return None
        profile = self._profile_path()
        if not profile.is_file():
            raise FileNotFoundError(f"SO-101 profile not found: {profile}")
        ports = self._transport_ports()
        python_value = str(self.config.option("python", ""))
        python = str(Path(python_value).expanduser()) if python_value else sys.executable
        command = [
            python,
            str(Path(__file__).resolve().parents[1] / "tools" / "so101_follower_service.py"),
            "--profile",
            str(profile),
            "--command-port",
            str(ports["command_port"]),
            "--status-port",
            str(ports["status_port"]),
            "--godot-status-port",
            str(ports["godot_status_port"]),
            "--editor-status-port",
            str(ports["editor_status_port"]),
        ]
        if bool(self.config.option("dry_run", False)):
            command.append("--dry-run")
        if bool(self.config.option("hold_on_connect", True)):
            command.append("--hold-on-connect")
        if bool(self.config.option("ignore_motor_6", False)):
            command.append("--ignore-motor-6")
        return LaunchSpec("SO-101 follower", tuple(command))

    def hold(self) -> None:
        if not self.config.enabled:
            return
        command_port = self._transport_ports()["command_port"]
        payload = json.dumps({"type": "arm_hold"}).encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
            peer.sendto(payload, ("127.0.0.1", command_port))

    def public_manifest(self) -> dict:
        manifest_path = Path(__file__).resolve().parents[1] / "robot.json"
        return RobotModuleManifest.load(manifest_path).public_dict()

    def operator_environment(self) -> dict[str, str]:
        environment: dict[str, str] = {}
        if self.config.enabled:
            ports = self._transport_ports()
            environment.update(
                ROBOT_TELEOP_COMMAND_PORT=str(ports["command_port"]),
                ROBOT_TELEOP_STATUS_PORT=str(ports["status_port"]),
            )
        view_devices = {
            "wrist_rgb": str(self.config.option("wrist_camera_device", "")),
        }
        environment["ROBOT_TELEOP_VIEW_DEVICES"] = json.dumps(view_devices)
        return environment
