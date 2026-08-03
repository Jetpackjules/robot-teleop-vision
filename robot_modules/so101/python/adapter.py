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

    def _profile_path(self) -> Path:
        profile_value = str(self.config.option("profile", ""))
        if not profile_value:
            raise ValueError("SO-101 robot option 'profile' is required when hardware is enabled")
        profile = Path(profile_value).expanduser()
        return profile if profile.is_absolute() else (REPO_ROOT / profile).resolve()

    def launch_spec(self) -> LaunchSpec | None:
        if not self.config.enabled:
            return None
        profile = self._profile_path()
        if not profile.is_file():
            raise FileNotFoundError(f"SO-101 profile not found: {profile}")
        python_value = str(self.config.option("python", ""))
        python = str(Path(python_value).expanduser()) if python_value else sys.executable
        command = [
            python,
            str(Path(__file__).resolve().parents[1] / "tools" / "so101_follower_service.py"),
            "--profile",
            str(profile),
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
        profile = self._profile_path()
        data = json.loads(profile.read_text(encoding="utf-8"))
        command_port = int(data.get("command_port", 4248))
        payload = json.dumps({"type": "arm_hold"}).encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
            peer.sendto(payload, ("127.0.0.1", command_port))

    def public_manifest(self) -> dict:
        manifest_path = Path(__file__).resolve().parents[1] / "robot.json"
        return RobotModuleManifest.load(manifest_path).public_dict()

    def operator_environment(self) -> dict[str, str]:
        environment: dict[str, str] = {}
        if self.config.enabled:
            data = json.loads(self._profile_path().read_text(encoding="utf-8"))
            environment.update(
                ROBOT_TELEOP_COMMAND_PORT=str(int(data.get("command_port", 4248))),
                ROBOT_TELEOP_STATUS_PORT=str(int(data.get("status_port", 4249))),
            )
        view_devices = {
            "wrist_rgb": str(self.config.option("wrist_camera_device", "")),
        }
        environment["ROBOT_TELEOP_VIEW_DEVICES"] = json.dumps(view_devices)
        return environment
