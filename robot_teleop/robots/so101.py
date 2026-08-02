from __future__ import annotations

import json
import socket
import sys
from pathlib import Path

from robot_teleop.config import REPO_ROOT, RobotConfig
from robot_teleop.interfaces import LaunchSpec
from robot_teleop.registry import register


class So101RobotAdapter:
    name = "so101"

    def __init__(self, config: RobotConfig) -> None:
        self.config = config

    def launch_spec(self) -> LaunchSpec | None:
        if not self.config.enabled:
            return None
        profile = Path(self.config.profile).expanduser()
        if not profile.is_absolute():
            profile = (REPO_ROOT / profile).resolve()
        if not profile.is_file():
            raise FileNotFoundError(f"SO-101 profile not found: {profile}")
        python = str(Path(self.config.python).expanduser()) if self.config.python else sys.executable
        command = [
            python,
            str(REPO_ROOT / "tools" / "so101_follower_service.py"),
            "--profile",
            str(profile),
        ]
        if self.config.dry_run:
            command.append("--dry-run")
        if self.config.hold_on_connect:
            command.append("--hold-on-connect")
        if self.config.ignore_motor_6:
            command.append("--ignore-motor-6")
        return LaunchSpec("SO-101 follower", tuple(command))

    def hold(self) -> None:
        if not self.config.enabled:
            return
        profile = Path(self.config.profile).expanduser()
        if not profile.is_absolute():
            profile = (REPO_ROOT / profile).resolve()
        data = json.loads(profile.read_text(encoding="utf-8"))
        command_port = int(data.get("command_port", 4248))
        payload = json.dumps({"type": "arm_hold"}).encode("utf-8")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as peer:
            peer.sendto(payload, ("127.0.0.1", command_port))


register("robot", So101RobotAdapter.name, So101RobotAdapter)
