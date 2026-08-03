from __future__ import annotations

import re
import time
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

PROTOCOL = "robot-teleop/v1"
CHANNELS = frozenset({"robot", "view", "camera"})
IDENTIFIER = re.compile(r"^[a-z][a-z0-9_.-]{0,63}$")


class TeleopProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class TeleopCommand:
    """Versioned, transport-neutral command negotiated through module capabilities."""

    module: str
    channel: str
    control_space: str
    command: str
    sequence: int
    sent_unix_ms: int
    payload: dict[str, Any] = field(default_factory=dict)
    protocol: str = PROTOCOL
    type: str = "teleop_command"

    @classmethod
    def from_mapping(
        cls,
        value: Mapping[str, Any],
        *,
        allowed_control_spaces: set[str] | frozenset[str] | None = None,
        now_unix_ms: int | None = None,
        max_age_ms: int | None = None,
    ) -> TeleopCommand:
        if value.get("protocol") != PROTOCOL or value.get("type") != "teleop_command":
            raise TeleopProtocolError("unsupported teleoperation protocol or message type")
        module = str(value.get("module", ""))
        channel = str(value.get("channel", ""))
        control_space = str(value.get("control_space", ""))
        command = str(value.get("command", ""))
        if not IDENTIFIER.fullmatch(module):
            raise TeleopProtocolError("module must be a safe identifier")
        if channel not in CHANNELS:
            raise TeleopProtocolError(f"channel must be one of {sorted(CHANNELS)}")
        if not IDENTIFIER.fullmatch(control_space) or not IDENTIFIER.fullmatch(command):
            raise TeleopProtocolError("control_space and command must be safe identifiers")
        if allowed_control_spaces is not None and control_space not in allowed_control_spaces:
            raise TeleopProtocolError(f"control space {control_space!r} was not negotiated")
        try:
            sequence = int(value["sequence"])
            sent_unix_ms = int(value["sent_unix_ms"])
        except (KeyError, TypeError, ValueError) as exc:
            raise TeleopProtocolError("sequence and sent_unix_ms must be integers") from exc
        if sequence < 0 or sent_unix_ms < 0:
            raise TeleopProtocolError("sequence and sent_unix_ms must not be negative")
        payload = value.get("payload", {})
        if not isinstance(payload, dict):
            raise TeleopProtocolError("payload must be an object")
        if max_age_ms is not None:
            now = int(time.time() * 1000) if now_unix_ms is None else now_unix_ms
            age = now - sent_unix_ms
            if age > max_age_ms or age < -max_age_ms:
                raise TeleopProtocolError(f"command timestamp is outside the {max_age_ms}ms window")
        return cls(module, channel, control_space, command, sequence, sent_unix_ms, dict(payload))

    def to_mapping(self) -> dict[str, Any]:
        return {
            "protocol": self.protocol,
            "type": self.type,
            "module": self.module,
            "channel": self.channel,
            "control_space": self.control_space,
            "command": self.command,
            "sequence": self.sequence,
            "sent_unix_ms": self.sent_unix_ms,
            "payload": self.payload,
        }
