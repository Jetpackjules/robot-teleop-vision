from __future__ import annotations

import os
import socket
import sys
from collections.abc import Mapping
from pathlib import Path

DEFAULT_JOURNAL_SOCKET = Path("/run/systemd/journal/socket")


def stream_is_journal(environment: Mapping[str, str]) -> bool:
    """Return true only when JOURNAL_STREAM still describes stdout or stderr."""

    value = environment.get("JOURNAL_STREAM", "")
    try:
        expected = tuple(int(part) for part in value.split(":", 1))
        if len(expected) != 2:
            return False
    except ValueError:
        return False
    for stream in (sys.stdout, sys.stderr):
        try:
            stat = os.fstat(stream.fileno())
        except (AttributeError, OSError):
            continue
        if expected == (stat.st_dev, stat.st_ino):
            return True
    return False


def journal_entry_payload(message: str, *, priority: int, component: str) -> bytes:
    """Build a small native-journal datagram without requiring python-systemd."""

    clean_message = message.replace("\x00", "").replace("\r", " ").replace("\n", " ")
    clean_component = component.replace("\x00", "").replace("\r", " ").replace("\n", " ")
    fields = (
        "SYSLOG_IDENTIFIER=robot-teleop",
        f"PRIORITY={max(0, min(7, priority))}",
        f"TELEOP_COMPONENT={clean_component or 'supervisor'}",
        f"MESSAGE={clean_message}",
    )
    return ("\n".join(fields) + "\n").encode("utf-8", errors="replace")


class JournalSink:
    """Best-effort journal mirror that always leaves console logging intact."""

    def __init__(
        self,
        socket_path: Path = DEFAULT_JOURNAL_SOCKET,
        *,
        environment: Mapping[str, str] | None = None,
    ) -> None:
        environment = os.environ if environment is None else environment
        # systemd already copies stdout to the journal only when JOURNAL_STREAM
        # still describes the actual stream (not an inherited, subsequently piped fd).
        self.enabled = (
            os.name == "posix"
            and sys.platform.startswith("linux")
            and socket_path.exists()
            and not stream_is_journal(environment)
        )
        self.socket_path = socket_path
        self._socket: socket.socket | None = None

    def emit(self, message: str, *, priority: int = 6, component: str = "supervisor") -> None:
        if not self.enabled:
            return
        try:
            if self._socket is None:
                self._socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            self._socket.sendto(
                journal_entry_payload(message, priority=priority, component=component),
                str(self.socket_path),
            )
        except OSError:
            # Diagnostics must never bring down capture, control, or safe shutdown.
            self.enabled = False
            self.close()

    def close(self) -> None:
        if self._socket is not None:
            self._socket.close()
            self._socket = None
