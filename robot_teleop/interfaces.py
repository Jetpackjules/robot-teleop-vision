from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol


@dataclass(frozen=True)
class DeviceInfo:
    adapter: str
    identifier: str
    label: str
    metadata: dict[str, Any]


@dataclass(frozen=True)
class LaunchSpec:
    name: str
    command: tuple[str, ...]
    environment: dict[str, str] = field(default_factory=dict)


class CameraAdapter(Protocol):
    name: str

    def discover(self) -> list[DeviceInfo]: ...


class RobotAdapter(Protocol):
    name: str

    def launch_spec(self) -> LaunchSpec | None: ...

    def hold(self) -> None: ...

    def public_manifest(self) -> dict[str, Any]: ...

    def operator_environment(self) -> dict[str, str]: ...

    def godot_configuration(
        self,
        *,
        calibration_status_port: int,
        reserved_udp_ports: dict[str, int] | None = None,
    ) -> dict[str, Any]:
        """Return non-secret settings; reject collisions with shared UDP listeners."""
        ...
