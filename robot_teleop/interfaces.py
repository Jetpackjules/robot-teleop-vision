from __future__ import annotations

from dataclasses import dataclass
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


class CameraAdapter(Protocol):
    name: str

    def discover(self) -> list[DeviceInfo]: ...


class RobotAdapter(Protocol):
    name: str

    def launch_spec(self) -> LaunchSpec | None: ...

    def hold(self) -> None: ...


class TrackingAdapter(Protocol):
    name: str

    def describe(self) -> dict[str, Any]: ...
