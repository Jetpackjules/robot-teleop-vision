from __future__ import annotations

import os
try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10
    import tomli as tomllib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / "config" / "local.toml"


def _path(value: str, root: Path = REPO_ROOT) -> Path:
    expanded = Path(os.path.expandvars(value)).expanduser()
    return expanded.resolve() if expanded.is_absolute() else (root / expanded).resolve()


@dataclass(frozen=True)
class StackConfig:
    host: str = "0.0.0.0"
    https_port: int = 8765
    stream_port: int = 8780
    tracking_port: int = 4247
    calibration_status_port: int = 4251
    public_mode: str = "quick"
    password_env: str = "GODOT_REMOTE_PASSWORD"
    password_default: str = "change-me"


@dataclass(frozen=True)
class GodotConfig:
    executable: str = ""
    project: str = "."
    launch_runtime: bool = True
    packaged_runtime: bool = False


@dataclass(frozen=True)
class RobotConfig:
    adapter: str = "disabled"
    enabled: bool = False
    module_paths: tuple[str, ...] = ()
    options: dict[str, Any] = field(default_factory=dict)

    def option(self, name: str, default: Any = None) -> Any:
        """Return a module-owned setting without teaching core about its meaning."""

        return self.options.get(name, default)


@dataclass(frozen=True)
class CameraConfig:
    adapters: tuple[str, ...] = ("realsense",)


@dataclass(frozen=True)
class TrackingConfig:
    adapter: str = "browser_mediapipe"


@dataclass(frozen=True)
class AppConfig:
    source: Path = DEFAULT_CONFIG
    stack: StackConfig = field(default_factory=StackConfig)
    godot: GodotConfig = field(default_factory=GodotConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    cameras: CameraConfig = field(default_factory=CameraConfig)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)

    @property
    def project_root(self) -> Path:
        return _path(self.godot.project)

    @property
    def robot_profile(self) -> Path | None:
        profile = str(self.robot.option("profile", ""))
        return _path(profile) if profile else None


def _robot_config(raw: object) -> RobotConfig:
    if not isinstance(raw, dict):
        raise ValueError("robot configuration must be a TOML table")
    data = dict(raw)
    adapter = str(data.pop("adapter", "disabled"))
    enabled = bool(data.pop("enabled", False))
    module_paths_value = data.pop("module_paths", ())
    if isinstance(module_paths_value, str):
        module_paths = (module_paths_value,)
    else:
        module_paths = tuple(str(item) for item in module_paths_value)
    explicit_options = data.pop("options", {})
    if not isinstance(explicit_options, dict):
        raise ValueError("robot.options must be a TOML table")

    # Unknown keys are intentionally module-owned. This also keeps existing
    # local files compatible while moving robot-specific schema out of core.
    options = dict(data)
    options.update(explicit_options)
    return RobotConfig(adapter, enabled, module_paths, options)


def load_config(path: str | Path = DEFAULT_CONFIG, *, require: bool = True) -> AppConfig:
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        if require:
            raise FileNotFoundError(
                f"Configuration not found: {source}. Run `robot-teleop init` first."
            )
        return AppConfig(source=source)
    data = tomllib.loads(source.read_text(encoding="utf-8"))
    stack = StackConfig(**data.get("stack", {}))
    godot = GodotConfig(**data.get("godot", {}))
    robot = _robot_config(data.get("robot", {}))
    camera_data = dict(data.get("cameras", {}))
    camera_data["adapters"] = tuple(camera_data.get("adapters", ("realsense",)))
    cameras = CameraConfig(**camera_data)
    tracking = TrackingConfig(**data.get("tracking", {}))
    if stack.public_mode not in {"off", "quick"}:
        raise ValueError("stack.public_mode must be 'off' or 'quick'")
    return AppConfig(source, stack, godot, robot, cameras, tracking)
