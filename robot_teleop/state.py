from __future__ import annotations

import os
import shutil
from pathlib import Path
from typing import Iterable


SHARED_DURABLE_STATE_FILES = (
    "camera_alignment_registry.json",
    "realsense_alignment_ground_truth.json",
    "realsense_cloud_alignment_result.json",
    "screen_setup.json",
    "unified_world_level.json",
)


def godot_app_userdata_root() -> Path:
    if os.name == "nt":
        appdata = Path(os.environ.get("APPDATA", Path.home() / "AppData" / "Roaming"))
        return appdata / "Godot" / "app_userdata"
    return Path.home() / ".local" / "share" / "godot" / "app_userdata"


def migrate_project_state(
    source_project: str,
    target_project: str = "Robot Teleop Vision",
    *,
    root: Path | None = None,
    force: bool = False,
    extra_state_files: Iterable[str] = (),
) -> list[Path]:
    """Copy only durable calibration state; never copy captures, logs, or device dumps."""
    base = root or godot_app_userdata_root()
    source = base / source_project
    target = base / target_project
    if not source.is_dir():
        raise FileNotFoundError(f"Godot state directory not found: {source}")
    target.mkdir(parents=True, exist_ok=True)
    copied: list[Path] = []
    state_files = list(SHARED_DURABLE_STATE_FILES)
    for name in extra_state_files:
        if Path(name).name != name or not name:
            raise ValueError(f"unsafe durable state file name: {name!r}")
        if name not in state_files:
            state_files.append(name)
    for name in state_files:
        source_file = source / name
        target_file = target / name
        if not source_file.is_file() or (target_file.exists() and not force):
            continue
        shutil.copy2(source_file, target_file)
        copied.append(target_file)
    return copied
