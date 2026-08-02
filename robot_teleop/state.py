from __future__ import annotations

import os
import shutil
from pathlib import Path


DURABLE_STATE_FILES = (
    "camera_alignment_registry.json",
    "realsense_alignment_ground_truth.json",
    "realsense_cloud_alignment_result.json",
    "screen_setup.json",
    "so101_robot_registration.depth_validated_anchor.json",
    "so101_robot_registration.json",
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
) -> list[Path]:
    """Copy only durable calibration state; never copy captures, logs, or device dumps."""
    base = root or godot_app_userdata_root()
    source = base / source_project
    target = base / target_project
    if not source.is_dir():
        raise FileNotFoundError(f"Godot state directory not found: {source}")
    target.mkdir(parents=True, exist_ok=True)
    copied: list[Path] = []
    for name in DURABLE_STATE_FILES:
        source_file = source / name
        target_file = target / name
        if not source_file.is_file() or (target_file.exists() and not force):
            continue
        shutil.copy2(source_file, target_file)
        copied.append(target_file)
    return copied
