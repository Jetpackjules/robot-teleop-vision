"""Repair a hardware-calibrated LeRobot follower profile without motor I/O."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import tempfile
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot_teleop.config import load_config  # noqa: E402
from robot_modules.so101.tools.so101_arm_common import ArmCalibration  # noqa: E402


def repair_profile(profile_path: Path, report_path: Path, *, apply: bool = False) -> dict:
    original = profile_path.read_bytes()
    profile = json.loads(original)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    calibration = profile["follower"]["calibration"]
    if calibration.get("coordinate_system", "legacy") not in ("legacy", "lerobot_urdf"):
        raise ValueError("Unknown coordinate system; profile was not changed")
    rows = report.get("motors", [])
    if report.get("port") != profile["follower"]["port"] or len(rows) != 6:
        raise ValueError("Diagnostic must contain all six motors on this follower's configured port")
    for index, row in enumerate(rows):
        if row.get("id") != index + 1 or row.get("name") != profile["motor_names"][index]:
            raise ValueError("Diagnostic motor identity does not match this profile")
        if row.get("errors"):
            raise ValueError("Diagnostic contains motor read failures; profile was not changed")
        registers = row["registers"]
        for register, field in (("Homing_Offset", "homing_offset"),
                                ("Min_Position_Limit", "start_pos"),
                                ("Max_Position_Limit", "end_pos")):
            if registers.get(register) != calibration[field][index]:
                raise ValueError(f"Motor {index + 1} {register} does not match the profile")
    candidate = {**calibration, "coordinate_system": "lerobot_urdf"}
    converted = ArmCalibration.from_dict(candidate)
    raw_pose = [int(row["registers"]["Present_Position"]) for row in rows]
    normalized = converted.raw_to_normalized(raw_pose)
    if converted.normalized_to_raw(normalized) != raw_pose:
        raise ValueError("Coordinate conversion did not preserve the measured encoder pose")
    result = {"normalized_pose": normalized, "changed": False, "backup": None}
    if not apply or calibration.get("coordinate_system") == "lerobot_urdf":
        return result
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    backup = profile_path.with_name(f"{profile_path.name}.before_coordinates.{stamp}.{uuid.uuid4().hex[:8]}.bak")
    with backup.open("xb") as stream:
        stream.write(original)
    profile["follower"]["calibration"] = candidate
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=profile_path.parent,
                                         prefix=profile_path.name + ".", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(profile, stream, indent=2)
            stream.write("\n")
        # Do not overwrite a concurrent edit made after the diagnostic was checked.
        if profile_path.read_bytes() != original:
            raise ValueError("Profile changed during repair; original was not overwritten")
        os.replace(temporary, profile_path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    result.update(changed=True, backup=str(backup))
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true", help="Back up and update only the follower profile")
    parser.add_argument("--config", type=Path, default=ROOT / "config/local.toml")
    args = parser.parse_args()
    config = load_config(args.config)
    if config.robot.adapter != "so101" or config.robot_profile is None:
        parser.error("The local configuration must select an SO-101 profile")
    result = repair_profile(config.robot_profile, ROOT / ".teleop/so101_motor_diagnostics.json", apply=args.apply)
    print("FILE-ONLY check: no serial connection, motor writes, or camera calibration changes.")
    print("Corrected model pose:", [round(value, 2) for value in result["normalized_pose"]])
    if result["changed"]:
        print("Follower coordinate conversion repaired. Backup:", result["backup"])
    elif args.apply:
        print("Follower profile already uses the corrected coordinate conversion.")
    else:
        print("Validated hardware/profile match. Run with --apply to back up and update the profile.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"Profile repair refused: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
