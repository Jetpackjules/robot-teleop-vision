"""Set per-pair physical leader motion directions without connecting to motors."""

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
from robot_modules.so101.tools.so101_arm_common import ArmPairProfile  # noqa: E402


def set_directions(profile_path: Path, invert: list[int], *, apply: bool = False) -> dict:
    if any(type(joint) is not int or not 1 <= joint <= 6 for joint in invert):
        raise ValueError("Inverted joints must be motor IDs 1 through 6")
    profile_path = profile_path.expanduser().resolve()
    original = profile_path.read_bytes()
    payload = json.loads(original)
    current = ArmPairProfile.load(profile_path)
    directions = [-1 if joint in invert else 1 for joint in range(1, 7)]
    result = {"profile": str(profile_path), "directions": directions, "changed": False, "backup": None}
    if not apply or list(current.leader_joint_directions) == directions:
        return result
    payload["leader_joint_directions"] = directions
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=profile_path.parent,
                                         prefix=profile_path.name + ".", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(payload, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
        ArmPairProfile.load(temporary)  # Validate the complete candidate before replacing anything.
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        backup = profile_path.with_name(f"{profile_path.name}.before_directions.{stamp}.{uuid.uuid4().hex[:8]}.bak")
        with backup.open("xb") as stream:
            stream.write(original)
        if profile_path.read_bytes() != original:
            raise ValueError("Profile changed during update; concurrent edit was not overwritten")
        os.replace(temporary, profile_path)
        result.update(changed=True, backup=str(backup))
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--profile", type=Path, help="Arm-pair JSON; otherwise use the active local configuration")
    source.add_argument("--config", type=Path, default=ROOT / "config/local.toml")
    directions = parser.add_mutually_exclusive_group(required=True)
    directions.add_argument("--invert", type=int, nargs="+", choices=range(1, 7), metavar="ID",
                            help="Invert only these motor IDs; all others keep the normal direction")
    directions.add_argument("--reset", action="store_true", help="Restore normal directions for all six motors")
    parser.add_argument("--apply", action="store_true", help="Back up and update the profile (otherwise preview)")
    args = parser.parse_args()
    path = args.profile
    if path is None:
        config = load_config(args.config)
        if config.robot.adapter != "so101" or config.robot_profile is None:
            parser.error("The local configuration must select an SO-101 profile, or provide --profile")
        path = config.robot_profile
    result = set_directions(path, args.invert or [], apply=args.apply)
    print("FILE-ONLY: updates relative leader motion directions; no serial connection or motor writes.")
    print("Profile:", result["profile"])
    print("Leader joint directions (motors 1-6):", result["directions"])
    if result["changed"]:
        print("Saved. Backup:", result["backup"])
        print("Restart the complete Python launcher/follower to load this setting.")
    elif args.apply:
        print("Profile already has these directions. Restart the launcher if it was running before the update.")
    else:
        print("Preview only. Use --apply to save this setting, then restart the launcher.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"Direction update refused: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
