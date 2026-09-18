"""Save six operator-confirmed raw encoder positions as this arm's rest pose."""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_modules.so101.tools.so101_arm_common import ArmPairProfile
from robot_modules.so101.tools.so101_rest_pose import save_rest_pose
from robot_teleop.config import load_config


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw", type=int, nargs=6, required=True, metavar="POSITION")
    parser.add_argument("--config", type=Path, default=ROOT / "config/local.toml")
    parser.add_argument("--apply", action="store_true", help="Save after validation; otherwise only check")
    args = parser.parse_args()
    config = load_config(args.config)
    if config.robot.adapter != "so101" or config.robot_profile is None:
        parser.error("The local configuration must select an SO-101 profile")
    result = save_rest_pose(ArmPairProfile.load(config.robot_profile), args.raw, apply=args.apply)
    print("FILE-ONLY: no serial connection, motor writes, or camera calibration changes.")
    print(f"Lowest modeled moving-link point: {result['height_m'] * 1000:.1f} mm above base origin.")
    print("Saved rest pose:" if result["saved"] else "Validated; add --apply to save:", result["path"])
    if result["backup"]:
        print("Previous rest pose backed up:", result["backup"])
    if result["saved"]:
        print("Return to Rest reloads this pose on its next request with the updated follower service.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"Rest pose was not saved: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
