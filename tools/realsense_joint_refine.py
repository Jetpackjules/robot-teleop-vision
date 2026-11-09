#!/usr/bin/env python3
"""Worker for Auto Align's joint refinement stage; writes results, never calibration."""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robot_teleop.joint_camera_refinement import run


def write(path, data):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", type=Path, required=True)
    args = parser.parse_args()
    request = json.loads(args.request.read_text(encoding="utf-8"))
    folder = args.request.parent
    try:
        result = run(
            request, folder, lambda status: write(folder / "progress.json", {"status": status})
        )
    except Exception as error:  # noqa: BLE001 - report worker/dependency failures without saving poses.
        result = {
            "ok": False,
            "request_id": request["request_id"],
            "status": f"Joint refinement failed: {error}. Previous alignment preserved.",
        }
    write(folder / "result.json", result)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
