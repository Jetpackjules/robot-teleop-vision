#!/usr/bin/env python3
"""Repository-local CLI wrapper used by the Godot setup dock."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot_teleop.cli import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
