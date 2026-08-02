#!/usr/bin/env python3
"""Migrate an SO-101 registration after calibrating shoulder-pan encoder zero.

The registration is the transform of the robot model root.  Changing the
shoulder-pan offset would otherwise rotate every moving link in world space.
This migration counter-transforms the root about the real shoulder-pan pivot,
so the already depth-validated moving-arm pose remains the starting point for
the next calibration.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path

import numpy as np

from so101_kinematics import BASE_STRAIGHT_NORMALIZED_DEG, JOINT_ORIGINS, _transform


ROS_TO_GODOT = np.array(
    (
        (0.0, -1.0, 0.0),
        (0.0, 0.0, 1.0),
        (-1.0, 0.0, 0.0),
    ),
    dtype=float,
)


def _payload_transform(payload: dict) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = np.column_stack(
        (payload["basis_x"], payload["basis_y"], payload["basis_z"])
    )
    result[:3, 3] = payload["origin"]
    return result


def _write_transform(payload: dict, transform: np.ndarray) -> None:
    payload["basis_x"] = transform[:3, 0].tolist()
    payload["basis_y"] = transform[:3, 1].tolist()
    payload["basis_z"] = transform[:3, 2].tolist()
    payload["origin"] = transform[:3, 3].tolist()


def migrate(payload: dict) -> dict:
    if int(payload.get("kinematics_version", 0)) != 9:
        raise ValueError("input registration must use kinematics version 9")

    model_root = np.eye(4)
    model_root[:3, :3] = ROS_TO_GODOT
    shoulder_origin = _transform(*JOINT_ORIGINS[0])
    old_to_new = np.eye(4)
    angle = math.radians(BASE_STRAIGHT_NORMALIZED_DEG)
    old_to_new[:3, :3] = np.array(
        (
            (math.cos(angle), -math.sin(angle), 0.0),
            (math.sin(angle), math.cos(angle), 0.0),
            (0.0, 0.0, 1.0),
        )
    )

    pivot_frame = model_root @ shoulder_origin
    correction = pivot_frame @ old_to_new @ np.linalg.inv(pivot_frame)
    migrated = dict(payload)
    _write_transform(migrated, _payload_transform(payload) @ correction)
    migrated["kinematics_version"] = 10
    migrated["registration_source"] = (
        str(payload.get("registration_source", "legacy")) + "+base_zero_v10"
    )
    migrated["base_zero_migration_degrees"] = BASE_STRAIGHT_NORMALIZED_DEG
    return migrated


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()

    payload = json.loads(args.source.read_text())
    migrated = migrate(payload)
    if args.destination.exists():
        backup = args.destination.with_name(args.destination.name + ".before_base_zero_v10")
        shutil.copy2(args.destination, backup)
    args.destination.write_text(json.dumps(migrated, indent="\t") + "\n")
    print(
        json.dumps(
            {
                "source": str(args.source),
                "destination": str(args.destination),
                "old_origin": payload["origin"],
                "new_origin": migrated["origin"],
                "kinematics_version": migrated["kinematics_version"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
