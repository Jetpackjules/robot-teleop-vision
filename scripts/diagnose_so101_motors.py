#!/usr/bin/env python3
"""Read SO-101 registers without commanding motion or changing motor settings."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot_teleop.config import load_config  # noqa: E402

REGISTERS = (
    "Torque_Enable", "Operating_Mode", "Present_Position", "Goal_Position",
    "Min_Position_Limit", "Max_Position_Limit", "Status", "Present_Load",
    "Present_Temperature", "Present_Voltage", "Torque_Limit", "Max_Torque_Limit",
    "Goal_Velocity", "Goal_Time", "Homing_Offset", "Phase",
)


def collect_registers(bus, motor_names: list[str]) -> list[dict]:
    """Read each register independently so one fault does not hide other data."""
    rows = []
    try:
        # Skip the all-motor handshake: an overloaded motor can reject its
        # reads, but the remaining motors still contain useful evidence.
        bus.connect(handshake=False)
        for motor_id, name in enumerate(motor_names, start=1):
            row = {"id": motor_id, "name": name, "registers": {}, "errors": {}}
            for register in REGISTERS:
                try:
                    row["registers"][register] = bus.read(
                        register, name, normalize=False, num_retry=0,
                    )
                except Exception as exc:
                    row["errors"][register] = str(exc)
            rows.append(row)
    finally:
        # Closing the serial handle directly also handles partial connection
        # failures and deliberately avoids disconnect's default torque write.
        original_error = sys.exc_info()[1]
        try:
            bus.port_handler.closePort()
        except Exception:
            if original_error is None:
                raise
    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "local.toml")
    parser.add_argument("--follower-python", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    config = load_config(args.config)
    if config.robot.adapter != "so101" or not config.robot.enabled or config.robot_profile is None:
        parser.error("the local configuration must select an enabled SO-101 profile")
    runtime = str(config.robot.option("python", ""))
    if runtime and not args.follower_python:
        return subprocess.run(
            [str(Path(runtime).expanduser()), str(Path(__file__).resolve()),
             "--config", str(config.source), "--follower-python"],
            check=False,
        ).returncode

    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    profile = json.loads(config.robot_profile.read_text(encoding="utf-8"))
    names = profile["motor_names"][:5 if config.robot.option("ignore_motor_6", False) else 6]
    port = profile["follower"]["port"]
    print(f"READ-ONLY motor check on {port}. Stop the launcher first; leave motor power connected.", flush=True)
    motors = {name: Motor(index + 1, "sts3215", MotorNormMode.RANGE_M100_100)
              for index, name in enumerate(names)}
    bus = FeetechMotorsBus(port=port, motors=motors)
    rows = collect_registers(bus, names)
    output = ROOT / ".teleop" / "so101_motor_diagnostics.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({"port": port, "motors": rows}, indent=2), encoding="utf-8")
    print("ID joint            torque mode position goal   min   max status load temp voltage_raw")
    for row in rows:
        values = row["registers"]
        columns = [values.get(key, "?") for key in REGISTERS[:10]]
        print(f"{row['id']:2} {row['name']:16} " + " ".join(f"{value:>5}" for value in columns))
        if row["errors"]:
            first = next(iter(row["errors"].values()))
            print(f"   {len(row['errors'])} register read error(s): {first}")
    print(f"Full report: {output}")
    print("Torque values are from AFTER launcher shutdown. This check sends no motor writes.")
    return 1 if any(row["errors"] for row in rows) else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"Read-only diagnostic failed: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
