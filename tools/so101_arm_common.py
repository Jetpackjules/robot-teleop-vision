#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


MOTOR_COUNT = 6
DEFAULT_PROFILE = Path(__file__).with_name("so101_arm_pair.json")


class ArmProtocolError(ValueError):
    pass


@dataclass(frozen=True)
class ArmCalibration:
    homing_offset: tuple[int, ...]
    drive_mode: tuple[int, ...]
    start_pos: tuple[int, ...]
    end_pos: tuple[int, ...]
    calib_mode: tuple[str, ...]

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ArmCalibration":
        fields = {
            "homing_offset": tuple(int(v) for v in data["homing_offset"]),
            "drive_mode": tuple(int(v) for v in data["drive_mode"]),
            "start_pos": tuple(int(v) for v in data["start_pos"]),
            "end_pos": tuple(int(v) for v in data["end_pos"]),
            "calib_mode": tuple(str(v).upper() for v in data["calib_mode"]),
        }
        if any(len(values) != MOTOR_COUNT for values in fields.values()):
            raise ValueError("SO-101 calibration must contain six values per field")
        return cls(**fields)

    def raw_to_normalized(self, positions: list[int] | tuple[int, ...]) -> list[float]:
        require_positions(positions)
        result: list[float] = []
        for index, raw in enumerate(positions):
            if self.calib_mode[index] == "LINEAR":
                span = self.end_pos[index] - self.start_pos[index]
                if span == 0:
                    raise ValueError(f"zero calibration span for motor {index + 1}")
                result.append((float(raw) - self.start_pos[index]) / span * 100.0)
                continue
            directed = -float(raw) if self.drive_mode[index] else float(raw)
            result.append((directed + self.homing_offset[index]) / 2048.0 * 180.0)
        return result

    def normalized_to_raw(self, positions: list[float] | tuple[float, ...]) -> list[int]:
        require_positions(positions)
        result: list[int] = []
        for index, value in enumerate(positions):
            if self.calib_mode[index] == "LINEAR":
                calibrated_value = max(0.0, min(100.0, float(value)))
                raw = self.start_pos[index] + calibrated_value / 100.0 * (
                    self.end_pos[index] - self.start_pos[index]
                )
            else:
                raw = float(value) / 180.0 * 2048.0 - self.homing_offset[index]
                if self.drive_mode[index]:
                    raw = -raw
            result.append(max(0, min(4095, int(round(raw)))))
        return result


@dataclass(frozen=True)
class ArmPairProfile:
    path: Path
    leader_serial: str
    leader_port: str
    leader_calibration: ArmCalibration
    follower_serial: str
    follower_port: str
    follower_calibration: ArmCalibration
    motor_names: tuple[str, ...]
    command_port: int
    status_port: int
    godot_status_port: int
    watchdog_ms: int
    start_pose_tolerance: tuple[float, ...]
    max_step: tuple[float, ...]

    @classmethod
    def load(cls, path: str | Path = DEFAULT_PROFILE) -> "ArmPairProfile":
        profile_path = Path(path).expanduser().resolve()
        data = json.loads(profile_path.read_text(encoding="utf-8"))
        leader = data["leader"]
        follower = data["follower"]
        motor_names = tuple(str(v) for v in data["motor_names"])
        tolerances = tuple(float(v) for v in data["start_pose_tolerance"])
        max_step = tuple(float(v) for v in data["max_step"])
        if len(motor_names) != MOTOR_COUNT or len(tolerances) != MOTOR_COUNT or len(max_step) != MOTOR_COUNT:
            raise ValueError("SO-101 profile arrays must contain six values")
        return cls(
            path=profile_path,
            leader_serial=str(leader["serial"]),
            leader_port=str(leader["port"]),
            leader_calibration=ArmCalibration.from_dict(leader["calibration"]),
            follower_serial=str(follower["serial"]),
            follower_port=str(follower["port"]),
            follower_calibration=ArmCalibration.from_dict(follower["calibration"]),
            motor_names=motor_names,
            command_port=int(data.get("command_port", 4248)),
            status_port=int(data.get("status_port", 4249)),
            godot_status_port=int(data.get("godot_status_port", 4250)),
            watchdog_ms=int(data.get("watchdog_ms", 250)),
            start_pose_tolerance=tolerances,
            max_step=max_step,
        )

    def map_leader_to_follower(self, leader_raw: list[int]) -> tuple[list[float], list[int]]:
        normalized = self.leader_calibration.raw_to_normalized(leader_raw)
        return normalized, self.follower_calibration.normalized_to_raw(normalized)


def require_positions(values: Any) -> None:
    if not isinstance(values, (list, tuple)) or len(values) != MOTOR_COUNT:
        raise ArmProtocolError("positions must be an array of six numbers")
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise ArmProtocolError("positions must contain finite numbers")


def validate_browser_arm_message(data: Any, *, now_ms: int | None = None) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise ArmProtocolError("arm message must be an object")
    kind = data.get("type")
    if kind == "arm_command":
        positions = data.get("positions")
        require_positions(positions)
        raw = [int(v) for v in positions]
        if any(value < 0 or value > 4095 for value in raw):
            raise ArmProtocolError("raw motor positions must be between 0 and 4095")
        seq = data.get("seq")
        sent_ms = data.get("sent_unix_ms")
        if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
            raise ArmProtocolError("seq must be a non-negative integer")
        if isinstance(sent_ms, bool) or not isinstance(sent_ms, (int, float)):
            raise ArmProtocolError("sent_unix_ms must be numeric")
        return {"type": kind, "seq": seq, "sent_unix_ms": int(sent_ms), "positions": raw}
    if kind == "arm_cartesian_velocity":
        linear = data.get("linear")
        angular = data.get("angular")
        wrist = data.get("wrist", [0.0, 0.0])
        if not isinstance(linear, (list, tuple)) or len(linear) != 3:
            raise ArmProtocolError("linear must contain three numbers")
        if not isinstance(angular, (list, tuple)) or len(angular) != 3:
            raise ArmProtocolError("angular must contain three numbers")
        if not isinstance(wrist, (list, tuple)) or len(wrist) != 2:
            raise ArmProtocolError("wrist must contain pitch and roll")
        values = [*linear, *angular, *wrist, data.get("gripper", 0.0)]
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in values
        ):
            raise ArmProtocolError("Cartesian velocity values must be finite numbers")
        if any(abs(float(value)) > 1.0 for value in values):
            raise ArmProtocolError("Cartesian velocity values must be between -1 and 1")
        seq = data.get("seq")
        sent_ms = data.get("sent_unix_ms")
        if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
            raise ArmProtocolError("seq must be a non-negative integer")
        if isinstance(sent_ms, bool) or not isinstance(sent_ms, (int, float)):
            raise ArmProtocolError("sent_unix_ms must be numeric")
        deadman = data.get("deadman")
        if not isinstance(deadman, bool):
            raise ArmProtocolError("deadman must be boolean")
        if not deadman and any(abs(float(value)) > 0.0001 for value in values):
            raise ArmProtocolError("disabled keyboard control cannot contain motion")
        frame = str(data.get("frame", "base"))
        if frame not in ("base", "wrist", "plane"):
            raise ArmProtocolError("Cartesian velocity frame must be base, wrist, or plane")
        angular_frame = str(data.get("angular_frame", frame if frame in ("base", "wrist") else "base"))
        if angular_frame not in ("base", "wrist"):
            raise ArmProtocolError("Cartesian angular velocity frame must be base or wrist")
        return {
            "type": kind,
            "seq": seq,
            "sent_unix_ms": int(sent_ms),
            "linear": [float(value) for value in linear],
            "angular": [float(value) for value in angular],
            "wrist": [float(value) for value in wrist],
            "gripper": float(data.get("gripper", 0.0)),
            "frame": frame,
            "angular_frame": angular_frame,
            "precision": bool(data.get("precision", False)),
            "deadman": deadman,
        }
    if kind in ("arm_joint_velocity", "arm_tool_velocity"):
        field = "motions" if kind == "arm_tool_velocity" else "velocities"
        velocities = data.get(field)
        if not isinstance(velocities, (list, tuple)) or len(velocities) != MOTOR_COUNT:
            raise ArmProtocolError(f"{field} must contain six numbers")
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in velocities
        ):
            raise ArmProtocolError(f"{field} values must be finite numbers")
        if any(abs(float(value)) > 1.0 for value in velocities):
            raise ArmProtocolError(f"{field} values must be between -1 and 1")
        seq = data.get("seq")
        sent_ms = data.get("sent_unix_ms")
        if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
            raise ArmProtocolError("seq must be a non-negative integer")
        if isinstance(sent_ms, bool) or not isinstance(sent_ms, (int, float)):
            raise ArmProtocolError("sent_unix_ms must be numeric")
        deadman = data.get("deadman")
        if not isinstance(deadman, bool):
            raise ArmProtocolError("deadman must be boolean")
        if not deadman and any(abs(float(value)) > 0.0001 for value in velocities):
            raise ArmProtocolError("disabled keyboard control cannot contain motion")
        return {
            "type": kind,
            "seq": seq,
            "sent_unix_ms": int(sent_ms),
            field: [float(value) for value in velocities],
            "precision": bool(data.get("precision", False)),
            "deadman": deadman,
        }
    if kind == "arm_enable":
        source = str(data.get("source", "leader"))
        if source not in ("leader", "keyboard"):
            raise ArmProtocolError("arm source must be leader or keyboard")
        return {"type": kind, "source": source}
    if kind == "arm_feedback_settings":
        keys = (
            "measured_feedback_enabled",
            "target_ghost_enabled",
            "following_error_safety_enabled",
            "freeze_overlay_on_stale_enabled",
            "d455_visual_correction_enabled",
        )
        result: dict[str, Any] = {"type": kind}
        for key in keys:
            value = data.get(key)
            if not isinstance(value, bool):
                raise ArmProtocolError(f"{key} must be boolean")
            result[key] = value
        return result
    if kind == "arm_idle_return_settings":
        enabled = data.get("enabled")
        timeout_seconds = data.get("timeout_seconds")
        if not isinstance(enabled, bool):
            raise ArmProtocolError("enabled must be boolean")
        if (
            isinstance(timeout_seconds, bool)
            or not isinstance(timeout_seconds, (int, float))
            or not math.isfinite(float(timeout_seconds))
            or not 60.0 <= float(timeout_seconds) <= 3600.0
        ):
            raise ArmProtocolError("timeout_seconds must be between 60 and 3600")
        return {
            "type": kind,
            "enabled": enabled,
            "timeout_seconds": float(timeout_seconds),
        }
    if kind in (
        "arm_hold",
        "arm_release_torque",
        "arm_release_gripper_torque",
        "arm_restart",
        "arm_status_request",
        "arm_return_to_rest",
    ):
        return {"type": kind}
    raise ArmProtocolError("unsupported arm message type")


def limit_step(previous: list[float], target: list[float], max_step: tuple[float, ...]) -> list[float]:
    require_positions(previous)
    require_positions(target)
    deltas = [wanted - old for old, wanted in zip(previous, target, strict=True)]
    largest_ratio = max(
        (abs(delta) / limit if limit > 0 else math.inf)
        for delta, limit in zip(deltas, max_step, strict=True)
    )
    scale = 1.0 if largest_ratio <= 1.0 else 1.0 / largest_ratio
    return [
        old + delta * scale
        for old, delta in zip(previous, deltas, strict=True)
    ]


def pose_is_close(actual: list[float], target: list[float], tolerance: tuple[float, ...]) -> tuple[bool, list[float]]:
    require_positions(actual)
    require_positions(target)
    errors = [abs(a - b) for a, b in zip(actual, target, strict=True)]
    return all(error <= limit for error, limit in zip(errors, tolerance, strict=True)), errors
