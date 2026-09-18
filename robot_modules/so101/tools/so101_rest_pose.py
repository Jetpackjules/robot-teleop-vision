"""Per-arm encoder rest poses. This module never opens the motor bus."""

import hashlib
import json
import math
import os
import tempfile
import uuid
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

if __package__:
    from .so101_arm_common import ArmPairProfile
else:
    from so101_arm_common import ArmPairProfile

DEFAULT_REST_POSE = Path(__file__).resolve().with_name("so101_rest_pose.json")


def rest_pose_path(profile: ArmPairProfile) -> Path:
    return profile.path.with_name("so101_rest_pose.json")


def calibration_fingerprint(profile: ArmPairProfile) -> str:
    data = {"calibration": asdict(profile.follower_calibration), "motors": profile.motor_names}
    return hashlib.sha256(json.dumps(data, sort_keys=True).encode("utf-8")).hexdigest()


def validate_rest_pose(profile: ArmPairProfile, payload: dict) -> dict:
    if not isinstance(payload, dict):
        raise TypeError("rest-pose file must contain a JSON object")
    version = payload.get("version", 0)
    if version not in (1, 2):
        raise RuntimeError("unsupported rest-pose file version")
    if payload.get("follower_serial") != profile.follower_serial:
        raise RuntimeError("rest pose belongs to a different follower arm")
    if version == 2 and payload.get("calibration_fingerprint") != calibration_fingerprint(profile):
        raise RuntimeError("rest pose belongs to a different servo calibration; save it again after verification")
    raw = payload.get("raw_positions")
    saved = payload.get("normalized_positions")
    if not isinstance(raw, list) or len(raw) != 6 or any(
        isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 4095 for v in raw
    ):
        raise RuntimeError("rest pose must contain six valid raw encoder positions")
    if not isinstance(saved, list) or len(saved) != 6 or any(
        isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in saved
    ):
        raise RuntimeError("rest pose must contain six finite normalized positions")
    calibration = profile.follower_calibration
    if calibration.coordinate_system == "lerobot_urdf":
        for i, (value, low, high) in enumerate(zip(raw, calibration.start_pos, calibration.end_pos, strict=True)):
            if not low <= value <= high:
                raise RuntimeError(f"saved rest pose exceeds calibrated limits for motor ID {i + 1}")
    normalized = calibration.raw_to_normalized(raw)
    mismatch = max(abs(current - previous) for current, previous in zip(normalized, saved, strict=True))
    if mismatch > (1e-6 if version == 2 else 2.0):
        raise RuntimeError(f"rest pose does not match the current servo calibration ({mismatch:.1f} deg mismatch)")
    return {**payload, "raw_positions": list(raw), "normalized_positions": normalized}


def load_rest_pose(profile: ArmPairProfile, path: Path | None = None) -> dict:
    if path is None:
        path = rest_pose_path(profile)
        if not path.exists() and DEFAULT_REST_POSE.exists():
            path = DEFAULT_REST_POSE  # Existing installations retain their saved pose.
    if not path.exists():
        raise RuntimeError(f"no rest pose is saved at {path}; use scripts/save_so101_rest_pose.py")
    return validate_rest_pose(profile, json.loads(path.read_text(encoding="utf-8")))


def save_rest_pose(profile: ArmPairProfile, raw: list[int], *, apply: bool = False) -> dict:
    if profile.follower_calibration.coordinate_system != "lerobot_urdf":
        raise ValueError("Repair the follower coordinate convention before saving a new encoder rest pose")
    payload = {
        "version": 2,
        "name": "operator-confirmed rest",
        "follower_serial": profile.follower_serial,
        "calibration_fingerprint": calibration_fingerprint(profile),
        "raw_positions": raw,
        "normalized_positions": profile.follower_calibration.raw_to_normalized(raw),
    }
    payload = validate_rest_pose(profile, payload)
    if profile.follower_calibration.normalized_to_raw(payload["normalized_positions"]) != raw:
        raise ValueError("Rest pose did not round-trip to the supplied encoder positions")
    # Import lazily to keep identity/JSON validation usable without numpy.
    if __package__:
        from .so101_kinematics import rest_mesh_minimum_height
    else:
        from so101_kinematics import rest_mesh_minimum_height
    height = rest_mesh_minimum_height(payload["normalized_positions"])
    if height < -0.002:
        raise ValueError(f"Rest mesh is below the modeled base plane ({height * 1000:.1f} mm)")
    path = rest_pose_path(profile)
    result = {"path": str(path), "height_m": height, "saved": False, "backup": None}
    if not apply:
        return result
    original = path.read_bytes() if path.exists() else None
    if original is not None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        backup = path.with_name(f"{path.name}.{stamp}.{uuid.uuid4().hex[:8]}.bak")
        with backup.open("xb") as stream:
            stream.write(original)
        result["backup"] = str(backup)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=path.name + ".", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(payload, stream, indent=2)
            stream.write("\n")
        if (path.read_bytes() if path.exists() else None) != original:
            raise ValueError("Saved rest pose changed concurrently; it was not overwritten")
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()
    return {**result, "saved": True}
