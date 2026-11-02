"""Read-only, bounded measured-encoder history for camera-clock animation."""
from __future__ import annotations

import json
import math
import struct
import threading
import time
from collections import deque


class MeasuredJointHistory:
    def __init__(self):
        self._samples = deque(maxlen=160)
        self._lock = threading.Condition()

    def observe(self, status, now_ms=None):
        values = status.get("follower_normalized")
        sent, age = status.get("sent_unix_ms"), status.get("last_read_age_ms")
        now_ms = time.time() * 1000 if now_ms is None else now_ms
        if status.get("follower_connected") is not True or status.get("measured_feedback_fresh") is not True:
            return
        if not isinstance(values, list) or len(values) != 6:
            return
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in [*values, sent, age]):
            return
        if sent - age <= 0 or not 0 <= age <= 350 or not -100 <= now_ms - sent <= 1000:
            return
        stamp = sent - age
        with self._lock:
            # Statuses can repeat one encoder read with a recomputed age.
            # Infer the read instant in the same host clock as camera capture.
            if self._samples and stamp <= self._samples[-1]["unix_ms"] + 2:
                return
            self._samples.append({"unix_ms": stamp, "normalized": list(values)})
            self._lock.notify_all()

    def attach(self, packet, wait_for_bracket_ms=0):
        if not packet.startswith(b"RGBD1") or len(packet) < 9:
            return packet
        size = struct.unpack_from("<I", packet, 5)[0]
        if size > 4 * 1024 * 1024 or size + 9 > len(packet):
            return packet
        try:
            metadata = json.loads(packet[9:9 + size])
            robot = metadata.get("robot")
            if not isinstance(robot, dict) or robot.get("kinematics", {}).get("schema") != "so101_measured_fk_v1":
                return packet
            target = metadata.get("capture_unix_ms", 0)
            deadline = time.monotonic() + min(100, max(0, wait_for_bracket_ms)) / 1000
            with self._lock:
                # At most one feedback interval; never extrapolate a motor goal.
                # Disconnected/stale encoders cannot hold up camera delivery.
                while (wait_for_bracket_ms and self._samples
                       and isinstance(target, (int, float)) and math.isfinite(target)
                       and not robot["kinematics"].get("manual_active")
                       and self._samples[-1]["unix_ms"] < target <= time.time() * 1000 + 50
                       and time.time() * 1000 - self._samples[-1]["unix_ms"] < 200):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    self._lock.wait(remaining)
                samples = list(self._samples)
            robot["measured_joint_history"] = samples
            robot["joint_clock"] = "host_unix_ms_inferred_read_time"
            header = json.dumps(metadata, separators=(",", ":"), allow_nan=False).encode()
            # Preserve every RGB/depth byte and every capture timestamp.
            return b"RGBD1" + struct.pack("<I", len(header)) + header + packet[9 + size:]
        except (ValueError, TypeError, AttributeError):
            return packet
