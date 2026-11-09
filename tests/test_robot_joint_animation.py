import json
import struct
import threading
import time

from robot_teleop.robot_joint_animation import MeasuredJointHistory


def status(stamp=1000, age=20, values=None):
    return {"sent_unix_ms": stamp, "last_read_age_ms": age,
            "follower_connected": True, "measured_feedback_fresh": True,
            "follower_normalized": values or [1, 2, 3, 4, 5, 6],
            "applied_normalized": [90] * 6}


def packet(metadata):
    header = json.dumps(metadata).encode()
    return b"RGBD1" + struct.pack("<I", len(header)) + header + b"EXACT RGB AND DEPTH BYTES"


def unpack(raw):
    n = struct.unpack_from("<I", raw, 5)[0]
    return json.loads(raw[9:9+n]), raw[9+n:]


def test_measured_history_preserves_capture_clock_calibration_and_pixels():
    history = MeasuredJointHistory()
    history.observe(status(), now_ms=1000)
    history.observe(status(1010, 30), now_ms=1010)  # Same physical encoder read.
    history.observe(status(1100, 20), now_ms=1100)
    original = {"capture_unix_ms": 1030, "cameras": [{"sequence": 12}],
                "robot": {"kinematics": {"schema": "so101_measured_fk_v1", "calibration": [5, 6, 7]}, "links": {"base": "unchanged"}}}
    encoded = packet(original)
    metadata, pixels = unpack(history.attach(encoded))
    samples = metadata["robot"].pop("measured_joint_history")
    metadata["robot"].pop("joint_clock")
    assert metadata == original
    assert pixels == unpack(encoded)[1]
    assert [s["unix_ms"] for s in samples] == [980, 1080]
    assert samples[0]["normalized"] == [1, 2, 3, 4, 5, 6]


def test_stale_faulty_nonfinite_and_command_only_data_are_never_animated():
    history = MeasuredJointHistory()
    for change in [{"follower_connected": False}, {"measured_feedback_fresh": False},
                   {"last_read_age_ms": 351}, {"last_read_age_ms": None},
                   {"follower_normalized": None}, {"follower_normalized": [float("nan")] * 6},
                   {"sent_unix_ms": 5000}, {"sent_unix_ms": 0}]:
        history.observe({**status(), **change}, now_ms=1000)
    raw = history.attach(packet({"robot": {"kinematics": {"schema": "so101_measured_fk_v1"}}}))
    assert unpack(raw)[0]["robot"]["measured_joint_history"] == []


def test_history_is_bounded_and_legacy_or_invalid_packets_are_unchanged():
    history = MeasuredJointHistory()
    for t in range(1000, 11000, 10):
        history.observe(status(t), now_ms=t)
    raw = history.attach(packet({"robot": {"kinematics": {"schema": "so101_measured_fk_v1"}}}))
    assert len(unpack(raw)[0]["robot"]["measured_joint_history"]) == 160
    for raw in [b"other", b"RGBD1", packet({"robot": {}}), b"RGBD1"+struct.pack("<I", 9999999)]:
        assert history.attach(raw) == raw


def test_wait_for_measured_bracket_wakes_without_predicting_or_altering_pixels():
    history = MeasuredJointHistory()
    now = time.time() * 1000
    history.observe(status(now, 40), now_ms=now)
    raw = packet({"capture_unix_ms": now - 10,
                  "robot": {"kinematics": {"schema": "so101_measured_fk_v1"}}})
    timer = threading.Timer(.015, lambda: history.observe(status(time.time()*1000, 0)))
    timer.start()
    try:
        result = history.attach(raw, wait_for_bracket_ms=100)
    finally:
        timer.join()
    metadata, pixels = unpack(result)
    assert metadata["robot"]["measured_joint_history"][-1]["unix_ms"] >= now - 10
    assert pixels == unpack(raw)[1]


def test_missing_feedback_never_waits_and_stalled_feedback_wait_is_bounded():
    history = MeasuredJointHistory()
    now = time.time()*1000
    raw = packet({"capture_unix_ms": now,
                  "robot": {"kinematics": {"schema": "so101_measured_fk_v1"}}})
    start = time.monotonic()
    history.attach(raw, wait_for_bracket_ms=100)
    assert time.monotonic()-start < .05
    history.observe(status(now, 10), now_ms=now)
    start = time.monotonic()
    history.attach(raw, wait_for_bracket_ms=20)
    assert .015 <= time.monotonic()-start < .2
