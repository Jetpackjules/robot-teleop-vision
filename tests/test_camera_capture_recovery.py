import json

import pytest

from robot_teleop import camera_capture_recovery as recovery


def test_policy_is_explicit_and_camera_scoped(tmp_path, monkeypatch):
    monkeypatch.setattr(recovery, "godot_app_userdata_root", lambda: tmp_path)
    assert recovery.read_policy() == {}
    path = tmp_path / "Robot Teleop Vision/camera_color_policy.json"
    path.parent.mkdir()
    policy = {"schema_version": 1, "cameras": {"123": {"auto_white_balance": True}}}
    path.write_text(json.dumps(policy))
    assert recovery.read_policy() == policy
    for value in [[], {"schema_version": 1, "cameras": {"123": {"auto_white_balance": 1}}},
                  {"schema_version": 1, "cameras": {"bad/path": {"auto_white_balance": True}}}]:
        path.write_text(json.dumps(value))
        with pytest.raises(ValueError):
            recovery.read_policy()


def setup_policy(monkeypatch):
    policy = {"schema_version": 1, "cameras": {}, "recover_capture_after_calibration": True}
    monkeypatch.setattr(recovery, "read_policy", lambda: policy)
    monkeypatch.setattr(recovery.sys, "platform", "linux")
    monkeypatch.setattr(recovery, "apply_color_policy", lambda policy: None)


def test_healthy_cameras_never_restart(monkeypatch, tmp_path):
    setup_policy(monkeypatch)
    monkeypatch.setattr(recovery, "wait_ready", lambda *args: [])
    monkeypatch.setattr(recovery, "restart_capture", lambda *args: pytest.fail("healthy capture restarted"))
    recovery.prepare_calibration_capture({"source_url": "http://127.0.0.1:8781/rgbd", "cameras": ["A"]}, tmp_path, lambda _: None)


@pytest.mark.parametrize("url", ["http://example.com:8781/rgbd", "http://127.0.0.1:8780/rgbd",
                                "http://127.0.0.1:8781/rgbd?profile=other", "https://127.0.0.1:8781/rgbd"])
def test_other_sources_cannot_restart_capture(monkeypatch, tmp_path, url):
    setup_policy(monkeypatch)
    monkeypatch.setattr(recovery, "wait_ready", lambda *args: pytest.fail("wrong capture source checked"))
    monkeypatch.setattr(recovery, "restart_capture", lambda *args: pytest.fail("wrong capture source restarted"))
    recovery.prepare_calibration_capture({"source_url": url, "cameras": ["A"]}, tmp_path, lambda _: None)


def test_recovery_is_once_and_only_for_stale_cameras(monkeypatch, tmp_path):
    setup_policy(monkeypatch)
    responses = iter([["B"], []])
    monkeypatch.setattr(recovery, "wait_ready", lambda *args: next(responses))
    calls = []
    monkeypatch.setattr(recovery, "restart_capture", lambda ids, folder: calls.append(ids))
    recovery.prepare_calibration_capture({"source_url": "http://127.0.0.1:8781/rgbd", "cameras": ["A", "B"]}, tmp_path, lambda _: None)
    assert calls == [["B"]]


def test_failed_recovery_stops_before_validation(monkeypatch, tmp_path):
    setup_policy(monkeypatch)
    monkeypatch.setattr(recovery, "wait_ready", lambda *args: ["B"])
    calls = []
    monkeypatch.setattr(recovery, "restart_capture", lambda ids, folder: calls.append(ids))
    with pytest.raises(ValueError, match="did not recover: B"):
        recovery.prepare_calibration_capture({"source_url": "http://127.0.0.1:8781/rgbd", "cameras": ["A", "B"]}, tmp_path, lambda _: None)
    assert calls == [["B"]]


def test_pid_metadata_preserves_unrelated_services(tmp_path, monkeypatch):
    monkeypatch.setattr(recovery, "ROOT", tmp_path)
    folder = tmp_path / ".teleop"
    folder.mkdir()
    path = folder / "latest-web-ui-session.json"
    path.write_text(json.dumps({"pids": {"capture": 12, "runtime": 99, "follower": 100}, "public_url": "unchanged"}))
    recovery._record_capture_pid(12, 13)
    assert json.loads(path.read_text()) == {"pids": {"capture": 13, "runtime": 99, "follower": 100}, "public_url": "unchanged"}
