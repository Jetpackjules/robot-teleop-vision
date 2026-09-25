"""Fault diagnostics must survive missing UI and must not disrupt shutdown."""

import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_modules/so101/tools"))

import so101_follower_service as service
from so101_arm_common import ArmPairProfile


def controller_for_report(tmp_path):
    profile = ArmPairProfile.load(ROOT / "tests/fixtures/so101_arm_pair.json")
    bus = service.FakeFollowerBus([2048, 2048, 2048, 2048, 2048, 2000])
    controller = service.FollowerController(
        profile, bus, rest_pose_path=tmp_path / "rest.json",
        fault_report_path=tmp_path / "记录" / "so101_follower_fault.json",
    )
    controller.connect()
    bus.enable_torque()
    controller.torque_enabled = True
    controller.state = "armed"
    controller.control_source = "leader"
    controller.leader_raw = [2050, 2090, 2170, 2010, 2080, 1800]
    controller.target_normalized = [0, 4, 9, -3, 2, 40]
    controller.last_seq = 42
    return controller, bus


def test_fault_saves_original_exception_and_pose_without_building_status(tmp_path, monkeypatch, capsys):
    controller, bus = controller_for_report(tmp_path)

    def broken_status():
        raise AssertionError("Fault logging must not need a working status/kinematics path")

    monkeypatch.setattr(controller, "status", broken_status)
    try:
        raise RuntimeError("Failed to write Goal_Position on id=2: overload")
    except RuntimeError as exc:
        controller.fail(str(exc))
    assert controller.state == "fault"
    assert not bus.torque
    report = json.loads(controller.fault_report_path.read_text(encoding="utf-8"))
    assert report["state_before_fault"] == "armed"
    assert report["torque_enabled_before_fault"] is True
    assert report["torque_disable_error"] == ""
    assert report["leader_raw"] == controller.leader_raw
    assert report["follower_raw"] == bus.positions
    assert report["target_normalized"] == controller.target_normalized
    assert report["last_seq"] == 42
    assert "RuntimeError: Failed to write Goal_Position" in report["traceback"]
    assert "Failed to write Goal_Position on id=2" in capsys.readouterr().err

    first_report = controller.fault_report_path.read_bytes()
    controller.fail("a secondary error while already faulted")
    assert controller.fault_report_path.read_bytes() == first_report
    assert capsys.readouterr().err == ""

    controller.connect()
    controller.fail("new fault after recovery")
    assert json.loads(controller.fault_report_path.read_text(encoding="utf-8"))["error"] == "new fault after recovery"


def test_broken_report_destination_and_console_do_not_prevent_stop(tmp_path, monkeypatch):
    controller, bus = controller_for_report(tmp_path)
    obstacle = tmp_path / "not-a-directory"
    obstacle.write_text("keep", encoding="utf-8")
    controller.fault_report_path = obstacle / "report.json"

    class BrokenConsole:
        def write(self, _text):
            raise BrokenPipeError("console closed")

    monkeypatch.setattr(service.sys, "stderr", BrokenConsole())
    controller.fail("motor communication failure")
    assert not bus.torque
    assert controller.state == "fault"
    assert controller.hardware_fault == "motor communication failure"
    assert obstacle.read_text(encoding="utf-8") == "keep"


def test_torque_disable_failure_is_recorded_without_hiding_original_error(tmp_path, monkeypatch):
    controller, _bus = controller_for_report(tmp_path)

    def fail_disable():
        raise OSError("USB device disconnected during torque release")

    monkeypatch.setattr(controller.bus, "disable_torque", fail_disable)
    controller.fail("original motor write failed")
    report = json.loads(controller.fault_report_path.read_text(encoding="utf-8"))
    assert report["error"] == "original motor write failed"
    assert report["torque_disable_error"] == "USB device disconnected during torque release"
    assert controller.state == "fault"


def test_uncaught_command_receive_error_is_saved_before_service_exits(tmp_path, monkeypatch):
    controller, bus = controller_for_report(tmp_path)
    monkeypatch.setattr(service, "FollowerController", lambda *args, **kwargs: controller)
    monkeypatch.setattr(service.signal, "signal", lambda *args: None)

    class Socket:
        def bind(self, _address):
            pass

        def setblocking(self, _flag):
            pass

        def recvfrom(self, _size):
            raise OSError("command socket receive failed")

        def close(self):
            pass

    monkeypatch.setattr(service.socket, "socket", lambda *args: Socket())
    with pytest.raises(OSError, match="command socket receive failed"):
        service.run_service(controller.profile, bus, 14248, 14249, 0, 0, 30, 20, True)
    report = json.loads(controller.fault_report_path.read_text(encoding="utf-8"))
    assert report["error"] == "Follower service exiting: command socket receive failed"
    assert "OSError: command socket receive failed" in report["traceback"]
    assert not bus.connected
    assert not bus.torque
