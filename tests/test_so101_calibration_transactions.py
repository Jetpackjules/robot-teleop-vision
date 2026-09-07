"""Hardware-free coverage of calibration request acknowledgement and cancellation."""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_modules" / "so101" / "tools"))

import so101_follower_service as service
from so101_arm_common import ArmPairProfile

COMMANDS = (
    ("arm_calibration_sweep", "base", 1),
    ("arm_joint_calibration_sweep", "joints", 3),
    ("arm_base_axis_sweep", "axis", 3),
    ("arm_wrist_calibration_sweep", "wrist", 3),
    ("arm_claw_calibration_sweep", "claw", 5),
)


class CountingBus(service.FakeFollowerBus):
    def __init__(self, initial):
        super().__init__(initial)
        self.writes = 0
        self.torque_enables = 0

    def write_positions(self, positions):
        self.writes += 1
        super().write_positions(positions)

    def enable_torque(self):
        self.torque_enables += 1
        super().enable_torque()


@pytest.fixture
def rig(monkeypatch):
    pair = ArmPairProfile.load(ROOT / "tests" / "fixtures" / "so101_arm_pair.json")
    initial = pair.follower_calibration.normalized_to_raw([-3, 130, 38, 78, -8, 2.5])
    bus = CountingBus(initial)
    clock = [10.0]
    controller = service.FollowerController(
        pair,
        bus,
        now=lambda: clock[0],
        rest_pose_path=ROOT / "tests" / "fixtures" / "so101_rest_pose.json",
    )
    controller.connect()
    plans = []

    def stationary_plan(start, *attempt):
        plans.append((list(start), attempt, controller.status()))
        return [(list(start), 0.2, "sampling stationary test pose")]

    # Isolate transaction behavior from geometry solving. Real clearance and
    # bounded-motion regressions remain in test_so101_arm.py.
    for name in (
        "build_calibration_sweep_waypoints",
        "build_joint_calibration_sweep_waypoints",
        "build_base_axis_sweep_waypoints",
        "build_wrist_calibration_sweep_waypoints",
        "build_claw_calibration_sweep_waypoints",
    ):
        monkeypatch.setattr(service, name, stationary_plan)
    return SimpleNamespace(controller=controller, bus=bus, clock=clock, plans=plans)


def command(request_id="request-1", action="start", kind="arm_base_axis_sweep"):
    payload = {"type": kind, "action": action}
    if request_id is not None:
        payload["calibration_request_id"] = request_id
    return payload


@pytest.mark.parametrize("kind,mode,maximum_attempt", COMMANDS)
def test_each_sweep_echoes_request_id_and_resets_old_rejection_before_planning(
    rig, kind, mode, maximum_attempt
):
    controller = rig.controller
    controller.calibration_rejection = "old plan was not safe"
    payload = command(kind=kind)
    payload["view_strategy_attempt"] = 99
    controller.receive(payload)

    assert controller.calibration_sweep_active
    assert controller.calibration_sweep_mode == mode
    assert controller.status()["calibration_request_id"] == "request-1"
    assert controller.status()["calibration_request_state"] == "running"
    assert controller.calibration_rejection == ""
    _, attempt, planning_status = rig.plans[0]
    assert planning_status["calibration_request_id"] == "request-1"
    assert planning_status["calibration_rejection"] == ""
    assert planning_status["calibration_request_state"] == "planning"
    assert not planning_status["calibration_sweep_active"]
    assert attempt == (() if mode == "base" else (maximum_attempt,))


@pytest.mark.parametrize("ending", ("active", "hold", "stop", "complete", "fault"))
def test_duplicate_request_never_restarts_motion_including_after_termination(rig, ending):
    controller = rig.controller
    controller.receive(command())
    if ending == "hold":
        controller.receive({"type": "arm_hold"})
    elif ending == "stop":
        controller.receive(command(action="stop"))
    elif ending == "complete":
        rig.clock[0] += 1
        controller.update()
    elif ending == "fault":
        controller.fail("simulated hardware failure")
    before = (len(rig.plans), rig.bus.writes, rig.bus.torque_enables, controller.state)
    expected_state = {
        "active": "running", "hold": "cancelled", "stop": "cancelled",
        "complete": "completed", "fault": "fault",
    }[ending]
    assert controller.calibration_request_state == expected_state

    controller.receive(command())

    assert (len(rig.plans), rig.bus.writes, rig.bus.torque_enables, controller.state) == before
    assert controller.status()["calibration_request_id"] == "request-1"
    assert controller.calibration_request_state == expected_state


def test_old_start_and_stop_cannot_replace_or_cancel_newer_transaction(rig):
    controller = rig.controller
    controller.receive(command("old"))
    controller.receive(command("old", "stop"))
    controller.receive(command("new"))
    before = (len(rig.plans), rig.bus.writes, rig.bus.torque_enables)

    controller.receive(command("old"))
    controller.receive(command("old", "stop"))

    assert (len(rig.plans), rig.bus.writes, rig.bus.torque_enables) == before
    assert controller.calibration_sweep_active
    assert controller.calibration_request_id == "new"
    controller.receive(command("new", "stop"))
    assert not controller.calibration_sweep_active


def test_cancel_before_start_prevents_reordered_packet_from_starting_motion(rig):
    rig.controller.receive(command("cancelled", "stop"))
    rig.controller.receive(command("cancelled"))

    assert rig.plans == []
    assert rig.bus.writes == 0
    assert rig.bus.torque_enables == 0


def test_disconnected_replacement_cannot_report_previous_sweep_as_active(rig):
    controller = rig.controller
    controller.receive(command("old"))
    rig.bus.connected = False
    controller.receive(command("disconnected-replacement"))
    assert not controller.calibration_sweep_active
    assert controller.calibration_sweep_waypoints == []
    assert controller.calibration_request_state == "rejected"
    assert controller.calibration_request_id == "disconnected-replacement"


def test_hold_remains_unconditional_and_preserves_correlated_interruption(rig):
    controller = rig.controller
    controller.receive(command())
    controller.receive({"type": "arm_hold", "control_session": "browser"})

    status = controller.status()
    assert not status["calibration_sweep_active"]
    assert status["calibration_request_id"] == "request-1"
    assert status["state"] == "hold"
    assert status["message"] == "operator hold during automatic calibration"
    assert status["calibration_request_state"] == "cancelled"
    assert rig.bus.torque


def test_legacy_start_resets_transaction_and_legacy_stop_remains_unconditional(rig):
    controller = rig.controller
    controller.receive(command("identified"))
    controller.receive(command(None))
    assert controller.calibration_request_id == ""
    assert len(rig.plans) == 2
    controller.receive(command("identified", "stop"))
    assert controller.calibration_sweep_active
    controller.receive(command(None, "stop"))
    assert not controller.calibration_sweep_active
    controller.receive(command(None))
    assert controller.calibration_sweep_active
    assert len(rig.plans) == 3


@pytest.mark.parametrize("failure", ("fault", "disconnected", "read", "write", "torque"))
def test_start_failures_expose_correlated_specific_rejection_without_retry(rig, monkeypatch, failure):
    controller = rig.controller

    def fail_operation(*_args):
        raise RuntimeError("simulated " + failure + " failure")

    if failure == "fault":
        controller.fail("existing motor fault")
    elif failure == "disconnected":
        rig.bus.connected = False
    else:
        method = {"read": "read_positions", "write": "write_positions", "torque": "enable_torque"}
        monkeypatch.setattr(rig.bus, method[failure], fail_operation)
    controller.receive(command("failed-request"))

    status = controller.status()
    assert status["calibration_request_id"] == "failed-request"
    assert not status["calibration_sweep_active"]
    assert status["calibration_rejection"]
    assert status["calibration_request_state"] == (
        "rejected" if failure == "disconnected" else "fault"
    )
    if failure == "fault":
        assert "existing motor fault" in status["calibration_rejection"]
    elif failure == "disconnected":
        assert "not connected" in status["calibration_rejection"]
    else:
        assert "simulated " + failure + " failure" in status["calibration_rejection"]
    assert not rig.bus.torque
    before = (len(rig.plans), rig.bus.writes, rig.bus.torque_enables)
    controller.receive(command("failed-request"))
    assert (len(rig.plans), rig.bus.writes, rig.bus.torque_enables) == before
    controller.receive(command("failed-request", "stop"))
    assert controller.calibration_request_state == status["calibration_request_state"]


def test_rejected_geometry_keeps_hold_and_requires_new_id_for_retry(rig, monkeypatch):
    controller = rig.controller
    controller.receive(command("first"))
    controller.receive({"type": "arm_hold"})
    original_planner = service.build_base_axis_sweep_waypoints

    def reject(*_args):
        raise RuntimeError("simulated clearance rejection")

    monkeypatch.setattr(service, "build_base_axis_sweep_waypoints", reject)
    before = (rig.bus.writes, rig.bus.torque_enables)
    controller.receive(command("rejected"))
    assert controller.state == "hold"
    assert rig.bus.torque
    assert controller.calibration_rejection == "simulated clearance rejection"
    assert controller.calibration_request_state == "rejected"
    assert (rig.bus.writes, rig.bus.torque_enables) == before

    monkeypatch.setattr(service, "build_base_axis_sweep_waypoints", original_planner)
    controller.receive(command("rejected"))
    assert (rig.bus.writes, rig.bus.torque_enables) == before
    controller.receive(command("new-attempt"))
    assert controller.calibration_sweep_active
    assert controller.calibration_rejection == ""
    assert controller.calibration_request_id == "new-attempt"


@pytest.mark.parametrize("interruption", ("direct-hold", "release-torque", "restart"))
def test_other_safety_interruptions_cancel_transaction_without_allowing_replay(rig, interruption):
    controller = rig.controller
    controller.receive(command())
    if interruption == "direct-hold":
        controller.hold("bad local command")
    elif interruption == "release-torque":
        controller.release_torque()
    else:
        controller.restart_hardware()
    assert not controller.calibration_sweep_active
    assert controller.calibration_request_state == "cancelled"
    assert controller.calibration_request_id == "request-1"
    before = (len(rig.plans), rig.bus.writes, rig.bus.torque_enables)
    controller.receive(command())
    assert (len(rig.plans), rig.bus.writes, rig.bus.torque_enables) == before


def test_expired_start_is_rejected_before_encoder_read_or_planning(rig, monkeypatch):
    monkeypatch.setattr(service.time, "time", lambda: 100.0)

    def unexpected_read():
        pytest.fail("expired request accessed motor feedback")

    monkeypatch.setattr(rig.bus, "read_positions", unexpected_read)
    payload = command("expired")
    payload["calibration_request_expires_unix_ms"] = 100_000
    rig.controller.receive(payload)

    assert rig.controller.calibration_request_state == "rejected"
    assert "expired" in rig.controller.calibration_rejection
    assert not rig.controller.calibration_sweep_active
    assert rig.plans == []
    assert rig.bus.writes == rig.bus.torque_enables == 0


@pytest.mark.parametrize("slow_operation", ("read", "plan", "write", "enable-torque"))
def test_expiry_during_synchronous_work_prevents_starting_sweep(rig, monkeypatch, slow_operation):
    wall_clock = [100.0]
    monkeypatch.setattr(service.time, "time", lambda: wall_clock[0])
    if slow_operation == "plan":
        owner, name = service, "build_base_axis_sweep_waypoints"
    else:
        owner = rig.bus
        name = {
            "read": "read_positions", "write": "write_positions",
            "enable-torque": "enable_torque",
        }[slow_operation]
    original = getattr(owner, name)

    def slow(*args):
        result = original(*args)
        wall_clock[0] = 111.0
        return result

    monkeypatch.setattr(owner, name, slow)
    payload = command("slow")
    payload["calibration_request_expires_unix_ms"] = 110_000
    rig.controller.receive(payload)

    assert rig.controller.calibration_request_state == "rejected"
    assert "expired" in rig.controller.calibration_rejection
    assert not rig.controller.calibration_sweep_active
    assert rig.controller.calibration_sweep_waypoints == []
    assert rig.bus.writes == (1 if slow_operation in ("write", "enable-torque") else 0)
    assert rig.bus.torque_enables == (1 if slow_operation == "enable-torque" else 0)
    before = (rig.bus.writes, rig.bus.torque_enables)
    rig.controller.receive(payload)
    assert (rig.bus.writes, rig.bus.torque_enables) == before


@pytest.mark.parametrize("expiry", (-1, float("nan"), float("inf"), "invalid", None))
def test_invalid_expiry_is_rejected_without_motor_commands(rig, expiry):
    payload = command("invalid-expiry")
    payload["calibration_request_expires_unix_ms"] = expiry
    rig.controller.receive(payload)
    assert rig.controller.calibration_request_state == "rejected"
    assert "invalid" in rig.controller.calibration_rejection
    assert rig.plans == []
    assert rig.bus.writes == rig.bus.torque_enables == 0


def test_start_deadline_does_not_stop_already_running_calibration(rig, monkeypatch):
    wall_clock = [100.0]
    monkeypatch.setattr(service.time, "time", lambda: wall_clock[0])
    payload = command("on-time")
    payload["calibration_request_expires_unix_ms"] = 110_000
    rig.controller.receive(payload)
    assert rig.controller.calibration_request_state == "running"
    assert rig.controller.status()["calibration_request_expires_unix_ms"] == 110_000
    wall_clock[0] = 120.0
    rig.clock[0] += 1
    rig.controller.update()
    assert rig.controller.calibration_request_state == "completed"
    assert rig.controller.calibration_rejection == ""


def test_expired_replacement_does_not_leave_rest_return_flag_latched(rig, monkeypatch):
    monkeypatch.setattr(service.time, "time", lambda: 100.0)
    rig.controller.rest_return_active = True
    rig.controller.state = "returning_rest"
    payload = command("expired-replacement")
    payload["calibration_request_expires_unix_ms"] = 99_000
    rig.controller.receive(payload)
    assert not rig.controller.rest_return_active
    assert rig.controller.calibration_request_state == "rejected"
    assert rig.bus.writes == rig.bus.torque_enables == 0
