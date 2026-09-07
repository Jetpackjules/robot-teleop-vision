from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

from robot_modules.so101.python.adapter import So101RobotAdapter
from robot_modules.so101.tools.so101_arm_common import ArmPairProfile
from robot_teleop.config import AppConfig, GodotConfig, RobotConfig, StackConfig
from robot_teleop.supervisor import Supervisor, write_runtime_configuration

FIXTURE = Path(__file__).parent / "fixtures" / "so101_arm_pair.json"
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_modules/so101/tools"))
import so101_follower_service


def profile_file(tmp_path: Path, **overrides) -> Path:
    profile = json.loads(FIXTURE.read_text(encoding="utf-8"))
    profile.update(overrides)
    path = tmp_path / "custom arm pair.json"
    path.write_text(json.dumps(profile), encoding="utf-8")
    return path


def adapter(profile: Path, *, enabled: bool = True) -> So101RobotAdapter:
    return So101RobotAdapter(
        RobotConfig(
            adapter="so101",
            enabled=enabled,
            options={"profile": str(profile), "dry_run": True, "hold_on_connect": False},
        )
    )


def test_legacy_profile_keeps_default_editor_telemetry_port():
    assert ArmPairProfile.load(FIXTURE).editor_status_port == 4252


def test_custom_ports_are_shared_by_follower_operator_and_godot(tmp_path):
    path = profile_file(
        tmp_path,
        command_port=21448,
        status_port=21449,
        godot_status_port=21450,
        editor_status_port=21452,
    )
    robot = adapter(path)
    assert ArmPairProfile.load(path).editor_status_port == 21452
    assert robot.godot_configuration(calibration_status_port=21451) == {
        "command_port": 21448,
        "status_port": 21449,
        "telemetry_port": 21450,
        "editor_telemetry_port": 21452,
    }
    spec = robot.launch_spec()
    assert spec is not None
    for flag, expected in (
        ("--command-port", "21448"),
        ("--status-port", "21449"),
        ("--godot-status-port", "21450"),
        ("--editor-status-port", "21452"),
    ):
        assert spec.command[spec.command.index(flag) + 1] == expected
    environment = robot.operator_environment()
    assert environment["ROBOT_TELEOP_COMMAND_PORT"] == "21448"
    assert environment["ROBOT_TELEOP_STATUS_PORT"] == "21449"
    assert "--dry-run" in spec.command
    assert "--hold-on-connect" not in spec.command


@pytest.mark.parametrize(
    "field", ["command_port", "status_port", "godot_status_port", "editor_status_port"]
)
@pytest.mark.parametrize("invalid", [0, -1, 65536, True, "21448", 21448.5, None])
def test_launched_stack_rejects_invalid_or_disabled_ports(tmp_path, field, invalid):
    robot = adapter(profile_file(tmp_path, **{field: invalid}))
    with pytest.raises(ValueError, match=field):
        robot.godot_configuration(calibration_status_port=4251)
    with pytest.raises(ValueError, match=field):
        robot.launch_spec()


def test_launched_stack_rejects_duplicate_module_ports(tmp_path):
    robot = adapter(profile_file(tmp_path, editor_status_port=14250))
    with pytest.raises(ValueError, match="distinct"):
        robot.godot_configuration(calibration_status_port=4251)


@pytest.mark.parametrize("calibration_port", [14248, 14249, 14250, 4252])
def test_launched_stack_rejects_calibration_status_port_collision(tmp_path, calibration_port):
    robot = adapter(profile_file(tmp_path))
    with pytest.raises(ValueError, match="stack.calibration_status_port"):
        robot.godot_configuration(calibration_status_port=calibration_port)


@pytest.mark.parametrize(
    "field", ["command_port", "status_port", "godot_status_port", "editor_status_port"]
)
def test_module_rejects_ports_reserved_by_shared_tracking(tmp_path, field):
    robot = adapter(profile_file(tmp_path, **{field: 4247}))
    with pytest.raises(ValueError, match="shared tracker"):
        robot.godot_configuration(
            calibration_status_port=4251,
            reserved_udp_ports={"shared tracker": 4247},
        )


def test_disabled_module_does_not_read_profile_or_open_transport(tmp_path):
    robot = adapter(tmp_path / "missing.json", enabled=False)
    assert robot.godot_configuration(calibration_status_port=4251) == {}
    assert robot.launch_spec() is None
    assert "ROBOT_TELEOP_COMMAND_PORT" not in robot.operator_environment()


def test_profile_edit_cannot_split_a_running_transport_or_redirect_shutdown_hold(
    tmp_path, monkeypatch
):
    path = profile_file(tmp_path)
    robot = adapter(path)
    initial = robot.godot_configuration(calibration_status_port=4251)
    profile_file(tmp_path, command_port=24248, status_port=24249, godot_status_port=24250)
    assert robot.godot_configuration(calibration_status_port=4251) == initial
    assert robot.operator_environment()["ROBOT_TELEOP_COMMAND_PORT"] == "14248"
    spec = robot.launch_spec()
    assert spec.command[spec.command.index("--command-port") + 1] == "14248"
    messages = []

    class FakeSocket:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return None

        def sendto(self, payload, address):
            messages.append((json.loads(payload), address))

    monkeypatch.setattr("robot_modules.so101.python.adapter.socket.socket", lambda *_: FakeSocket())
    robot.hold()
    assert messages == [({"type": "arm_hold"}, ("127.0.0.1", 14248))]


def test_supervisor_shares_exact_nonsecret_configuration_before_spawning(tmp_path, monkeypatch):
    path = profile_file(tmp_path, editor_status_port=14252)
    config = AppConfig(
        source=tmp_path / "local.toml",
        stack=StackConfig(
            public_mode="off", calibration_status_port=14251, password_default="private-value"
        ),
        godot=GodotConfig(project=str(tmp_path), launch_runtime=True, packaged_runtime=True),
        robot=adapter(path).config,
    )
    supervisor = Supervisor(config)
    monkeypatch.setattr("robot_teleop.supervisor._port_available", lambda _port: True)
    monkeypatch.setattr("robot_teleop.supervisor.find_godot", lambda _path: Path("fake-godot"))
    monkeypatch.setattr(supervisor, "_publish_state", lambda *_args: None)
    monkeypatch.setattr(supervisor, "_verify_local_url", lambda: None)
    launches = []
    destination = tmp_path / ".teleop" / "runtime_config.json"

    def spawn(spec):
        assert destination.is_file(), "editor configuration must exist before a child starts"
        launches.append(spec)

    monkeypatch.setattr(supervisor, "_spawn", spawn)
    try:
        supervisor.start()
    finally:
        supervisor.journal.close()
    runtime = next(spec for spec in launches if spec.name == "Godot runtime")
    shared = json.loads(runtime.environment["ROBOT_TELEOP_RUNTIME_CONFIG"])
    assert json.loads(destination.read_text(encoding="utf-8")) == shared
    assert shared == {
        "schema_version": 1,
        "module": "so101",
        "enabled": True,
        "calibration_status_port": 14251,
        "module_settings": {
            "command_port": 14248,
            "status_port": 14249,
            "telemetry_port": 14250,
            "editor_telemetry_port": 14252,
        },
    }
    assert "private-value" not in json.dumps(shared)
    assert str(path) not in json.dumps(shared)
    operator = next(spec for spec in launches if spec.name == "operator server")
    assert operator.environment["ROBOT_TELEOP_COMMAND_PORT"] == "14248"
    assert operator.environment["ROBOT_TELEOP_STATUS_PORT"] == "14249"


@pytest.mark.parametrize("invalid", [True, "4251", 0, 65536, 14248])
def test_supervisor_invalid_transport_fails_before_any_child_or_config_write(
    tmp_path, monkeypatch, invalid
):
    path = profile_file(tmp_path)
    config = AppConfig(
        source=tmp_path / "local.toml",
        stack=StackConfig(public_mode="off", calibration_status_port=invalid),
        godot=GodotConfig(project=str(tmp_path), launch_runtime=False),
        robot=adapter(path).config,
    )
    supervisor = Supervisor(config)
    monkeypatch.setattr(supervisor, "_spawn", lambda _: pytest.fail("must not start hardware"))
    try:
        with pytest.raises(ValueError, match="calibration_status_port"):
            supervisor.start()
    finally:
        supervisor.journal.close()
    assert not (tmp_path / ".teleop" / "runtime_config.json").exists()


@pytest.mark.parametrize(
    "tracking_port, command_port, label",
    [
        (14248, 14248, "stack.tracking_port"),
        (24247, 4247, "Godot remote-control gateway"),
    ],
)
def test_supervisor_reserves_configured_and_actual_tracking_listener_before_spawning(
    tmp_path, monkeypatch, tracking_port, command_port, label
):
    path = profile_file(tmp_path, command_port=command_port)
    config = AppConfig(
        source=tmp_path / "local.toml",
        stack=StackConfig(public_mode="off", tracking_port=tracking_port),
        godot=GodotConfig(project=str(tmp_path), launch_runtime=False),
        robot=adapter(path).config,
    )
    supervisor = Supervisor(config)
    monkeypatch.setattr(supervisor, "_spawn", lambda _: pytest.fail("must not start hardware"))
    try:
        with pytest.raises(ValueError, match=label):
            supervisor.start()
    finally:
        supervisor.journal.close()
    assert not (tmp_path / ".teleop" / "runtime_config.json").exists()


def test_atomic_runtime_configuration_replaces_previous_snapshot(tmp_path):
    write_runtime_configuration(tmp_path, {"enabled": True})
    write_runtime_configuration(tmp_path, {"enabled": False})
    target = tmp_path / ".teleop" / "runtime_config.json"
    assert json.loads(target.read_text(encoding="utf-8")) == {"enabled": False}
    assert list(target.parent.glob("*.tmp")) == []


@pytest.mark.parametrize(
    "flags, expected",
    [
        ([], (14248, 14249, 14250, 14252)),
        (
            [
                "--command-port",
                "22448",
                "--status-port",
                "22449",
                "--godot-status-port",
                "22450",
                "--editor-status-port",
                "22452",
            ],
            (22448, 22449, 22450, 22452),
        ),
        (["--godot-status-port", "0", "--editor-status-port", "0"], (14248, 14249, 0, 0)),
    ],
)
def test_standalone_follower_cli_preserves_explicit_overrides_and_optional_zero(
    tmp_path, monkeypatch, flags, expected
):
    path = profile_file(tmp_path, editor_status_port=14252)
    monkeypatch.setattr(sys, "argv", ["follower", "--profile", str(path), "--dry-run", *flags])
    calls = []
    monkeypatch.setattr(
        so101_follower_service, "run_service", lambda *args: calls.append(args) or 0
    )
    assert so101_follower_service.main() == 0
    assert calls[0][2:6] == expected


@pytest.mark.parametrize(
    "flag, invalid",
    [
        ("--command-port", "0"),
        ("--status-port", "0"),
        ("--godot-status-port", "-1"),
        ("--editor-status-port", "65536"),
    ],
)
def test_standalone_follower_cli_rejects_invalid_ports_before_bus_creation(
    monkeypatch, flag, invalid
):
    monkeypatch.setattr(
        sys, "argv", ["follower", "--profile", str(FIXTURE), "--dry-run", flag, invalid]
    )
    monkeypatch.setattr(
        so101_follower_service, "FakeFollowerBus", lambda *_: pytest.fail("must not create a bus")
    )
    with pytest.raises(SystemExit) as error:
        so101_follower_service.main()
    assert error.value.code == 2


@pytest.mark.parametrize("port", ["COM5", "COM10", "COM256", r"\\.\COM12"])
def test_windows_serial_preflight_does_not_treat_com_names_as_files(port):
    assert so101_follower_service._configured_follower_port_exists(port, platform="win32")


def test_linux_serial_preflight_still_checks_the_configured_device_path(tmp_path):
    absent = tmp_path / "missing-device"
    assert not so101_follower_service._configured_follower_port_exists(
        str(absent), platform="linux"
    )
    # A fixture path is sufficient: this is path preflight only, never a claim
    # that a successful hardware handshake has occurred.
    assert so101_follower_service._configured_follower_port_exists(str(FIXTURE), platform="linux")


def test_windows_cli_passes_only_the_exact_configured_com_port_to_the_bus(tmp_path, monkeypatch):
    path = profile_file(tmp_path)
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["follower"]["port"] = "COM128"
    path.write_text(json.dumps(payload), encoding="utf-8")
    monkeypatch.setattr(sys, "argv", ["follower", "--profile", str(path)])
    preflight = so101_follower_service._configured_follower_port_exists
    monkeypatch.setattr(
        so101_follower_service,
        "_configured_follower_port_exists",
        lambda port: preflight(port, platform="win32"),
    )
    buses = []

    def fake_bus(port, motor_names):
        result = object()
        buses.append((port, motor_names, result))
        return result

    monkeypatch.setattr(so101_follower_service, "LeRobotFollowerBus", fake_bus)
    launches = []
    monkeypatch.setattr(
        so101_follower_service, "run_service", lambda *args: launches.append(args) or 0
    )
    assert so101_follower_service.main() == 0
    assert len(buses) == 1
    assert buses[0][0] == "COM128"
    assert launches[0][0].follower_port == "COM128"
    assert launches[0][1] is buses[0][2]


def test_linux_cli_missing_configured_device_rejects_before_bus_creation(tmp_path, monkeypatch):
    path = profile_file(tmp_path)
    monkeypatch.setattr(sys, "argv", ["follower", "--profile", str(path)])
    preflight = so101_follower_service._configured_follower_port_exists
    monkeypatch.setattr(
        so101_follower_service,
        "_configured_follower_port_exists",
        lambda port: preflight(port, platform="linux"),
    )
    monkeypatch.setattr(
        so101_follower_service,
        "LeRobotFollowerBus",
        lambda *_: pytest.fail("missing configured device must not construct a bus"),
    )
    assert so101_follower_service.main() == 2
