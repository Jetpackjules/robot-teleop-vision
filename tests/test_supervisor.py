from pathlib import Path

import pytest

import robot_teleop.robots  # noqa: F401 - register the vision-only adapter
from robot_teleop.config import AppConfig, GodotConfig, StackConfig
from robot_teleop.supervisor import Supervisor, operator_ui_response_ready


def test_operator_health_accepts_controller_shell():
    assert operator_ui_response_ready(
        "https://127.0.0.1:8765/controller.html",
        200,
        b'<canvas id="hybrid-canvas"></canvas>',
    )


def test_operator_health_accepts_expected_password_gate():
    assert operator_ui_response_ready(
        "https://127.0.0.1:8765/login?next=%2Fcontroller.html",
        200,
        b'<form method="post" action="/login"><input name="password"></form>',
    )


def test_operator_health_rejects_unrelated_success_page():
    assert not operator_ui_response_ready(
        "https://127.0.0.1:8765/error",
        200,
        b"temporarily unavailable",
    )


def test_operator_password_starting_with_dash_is_passed_as_one_argument(tmp_path, monkeypatch):
    config = AppConfig(
        source=tmp_path / "local.toml",
        stack=StackConfig(public_mode="off", password_default="-generated-password"),
        godot=GodotConfig(launch_runtime=False, project=str(tmp_path)),
    )
    supervisor = Supervisor(config)
    launches = []
    monkeypatch.setattr("robot_teleop.supervisor._port_available", lambda _port: True)
    monkeypatch.setattr(supervisor, "_publish_state", lambda *_args: None)
    monkeypatch.setattr(supervisor, "_verify_local_url", lambda: None)
    monkeypatch.setattr(supervisor, "_spawn", launches.append)

    supervisor.start()

    assert len(launches) == 1
    assert "--password=-generated-password" in launches[0].command
    assert "--password" not in launches[0].command


def test_source_runtime_bootstraps_extension_index_before_launch(tmp_path, monkeypatch):
    config = AppConfig(
        source=tmp_path / "local.toml",
        stack=StackConfig(public_mode="off"),
        godot=GodotConfig(
            executable="configured-godot",
            project=str(tmp_path),
            launch_runtime=True,
            packaged_runtime=False,
        ),
    )
    supervisor = Supervisor(config)
    calls = []
    monkeypatch.setattr("robot_teleop.supervisor._port_available", lambda _port: True)
    monkeypatch.setattr(supervisor, "_publish_state", lambda *_args: None)
    monkeypatch.setattr(
        "robot_teleop.supervisor.find_godot",
        lambda _configured: Path("godot"),
    )

    def ensure(godot, project_root):
        calls.append((godot, project_root))
        raise RuntimeError("extension-index sentinel")

    monkeypatch.setattr("robot_teleop.supervisor.ensure_godot_extension_index", ensure)

    with pytest.raises(RuntimeError, match="extension-index sentinel"):
        supervisor.start()

    assert calls == [(Path("godot"), tmp_path)]


def test_source_runtime_refuses_to_launch_an_unusable_native_extension(tmp_path, monkeypatch):
    config = AppConfig(
        source=tmp_path / "local.toml",
        stack=StackConfig(public_mode="off"),
        godot=GodotConfig(
            executable="configured-godot",
            project=str(tmp_path),
            launch_runtime=True,
            packaged_runtime=False,
        ),
    )
    supervisor = Supervisor(config)
    calls = []
    monkeypatch.setattr("robot_teleop.supervisor._port_available", lambda _port: True)
    monkeypatch.setattr(supervisor, "_publish_state", lambda *_args: None)
    monkeypatch.setattr(
        "robot_teleop.supervisor.find_godot",
        lambda _configured: Path("godot"),
    )
    monkeypatch.setattr(
        "robot_teleop.supervisor.ensure_godot_extension_index",
        lambda godot, project_root: calls.append(("index", godot, project_root)),
    )

    def smoke(godot, project_root):
        calls.append(("smoke", godot, project_root))
        return False, "loaded native extension is stale"

    monkeypatch.setattr("robot_teleop.supervisor.godot_extension_smoke_status", smoke)
    monkeypatch.setattr(
        supervisor,
        "_spawn",
        lambda _spec: pytest.fail("runtime must not launch after a failed native smoke check"),
    )

    with pytest.raises(RuntimeError, match="native RGB-D extension check failed.*stale"):
        supervisor.start()

    assert calls == [
        ("index", Path("godot"), tmp_path),
        ("smoke", Path("godot"), tmp_path),
    ]
