from pathlib import Path
from types import SimpleNamespace

from robot_teleop.cli import main


def test_godot_path_prints_the_resolved_executable(monkeypatch, capsys):
    configured = "configured-godot"
    resolved = Path("C:/tools/Godot/Godot_console.exe")
    monkeypatch.setattr(
        "robot_teleop.cli.load_config",
        lambda _path: SimpleNamespace(godot=SimpleNamespace(executable=configured)),
    )
    monkeypatch.setattr(
        "robot_teleop.cli.find_godot",
        lambda value: resolved if value == configured else None,
    )

    assert main(["godot-path"]) == 0
    assert capsys.readouterr().out.strip() == str(resolved)


def test_godot_path_returns_failure_when_godot_is_missing(monkeypatch, capsys):
    monkeypatch.setattr(
        "robot_teleop.cli.load_config",
        lambda _path: SimpleNamespace(godot=SimpleNamespace(executable="missing")),
    )
    monkeypatch.setattr("robot_teleop.cli.find_godot", lambda _value: None)

    assert main(["godot-path"]) == 1
    assert capsys.readouterr().out == ""
