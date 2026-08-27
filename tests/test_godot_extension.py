import json
import subprocess
from pathlib import Path

import pytest

from robot_teleop.doctor import (
    GODOT_EXTENSION_CLASSES,
    GODOT_EXTENSION_METHODS,
    GODOT_EXTENSION_RESOURCE,
    GODOT_SMOKE_MARKER,
    ensure_godot_extension_index,
    godot_console_executable,
    godot_extension_index_status,
    godot_extension_smoke_status,
)


def write_extension_index(project_root: Path) -> None:
    index = project_root / ".godot" / "extension_list.cfg"
    index.parent.mkdir(parents=True, exist_ok=True)
    index.write_text(GODOT_EXTENSION_RESOURCE + "\n", encoding="utf-8")


def test_godot_diagnostics_prefer_the_windows_console_wrapper(tmp_path: Path):
    gui = tmp_path / "Godot_v4.7.1-stable_win64.exe"
    console = tmp_path / "Godot_v4.7.1-stable_win64_console.exe"
    gui.touch()
    console.touch()

    assert godot_console_executable(gui) == console
    assert godot_console_executable(console) == console


def test_extension_index_status_reports_a_clean_clone(tmp_path: Path):
    ok, detail = godot_extension_index_status(tmp_path)

    assert not ok
    assert "extension_list.cfg" in detail
    assert "editor import scan" in detail


def test_ensure_extension_index_runs_editor_scan_once(tmp_path: Path, monkeypatch):
    calls = []

    def run(command, **kwargs):
        calls.append((command, kwargs))
        write_extension_index(tmp_path)
        return subprocess.CompletedProcess(command, 0, "scan complete", "")

    monkeypatch.setattr("robot_teleop.doctor.subprocess.run", run)

    ensure_godot_extension_index(Path("godot"), tmp_path)
    ensure_godot_extension_index(Path("godot"), tmp_path)

    assert len(calls) == 1
    assert calls[0][0] == [
        "godot",
        "--headless",
        "--editor",
        "--path",
        str(tmp_path),
        "--quit",
    ]


def test_ensure_extension_index_fails_if_scan_does_not_create_it(tmp_path: Path, monkeypatch):
    monkeypatch.setattr(
        "robot_teleop.doctor.subprocess.run",
        lambda command, **kwargs: subprocess.CompletedProcess(command, 0, "", ""),
    )

    with pytest.raises(RuntimeError, match="did not index the native extension"):
        ensure_godot_extension_index(Path("godot"), tmp_path)


def test_godot_smoke_check_verifies_class_and_required_methods(tmp_path: Path, monkeypatch):
    write_extension_index(tmp_path)

    def run(command, **kwargs):
        assert command[command.index("--quit-after") + 1] == "120"
        script = Path(command[command.index("--script") + 1]).read_text(encoding="utf-8")
        assert "for class_name in" not in script
        assert all(class_name in script for class_name in GODOT_EXTENSION_CLASSES)
        assert all(method in script for method in GODOT_EXTENSION_METHODS)
        payload = {"missing_classes": [], "missing_methods": []}
        return subprocess.CompletedProcess(
            command,
            0,
            GODOT_SMOKE_MARKER + json.dumps(payload),
            "",
        )

    monkeypatch.setattr("robot_teleop.doctor.subprocess.run", run)

    ok, detail = godot_extension_smoke_status(Path("godot"), tmp_path)

    assert ok
    assert "4 native classes" in detail
    assert "4 required RGB-D methods" in detail


def test_godot_smoke_check_reports_a_stale_native_binary(tmp_path: Path, monkeypatch):
    write_extension_index(tmp_path)
    payload = {
        "missing_classes": [],
        "missing_methods": ["get_depth_u16_frame", "request_rgbd_encode"],
    }
    monkeypatch.setattr(
        "robot_teleop.doctor.subprocess.run",
        lambda command, **kwargs: subprocess.CompletedProcess(
            command,
            3,
            GODOT_SMOKE_MARKER + json.dumps(payload),
            "",
        ),
    )

    ok, detail = godot_extension_smoke_status(Path("godot"), tmp_path)

    assert not ok
    assert "stale" in detail
    assert "missing required methods" in detail
    assert "get_depth_u16_frame" in detail


def test_godot_smoke_check_reports_a_missing_native_class(tmp_path: Path, monkeypatch):
    write_extension_index(tmp_path)
    payload = {
        "missing_classes": ["RealSensePairCalibrator"],
        "missing_methods": [],
    }
    monkeypatch.setattr(
        "robot_teleop.doctor.subprocess.run",
        lambda command, **kwargs: subprocess.CompletedProcess(
            command,
            3,
            GODOT_SMOKE_MARKER + json.dumps(payload),
            "",
        ),
    )

    ok, detail = godot_extension_smoke_status(Path("godot"), tmp_path)

    assert not ok
    assert "missing required classes" in detail
    assert "RealSensePairCalibrator" in detail
