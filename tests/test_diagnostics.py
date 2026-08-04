import json
import zipfile
from pathlib import Path

from robot_teleop.config import AppConfig, CameraConfig, RobotConfig, StackConfig
from robot_teleop.diagnostics import create_support_bundle, sanitize_text, sanitize_url
from robot_teleop.doctor import Check
from robot_teleop.journal import journal_entry_payload, stream_is_journal


def test_native_journal_payload_has_stable_identifier_and_component():
    payload = journal_entry_payload(
        "camera ready\nnext line",
        priority=4,
        component="Godot runtime",
    ).decode("utf-8")

    assert "SYSLOG_IDENTIFIER=robot-teleop\n" in payload
    assert "PRIORITY=4\n" in payload
    assert "TELEOP_COMPONENT=Godot runtime\n" in payload
    assert "MESSAGE=camera ready next line\n" in payload


def test_stale_or_malformed_journal_stream_marker_does_not_disable_mirroring():
    assert not stream_is_journal({})
    assert not stream_is_journal({"JOURNAL_STREAM": "not-an-inode"})
    assert not stream_is_journal({"JOURNAL_STREAM": "0:0"})


def test_text_sanitizer_removes_credentials_home_and_url_queries():
    text = (
        f"password=hunter2 token:abcd1234 path={Path.home()}/capture "
        "url=https://named.example/path?access_token=bad"
    )

    sanitized = sanitize_text(text, secrets=["hunter2"])

    assert "hunter2" not in sanitized
    assert "abcd1234" not in sanitized
    assert str(Path.home()) not in sanitized
    assert "access_token" not in sanitized
    assert "named.example" not in sanitized


def test_quick_tunnel_url_is_replaced_but_route_is_retained():
    assert sanitize_url("https://words.trycloudflare.com/controller.html?x=1") == (
        "https://<temporary-quick-tunnel>/controller.html"
    )


def test_support_bundle_excludes_raw_secrets_profiles_serials_and_imagery(tmp_path, monkeypatch):
    config = AppConfig(
        source=tmp_path / "local.toml",
        stack=StackConfig(password_default="do-not-share-this"),
        cameras=CameraConfig(("realsense",)),
        robot=RobotConfig(
            adapter="example",
            enabled=True,
            options={
                "profile": "/private/robot.json",
                "api_key": "module-secret-value",
            },
        ),
    )
    devices = [
        {
            "adapter": "realsense",
            "identifier": "123456789",
            "label": "Intel RealSense D455 (123456789)",
            "metadata": {"firmware": "5.16.0", "physical_port": "usb-3"},
        }
    ]
    monkeypatch.setattr(
        "robot_teleop.diagnostics.run_checks",
        lambda _config: ([Check("camera", True, "serial 123456789")], devices),
    )
    monkeypatch.setattr(
        "robot_teleop.diagnostics.collect_journal",
        lambda _lines: {
            "available": True,
            "detail": "test",
            "text": "serial=123456789 password=do-not-share-this api_key=module-secret-value",
        },
    )
    monkeypatch.setattr(
        "robot_teleop.diagnostics.read_state",
        lambda: {
            "running": True,
            "public_url": "https://words.trycloudflare.com/controller.html?token=bad",
        },
    )
    monkeypatch.setenv(config.stack.password_env, "environment-secret-value")
    output = tmp_path / "support.zip"

    created = create_support_bundle(config, output)

    assert created == output
    with zipfile.ZipFile(created) as archive:
        assert sorted(archive.namelist()) == ["README.txt", "journal.log", "summary.json"]
        combined = "\n".join(
            archive.read(name).decode("utf-8") for name in archive.namelist()
        )
        summary = json.loads(archive.read("summary.json"))
    for forbidden in (
        "123456789",
        "do-not-share-this",
        "module-secret-value",
        "environment-secret-value",
        "/private/robot.json",
        "token=bad",
    ):
        assert forbidden not in combined
    assert summary["privacy"]["sanitized"] is True
    assert summary["configuration"]["robot"]["option_names"] == ["api_key", "profile"]
    assert summary["devices"][0]["identifier"].startswith("<camera-")
    assert "calibration" not in archive.namelist()
    assert not any(name.endswith((".jpg", ".png", ".rgb", ".depth")) for name in archive.namelist())


def test_support_bundle_refuses_to_overwrite(tmp_path):
    output = tmp_path / "existing.zip"
    output.write_bytes(b"keep")

    try:
        create_support_bundle(AppConfig(), output)
    except FileExistsError:
        pass
    else:
        raise AssertionError("expected FileExistsError")

    assert output.read_bytes() == b"keep"
