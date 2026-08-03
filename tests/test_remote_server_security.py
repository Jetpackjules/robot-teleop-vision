from __future__ import annotations

import json
import socket
import sys
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from lan_remote_view_server import LanRemoteHandler, LanRemoteServer
from robot_teleop.operator import load_robot_operator


def bare_handler(headers=None, server=None):
    handler = object.__new__(LanRemoteHandler)
    handler.headers = headers or {}
    handler.server = server or SimpleNamespace(robot_operator=load_robot_operator("so101"))
    return handler


def test_login_redirect_target_never_leaves_origin():
    handler = bare_handler()
    assert handler.safe_login_next("/controller.html?mode=quality") == "/controller.html?mode=quality"
    assert handler.safe_login_next("https://evil.example/steal") == "/controller.html"
    assert handler.safe_login_next("//evil.example/steal") == "/controller.html"


def test_login_form_uses_post_and_does_not_put_password_in_query():
    handler = bare_handler()
    page = handler.login_html("/controller.html", "").decode("utf-8")
    assert 'method="post"' in page
    assert 'action="/login"' in page
    assert 'name="password"' in page


def test_websocket_origin_must_match_request_host():
    handler = bare_handler({"Origin": "https://window.example.com", "Host": "window.example.com"})
    assert handler.websocket_origin_allowed()
    handler.headers = {"Origin": "https://evil.example", "Host": "window.example.com"}
    assert not handler.websocket_origin_allowed()
    handler.headers = {"Host": "window.example.com"}
    assert not handler.websocket_origin_allowed()


def test_quick_tunnel_cannot_authorize_arm_but_named_access_can():
    server = SimpleNamespace(
        public_robot=True,
        public_hostname="window.example.com",
        verify_access_token=lambda token: token == "valid",
        password="test-password",
        session_cookie="cookie",
    )
    quick = bare_handler(
        {"Origin": "https://random.trycloudflare.com", "Host": "random.trycloudflare.com", "Cf-Ray": "abc"},
        server,
    )
    assert not quick.robot_authorized()
    named = bare_handler(
        {
            "Origin": "https://window.example.com",
            "Host": "window.example.com",
            "Cf-Ray": "abc",
            "Cf-Access-Jwt-Assertion": "valid",
        },
        server,
    )
    assert named.robot_authorized()


def test_explicit_quick_tunnel_mode_uses_password_session_cookie():
    server = SimpleNamespace(
        public_robot=False,
        public_hostname="",
        allow_quick_tunnel_robot=True,
        verify_access_token=lambda _token: False,
        password="test-password",
        session_cookie="cookie",
    )
    handler = bare_handler(
        {
            "Origin": "https://temporary.trycloudflare.com",
            "Host": "temporary.trycloudflare.com",
            "Cf-Ray": "abc",
            "Cookie": "godot_remote_session=cookie",
        },
        server,
    )
    assert handler.robot_authorized()


def test_only_one_arm_controller_can_hold_lease(tmp_path):
    server = LanRemoteServer(
        ("127.0.0.1", 0),
        LanRemoteHandler,
        tmp_path,
        "127.0.0.1",
        4247,
        "127.0.0.1",
        8780,
        "window",
        robot_module="so101",
        robot_status_port=0,
        robot_calibration_status_port=0,
    )
    try:
        first = server.claim_robot_controller("first")
        assert first
        assert server.claim_robot_controller("second") is None
        server.release_robot_controller(first)
        assert server.claim_robot_controller("second")
    finally:
        server.server_close()


def test_arm_transport_jitter_drops_one_packet_without_forcing_hold(tmp_path):
    server = LanRemoteServer(
        ("127.0.0.1", 0),
        LanRemoteHandler,
        tmp_path,
        "127.0.0.1",
        4247,
        "127.0.0.1",
        8780,
        "window",
        robot_module="so101",
        robot_status_port=0,
        robot_calibration_status_port=0,
    )
    try:
        initial = {"sent_unix_ms": 9_900}
        accepted, age_ms = server.normalize_browser_robot_timing(initial, 10_000)
        assert accepted
        assert age_ms == 0

        delayed = {"sent_unix_ms": 9_600}
        accepted, age_ms = server.normalize_browser_robot_timing(delayed, 10_000)
        assert not accepted
        assert age_ms == 300
        assert server.robot_stale_command_drop_count == 1

        fresh = {"sent_unix_ms": 9_966}
        accepted, age_ms = server.normalize_browser_robot_timing(fresh, 10_066)
        assert accepted
        assert age_ms == 0
    finally:
        server.server_close()


def test_arm_transport_rebases_after_stable_latency_path_shift(tmp_path):
    server = LanRemoteServer(
        ("127.0.0.1", 0),
        LanRemoteHandler,
        tmp_path,
        "127.0.0.1",
        4247,
        "127.0.0.1",
        8780,
        "window",
        robot_module="so101",
        robot_status_port=0,
        robot_calibration_status_port=0,
    )
    try:
        assert server.normalize_browser_robot_timing(
            {"sent_unix_ms": 9_900},
            10_000,
        )[0]
        offsets = (405, 399, 408, 402)
        decisions = []
        for index, offset in enumerate(offsets):
            bridge_ms = 10_100 + index * 33
            message = {"sent_unix_ms": bridge_ms - offset}
            decisions.append(
                server.normalize_browser_robot_timing(message, bridge_ms)[0]
            )
        assert decisions == [False, False, False, True]
        assert server.robot_browser_clock_offset_ms == pytest.approx(404, abs=1)
    finally:
        server.server_close()


def test_keyboard_auto_resume_only_allows_same_session_watchdog_pause(tmp_path):
    server = LanRemoteServer(
        ("127.0.0.1", 0),
        LanRemoteHandler,
        tmp_path,
        "127.0.0.1",
        4247,
        "127.0.0.1",
        8780,
        "window",
        robot_module="so101",
        robot_status_port=0,
        robot_calibration_status_port=0,
    )
    forwarded = []
    try:
        controller_id = server.claim_robot_controller("browser")
        assert controller_id
        with server.robot_status_lock:
            server.robot_status = {
                "type": "arm_status",
                "state": "hold",
                "message": "command watchdog expired",
                "control_session": controller_id,
            }
            server.robot_status_received_at = time.monotonic()
        server.forward_robot = forwarded.append
        command = {"type": "arm_cartesian_velocity", "deadman": True}
        assert server.resume_after_transient_watchdog(
            controller_id,
            command,
        )
        assert forwarded == [
            {
                "type": "arm_enable",
                "source": "keyboard",
                "control_session": controller_id,
            }
        ]

        with server.robot_status_lock:
            server.robot_status["message"] = (
                "Contact / following error stopped motion at servo 2"
            )
        assert not server.resume_after_transient_watchdog(
            controller_id,
            command,
        )
        assert len(forwarded) == 1
    finally:
        server.server_close()


def test_latest_rgbd_page_preempts_and_supersedes_older_page(tmp_path):
    server = LanRemoteServer(
        ("127.0.0.1", 0),
        LanRemoteHandler,
        tmp_path,
        "127.0.0.1",
        4247,
        "127.0.0.1",
        8780,
        "window",
        robot_module="so101",
        robot_status_port=0,
        robot_calibration_status_port=0,
    )

    class FakeHandler:
        def __init__(self):
            self.preempted = False

        def preempt_rgbd_websocket(self):
            self.preempted = True

    first = FakeHandler()
    second = FakeHandler()
    try:
        assert server.claim_rgbd_client("first-page", first)
        assert server.claim_rgbd_client("second-page", second)
        assert first.preempted
        assert not second.preempted
        assert not server.claim_rgbd_client("first-page", first)
        assert server.claim_rgbd_client("second-page", second)
    finally:
        server.server_close()


def test_newest_page_preempts_stream_and_arm_control_together(tmp_path):
    server = LanRemoteServer(
        ("127.0.0.1", 0),
        LanRemoteHandler,
        tmp_path,
        "127.0.0.1",
        4247,
        "127.0.0.1",
        8780,
        "window",
        robot_module="so101",
        robot_status_port=0,
        robot_calibration_status_port=0,
    )

    class FakeHandler:
        def __init__(self):
            self.robot_preempted = False
            self.rgbd_preempted = False

        def preempt_robot_websocket(self):
            self.robot_preempted = True

        def preempt_rgbd_websocket(self):
            self.rgbd_preempted = True

    old_arm = FakeHandler()
    new_rgbd = FakeHandler()
    forwarded = []
    server.forward_robot = forwarded.append
    try:
        assert server.claim_robot_controller("old", "old-page", old_arm)
        assert server.claim_rgbd_client("new-page", new_rgbd)
        assert old_arm.robot_preempted
        assert {"type": "arm_hold"} in forwarded
        assert not server.claim_rgbd_client("old-page", FakeHandler())
    finally:
        server.server_close()


def test_robot_calibration_status_is_merged_into_robot_status(tmp_path):
    server = LanRemoteServer(
        ("127.0.0.1", 0),
        LanRemoteHandler,
        tmp_path,
        "127.0.0.1",
        4247,
        "127.0.0.1",
        8780,
        "window",
        robot_module="so101",
        robot_status_port=0,
        robot_calibration_status_port=0,
    )
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        port = server.robot_calibration_socket.getsockname()[1]
        sender.sendto(
            json.dumps(
                {
                    "type": "robot_calibration_status",
                    "state": "capturing",
                    "frames": 7,
                    "confidence": 0.0,
                    "message": "Move the arm",
                }
            ).encode("utf-8"),
            ("127.0.0.1", port),
        )
        deadline = time.monotonic() + 1.0
        status = {}
        while time.monotonic() < deadline:
            status = server.latest_robot_status().get("robot_calibration", {})
            if status.get("frames") == 7:
                break
            time.sleep(0.01)
        assert status["state"] == "capturing"
        assert status["frames"] == 7
    finally:
        sender.close()
        server.server_close()


def test_static_path_containment_uses_real_path_boundaries(tmp_path):
    root = tmp_path / "web"
    sibling = tmp_path / "web-secret"
    root.mkdir()
    sibling.mkdir()
    assert not (sibling / "secret.txt").resolve().is_relative_to(root.resolve())


def test_websocket_rate_limit_is_bounded():
    handler = bare_handler()
    window = deque()
    handler.check_websocket_rate(window, 2)
    handler.check_websocket_rate(window, 2)
    with pytest.raises(ValueError, match="rate"):
        handler.check_websocket_rate(window, 2)


def test_remote_view_settings_are_typed_and_clamped():
    handler = bare_handler()
    result = handler.validate_view_settings(
        {
            "type": "view_settings",
            "inspect_enabled": True,
            "max_yaw": 999,
            "orbit_distance": 0.01,
            "dolly_enabled": False,
            "robot_overlay_enabled": True,
            "robot_overlay_style": "alignment",
            "arm_measured_feedback_enabled": True,
            "arm_target_ghost_enabled": False,
            "arm_following_error_safety_enabled": True,
            "arm_freeze_overlay_on_stale_enabled": True,
            "arm_d455_visual_correction_enabled": False,
            "white_background_enabled": True,
            "calibrate_robot_position": True,
            "focus_pick_uv": [1.5, -0.25],
            "focus_pick_sent_unix_ms": 123456,
        }
    )
    assert result["max_yaw"] == 180.0
    assert result["orbit_distance"] == 0.1
    assert result["dolly_enabled"] is False
    assert result["robot_overlay_style"] == "alignment"
    assert result["calibrate_robot_position"] is True
    assert result["arm_measured_feedback_enabled"] is True
    assert result["arm_target_ghost_enabled"] is False
    assert result["white_background_enabled"] is True
    assert result["focus_pick_uv"] == [1.0, 0.0]
    assert result["focus_pick_sent_unix_ms"] == 123456
    with pytest.raises(ValueError, match="boolean"):
        handler.validate_view_settings({"inspect_enabled": "yes"})
    with pytest.raises(ValueError, match="alignment or solid"):
        handler.validate_view_settings({"robot_overlay_style": "wireframe"})
    with pytest.raises(ValueError, match="two coordinates"):
        handler.validate_view_settings({"focus_pick_uv": [0.5]})
