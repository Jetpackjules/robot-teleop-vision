from __future__ import annotations

import json
import socket
import ssl
import sys
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from lan_remote_view_server import LanRemoteHandler, LanRemoteServer, ensure_cert

from robot_teleop.operator import load_robot_operator


def bare_handler(headers=None, server=None):
    handler = object.__new__(LanRemoteHandler)
    handler.headers = headers or {}
    handler.server = server or SimpleNamespace(robot_operator=load_robot_operator("so101"))
    return handler


def test_tls_certificate_generation_does_not_require_openssl(tmp_path):
    cert = tmp_path / "tls" / "lan_remote.crt"
    key = tmp_path / "tls" / "lan_remote.key"

    ensure_cert(cert, key, "127.0.0.1")

    assert cert.is_file()
    assert key.is_file()
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certfile=cert, keyfile=key)


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


def test_desktop_video_is_behind_existing_login_gate():
    handler = bare_handler(server=SimpleNamespace(godot_desktop_stream=SimpleNamespace(
        handle=lambda *args: pytest.fail("unauthenticated desktop capture"))))
    handler.path = "/godot/desktop.mp4"
    handler.auth_required = lambda: True
    handler.is_authorized = lambda: False
    redirected = []
    handler.redirect_to_login = lambda: redirected.append(True)
    handler.do_GET()
    assert redirected == [True]


@pytest.mark.parametrize("path", ["/camera-rgb?w=3840&q=95", "/camera-rgb-stream?w=3840&q=95", "/api/v1/cameras"])
def test_original_camera_rgb_is_behind_existing_login_gate(path):
    handler = bare_handler()
    handler.path = path
    handler.auth_required = lambda: True
    handler.is_authorized = lambda: False
    handler.serve_camera_rgb = lambda _: pytest.fail("unauthorized camera access")
    redirected = []
    handler.redirect_to_login = lambda: redirected.append(True)
    handler.do_GET()
    assert redirected == [True]


def test_camera_list_uses_only_trusted_capture_endpoint(monkeypatch):
    import lan_remote_view_server as remote
    calls, results = [], []
    class Connection:
        def __init__(self, host, port, timeout):
            assert (host,port,timeout)==('127.0.0.1',8781,5)
        def request(self, method, path): calls.append((method,path))
        def getresponse(self):
            return SimpleNamespace(status=200, read=lambda maximum:json.dumps({'cameras':[
                {'id':'1234','name':'D435 A','model':'D435','private_field':'discard'}]}).encode())
        def close(self): pass
    monkeypatch.setattr(remote.http.client,'HTTPConnection',Connection)
    handler=bare_handler(server=SimpleNamespace(stream_host='127.0.0.1',capture_port=8781))
    handler.serve_json=results.append
    handler.serve_camera_list()
    assert calls==[('GET','/cameras')]
    assert results==[{'cameras':[{'id':'1234','name':'D435 A','model':'D435'}]}]


def test_original_rgb_proxy_uses_fixed_endpoint_and_bounded_options(monkeypatch):
    from io import BytesIO
    from threading import BoundedSemaphore
    from urllib.parse import parse_qs, urlsplit
    import lan_remote_view_server as remote

    requests = []
    class Response:
        status = 200
        def read(self, maximum): return b"jpeg-original"
        def getheader(self, name, default=None):
            return {"Content-Type": "image/jpeg", "X-Frame-Width": "1280",
                    "X-Frame-Height": "720", "X-Camera-Id": "1234"}.get(name, default)
    class Connection:
        def __init__(self, host, port, timeout):
            assert (host, port, timeout) == ("127.0.0.1", 8781, 5)
        def request(self, method, path): requests.append((method, path))
        def getresponse(self): return Response()
        def close(self): pass
    monkeypatch.setattr(remote.http.client, "HTTPConnection", Connection)
    handler = bare_handler(server=SimpleNamespace(stream_host="127.0.0.1", stream_port=8780, capture_port=8781,
        rgbd_http_slots=BoundedSemaphore(1)))
    headers = {}
    errors = []
    handler.wfile = BytesIO()
    handler.send_response = lambda code: headers.update(status=code)
    handler.send_header = lambda name, value: headers.update({name: value})
    handler.end_headers = lambda: None
    handler.send_error = lambda code, _: errors.append(code)
    handler.serve_camera_rgb("w=99999&q=999&camera=1234&url=https://untrusted.example")
    assert requests[0][0] == "GET"
    path = urlsplit(requests[0][1])
    assert path.path == "/camera-rgb"
    assert parse_qs(path.query) == {"w": ["3840"], "q": ["100"], "camera": ["1234"]}
    assert headers["X-Frame-Width"] == "1280"
    assert handler.wfile.getvalue() == b"jpeg-original"
    handler.serve_camera_rgb("camera=bad%0d%0aHeader")
    assert errors == [400] and len(requests) == 1
    assert handler.server.rgbd_http_slots.acquire(blocking=False)


def test_original_rgb_websocket_relays_latest_frame_from_capture_port(monkeypatch):
    import threading
    import struct
    import lan_remote_view_server as remote
    stopped = threading.Event()
    urls = []
    class Reader:
        def __init__(self, url):
            urls.append(url)
            self.first = True
        def read_part(self):
            if self.first:
                self.first = False
                return {"x-camera-id":"1234", "x-frame-sequence":"17", "x-capture-epoch":"owner"}, b"jpeg-original"
            stopped.wait(2)
            raise OSError("closed")
        def close(self): stopped.set()
    monkeypatch.setattr(remote,"MultipartRawFrameReader", Reader)
    handler = bare_handler(server=SimpleNamespace(stream_host="127.0.0.1",capture_port=8781,
        rgbd_http_slots=threading.BoundedSemaphore(1)))
    handler.path="/camera-rgb-stream?w=99999&q=101&camera=1234&url=https://untrusted.example"
    handler.accept_websocket=lambda: True
    handler.connection=SimpleNamespace(settimeout=lambda _:None)
    # This test isolates packet forwarding; real peer close handling is
    # exercised with sockets in test_camera_stream_recovery.py.
    handler.watch_stream_peer=lambda *_args:None
    packets=[]
    def send(payload,opcode):
        packets.append((payload,opcode))
        raise BrokenPipeError("viewer closed")
    handler.send_ws_frame=send
    handler.handle_camera_rgb_websocket()
    assert urls==["http://127.0.0.1:8781/camera-rgb-stream?camera=1234&w=3840&q=100"]
    packet,opcode=packets[0]
    assert opcode==2
    length=struct.unpack_from("<I",packet)[0]
    assert json.loads(packet[4:4+length])["x-capture-epoch"]=="owner"
    assert packet[4+length:]==b"jpeg-original"
    assert stopped.is_set() and handler.server.rgbd_http_slots.acquire(blocking=False)


@pytest.mark.parametrize("setting", ["camera_locations_enabled", "depth_only_enabled", "workspace_crop_enabled", "workspace_crop_show_box", "modeled_workspace_enabled"])
def test_camera_display_setting_is_typed_and_can_be_saved(setting):
    handler = bare_handler()
    assert handler.validate_view_settings({setting: True})[setting] is True
    assert handler.validate_site_settings({setting: False})[setting] is False
    with pytest.raises(ValueError, match="boolean"):
        handler.validate_view_settings({setting: "yes"})


def test_workspace_crop_bounds_are_validated_and_persisted():
    handler = bare_handler()
    bounds = {"workspace_crop_enabled": True, "workspace_crop_min": [-.5, -.2, -1.4], "workspace_crop_max": [.5, .8, -.4]}
    saved = handler.validate_site_settings(bounds)
    assert all(saved[k] == v for k, v in bounds.items())
    for bad in ([0, 1], [False, 0, 1], ["0", 0, 1], [float("nan"), 0, 1], [21, 0, 1]):
        with pytest.raises(ValueError, match="three finite"):
            handler.validate_view_settings({"workspace_crop_min": bad})
    with pytest.raises(ValueError, match="below maximum"):
        handler.validate_view_settings({**bounds, "workspace_crop_max": [-.6, .8, -.4]})


def test_direct_workspace_transforms_are_validated_and_persisted():
    handler = bare_handler()
    settings = {"workspace_crop_rotation": [18, -129, 12], "modeled_workspace_table_height": 0.82}
    assert all(handler.validate_site_settings(settings)[key] == value for key, value in settings.items())
    for value in ([0, 0], [False, 0, 0], [0, float("inf"), 0], [0, 361, 0], ["0", 0, 0]):
        with pytest.raises(ValueError, match="finite angles"):
            handler.validate_view_settings({"workspace_crop_rotation": value})


def test_modeled_workspace_settings_are_typed_and_persisted():
    handler = bare_handler()
    settings = {"modeled_workspace_enabled": True, "modeled_workspace_center": [.12, -.3, -1.2],
                "modeled_workspace_width": .9, "modeled_workspace_depth": .6,
                "modeled_workspace_heading": 35, "modeled_workspace_clearance": .006,
                "modeled_workspace_height": .8, "modeled_workspace_light_angle": -40}
    saved = handler.validate_site_settings(settings)
    assert all(saved[k] == v for k, v in settings.items())
    for bad in ([0, 1], [False, 0, 1], ["0", 0, 1], [float("nan"), 0, 1], [21, 0, 1]):
        with pytest.raises(ValueError, match="three finite"):
            handler.validate_view_settings({"modeled_workspace_center": bad})
    for key in ("width", "depth", "heading", "clearance", "height", "light_angle"):
        with pytest.raises(ValueError, match="finite"):
            handler.validate_view_settings({f"modeled_workspace_{key}": float("inf")})
    bounded = handler.validate_view_settings({"modeled_workspace_clearance": -1, "modeled_workspace_width": 100})
    assert bounded["modeled_workspace_clearance"] == 0
    assert bounded["modeled_workspace_width"] == 4


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
            "head_tracking_enabled": False,
            "workspace_surface_enabled": True,
            "workspace_boundary_enabled": False,
            "workspace_surface_offset": -99,
            "workspace_surface_size": 99,
            "workspace_surface_opacity": 0.001,
            "display_mode": "rgb_camera",
            "stream_preset": "latency",
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
    assert result["head_tracking_enabled"] is False
    assert result["workspace_surface_enabled"] is True
    assert result["workspace_boundary_enabled"] is False
    assert result["workspace_surface_offset"] == -1.0
    assert result["workspace_surface_size"] == 10.0
    assert result["workspace_surface_opacity"] == 0.01
    assert result["display_mode"] == "rgb_camera"
    assert result["stream_preset"] == "latency"
    assert result["focus_pick_uv"] == [1.0, 0.0]
    assert result["focus_pick_sent_unix_ms"] == 123456
    with pytest.raises(ValueError, match="boolean"):
        handler.validate_view_settings({"inspect_enabled": "yes"})
    with pytest.raises(ValueError, match="alignment or solid"):
        handler.validate_view_settings({"robot_overlay_style": "wireframe"})
    with pytest.raises(ValueError, match="quality or latency"):
        handler.validate_view_settings({"stream_preset": "automatic"})
    with pytest.raises(ValueError, match="two coordinates"):
        handler.validate_view_settings({"focus_pick_uv": [0.5]})


def test_site_settings_persist_only_stable_setup_and_clamp_navigation():
    handler = bare_handler()
    result = handler.validate_site_settings(
        {
            "type": "view_settings",
            "settings_version": 10,
            "display_mode": "point_cloud",
            "stream_preset": "quality",
            "head_tracking_enabled": False,
            "workspace_surface_offset": 0.025,
            "arm_idle_return_enabled": True,
            "calibrate_robot_position": True,
            "manual_claw_calibration_save": True,
            "robot_visual_calibration_action": {"operation": "import", "request_id": "test", "file": {"type": "so101_visual_calibration"}},
            "default_navigation": {
                "orbit_yaw": 900,
                "orbit_pitch": -900,
                "dolly": 99,
                "pan": [99, -99, 1.5],
            },
        }
    )
    assert result["settings_version"] == 10
    assert result["display_mode"] == "point_cloud"
    assert result["stream_preset"] == "quality"
    assert result["head_tracking_enabled"] is False
    assert result["workspace_surface_offset"] == 0.025
    assert result["arm_idle_return_enabled"] is True
    assert "calibrate_robot_position" not in result
    assert "manual_claw_calibration_save" not in result
    assert "robot_visual_calibration_action" not in result
    assert result["default_navigation"] == {
        "orbit_yaw": 180.0,
        "orbit_pitch": -80.0,
        "dolly": 8.0,
        "pan": [20.0, -20.0, 1.5],
    }
    unified = handler.validate_site_settings({"settings_version": 13, "display_mode": "unified_mesh"})
    assert unified["display_mode"] == "unified_mesh"
    assert handler.validate_view_settings(unified)["display_mode"] == "unified_mesh"
    blended = handler.validate_site_settings({"settings_version": 14, "display_mode": "blendpcr"})
    assert blended["display_mode"] == "blendpcr"
    assert handler.validate_view_settings(blended)["display_mode"] == "blendpcr"
    with pytest.raises(ValueError, match="settings_version"):
        handler.validate_site_settings({"settings_version": "ten"})


def test_site_settings_file_round_trip_uses_explicit_site_path(tmp_path):
    settings_path = tmp_path / "site" / "operator.json"
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
        site_settings_path=settings_path,
    )
    try:
        assert server.load_site_settings() == {}
        server.save_site_settings({"settings_version": 10, "fov": 86.0})
        assert server.load_site_settings() == {
            "settings_version": 10,
            "fov": 86.0,
        }
        assert settings_path.is_file()
    finally:
        server.server_close()


def test_controller_keeps_daily_controls_separate_from_advanced_setup():
    root = Path(__file__).resolve().parents[1]
    page = (root / "web" / "controller.html").read_text(encoding="utf-8")
    assert 'id="display-mode"' in page
    assert '<option value="rgb_camera">RGB camera</option>' in page
    assert 'id="open-advanced"' in page
    assert 'id="advanced-setup"' in page
    assert 'id="robot-settings-root"' in page
    assert 'id="robot-setup-root"' in page
    assert page.index('id="robot-settings-root"') < page.index('id="advanced-setup"')
    assert page.index('id="robot-setup-root"') > page.index('id="advanced-setup"')
    assert 'id="stream-preset"' in page
    assert '>Reconnect Video</button>' in page
    assert page.count('id="recenter"') == 1
    assert 'id="inspect-recenter"' not in page
    assert 'id="inspect-enabled"' not in page
    assert 'id="head-tracking-options"' in page
    assert 'id="persistent-temporal-reference-enabled"' not in page
