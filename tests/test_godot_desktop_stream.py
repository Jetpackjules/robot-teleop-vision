from io import BytesIO
from types import SimpleNamespace

import pytest

from tools.godot_desktop_stream import DesktopStreamConfig, GodotDesktopStream
from tools.godot_desktop_proxy import proxy_novnc


def config(**changes):
    return DesktopStreamConfig(**{"display": ":99", "xauthority": "/private/Xauthority", **changes})


@pytest.mark.parametrize("changes", [
    {"display": "remote.example:0"}, {"display": ":99; echo unsafe"},
    {"size": "1441x900"}, {"size": "99999x99999"}, {"fps": 0},
    {"fps": 120}, {"device": "https://example.com"}, {"xauthority": "relative"},
    {"vnc_port":0},
])
def test_capture_options_reject_remote_inputs_and_unbounded_sizes(changes):
    with pytest.raises(ValueError):
        config(**changes)


def test_encoder_preserves_desktop_size_and_uses_hardware_video_engine():
    command = config().command()
    assert command[command.index("-video_size") + 1] == "1440x900"
    assert command[command.index("-c:v") + 1] == "h264_vaapi"
    assert command[command.index("-bf") + 1] == "0"
    assert "scale=" not in " ".join(command)
    assert command[-1] == "pipe:1"


def test_unknown_route_does_not_start_encoder(monkeypatch):
    monkeypatch.setattr("tools.godot_desktop_stream.subprocess.Popen", lambda *a, **k: pytest.fail("encoder started"))
    assert GodotDesktopStream(config()).handle(None, "/controller.html") is False


@pytest.mark.parametrize("authorized,origin_ok",[(False,True),(True,False)])
def test_private_input_bridge_rejects_unauthorized_or_foreign_origin(monkeypatch,authorized,origin_ok):
    monkeypatch.setattr("tools.godot_desktop_proxy.socket.create_connection", lambda *a, **k: pytest.fail("upstream opened"))
    errors=[]
    handler=SimpleNamespace(path='/godot/websockify',headers={'Upgrade':'websocket'},
        auth_required=lambda:True,is_authorized=lambda:authorized,
        websocket_origin_allowed=lambda:origin_ok,send_error=lambda code,message:errors.append(code))
    assert proxy_novnc(handler,6085)
    assert errors==[403]


def test_disconnect_stops_encoder_and_releases_viewer_slot(monkeypatch):
    class Process:
        stdout = BytesIO(b"mp4 header")
        terminated = False
        def terminate(self): self.terminated = True
        def wait(self, timeout): pass
    class Handler:
        close_connection = False
        def send_response(self, code): assert code == 200
        def send_header(self, *args): pass
        def end_headers(self): pass
        def send_error(self, *args): pytest.fail(str(args))
        def write(self, data): raise BrokenPipeError()
        def flush(self): pass
        def log_error(self, *args): pytest.fail(str(args))
    process = Process()
    monkeypatch.setattr("tools.godot_desktop_stream.subprocess.Popen", lambda *a, **k: process)
    monkeypatch.setattr("tools.godot_desktop_stream.select.select", lambda *a: ([process.stdout], [], []))
    handler = Handler()
    handler.wfile = handler
    stream = GodotDesktopStream(config())
    assert stream.handle(handler, "/godot/desktop.mp4")
    assert process.terminated
    assert stream._slots.acquire(blocking=False)
    assert stream._slots.acquire(blocking=False)
    assert not stream._slots.acquire(blocking=False)
