"""Slow viewers must stop WAN queue growth without changing RGB-D pixels."""
import json
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import lan_remote_view_server as remote


def test_independent_rgbd_waits_for_browser_and_resumes_at_latest_capture(monkeypatch):
    full = threading.Event()
    closed = threading.Event()
    hooks = {}
    captured = [0]

    class Reader:
        def __init__(self, _url):
            self.sequence = 0

        def read_part(self):
            if closed.wait(0.003):
                raise OSError("closed")
            self.sequence += 1
            captured[0] = self.sequence
            return {}, f"unchanged-native-packet {self.sequence}".encode()

        def close(self):
            closed.set()

    monkeypatch.setattr(remote, "MultipartRawFrameReader", Reader)
    monkeypatch.setattr(remote, "crop_rgbd_for_client", lambda _server, packet, _client: packet)
    handler = object.__new__(remote.LanRemoteHandler)
    handler.path = "/rgbd-stream?client=flow&fps=30&flow_control=1&temporal_depth=0&temporal_color=0"
    handler.client_address = ("127.0.0.1", 1)
    handler.server = SimpleNamespace(stream_host="127.0.0.1", capture_port=8781,
        claim_rgbd_client=lambda *_: True, release_rgbd_client=lambda *_: None)
    handler.connection = SimpleNamespace(settimeout=lambda _: None, setsockopt=lambda *_: None)
    handler.accept_websocket = lambda: True

    def watcher(*_args, on_text=None):
        hooks["acknowledge"] = on_text

    handler.watch_stream_peer = watcher
    messages, packets = [], []

    def send(data, opcode):
        if opcode == 1:
            messages.append(json.loads(data))
        else:
            packets.append(data)
            if len(packets) == 4:
                full.set()
            if len(packets) == 5:
                raise BrokenPipeError("test finished")

    handler.send_ws_frame = send
    worker = threading.Thread(target=handler.handle_rgbd_websocket)
    try:
        worker.start()
        assert full.wait(2)
        time.sleep(0.06)
        assert len(packets) == 4, "A slow browser cannot accumulate an unbounded tunnel queue"
        hooks["acknowledge"](b'{"type":"rgbd-ack","sequence":9999}')
        time.sleep(0.02)
        assert len(packets) == 4, "An acknowledgement cannot grant credit for unsent frames"
        latest_before_acknowledgement = captured[0]
        hooks["acknowledge"](b'{"type":"rgbd-ack","sequence":4}')
        worker.join(2)
        assert not worker.is_alive()
        assert len(packets) == 5
        assert int(packets[4].split()[-1]) >= latest_before_acknowledgement
        assert all(p.startswith(b"unchanged-native-packet ") for p in packets)
        assert [m["sequence"] for m in messages] == [1, 2, 3, 4, 5]
    finally:
        closed.set()
        worker.join(3)
