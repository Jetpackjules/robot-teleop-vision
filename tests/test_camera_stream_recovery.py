"""A closed browser must release camera capacity even with capture paused."""
import json
import socket
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
import lan_remote_view_server as remote


def test_rgb_close_releases_reader_while_camera_has_no_frames(monkeypatch):
    server, client = socket.socketpair()
    created, released = threading.Event(), threading.Event()
    class PausedReader:
        def __init__(self, url):
            assert url.startswith('http://127.0.0.1:8781/camera-rgb-stream?')
            created.set()
        def read_part(self):
            released.wait(3)
            raise OSError('reader closed')
        def close(self): released.set()
    monkeypatch.setattr(remote, 'MultipartRawFrameReader', PausedReader)
    slots = threading.BoundedSemaphore(1)
    handler = object.__new__(remote.LanRemoteHandler)
    handler.path = '/camera-rgb-stream?camera=1234&w=3840&q=95'
    handler.connection = server
    handler.server = SimpleNamespace(rgbd_http_slots=slots, stream_host='127.0.0.1', capture_port=8781)
    handler.accept_websocket = lambda: True
    worker = threading.Thread(target=handler.handle_camera_rgb_websocket)
    try:
        worker.start()
        assert created.wait(1)
        # Masked zero-length Close frame, as sent by WebSocket.close().
        client.sendall(bytes.fromhex('888000000000'))
        worker.join(2)
        assert not worker.is_alive(), 'mode change must not wait for another camera frame'
        assert released.is_set(), 'native multipart reader must be closed'
        assert slots.acquire(blocking=False), 'next mode must be able to acquire stream capacity'
    finally:
        released.set();client.close();server.close();worker.join(3)


def test_stream_watcher_answers_ping_and_stops_on_disconnect():
    server, client = socket.socketpair()
    stop = threading.Event()
    handler = object.__new__(remote.LanRemoteHandler);handler.connection=server
    worker = handler.watch_stream_peer(stop, [threading.Condition()], threading.Lock())
    try:
        client.settimeout(1)
        client.sendall(bytes.fromhex('898100000000') + b'x')
        assert client.recv(3) == b'\x8a\x01x'
        client.shutdown(socket.SHUT_RDWR)
        assert stop.wait(1), 'TCP disconnect must wake a quiet camera stream'
    finally:
        stop.set();worker.join(1);client.close();server.close()


def test_multipart_close_unblocks_a_real_reader_without_waiting_for_timeout():
    server, client = socket.socketpair()
    server.settimeout(5)
    reader = object.__new__(remote.MultipartRawFrameReader)
    reader.sock = server;reader.buffer=b'';reader.boundary=b'--godotframe'
    stopped=threading.Event()
    def read():
        try: reader.read_part()
        except OSError: pass
        finally: stopped.set()
    worker=threading.Thread(target=read)
    try:
        worker.start();time.sleep(.03)
        reader.close()
        assert stopped.wait(.5), 'closing a quiet stream must interrupt recv immediately'
        client.settimeout(.5)
        assert client.recv(1)==b'', 'native peer must see EOF and release its slot'
    finally:
        reader.close();client.close();worker.join(6)


def test_public_image_levels_exclude_private_or_nonfinite_values(monkeypatch):
    class Connection:
        def __init__(self, *_args, **_kwargs): pass
        def request(self, *_args): pass
        def close(self): pass
        def getresponse(self):
            return SimpleNamespace(status=200, read=lambda _: json.dumps({'cameras': [{
                'id':'1234','name':'D435 A','model':'D435', 'private_field':'discard',
                'image_levels': {'mean_luma':25.0,'dark_percent':40.0,'auto_white_balance':1,
                    'white_balance_state':'ready','gain':float('nan'),'private_field':'discard'}
            }]}).encode())
    monkeypatch.setattr(remote.http.client, 'HTTPConnection', Connection)
    handler=object.__new__(remote.LanRemoteHandler)
    handler.server=SimpleNamespace(stream_host='127.0.0.1',capture_port=8781)
    results=[];handler.serve_json=results.append
    handler.serve_camera_list()
    assert results == [{'cameras': [{'id':'1234','name':'D435 A','model':'D435',
        'image_levels':{'mean_luma':25.0,'dark_percent':40.0,'auto_white_balance':1,'white_balance_state':'ready'}}]}]
