"""Bounded persistent-reference pipelining across a slow network send."""
import shutil
import subprocess
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"tools"))
import lan_remote_view_server as remote


def test_browser_transport_recovers_without_resurrecting_stopped_pages():
    node = shutil.which("node")
    if node is None:
        import pytest
        pytest.skip("Node.js is required for browser transport checks")
    result = subprocess.run([node, str(Path(__file__).with_name("rgbd_transport_recovery.cjs"))],
                            capture_output=True, text=True, timeout=15, check=False)
    assert result.returncode == 0, result.stdout + result.stderr

def test_persistent_reference_encoder_overlaps_send_and_keeps_order(monkeypatch):
    sending=threading.Event()
    second_encoded=threading.Event()
    closed=threading.Event()
    class Reader:
        def __init__(self,_): self.count=0
        def read_part(self):
            self.count+=1
            if self.count==1: return {},b"first"
            if self.count==2:
                assert sending.wait(2)
                return {},b"second"
            closed.wait(2)
            raise OSError("closed")
        def close(self): closed.set()
    class Encoder:
        def __init__(self,**_): pass
        def encode(self,packet):
            if packet==b"second": second_encoded.set()
            return b"encoded-"+packet,True
        def close(self): pass
    monkeypatch.setattr(remote,"MultipartRawFrameReader",Reader)
    monkeypatch.setattr(remote,"TemporalRgbdDepthEncoder",Encoder)
    handler=object.__new__(remote.LanRemoteHandler)
    handler.path="/rgbd-stream?client=test&temporal_depth=1&persistent_reference=1"
    handler.client_address=("127.0.0.1",1)
    handler.server=SimpleNamespace(stream_host="127.0.0.1",capture_port=8781,
        claim_rgbd_client=lambda *_:True,release_rgbd_client=lambda *_:None)
    handler.connection=SimpleNamespace(settimeout=lambda _:None,setsockopt=lambda *_:None)
    handler.accept_websocket=lambda:True
    # This transport stub exercises encoding/sending, not the separately tested socket reader.
    handler.watch_stream_peer=lambda *_:None
    sent=[]
    def send(packet,opcode):
        assert opcode==2
        sent.append(packet)
        if len(sent)==1:
            sending.set()
            assert second_encoded.wait(2),"next reference must encode while the previous send is blocked"
        else:
            raise BrokenPipeError("test viewer closed")
    handler.send_ws_frame=send
    handler.handle_rgbd_websocket()
    assert sent==[b"encoded-first",b"encoded-second"]
    assert closed.is_set()
