import fractions
from itertools import pairwise

import av
import numpy as np
import pytest

from robot_teleop.camera_preview import CameraPreviewEncoder, avc_codec_string


def camera_jpeg(width=320, height=180):
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :width // 3] = [30, 80, 200]
    image[:, width // 3:width * 2 // 3] = [210, 140, 20]
    image[:, width * 2 // 3:] = [250, 250, 250]
    encoder = av.CodecContext.create("mjpeg", "w")
    encoder.width, encoder.height, encoder.pix_fmt = width, height, "yuvj420p"
    encoder.time_base = fractions.Fraction(1, 30)
    frame = av.VideoFrame.from_ndarray(image, format="rgb24").reformat(format="yuvj420p")
    return b"".join(bytes(packet) for packet in encoder.encode(frame))


def test_native_size_full_range_colors_and_immediate_decode():
    jpeg = camera_jpeg()
    reference = av.CodecContext.create("mjpeg", "r").decode(av.Packet(jpeg))[0].to_ndarray(format="rgb24")
    encoder, decoder = CameraPreviewEncoder(), av.CodecContext.create("h264", "r")
    timestamps = []
    for index in range(6):
        metadata, data = encoder.encode(jpeg, keyframe=index == 3)
        frames = decoder.decode(av.Packet(data))
        assert len(frames) == 1, "Preview must not buffer frames for reordering"
        assert (frames[0].width, frames[0].height) == (320, 180)
        assert frames[0].color_range == 2, "JPEG's full range must survive H.264 VUI"
        reconstructed = frames[0].to_ndarray(format="rgb24")
        error = np.mean((reference.astype(float) - reconstructed.astype(float)) ** 2)
        assert 10 * np.log10(255 ** 2 / max(error, 1e-9)) > 40
        assert metadata["preview_sequence"] == index + 1
        assert metadata["encoding"] == "h264"
        timestamps.append(metadata["timestamp_us"])
        if index in (0, 3):
            assert metadata["keyframe"]
            assert metadata["codec"] == avc_codec_string(data)
    assert all(b > a for a, b in pairwise(timestamps))


def test_resolution_change_starts_independently_decodable_stream():
    encoder = CameraPreviewEncoder()
    encoder.encode(camera_jpeg())
    metadata, packet = encoder.encode(camera_jpeg(640, 360))
    assert metadata["keyframe"]
    decoded = av.CodecContext.create("h264", "r").decode(av.Packet(packet))
    assert (decoded[0].width, decoded[0].height) == (640, 360)


def test_codec_string_requires_sps():
    with pytest.raises(ValueError, match="SPS"):
        avc_codec_string(b"not an AVC stream")


def test_preview_close_releases_source_and_encoder_capacity(monkeypatch):
    import socket
    import threading
    from types import SimpleNamespace

    from test_camera_stream_recovery import remote

    server, client = socket.socketpair()
    created, released = threading.Event(), threading.Event()

    class PausedReader:
        def __init__(self, url):
            assert url == "http://127.0.0.1:8781/camera-rgb-stream?camera=1234&w=3840&q=95"
            created.set()

        def read_part(self):
            released.wait(3)
            raise OSError("closed")

        def close(self):
            released.set()

    monkeypatch.setattr(remote, "MultipartRawFrameReader", PausedReader)
    slots = threading.BoundedSemaphore(1)
    handler = object.__new__(remote.LanRemoteHandler)
    handler.connection = server
    handler.path = "/camera-preview-stream?camera=1234&w=3840&url=https://untrusted.example"
    handler.server = SimpleNamespace(ffmpeg_slots=slots, stream_host="127.0.0.1", capture_port=8781)
    handler.accept_websocket = lambda: True
    worker = threading.Thread(target=handler.handle_camera_preview_websocket)
    try:
        worker.start()
        assert created.wait(1)
        client.sendall(bytes.fromhex("888000000000"))
        worker.join(2)
        assert not worker.is_alive()
        assert released.is_set()
        assert slots.acquire(blocking=False)
    finally:
        released.set()
        client.close()
        server.close()
        worker.join(3)
