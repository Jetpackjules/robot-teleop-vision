"""Low-delay camera preview encoding. Capture and recorded RGB stay untouched."""
from __future__ import annotations

import fractions
import time

import av


def avc_codec_string(data: bytes) -> str:
    """Read profile/compatibility/level from an Annex B SPS for WebCodecs."""
    import re

    for nal in re.split(b"\x00\x00\x00?\x01", data):
        if len(nal) >= 4 and nal[0] & 31 == 7:
            return "avc1." + nal[1:4].hex()
    raise ValueError("Camera preview keyframe has no H.264 SPS")


class CameraPreviewEncoder:
    """Encode current RGB at native resolution, without lookahead or B frames.

    The caller drops stale *source images* before encoding. Encoded delta frames
    must remain ordered; dropping those would break their reference chain.
    """

    def __init__(self) -> None:
        self.jpeg = av.CodecContext.create("mjpeg", "r")
        self.jpeg.thread_count = 1
        self.encoder = None
        self.dimensions = None
        self.sequence = 0
        self.last_pts = -1
        self.started = time.monotonic()
        self.codec = ""

    def encode(self, jpeg: bytes, *, keyframe: bool = False) -> tuple[dict, bytes]:
        started = time.perf_counter()
        frames = self.jpeg.decode(av.Packet(jpeg))
        if not frames:
            raise ValueError("Original RGB JPEG did not decode")
        frame = frames[0]
        dimensions = frame.width, frame.height
        if any(value % 2 for value in dimensions):
            raise ValueError("H.264 preview requires even native image dimensions")
        if dimensions != self.dimensions:
            self.encoder = av.CodecContext.create("libx264", "w")
            self.encoder.width, self.encoder.height = dimensions
            self.encoder.pix_fmt = "yuvj420p"
            # JPEG uses full-range samples. Explicit VUI metadata prevents a
            # decoder interpreting them as limited-range (crushed darks/whites).
            self.encoder.color_range = frame.color_range
            self.encoder.colorspace = frame.colorspace
            self.encoder.time_base = fractions.Fraction(1, 1_000_000)
            self.encoder.framerate = fractions.Fraction(60, 1)
            self.encoder.thread_count = 2
            self.encoder.max_b_frames = 0
            self.encoder.gop_size = 30
            self.encoder.options = {
                "preset": "ultrafast", "tune": "zerolatency", "crf": "10",
                "profile": "baseline",
                "x264-params": "repeat-headers=1:scenecut=0:rc-lookahead=0",
            }
            self.encoder.open()
            self.dimensions = dimensions
            keyframe = True
        # Keep JPEG's actual full-range format as well as its metadata. Older
        # swscale versions narrow yuvj420p -> yuv420p values but retain the flag.
        frame = frame.reformat(format="yuvj420p")
        # Timestamp gaps represent skipped captures, never duplicate old images.
        frame.pts = max(self.last_pts + 1, int((time.monotonic() - self.started) * 1_000_000))
        self.last_pts = frame.pts
        frame.time_base = self.encoder.time_base
        if keyframe:
            frame.pict_type = av.video.frame.PictureType.I
        packets = self.encoder.encode(frame)
        if not packets:
            raise ValueError("Low-delay camera encoder buffered a frame")
        data = b"".join(bytes(packet) for packet in packets)
        is_key = any(packet.is_keyframe for packet in packets)
        if is_key:
            self.codec = avc_codec_string(data)
        self.sequence += 1
        return {
            "encoding": "h264", "codec": self.codec,
            "keyframe": is_key, "timestamp_us": frame.pts,
            "preview_sequence": self.sequence,
            "width": frame.width, "height": frame.height,
            "encode_ms": (time.perf_counter() - started) * 1000,
        }, data
