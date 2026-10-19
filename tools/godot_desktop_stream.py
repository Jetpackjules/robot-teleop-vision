"""Opt-in H.264 view of an operator-selected private X11 desktop.

The existing gateway authenticates requests before calling this module. Capture
starts only while a viewer is connected, uses the Intel/AMD VAAPI video engine,
and never opens a listener or a robot-control connection of its own.
"""
from __future__ import annotations

from dataclasses import dataclass
import http.client
import json
from pathlib import Path, PurePosixPath
import os
import re
import select
import subprocess
import tempfile
import threading
from tools.godot_desktop_proxy import proxy_novnc


@dataclass(frozen=True)
class DesktopStreamConfig:
    display: str
    xauthority: str
    ffmpeg: str = "ffmpeg"
    size: str = "1440x900"
    fps: int = 30
    device: str = "/dev/dri/renderD128"
    vnc_port: int = 6085

    def __post_init__(self):
        if not re.fullmatch(r":\d+(?:\.\d+)?", self.display):
            raise ValueError("Desktop capture requires an explicit local X11 display")
        match = re.fullmatch(r"(\d+)x(\d+)", self.size)
        if not match:
            raise ValueError("Desktop size must be WIDTHxHEIGHT")
        width, height = map(int, match.groups())
        if not (16 <= width <= 3840 and 16 <= height <= 2160) or width % 2 or height % 2:
            raise ValueError("Desktop dimensions must be even and at most 3840x2160")
        if not 1 <= self.fps <= 60:
            raise ValueError("Desktop FPS must be between 1 and 60")
        if not PurePosixPath(self.xauthority).is_absolute():
            raise ValueError("Use an absolute Xauthority path")
        if not re.fullmatch(r"/dev/dri/renderD\d+", self.device):
            raise ValueError("Use a local VAAPI render device")
        if not 1 <= self.vnc_port <= 65535:
            raise ValueError("Private noVNC port is outside the valid range")

    def command(self):
        return [self.ffmpeg, "-hide_banner", "-loglevel", "error", "-nostdin",
                "-vaapi_device", self.device, "-thread_queue_size", "2",
                "-f", "x11grab", "-draw_mouse", "0", "-framerate", str(self.fps),
                "-video_size", self.size, "-i", self.display, "-an",
                "-vf", "format=nv12,hwupload", "-c:v", "h264_vaapi",
                "-profile:v", "high", "-level:v", "4.1", "-rc_mode", "CQP",
                "-qp", "18", "-g", str(self.fps), "-bf", "0",
                "-movflags", "empty_moov+default_base_moof+frag_every_frame",
                "-flush_packets", "1", "-f", "mp4", "pipe:1"]


class GodotDesktopStream:
    def __init__(self, config: DesktopStreamConfig):
        self.config = config
        self._slots = threading.BoundedSemaphore(2)

    def handle(self, handler, path):
        if path == "/godot/live.html":
            page = Path(__file__).resolve().parents[1] / "web/godot_live.html"
            data = page.read_bytes()
            handler.send_response(200)
            handler.send_header("Content-Type", "text/html; charset=utf-8")
            handler.send_header("Cache-Control", "no-store")
            handler.send_header("Content-Length", str(len(data)))
            handler.end_headers()
            handler.wfile.write(data)
            return True
        if path == "/godot/desktop-status.json":
            status = {"size": self.config.size, "fps": self.config.fps,
                      "codec": "avc1.640029", "encoder": "VAAPI H.264"}
            connection = http.client.HTTPConnection(handler.server.stream_host,handler.server.stream_port,timeout=0.5)
            try:
                connection.request("GET","/diagnostics")
                response = connection.getresponse()
                if response.status == 200:
                    diagnostics = json.loads(response.read(65536))
                    status["scene_fps"] = diagnostics.get("scene_fps")
                    status["cameras"] = diagnostics.get("cameras",[])
            except (OSError,ValueError,http.client.HTTPException):
                pass
            finally:
                connection.close()
            handler.serve_json(status)
            return True
        if path != "/godot/desktop.mp4":
            return proxy_novnc(handler,self.config.vnc_port) if path.startswith("/godot/") else False
        if not self._slots.acquire(blocking=False):
            handler.send_error(503, "Two desktop video viewers are already connected")
            return True
        process = None
        try:
            env = {**os.environ, "DISPLAY": self.config.display, "XAUTHORITY": self.config.xauthority}
            # This is a private file, not an unconsumed stderr pipe that can
            # deadlock the encoder. Only the bounded tail is reported on error.
            with tempfile.TemporaryFile() as errors:
                process = subprocess.Popen(self.config.command(), env=env, stdin=subprocess.DEVNULL,
                                           stdout=subprocess.PIPE, stderr=errors)
                readable, _, _ = select.select([process.stdout], [], [], 8)
                first = process.stdout.read1(65536) if readable else b""
                if not first:
                    errors.seek(0)
                    handler.log_error("Desktop encoder did not start: %s", errors.read(4096).decode(errors="replace"))
                    handler.send_error(503, "Desktop GPU encoder unavailable; use the standard desktop view")
                    return True
                handler.send_response(200)
                handler.send_header("Content-Type", "video/mp4")
                handler.send_header("Cache-Control", "no-store, no-transform")
                handler.send_header("X-Accel-Buffering", "no")
                handler.send_header("Transfer-Encoding", "chunked")
                handler.end_headers()
                data = first
                while data:
                    # Backpressure stays in bounded OS pipes/socket buffers.
                    handler.wfile.write(f"{len(data):x}\r\n".encode()+data+b"\r\n")
                    handler.wfile.flush()
                    data = process.stdout.read1(65536)
                handler.wfile.write(b"0\r\n\r\n")
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionError, OSError) as error:
            handler.close_connection = True
            if process is None:
                handler.send_error(503, "Desktop encoder executable unavailable")
        finally:
            if process is not None:
                process.terminate()
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2)
                if process.stdout is not None:
                    process.stdout.close()
            self._slots.release()
        return True

    def proxy_controls(self, handler):
        return proxy_novnc(handler,self.config.vnc_port)
