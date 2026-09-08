#!/usr/bin/env python3
"""Serve a local, input-only controller for the isolated Godot alignment demo.

Standard library only. This process never imports the production operator,
follower, serial or camera services. Its sole output is validated simulation
JSON on a fixed loopback UDP endpoint.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import re
import socket
import struct
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HTTP_PORT = 14860
DEFAULT_UDP_PORT = 14861
MAX_FRAME = 4096
GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
STATIC_FILES = {
    "/": (ROOT / "web/simulation_controller.html", "text/html; charset=utf-8"),
    "/simulation_controller.js": (ROOT / "web/simulation_controller.js", "text/javascript"),
    "/godot_webcam_tracker_bridge.js": (ROOT / "web/godot_webcam_tracker_bridge.js", "text/javascript"),
}


class InvalidSimulationInput(ValueError):
    """A value does not satisfy the deliberately narrow simulation protocol."""


def _number(value: object, lower: float, upper: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise InvalidSimulationInput("Expected a finite number")
    if not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError("Number outside the simulation input range")
    return float(value)


def validate_packet(payload: object, *, now_ms: int | None = None) -> dict:
    """Build a fresh, narrow envelope; never forward arbitrary browser JSON."""
    if not isinstance(payload, dict):
        raise InvalidSimulationInput("Expected an object")
    if (payload.get("type") != "alignment_demo"
            or type(payload.get("version")) is not int or payload["version"] != 1):
        raise ValueError("Only alignment_demo version 1 is accepted")
    seq = payload.get("seq")
    if isinstance(seq, bool) or not isinstance(seq, int) or not 0 <= seq < 2**53:
        raise ValueError("Expected a nonnegative sequence number")
    now = int(time.time() * 1000) if now_ms is None else now_ms
    timestamp = _number(payload.get("timestamp_ms"), 0, 2**53 - 1)
    if not -250 <= now - timestamp <= 1000:
        raise ValueError("Input is stale or from a future clock")
    source = payload.get("source")
    if not isinstance(source, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", source):
        raise ValueError("Invalid source")
    head, arm = payload.get("head"), payload.get("arm")
    if not isinstance(head, dict) or not isinstance(arm, dict):
        raise InvalidSimulationInput("Expected head and arm input objects")
    if type(head.get("active")) is not bool or type(arm.get("active")) is not bool:
        raise ValueError("Input active flags must be booleans")
    if head.get("units") != "cm":
        raise ValueError("Head coordinates must be in cm")
    values = arm.get("normalized")
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError("Expected six normalized joint values")
    normalized = [_number(value, -360, 360) for value in values[:5]]
    normalized.append(_number(values[5], 0, 100))
    arm_source = arm.get("source", "manual")
    if arm_source not in ("manual", "leader"):
        raise ValueError("Unknown arm input source")
    result = {
        "type": "alignment_demo", "version": 1, "seq": seq,
        "timestamp_ms": int(timestamp), "source": source,
        "head": {
            "active": head["active"], "units": "cm",
            "x": _number(head.get("x"), -100, 100),
            "y": _number(head.get("y"), -100, 100),
            "z": _number(head.get("z"), 0, 500),
        },
        "arm": {"active": arm["active"], "source": arm_source, "normalized": normalized},
    }
    if "action" in payload:
        if payload["action"] not in ("reset", "recenter", "replay"):
            raise ValueError("Unknown simulation action")
        result["action"] = payload["action"]
    return result


def receive_frame(stream) -> tuple[int, bytes]:
    """Read one small masked RFC 6455 frame; reject fragmentation/extensions."""
    def exact(size: int) -> bytes:
        value = stream.read(size)
        if len(value) != size:
            raise EOFError
        return value

    first, second = exact(2)
    opcode = first & 15
    if first & 0x70 or not first & 0x80 or not second & 0x80:
        raise ValueError("Masked, complete WebSocket frames are required")
    if opcode not in (1, 8, 9, 10):
        raise ValueError("Only text and control WebSocket frames are accepted")
    length = second & 127
    if length == 126:
        length = struct.unpack("!H", exact(2))[0]
    elif length == 127:
        length = struct.unpack("!Q", exact(8))[0]
    if length > MAX_FRAME or (opcode >= 8 and length > 125):
        raise ValueError("WebSocket frame is too large")
    mask = exact(4)
    data = exact(length)
    return opcode, bytes(value ^ mask[index % 4] for index, value in enumerate(data))


def frame_bytes(opcode: int, payload: bytes) -> bytes:
    size = len(payload)
    prefix = bytes([0x80 | opcode, size]) if size < 126 else bytes([0x80 | opcode, 126]) + struct.pack("!H", size)
    return prefix + payload


class DemoServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = False

    def __init__(self, http_port: int = DEFAULT_HTTP_PORT, udp_port: int = DEFAULT_UDP_PORT):
        if not 1 <= udp_port <= 65535 or udp_port in {4248, 4249, 4250, 4251, 4252}:
            raise ValueError("Choose a separate simulation UDP port")
        self.udp_target = ("127.0.0.1", udp_port)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.controller_lock = threading.Lock()
        super().__init__(("127.0.0.1", http_port), DemoHandler)

    def server_bind(self) -> None:
        # Windows SO_REUSEADDR can permit two unrelated listeners on one port.
        # A second demo launch must fail instead of splitting browser requests.
        if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()

    def send_simulation(self, packet: dict) -> None:
        self.udp.sendto(json.dumps(packet, allow_nan=False, separators=(",", ":")).encode(), self.udp_target)

    def server_close(self) -> None:
        super().server_close()
        self.udp.close()


class DemoHandler(BaseHTTPRequestHandler):
    server: DemoServer
    protocol_version = "HTTP/1.1"

    def log_message(self, _format, *_args):
        pass

    def _response(self, status: int, content: bytes, content_type: str = "text/plain") -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Cross-Origin-Resource-Policy", "same-origin")
        self.send_header("Permissions-Policy", "camera=(self), microphone=(), serial=(self)")
        self.end_headers()
        self.wfile.write(content)

    def do_GET(self):
        port = self.server.server_port
        allowed_hosts = {f"127.0.0.1:{port}", f"localhost:{port}"}
        if self.headers.get("Host") not in allowed_hosts:
            self._response(403, b"Use the printed localhost URL")
            return
        path = urlsplit(self.path).path
        if path == "/input":
            self._websocket()
        elif path == "/config":
            self._response(200, json.dumps({
                "service": "robot-teleop-alignment-demo", "protocol": "alignment_demo",
                "version": 1, "udp_port": self.server.udp_target[1],
            }).encode(), "application/json")
        elif path in STATIC_FILES:
            file_path, content_type = STATIC_FILES[path]
            try:
                content = file_path.read_bytes()
            except OSError:
                self._response(500, b"Controller asset missing")
                return
            self._response(200, content, content_type)
        else:
            self._response(404, b"No route")

    def _websocket(self):
        origin = self.headers.get("Origin", "")
        expected = f"http://{self.headers.get('Host')}"
        if origin != expected:
            self._response(403, b"Same-origin input only")
            return
        try:
            key = self.headers.get("Sec-WebSocket-Key", "")
            decoded_key = base64.b64decode(key, validate=True)
        except ValueError:
            decoded_key = b""
        if (self.headers.get("Upgrade", "").lower() != "websocket"
                or "upgrade" not in self.headers.get("Connection", "").lower()
                or self.headers.get("Sec-WebSocket-Version") != "13" or len(decoded_key) != 16):
            self._response(400, b"Invalid WebSocket handshake")
            return
        if not self.server.controller_lock.acquire(blocking=False):
            self._response(409, b"A controller is already connected; close its tab first")
            return
        last_packet = None
        last_seq = -1
        source = "browser-" + uuid.uuid4().hex
        try:
            self.send_response(101)
            self.send_header("Upgrade", "websocket")
            self.send_header("Connection", "Upgrade")
            accept = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
            self.send_header("Sec-WebSocket-Accept", accept)
            self.end_headers()
            self.connection.settimeout(3)
            while True:
                opcode, data = receive_frame(self.rfile)
                if opcode == 8:
                    self.wfile.write(frame_bytes(8, b""))
                    break
                if opcode == 9:
                    self.wfile.write(frame_bytes(10, data))
                    continue
                if opcode == 10:
                    continue
                try:
                    packet = validate_packet(json.loads(data.decode("utf-8")))
                    if packet["seq"] <= last_seq:
                        raise ValueError("Sequence must advance")
                    packet["source"] = source
                    self.server.send_simulation(packet)
                    last_packet, last_seq = packet, packet["seq"]
                except (ValueError, UnicodeError):
                    self.wfile.write(frame_bytes(1, b'{"error":"Invalid or stale simulation input"}'))
        except (EOFError, OSError, ValueError):
            pass
        finally:
            if last_packet:
                last_packet.pop("action", None)
                last_packet["seq"] += 1
                last_packet["timestamp_ms"] = int(time.time() * 1000)
                last_packet["head"]["active"] = False
                last_packet["arm"]["active"] = False
                try:
                    self.server.send_simulation(last_packet)
                except OSError:
                    pass
            self.server.controller_lock.release()
            self.close_connection = True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=DEFAULT_HTTP_PORT)
    parser.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    args = parser.parse_args()
    if not 1024 <= args.port <= 65535:
        parser.error("--port must be from 1024 to 65535")
    try:
        server = DemoServer(args.port, args.udp_port)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    print(f"Simulation controller: http://127.0.0.1:{server.server_port}/", flush=True)
    print(f"Simulation input only: UDP 127.0.0.1:{args.udp_port}. No follower service is started.", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
