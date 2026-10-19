"""Authenticated, loopback-only noVNC assets and input bridge."""
import select
import socket
import ssl
from urllib.parse import urlsplit


def proxy_novnc(handler, port: int) -> bool:
    parts = urlsplit(handler.path)
    path = parts.path.removeprefix("/godot/")
    allowed = path in {"vnc.html", "websockify"} or path.startswith(("core/", "vendor/", "app/"))
    if not parts.path.startswith("/godot/") or not allowed:
        return False
    if not handler.auth_required() or not handler.is_authorized():
        handler.send_error(403, "Desktop access requires an authorized session")
        return True
    websocket = handler.headers.get("Upgrade", "").lower() == "websocket"
    if websocket and not handler.websocket_origin_allowed():
        handler.send_error(403, "WebSocket origin rejected")
        return True
    target = "/" + path + ("?" + parts.query if parts.query else "")
    try:
        upstream = socket.create_connection(("127.0.0.1", port), timeout=5)
    except OSError:
        handler.send_error(503, "Private noVNC bridge unavailable")
        return True
    timeout = handler.connection.gettimeout()
    try:
        lines = [f"GET {target} HTTP/1.1", f"Host: 127.0.0.1:{port}"]
        excluded = {"host", "cookie", "authorization", "proxy-authorization", "connection",
                    "proxy-connection", "content-length", "transfer-encoding"}
        for name, value in handler.headers.items():
            if name.lower() not in excluded:
                lines.append(f"{name}: {value}")
        lines.append("Connection: Upgrade" if websocket else "Connection: close")
        upstream.sendall(("\r\n".join(lines) + "\r\n\r\n").encode("iso-8859-1"))
        handler.close_connection = True
        handler.connection.setblocking(False)
        upstream.setblocking(False)
        peers = {handler.connection: upstream, upstream: handler.connection}
        buffers = {peer: bytearray() for peer in peers}
        closing = False
        while True:
            progressed = False
            if not closing:
                for source, destination in peers.items():
                    if len(buffers[destination]) >= 4 * 1024 * 1024:
                        continue
                    try:
                        data = source.recv(65536)
                    except (BlockingIOError, ssl.SSLWantReadError, ssl.SSLWantWriteError):
                        continue
                    if not data:
                        closing = True
                        break
                    buffers[destination].extend(data)
                    progressed = True
            for destination, pending in buffers.items():
                if not pending:
                    continue
                try:
                    sent = destination.send(pending[:65536])
                except (BlockingIOError, ssl.SSLWantReadError, ssl.SSLWantWriteError):
                    continue
                del pending[:sent]
                progressed = progressed or sent > 0
            if closing and not any(buffers.values()):
                return True
            if not progressed:
                readers = [] if closing else [source for source, destination in peers.items()
                    if len(buffers[destination]) < 4 * 1024 * 1024]
                writers = [destination for destination, pending in buffers.items() if pending]
                select.select(readers, writers, [], 30)
    except (ConnectionError, OSError):
        return True
    finally:
        upstream.close()
        try:
            handler.connection.settimeout(timeout)
        except OSError:
            pass
