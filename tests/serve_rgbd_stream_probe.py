"""Loopback-only browser probe: python tests/serve_rgbd_stream_probe.py."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]


class ProbeHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/":
            content = (ROOT / "tests/browser_rgbd_stream_probe.html").read_bytes()
            mime = "text/html; charset=utf-8"
        elif path == "/renderer.js":
            content = (ROOT / "web/hybrid_rgbd_renderer.js").read_bytes() + (
                b"\nexport {state,uploadCamera,VERTEX_SHADER,FRAGMENT_SHADER,identityMatrix,perspectiveMatrix,"
                b"inverseRigidTransform,multiplyMatrix};"
            )
            mime = "text/javascript"
        elif path.count("/") == 1 and path.endswith(".js") and (ROOT / "web" / path[1:]).is_file():
            content = (ROOT / "web" / path[1:]).read_bytes()
            mime = "text/javascript"
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", mime)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def log_message(self, *_):
        pass


if __name__ == "__main__":
    ThreadingHTTPServer(("127.0.0.1", 18766), ProbeHandler).serve_forever()
