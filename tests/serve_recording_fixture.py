"""A loopback-only recording rehearsal. Every sensor/robot sample is synthetic."""
from pathlib import Path
import argparse
import json
import socket
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tests'))
sys.path.insert(0, str(ROOT))
from test_recording import CaptureHandler, feedback
from http.server import ThreadingHTTPServer
from tools.lan_remote_view_server import LanRemoteHandler, LanRemoteServer


class FixtureHandler(LanRemoteHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        if self.path == '/api/v1/robot-module':
            self.serve_json({'id': 'disabled', 'label': 'Synthetic recording rehearsal', 'entrypoints': {}, 'capabilities': {}})
            return
        if self.path.startswith('/capture-'):
            self.serve_json({})
            return
        super().do_GET()

    def serve_file(self, rel, include_body=True):
        if rel != 'controller.html':
            return super().serve_file(rel, include_body)
        source = (ROOT/'web/controller.html').read_text(encoding='utf-8')
        source = source.replace('<script type="module" src="./controller_hybrid.js',
            '<style>#start-overlay{display:none!important}</style><script>window.robotTeleopPageId="recording-fixture";localStorage.setItem("robotTeleopViewSettings",JSON.stringify({settings_version:16,head_tracking_enabled:false}));</script><script type="module" src="./controller_hybrid.js')
        source = source.replace('<h3 id="recording-title" class="section-title">Demonstration recording</h3>',
            '<h3 id="recording-title" class="section-title">Demonstration recording</h3><p class="helper-text">Synthetic rehearsal. No hardware is connected.</p>')
        data = source.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        if include_body:
            self.wfile.write(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=18768)
    parser.add_argument('--output', type=Path, default=ROOT/'.teleop/recording-rehearsal')
    args = parser.parse_args()
    capture = ThreadingHTTPServer(('127.0.0.1', 0), CaptureHandler)
    capture.daemon_threads = True
    capture.stopping, capture.requests = threading.Event(), []
    threading.Thread(target=capture.serve_forever, daemon=True).start()
    server = LanRemoteServer(('127.0.0.1', args.port), FixtureHandler, ROOT/'web', '127.0.0.1', 0,
        '127.0.0.1', capture.server_port, '', robot_module='so101', robot_command_port=0,
        robot_status_port=0, robot_calibration_status_port=0, recording_root=args.output,
        site_settings_path=args.output/'fixture-site-settings.json',
        robot_view_devices={'wrist_rgb': 'unused'})
    original_source = server.episode_recorder.source_factory

    def synthetic_source():
        source = original_source()
        source.synthetic = True
        return source

    server.episode_recorder.source_factory = synthetic_source
    server.browser_client_id = 'recording-fixture'
    stopping = threading.Event()

    def publish():
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as publisher:
            while not stopping.is_set():
                publisher.sendto(json.dumps(feedback()).encode(), server.robot_status_socket.getsockname())
                stopping.wait(.01)

    threading.Thread(target=publish, daemon=True).start()
    print(json.dumps({'port': server.server_port, 'synthetic': True}), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stopping.set()
        server.server_close()
        capture.stopping.set()
        capture.shutdown()
        capture.server_close()


if __name__ == '__main__':
    main()
