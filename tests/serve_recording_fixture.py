"""A loopback-only recording rehearsal. Every sensor/robot sample is synthetic."""
from pathlib import Path
import argparse
import base64
import copy
import json
import socket
import sys
import threading
import time
import struct
from urllib.parse import parse_qs, urlsplit

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tests'))
sys.path.insert(0, str(ROOT))
from test_recording import CaptureHandler, feedback
from http.server import ThreadingHTTPServer
from tools.lan_remote_view_server import LanRemoteHandler, LanRemoteServer
from workspace_readiness_fixture import generate


class TrialCaptureHandler(CaptureHandler):
    """Two original RGB streams plus public synthetic RGB-D and actual CAD FK."""
    scene = None

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/cameras':
            payload = json.dumps({'cameras': [{'id': 'camera-test', 'name': 'Camera 1'},
                                              {'id': 'camera-right', 'name': 'Camera 2'}]}).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        if path == '/rgbd':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            scene = self.scene
            original = base64.b64decode(scene['source'])
            length = struct.unpack('<I', original[5:9])[0]
            metadata = json.loads(original[9:9+length])
            payload = original[9+length:]
            metadata['robot'] = {'available': True, 'registration': {'registered': True},
                                 'links': scene['poses'][0]['links']}
            eye, target = np.array([.8, .6, .25]), np.array([0, .12, -1])
            back = eye-target
            back /= np.linalg.norm(back)
            right = np.cross([0, 1, 0], back)
            right /= np.linalg.norm(right)
            up = np.cross(back, right)
            metadata['viewer']['transform'] = [*right, 0, *up, 0, *back, 0, *eye, 1]
            try:
                while not self.server.stopping.is_set():
                    now = time.time()*1000
                    metadata['capture_unix_ms'] = now
                    for camera in metadata['cameras']:
                        camera.update(capture_unix_ms=now, sequence=int(time.monotonic()*10))
                    header = json.dumps(metadata, separators=(',', ':')).encode()
                    packet = b'RGBD1'+struct.pack('<I', len(header))+header+payload
                    self.wfile.write(f'--frame\r\nContent-Length: {len(packet)}\r\n\r\n'.encode()+packet+b'\r\n')
                    self.wfile.flush()
                    self.server.stopping.wait(.1)
            except (OSError, ValueError):
                pass
            return
        if path == '/camera-rgb-stream':
            selected = parse_qs(urlsplit(self.path).query).get('camera', ['camera-test'])[0]
            from test_recording import JPEG
            self.server.requests.append(self.path)
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while not self.server.stopping.is_set():
                    sequence = time.monotonic_ns()//30_000_000
                    header = (f'--frame\r\nContent-Length: {len(JPEG)}\r\nX-Camera-Id: {selected}\r\n'
                              'X-Frame-Width: 64\r\nX-Frame-Height: 48\r\nX-Source-Width: 64\r\nX-Source-Height: 48\r\n'
                              f'X-Frame-Sequence: {sequence}\r\nX-Capture-Epoch: synthetic\r\n'
                              f'X-Capture-Unix-Ms: {time.time()*1000-15:.3f}\r\n\r\n').encode()
                    self.wfile.write(header+JPEG+b'\r\n')
                    self.wfile.flush()
                    self.server.stopping.wait(.03)
            except (OSError, ValueError):
                pass
            return
        super().do_GET()


class FixtureHandler(LanRemoteHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        if self.path == '/trial_fixture.js':
            payload = (ROOT/'tests/trial_fixture.js').read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'text/javascript')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)
            return
        if self.path == '/api/v1/site-settings':
            settings = {**self.server.fixture_scene['settings'], 'settings_version': 16,
                        'robot_overlay_enabled': True, 'robot_overlay_style': 'solid',
                        'robot_overlay_mask_scanned_robot': False, 'head_tracking_enabled': False,
                        'scene_lighting_enabled': True, 'scene_shadows_enabled': True,
                        'orbit_distance': 1.2, 'focus_vertical_offset': .25, 'depth_only_enabled': True}
            self.serve_json(settings)
            return
        if self.path.startswith('/capture-'):
            self.serve_json({})
            return
        super().do_GET()

    def serve_file(self, rel, include_body=True):
        if rel == 'controller_hybrid.js':
            # The production HTTP default uses the gateway's normal port. This
            # fixture gets an ephemeral port to coexist with running services.
            data = (ROOT/'web/controller_hybrid.js').read_text(encoding='utf-8').replace(
                'ws://${pageHost()}:8766/head-tracking', 'ws://${window.location.host}/head-tracking').encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/javascript; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            if include_body:
                self.wfile.write(data)
            return
        if rel != 'controller.html':
            return super().serve_file(rel, include_body)
        source = (ROOT/'web/controller.html').read_text(encoding='utf-8')
        source = source.replace('<script type="module" src="./controller_hybrid.js',
            '<style>#start-overlay{display:none!important}</style><script>window.robotTeleopPageId="recording-fixture";</script><script type="module" src="/trial_fixture.js"></script><script type="module" src="./controller_hybrid.js')
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
    scene = generate()
    TrialCaptureHandler.scene = scene
    capture = ThreadingHTTPServer(('127.0.0.1', 0), TrialCaptureHandler)
    capture.daemon_threads = True
    capture.stopping, capture.requests = threading.Event(), []
    threading.Thread(target=capture.serve_forever, daemon=True).start()
    commands = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    commands.bind(('127.0.0.1', 0))
    commands.settimeout(.1)
    server = LanRemoteServer(('127.0.0.1', args.port), FixtureHandler, ROOT/'web', '127.0.0.1', 0,
        '127.0.0.1', capture.server_port, '', robot_module='so101', robot_command_port=commands.getsockname()[1],
        robot_status_port=0, robot_calibration_status_port=0, recording_root=args.output,
        site_settings_path=args.output/'fixture-site-settings.json',
        robot_view_devices={'wrist_rgb': 'unused'})
    original_source = server.episode_recorder.source_factory
    server.fixture_scene = copy.deepcopy(scene)

    def synthetic_source():
        source = original_source()
        source.synthetic = True
        close = source.close

        def delayed_close():
            close()
            time.sleep(.6)  # Exercise the UI's real Saving state.

        source.close = delayed_close
        return source

    server.episode_recorder.source_factory = synthetic_source
    server.browser_client_id = 'recording-fixture'
    stopping = threading.Event()
    control_lock = threading.Lock()
    control_status = {'state': 'ready', 'armed': False, 'torque_enabled': False}

    def receive_commands():
        # Simulated acknowledgement only; this socket never forwards to hardware.
        while not stopping.is_set():
            try:
                packet, _ = commands.recvfrom(65536)
                message = json.loads(packet)
                if message.get('type') in ('arm_hold', 'arm_enable'):
                    armed = message['type'] == 'arm_enable'
                    with control_lock:
                        control_status.update(state='armed' if armed else 'hold', armed=armed, torque_enabled=True)
            except TimeoutError:
                continue
            except OSError:
                break

    def publish():
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as publisher:
            while not stopping.is_set():
                with control_lock:
                    status = {**feedback(), **control_status}
                publisher.sendto(json.dumps(status).encode(), server.robot_status_socket.getsockname())
                stopping.wait(.01)

    threading.Thread(target=publish, daemon=True).start()
    threading.Thread(target=receive_commands, daemon=True).start()
    print(json.dumps({'port': server.server_port, 'synthetic': True}), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stopping.set()
        commands.close()
        server.server_close()
        capture.stopping.set()
        capture.shutdown()
        capture.server_close()


if __name__ == '__main__':
    main()
