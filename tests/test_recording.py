from __future__ import annotations

from contextlib import contextmanager
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import av
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from robot_teleop.operator import load_robot_operator
from robot_teleop.recording import CaptureRgbSource, HostEpisodeRecorder, RecordingError, RgbFrame
from robot_teleop.recording_export import export_episodes, inspect_episode, resample_indices
from tools.lan_remote_view_server import LanRemoteHandler, LanRemoteServer


def jpeg():
    rgb = np.zeros((48, 64, 3), dtype=np.uint8)
    rgb[:, :, 0] = np.arange(64, dtype=np.uint8)*3
    rgb[:, :, 1] = np.arange(48, dtype=np.uint8)[:, None]*4
    rgb[:, :, 2] = 190
    codec = av.CodecContext.create('mjpeg', 'w')
    codec.width, codec.height, codec.pix_fmt = 64, 48, 'yuvj444p'
    return b''.join(bytes(p) for p in codec.encode(av.VideoFrame.from_ndarray(rgb, format='rgb24')))


JPEG = jpeg()


def feedback():
    return {'type': 'arm_status', 'follower_connected': True, 'measured_feedback_fresh': True,
            'last_read_age_ms': 0., 'last_write_age_ms': 2., 'follower_normalized': [1., 2., 3., 4., 5., 40.],
            'applied_normalized': [6., 7., 8., 9., 10., 55.], 'last_seq': 17, 'restart_count': 0}


class SyntheticSource:
    synthetic = True

    def __init__(self):
        self.ids = []
        self.closed = False
        self.age = .02
        self.epoch = 'test-capture'
        self.width = 64

    def list_cameras(self):
        return [{'id': 'left'}, {'id': 'right'}]

    def start(self, cameras):
        self.ids = cameras

    def snapshot(self):
        mono, unix = time.monotonic_ns(), time.time_ns()
        return [RgbFrame(name, JPEG, self.width, 48, mono//30_000_000, self.epoch,
                         unix-int(self.age*1e9), mono-int(self.age*1e9), mono) for name in self.ids]

    def close(self):
        self.closed = True


def wait_for(predicate, timeout=6):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('Timed out waiting for the recorder state')


@contextmanager
def recorder_at(root):
    source = SyntheticSource()
    recorder = HostEpisodeRecorder(root, load_robot_operator('so101'), lambda: source, minimum_free_bytes=0)
    stopping = threading.Event()
    status = feedback()

    def feed():
        while not stopping.is_set():
            recorder.observe_robot(status)
            stopping.wait(.01)

    worker = threading.Thread(target=feed, daemon=True)
    worker.start()
    wait_for(lambda: recorder.status()['ready'])
    try:
        yield recorder, source, status
    finally:
        recorder.close()
        stopping.set()
        worker.join(timeout=2)


def complete_episode(recorder, task='Put the block in the cup', outcome='success'):
    recorder.start('page-a', task)
    wait_for(lambda: recorder.status()['frames'] >= 4)
    recorder.stop('page-a', outcome=outcome)
    wait_for(lambda: recorder.status()['state'] not in ('preparing', 'recording', 'saving'))
    assert recorder.status()['state'] == 'idle', recorder.status()
    return recorder.root/recorder.status()['last_saved']['episode']


def test_host_episode_preserves_rgb_and_actual_applied_targets(tmp_path):
    with recorder_at(tmp_path) as (recorder, source, _):
        path = complete_episode(recorder)
        manifest, data = inspect_episode(path)
        assert manifest['status'] == 'complete' and manifest['synthetic'] is True
        assert manifest['outcome'] == 'success' and source.closed
        assert manifest['robot']['units'][-1] == 'percent'
        assert (path/str(data['obs.images.camera_1'][0])).read_bytes() == JPEG
        assert (path/str(data['obs.images.camera_2'][0])).read_bytes() == JPEG
        np.testing.assert_array_equal(data['obs.state'][0], feedback()['follower_normalized'])
        np.testing.assert_array_equal(data['action'][0], feedback()['applied_normalized'])
        assert data['obs.capture_unix_ns'].shape == (len(data['action']), 2)
        assert data['timestamp'][-1] > data['timestamp'][0]
        assert data['obs.robot_measured_ns'].dtype == np.int64
        assert np.all(data['action_applied_ns'] <= data['obs.robot_received_ns'])


TRIAL = {'session_id': 'test-session', 'index': 1, 'total': 3,
         'condition': 'fixed_rgb', 'display_mode': 'rgb_camera'}


def test_trial_metadata_survives_host_save(tmp_path):
    with recorder_at(tmp_path) as (recorder, _, _):
        supplied = dict(TRIAL)
        recorder.start('page-a', 'Match the viewing condition', trial=supplied)
        supplied['condition'] = 'head_3d'
        wait_for(lambda: recorder.status()['frames'] >= 3)
        assert recorder.status()['trial'] == TRIAL
        recorder.stop('page-a', outcome='success')
        wait_for(lambda: recorder.status()['state'] == 'idle')
        saved = recorder.status()['last_saved']
        assert saved['trial'] == TRIAL and saved['duration_seconds'] > 0
        manifest, _ = inspect_episode(tmp_path/saved['episode'])
        assert manifest['trial'] == TRIAL


@pytest.mark.parametrize('patch', [
    {'index': 0}, {'index': True}, {'index': 4}, {'total': 31}, {'session_id': '../test'},
    {'session_id': []}, {'condition': 'invented'}, {'display_mode': 'blendpcr'}, {'extra': 'unbounded'},
])
def test_invalid_trial_metadata_cannot_start_capture(tmp_path, patch):
    with recorder_at(tmp_path) as (recorder, _, _):
        with pytest.raises(RecordingError, match='[Tt]rial'):
            recorder.start('page-a', 'Invalid metadata', trial={**TRIAL, **patch})
        assert recorder.status()['state'] == 'idle' and recorder.worker is None


@pytest.mark.parametrize('change', ['stale_rgb', 'profile', 'robot', 'invalid_feedback'])
def test_source_faults_preserve_incomplete_episode(tmp_path, change):
    with recorder_at(tmp_path) as (recorder, source, status):
        recorder.start('page-a', 'Test source health')
        wait_for(lambda: recorder.status()['frames'] >= 2)
        if change == 'stale_rgb':
            source.age = 1.
        elif change == 'profile':
            source.epoch = 'changed'
        elif change == 'robot':
            status['restart_count'] = 1
        else:
            status['measured_feedback_fresh'] = False
        wait_for(lambda: recorder.status()['state'] == 'error')
        saved = recorder.status()['last_saved']
        assert saved['status'] == 'incomplete'
        assert (tmp_path/saved['episode']/'frames.jsonl').is_file()
        with pytest.raises(RecordingError, match='No matching complete episodes'):
            export_episodes(tmp_path, tmp_path/'export')


def test_discard_keeps_raw_capture_and_does_not_export_it(tmp_path):
    with recorder_at(tmp_path) as (recorder, _, _):
        recorder.start('page-a', 'Discard this capture')
        wait_for(lambda: recorder.status()['frames'] >= 2)
        recorder.stop('page-a', discard=True)
        wait_for(lambda: recorder.status()['state'] == 'idle')
        saved = recorder.status()['last_saved']
        assert saved['status'] == 'discarded'
        assert (tmp_path/saved['episode']/'episode.npz').is_file()
        with pytest.raises(RecordingError, match='No matching complete episodes'):
            export_episodes(tmp_path, tmp_path/'export', outcome='all')


def test_freshness_ownership_and_requests_are_enforced(tmp_path):
    recorder = HostEpisodeRecorder(tmp_path, load_robot_operator('so101'), SyntheticSource)
    with pytest.raises(RecordingError, match='Fresh measured'):
        recorder.start('a', 'Test')
    with recorder_at(tmp_path) as (recorder, _, _):
        for task, cameras, fps in [('', None, 15), ('ok', ['../escape'], 15), ('ok', ['left', 'left'], 15), ('ok', [{}], 15), ('ok', None, True)]:
            with pytest.raises(RecordingError):
                recorder.start('a', task, cameras, fps)
        recorder.start('a', 'Owned episode')
        with pytest.raises(RecordingError, match='another controller'):
            recorder.stop('b')
        with pytest.raises(RecordingError, match='already recording'):
            recorder.start('a', 'Second capture')
        recorder.interrupt('Controller changed')
        wait_for(lambda: recorder.status()['state'] == 'error')


def test_module_sample_contract_rejects_bad_or_automatic_motion():
    operator = load_robot_operator('so101')
    assert load_robot_operator('disabled').recording_spec() is None
    for changes in [{'follower_connected': False}, {'follower_normalized': [1, 2]},
                    {'applied_normalized': [float('nan')]*6}, {'last_read_age_ms': True},
                    {'last_write_age_ms': None},
                    {'calibration_sweep_active': True}, {'rest_return_active': True}]:
        assert operator.recording_sample({**feedback(), **changes}) is None
    assert operator.recording_sample(feedback())['action'] != operator.recording_sample(feedback())['state']


def test_existing_wrist_stream_is_reused_without_opening_or_stopping_hardware():
    process = SimpleNamespace(poll=lambda: None)
    hub = SimpleNamespace(condition=threading.Condition(), process=process, frame=JPEG,
                          sequence=5, capture_unix_ms=time.time()*1000-10)
    source = CaptureRgbSource('127.0.0.1', 1, lambda *_: pytest.fail('A new capture was opened'), {'wrist_rgb': hub})
    source.start(['aux_wrist_rgb'])
    frames = source.snapshot()
    assert len(frames) == 1 and frames[0].jpeg == JPEG
    assert (frames[0].width, frames[0].height) == (64, 48)
    source.close()
    assert hub.process is process and process.poll() is None


def test_recording_api_is_behind_the_existing_session_gate():
    handler = object.__new__(LanRemoteHandler)
    handler.path = '/api/v1/recording'
    handler.headers = {}
    handler.auth_required = lambda: True
    handler.is_authorized = lambda: False
    seen = []
    handler.redirect_to_login = lambda: seen.append('login')
    handler.send_response = lambda code: seen.append(code)
    handler.send_header = lambda *_: None
    handler.end_headers = lambda: None
    handler.serve_recording_status = lambda _: pytest.fail('Unauthenticated recording status')
    handler.handle_recording_command = lambda: pytest.fail('Unauthenticated recording command')
    handler.do_GET()
    handler.do_POST()
    assert seen == ['login', 401]


def test_export_never_overwrites_and_rejects_escaping_paths(tmp_path):
    target = tmp_path/'keep'
    target.mkdir()
    sentinel = target/'original.txt'
    sentinel.write_text('preserve', encoding='utf-8')
    with pytest.raises(RecordingError, match='already exists'):
        export_episodes(tmp_path, target)
    assert sentinel.read_text() == 'preserve'
    with recorder_at(tmp_path/'source') as (recorder, _, _):
        path = complete_episode(recorder)
    with np.load(path/'episode.npz', allow_pickle=False) as saved:
        arrays = dict(saved)
    arrays['obs.images.camera_1'] = np.asarray(['../outside.jpg']*len(arrays['action']))
    np.savez_compressed(path/'episode.npz', **arrays)
    with pytest.raises(RecordingError, match='invalid image path'):
        inspect_episode(path)


def test_export_resampling_retains_timing_and_rejects_large_gaps():
    np.testing.assert_array_equal(resample_indices(np.array([0., .07, .13, .2]), 15), [0, 1, 2, 3])
    with pytest.raises(RecordingError, match='gap exceeds'):
        resample_indices(np.array([0., 1.]), 15)


class CaptureHandler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        if self.path == '/cameras':
            body = json.dumps({'cameras': [{'id': 'camera-test', 'name': 'Synthetic camera'}]}).encode()
            self.send_response(200)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if not self.path.startswith('/camera-rgb-stream?'):
            self.send_error(404)
            return
        self.server.requests.append(self.path)
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        try:
            while not self.server.stopping.is_set():
                sequence = time.monotonic_ns()//30_000_000
                header = (f'--frame\r\nContent-Length: {len(JPEG)}\r\nX-Camera-Id: camera-test\r\n'
                          'X-Frame-Width: 64\r\nX-Frame-Height: 48\r\n'
                          'X-Source-Width: 64\r\nX-Source-Height: 48\r\n'
                          f'X-Frame-Sequence: {sequence}\r\nX-Capture-Epoch: synthetic\r\n'
                          f'X-Capture-Unix-Ms: {time.time()*1000-15:.3f}\r\n\r\n').encode()
                self.wfile.write(header+JPEG+b'\r\n')
                self.wfile.flush()
                self.server.stopping.wait(.03)
        except (OSError, ValueError):
            pass


def request(port, method, path, body=None, origin=True):
    connection = http.client.HTTPConnection('127.0.0.1', port, timeout=8)
    headers = {'Content-Type': 'application/json'}
    if origin:
        headers['Origin'] = f'http://127.0.0.1:{port}'
    connection.request(method, path, json.dumps(body).encode() if body is not None else None, headers)
    response = connection.getresponse()
    status, payload = response.status, response.read()
    connection.close()
    return status, json.loads(payload) if payload.startswith(b'{') else payload


def test_real_http_and_existing_multipart_rgb_record_without_motor_commands(tmp_path):
    capture = ThreadingHTTPServer(('127.0.0.1', 0), CaptureHandler)
    capture.daemon_threads = True
    capture.stopping, capture.requests = threading.Event(), []
    capture_thread = threading.Thread(target=capture.serve_forever, daemon=True)
    capture_thread.start()
    commands = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    commands.bind(('127.0.0.1', 0))
    commands.settimeout(.05)
    server = LanRemoteServer(('127.0.0.1', 0), LanRemoteHandler, tmp_path/'web', '127.0.0.1', 0,
        '127.0.0.1', capture.server_port, '', robot_module='so101', robot_command_port=commands.getsockname()[1],
        robot_status_port=0, robot_calibration_status_port=0, recording_root=tmp_path/'episodes',
        robot_view_devices={'wrist_rgb': 'unused'})
    server.browser_client_id = 'page-a'
    serving = threading.Thread(target=server.serve_forever, daemon=True)
    serving.start()
    stopping = threading.Event()

    def publish_feedback():
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as publisher:
            while not stopping.is_set():
                publisher.sendto(json.dumps(feedback()).encode(), server.robot_status_socket.getsockname())
                stopping.wait(.01)

    publisher = threading.Thread(target=publish_feedback, daemon=True)
    publisher.start()
    try:
        wait_for(lambda: server.episode_recorder.status()['ready'])
        assert request(server.server_port, 'POST', '/api/v1/recording', {'command': 'start', 'client': 'page-a', 'task': 'Synthetic HTTP trial'}, origin=False)[0] == 403
        assert request(server.server_port, 'POST', '/api/v1/recording', {'command': 'start', 'client': 'other', 'task': 'Wrong page'})[0] == 409
        assert request(server.server_port, 'POST', '/api/v1/recording', {'command': 'start', 'client': 'page-a', 'task': 'Synthetic HTTP trial', 'trial': {**TRIAL, 'index': 0}})[0] == 409
        assert request(server.server_port, 'POST', '/api/v1/recording', {'command': 'start', 'client': 'page-a', 'task': 'Synthetic HTTP trial', 'trial': TRIAL})[0] == 200
        wait_for(lambda: server.episode_recorder.status()['frames'] >= 4)
        assert request(server.server_port, 'GET', '/api/v1/recording?client=page-a')[1]['state'] == 'recording'
        assert request(server.server_port, 'POST', '/api/v1/recording', {'command': 'stop', 'client': 'page-a', 'outcome': 'success'})[0] == 200
        wait_for(lambda: server.episode_recorder.status()['state'] == 'idle')
        saved = server.episode_recorder.status()['last_saved']
        manifest, data = inspect_episode(tmp_path/'episodes'/saved['episode'])
        assert manifest['status'] == 'complete' and manifest['task'] == 'Synthetic HTTP trial'
        assert manifest['trial'] == TRIAL
        assert (tmp_path/'episodes'/saved['episode']/str(data['obs.images.camera_1'][0])).read_bytes() == JPEG
        assert all('w=3840' in path and 'q=95' in path for path in capture.requests)
        with pytest.raises(socket.timeout):
            commands.recvfrom(1024)
    finally:
        stopping.set()
        publisher.join(timeout=2)
        server.shutdown()
        server.server_close()
        serving.join(timeout=2)
        capture.stopping.set()
        capture.shutdown()
        capture.server_close()
        commands.close()


@pytest.mark.skipif(not os.environ.get('LEROBOT_TEST_PYTHON'), reason='requires isolated official LeRobot test environment')
def test_official_lerobot_export_and_training_dataloader(tmp_path):
    with recorder_at(tmp_path/'source') as (recorder, _, _):
        complete_episode(recorder)
        complete_episode(recorder)
        complete_episode(recorder, outcome='failure')
    output = tmp_path/'lerobot'
    env = {**os.environ, 'HF_HUB_OFFLINE': '1', 'HF_DATASETS_OFFLINE': '1', 'WANDB_MODE': 'disabled'}
    result = subprocess.run([os.environ['LEROBOT_TEST_PYTHON'], '-m', 'robot_teleop.recording_export',
                             '--source', str(tmp_path/'source'), '--output', str(output)],
                            cwd=ROOT, env=env, capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stderr[-6000:]
    exported = json.loads((output/'capture_export.json').read_text(encoding='utf-8'))
    assert len(exported['episodes']) == 2 and exported['uploaded'] is False
    code = """
from pathlib import Path
import torch,sys,json,av,numpy as np
from torch.utils.data import DataLoader
from lerobot.datasets.lerobot_dataset import LeRobotDataset
d=LeRobotDataset('local/robot-teleop',root=Path(sys.argv[1]))
assert d.num_episodes==2
b=next(iter(DataLoader(d,batch_size=2,num_workers=0)))
assert b['observation.state'].shape==(2,6)
assert b['action'].shape==(2,6)
assert b['observation.images.camera_1'].shape==(2,3,48,64)
assert not torch.equal(b['observation.state'],b['action'])
assert b['annotation.outcome'].eq(1).all()
raw=np.load(Path(sys.argv[2])/'episode.npz',allow_pickle=False)
image=Path(sys.argv[2])/str(raw['obs.images.camera_1'][0])
expected=av.CodecContext.create('mjpeg','r').decode(av.Packet(image.read_bytes()))[-1].to_ndarray(format='rgb24')
actual=(d[0]['observation.images.camera_1'].permute(1,2,0)*255).round().to(torch.uint8).numpy()
np.testing.assert_array_equal(actual,expected)
print(json.dumps({'episodes':d.num_episodes,'frames':len(d),'training_batch':True}))
"""
    loaded = subprocess.run([os.environ['LEROBOT_TEST_PYTHON'], '-c', code, str(output), str(sorted((tmp_path/'source').iterdir())[0])], cwd=ROOT,
                            env=env, capture_output=True, text=True, timeout=120)
    assert loaded.returncode == 0, loaded.stderr[-6000:]
