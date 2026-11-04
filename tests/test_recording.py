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


@pytest.mark.parametrize('total', [3, 12, 18])
def test_trial_metadata_survives_host_save(tmp_path, total):
    with recorder_at(tmp_path) as (recorder, _, _):
        expected = {**TRIAL, 'total': total, 'index': total,
                    'session_name': 'Participant α / test — 2026-10-28 16:00:00 GMT+1'}
        supplied = dict(expected)
        recorder.start('page-a', 'Match the viewing condition', trial=supplied)
        supplied['condition'] = 'head_3d'
        wait_for(lambda: recorder.status()['frames'] >= 3)
        assert recorder.status()['trial'] == expected
        recorder.stop('page-a', outcome='success')
        wait_for(lambda: recorder.status()['state'] == 'idle')
        saved = recorder.status()['last_saved']
        assert saved['trial'] == expected and saved['duration_seconds'] > 0
        manifest, _ = inspect_episode(tmp_path/saved['episode'])
        assert manifest['trial'] == expected


def test_pause_freezes_trial_time_without_dropping_capture_and_finishes_paused(tmp_path):
    with recorder_at(tmp_path) as (recorder, _, _):
        with pytest.raises(RecordingError):
            recorder.set_paused('page-a', True)
        recorder.start('page-a', 'Place the block in the cup', trial=TRIAL)
        wait_for(lambda: recorder.status()['frames'] >= 4)
        with pytest.raises(RecordingError, match='another controller'):
            recorder.set_paused('other', True)
        paused = recorder.set_paused('page-a', True)
        assert paused['state'] == 'paused' and paused['pause_supported']
        recorder.set_paused('page-a', True)  # A retried request is idempotent.
        wait_for(lambda: recorder.status()['frames'] >= paused['frames']+4)
        assert recorder.status()['elapsed_seconds'] == paused['elapsed_seconds']
        assert len(recorder.pauses) == 1 and recorder.pauses[0]['end_seconds'] is None
        recorder.set_paused('page-a', False)
        recorder.set_paused('page-a', False)
        wait_for(lambda: recorder.status()['elapsed_seconds'] > paused['elapsed_seconds']+.1)
        assert recorder.pauses[0]['end_seconds'] > recorder.pauses[0]['start_seconds']
        paused = recorder.set_paused('page-a', True)
        wait_for(lambda: recorder.status()['frames'] >= paused['frames']+3)
        recorder.stop('page-a', outcome='success')
        wait_for(lambda: recorder.status()['state'] == 'idle')
        saved = recorder.status()['last_saved']
        manifest, arrays = inspect_episode(tmp_path/saved['episode'])
        assert manifest['status'] == 'complete' and manifest['trial'] == TRIAL
        assert len(manifest['pauses']) == 2
        assert all(p['end_seconds'] > p['start_seconds'] for p in manifest['pauses'])
        assert saved['active_duration_seconds'] < saved['duration_seconds']-.25
        assert manifest['active_duration_seconds'] == saved['active_duration_seconds']
        mask = arrays['trial_paused']
        assert mask.sum() >= 4 and (~mask).sum() >= 4
        assert np.all(np.diff(arrays['timestamp']) > 0)
        assert np.all(np.diff(arrays['trial_elapsed_seconds']) >= -1e-6)
        assert np.ptp(arrays['trial_elapsed_seconds'][mask][-2:]) < 1e-6
        rows = [json.loads(line) for line in (tmp_path/saved['episode']/'frames.jsonl').read_text().splitlines()]
        assert [r['trial_paused'] for r in rows] == mask.tolist()
        assert all(len(list((tmp_path/saved['episode']/c['key']).glob('*.jpg'))) == manifest['frames'] for c in manifest['cameras'])

        class Dataset:
            @classmethod
            def create(cls, **kwargs):
                kwargs['root'].mkdir()
                cls.frames, cls.features = [], kwargs['features']
                return cls()

            def add_frame(self, frame):
                self.frames.append(frame)

            def save_episode(self):
                pass

            def finalize(self):
                pass

        summary = export_episodes(tmp_path/saved['episode'], tmp_path/'export', dataset_class=Dataset)
        assert summary['episodes'][0]['pauses'] == manifest['pauses']
        exported_mask = [int(frame['annotation.trial_paused'][0]) for frame in Dataset.frames]
        assert 0 in exported_mask and 1 in exported_mask
        assert 'annotation.trial_paused' in Dataset.features

        # Older captures remain exportable, with explicitly unpaused annotations.
        arrays.pop('trial_paused')
        arrays.pop('trial_elapsed_seconds')
        np.savez_compressed(tmp_path/saved['episode']/'episode.npz', **arrays)
        export_episodes(tmp_path/saved['episode'], tmp_path/'legacy-export', dataset_class=Dataset)
        assert all(frame['annotation.trial_paused'][0] == 0 for frame in Dataset.frames)


@pytest.mark.parametrize('patch', [
    {'index': 0}, {'index': True}, {'index': 4}, {'total': 31}, {'session_id': '../test'},
    {'session_id': []}, {'condition': 'invented'}, {'display_mode': 'blendpcr'}, {'extra': 'unbounded'},
    {'session_name': ''}, {'session_name': ' '}, {'session_name': []},
    {'session_name': 'x'*201}, {'session_name': 'name\n2026-10-28'},
])
def test_invalid_trial_metadata_cannot_start_capture(tmp_path, patch):
    with recorder_at(tmp_path) as (recorder, _, _):
        with pytest.raises(RecordingError, match='[Tt]rial'):
            recorder.start('page-a', 'Invalid metadata', trial={**TRIAL, **patch})
        assert recorder.status()['state'] == 'idle' and recorder.worker is None


@pytest.mark.parametrize('change', ['stale_rgb', 'profile', 'robot', 'invalid_feedback', 'camera_order'])
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
        elif change == 'camera_order':
            source.ids.reverse()
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


@pytest.mark.parametrize('has_aligned_feedback', [True, False])
def test_robot_feedback_must_align_with_every_camera(tmp_path, has_aligned_feedback):
    recorder = HostEpisodeRecorder(tmp_path, load_robot_operator('so101'), SyntheticSource)
    now = time.monotonic_ns()
    if has_aligned_feedback:
        recorder.observe_robot(feedback(), received_ns=now-50_000_000)
    recorder.observe_robot(feedback(), received_ns=now)
    frames = [RgbFrame(str(i), JPEG, 64, 48, 1, 'capture', time.time_ns(), now-age, now)
              for i, age in enumerate([10_000_000, 105_000_000])]
    source = SimpleNamespace(snapshot=lambda: frames)
    if has_aligned_feedback:
        _, robot = recorder._sample(source)
        assert max(abs(robot['measured_ns']-f.capture_monotonic_ns) for f in frames) <= 100_000_000
    else:
        with pytest.raises(RecordingError, match='100 ms'):
            recorder._sample(source)


def test_feedback_uses_producer_timestamp_despite_udp_delay(tmp_path):
    recorder = HostEpisodeRecorder(tmp_path, load_robot_operator('so101'), SyntheticSource)
    now = time.monotonic_ns()
    status = {**feedback(), 'sent_unix_ms': time.time_ns()/1_000_000-180, 'last_read_age_ms': 10}
    recorder.observe_robot(status, received_ns=now)
    sample = recorder.telemetry[-1]
    assert 185_000_000 < now-sample['measured_ns'] < 195_000_000
    assert sample['received_ns'] == now
    recorder.observe_robot({**status, 'sent_unix_ms': 1})
    assert not recorder.telemetry


def test_transient_mismatch_waits_for_a_genuinely_aligned_pair(tmp_path):
    recorder = HostEpisodeRecorder(tmp_path, load_robot_operator('so101'), SyntheticSource)
    mono = time.monotonic_ns()
    recorder.observe_robot(feedback(), received_ns=mono-200_000_000)
    frame = RgbFrame('left', JPEG, 64, 48, 1, 'capture', time.time_ns(), mono, mono)
    worker = threading.Timer(.025, lambda: recorder.observe_robot(feedback()))
    worker.start()
    try:
        frames, sample = recorder._sample(SimpleNamespace(snapshot=lambda: [frame]))
        assert abs(sample['measured_ns']-frames[0].capture_monotonic_ns) <= 100_000_000
    finally:
        worker.join()


def test_frozen_short_capture_is_not_reported_as_complete(tmp_path):
    with recorder_at(tmp_path) as (recorder, source, _):
        snapshot = source.snapshot
        cached = []

        def frozen():
            if not cached:
                cached.extend(snapshot())
            return cached

        source.snapshot = frozen
        recorder.start('page-a', 'Short frozen capture')
        wait_for(lambda: recorder.status()['frames'] >= 2)
        recorder.stop('page-a', outcome='success')
        wait_for(lambda: recorder.status()['state'] not in ('recording', 'saving'))
        saved = recorder.status()['last_saved']
        assert saved['status'] == 'incomplete'
        assert 'advanc' in recorder.status()['message'].lower()
        assert (tmp_path/saved['episode']/'frames.jsonl').is_file()


def test_failed_finalization_preserves_raw_evidence_and_cannot_export(tmp_path, monkeypatch):
    with recorder_at(tmp_path) as (recorder, _, _):
        recorder.start('page-a', 'Finalization failure')
        wait_for(lambda: recorder.status()['frames'] >= 3)

        def fail(*args, **kwargs):
            raise OSError('Synthetic disk failure')

        monkeypatch.setattr('robot_teleop.recording.np.savez_compressed', fail)
        recorder.stop('page-a', outcome='success')
        wait_for(lambda: recorder.status()['state'] == 'error')
        folder = tmp_path/recorder.episode
        assert (folder/'frames.jsonl').is_file() and list((folder/'camera_1').glob('*.jpg'))
        assert json.loads((folder/'episode.json').read_text())['status'] != 'complete'
        with pytest.raises(RecordingError, match='No matching complete episodes'):
            export_episodes(tmp_path, tmp_path/'export')


def test_disk_pressure_stops_capture_as_incomplete(tmp_path, monkeypatch):
    with recorder_at(tmp_path) as (recorder, _, _):
        recorder.minimum_free_bytes = 1
        monkeypatch.setattr('robot_teleop.recording.shutil.disk_usage',
                            lambda _: SimpleNamespace(free=2 if recorder.samples < 3 else 0))
        recorder.start('page-a', 'Disk pressure')
        wait_for(lambda: recorder.status()['state'] == 'error')
        saved = recorder.status()['last_saved']
        assert saved['status'] == 'incomplete' and saved['frames'] >= 3
        assert 'disk space' in recorder.status()['message']


@pytest.mark.parametrize('fault', ['camera_skew', 'robot_skew', 'backward_sequence', 'relative_time'])
def test_export_rejects_invalid_recorded_timing(tmp_path, fault):
    with recorder_at(tmp_path) as (recorder, _, _):
        root = complete_episode(recorder)
    with np.load(root/'episode.npz', allow_pickle=False) as data:
        arrays = dict(data)
    if fault == 'camera_skew':
        arrays['obs.capture_monotonic_ns'][:, 1] -= 200_000_000
        arrays['obs.capture_unix_ns'][:, 1] -= 200_000_000
    elif fault == 'robot_skew':
        arrays['obs.robot_measured_ns'] -= 200_000_000
    elif fault == 'backward_sequence':
        arrays['camera_sequence'][1, 0] = arrays['camera_sequence'][0, 0]-1
    else:
        arrays['timestamp'][1:] += .01
    np.savez_compressed(root/'episode.npz', **arrays)
    with pytest.raises(RecordingError):
        inspect_episode(root)


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


@pytest.mark.parametrize('pause_trial', [False, True])
@pytest.mark.parametrize('metric', ['pass_fail', 'block_count'])
def test_real_http_and_existing_multipart_rgb_record_without_motor_commands(tmp_path, monkeypatch, pause_trial, metric):
    trial = {**TRIAL, 'success_metric': metric}
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
        assert request(server.server_port, 'POST', '/api/v1/recording', {'command': 'start', 'client': 'page-a', 'task': 'Synthetic HTTP trial', 'trial': trial})[0] == 200
        wait_for(lambda: server.episode_recorder.status()['frames'] >= 4)
        assert request(server.server_port, 'GET', '/api/v1/recording?client=page-a')[1]['state'] == 'recording'
        if pause_trial:
            assert request(server.server_port, 'POST', '/api/v1/recording', {'command': 'pause', 'client': 'other'})[0] == 409
            with pytest.raises(socket.timeout):
                commands.recvfrom(1024)
            assert request(server.server_port, 'POST', '/api/v1/recording', {'command': 'pause', 'client': 'page-a'})[1]['state'] == 'paused'
            assert json.loads(commands.recvfrom(1024)[0])['type'] == 'arm_hold'
            assert request(server.server_port, 'POST', '/api/v1/recording', {'command': 'resume', 'client': 'page-a'})[1]['state'] == 'recording'
            with pytest.raises(socket.timeout):  # Recorder Resume never enables motors.
                commands.recvfrom(1024)
            with monkeypatch.context() as patch:
                def failed_hold(_):
                    raise OSError('simulated missing arm service')
                patch.setattr(server, 'forward_robot', failed_hold)
                code, result = request(server.server_port, 'POST', '/api/v1/recording', {'command': 'pause', 'client': 'page-a'})
                assert code == 409 and 'Hold could not reach' in result['error']
                assert server.episode_recorder.status()['state'] == 'paused'
            assert request(server.server_port, 'POST', '/api/v1/recording', {'command': 'pause', 'client': 'page-a'})[0] == 200
            assert json.loads(commands.recvfrom(1024)[0])['type'] == 'arm_hold'
        assert request(server.server_port, 'POST', '/api/v1/recording', {'command': 'stop', 'client': 'page-a', 'outcome': 'success'})[0] == 200
        wait_for(lambda: server.episode_recorder.status()['state'] == 'idle')
        saved = server.episode_recorder.status()['last_saved']
        manifest, data = inspect_episode(tmp_path/'episodes'/saved['episode'])
        assert manifest['status'] == 'complete' and manifest['task'] == 'Synthetic HTTP trial'
        assert manifest['trial'] == trial
        # The actual result popup sends session_id, including after a page refresh.
        result_fields = {'command': 'rate', 'outcome': 'failure'} if metric == 'pass_fail' else {'command': 'score', 'block_count': 0}
        result_fields.update(client='page-b', session_id=trial['session_id'])
        assert request(server.server_port, 'POST', '/api/v1/recording', result_fields)[0] == 409
        server.browser_client_id = 'page-b'
        assert request(server.server_port, 'POST', '/api/v1/recording', {**result_fields, 'session_id': 'wrong-session'})[0] == 409
        assert request(server.server_port, 'POST', '/api/v1/recording', {**result_fields, 'unexpected': True})[0] == 409
        code, reviewed = request(server.server_port, 'POST', '/api/v1/recording', result_fields)
        assert code == 200, reviewed
        assert reviewed['last_saved']['result_reviewed']
        manifest = json.loads((tmp_path/'episodes'/saved['episode']/'episode.json').read_text())
        assert manifest['result_reviewed']
        assert manifest['outcome'] == 'failure' if metric == 'pass_fail' else manifest['block_count'] == 0
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
assert b['annotation.trial_paused'].eq(0).all()
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


def test_original_rgb_retries_transient_startup_capacity_then_preserves_frame():
    attempts = []
    class Reader:
        closed = False
        def read_part(self):
            source.stopping.set()
            return {"x-frame-width":"64","x-frame-height":"48","x-source-width":"64","x-source-height":"48",
                    "x-capture-unix-ms":str(time.time_ns()/1e6),"x-camera-id":"camera_c","x-frame-sequence":"1"}, JPEG
        def close(self):self.closed=True
    reader=Reader()
    def factory(url):
        attempts.append(url)
        if len(attempts)<3:raise OSError("HTTP/1.1 503 Service Unavailable")
        return reader
    source=CaptureRgbSource('127.0.0.1',8781,factory)
    source.start(['camera_c']);source.threads[0].join(timeout=3)
    assert len(attempts)==3 and not source.error and reader.closed
    assert source.frames['camera_c'][0].jpeg==JPEG


def test_original_rgb_capacity_failure_names_camera_and_actual_http_error():
    attempts=[]
    def factory(url):
        attempts.append(url)
        raise OSError('HTTP/1.1 503 Service Unavailable')
    source=CaptureRgbSource('127.0.0.1',8781,factory)
    source.start(['camera_c']);source.threads[0].join(timeout=3)
    assert len(attempts)==4
    assert 'camera_c' in source.error and '503' in source.error
    assert not source.frames['camera_c']
    source.close()


def test_original_rgb_does_not_retry_or_hide_a_broken_stream():
    attempts=[]
    def factory(url):
        attempts.append(url)
        raise OSError('Connection reset by peer')
    source=CaptureRgbSource('127.0.0.1',8781,factory)
    source.start(['camera_c']);source.threads[0].join(timeout=2)
    assert len(attempts)==1 and 'Connection reset by peer' in source.error
    source.close()


def test_trial_host_deadline_saves_and_holds_without_browser_timer(tmp_path):
    with recorder_at(tmp_path) as (recorder, source, _):
        held=[];recorder.on_time_limit=lambda:held.append(True)
        trial={'session_id':'deadline','index':1,'total':3,'condition':'fixed_rgb','display_mode':'rgb_camera',
               'task_id':'count_blocks','time_limit_seconds':1}
        recorder.start('page-a','Count blocks',trial=trial)
        wait_for(lambda:recorder.status()['state']=='idle',timeout=4)
        saved=recorder.last_saved
        assert saved['timed_out'] and saved['status']=='complete' and held==[True]
        assert 1<=saved['active_duration_seconds']<1.3
        assert saved['trial']['time_limit_seconds']==1
        with pytest.raises(RecordingError):recorder.set_block_count('other',4)
        for bad in (-1,True,1.5,10000):
            with pytest.raises(RecordingError):recorder.set_block_count('page-a',bad)
        result=recorder.set_block_count('page-a',0)
        assert result['last_saved']['block_count']==0
        manifest=json.loads((tmp_path/saved['episode']/'episode.json').read_text())
        assert manifest['block_count']==0 and manifest['timed_out']


def test_trial_deadline_excludes_paused_time(tmp_path):
    with recorder_at(tmp_path) as (recorder, source, _):
        trial={'session_id':'paused-deadline','index':1,'total':3,'condition':'fixed_rgb','display_mode':'rgb_camera','time_limit_seconds':1}
        recorder.start('page-a','Pause deadline',trial=trial)
        wait_for(lambda:recorder.status()['state']=='recording')
        recorder.set_paused('page-a',True);time.sleep(1.1)
        assert recorder.state=='paused'
        recorder.set_paused('page-a',False)
        wait_for(lambda:recorder.state=='idle',timeout=3)
        assert recorder.last_saved['timed_out']
        assert recorder.last_saved['duration_seconds']>1.8


@pytest.mark.parametrize('task_id', ['single_block', 'custom'])
def test_binary_trial_finish_before_limit_passes_and_timeout_fails(tmp_path, task_id):
    trial = {'session_id': 'binary', 'index': 1, 'total': 18, 'condition': 'fixed_rgb',
             'display_mode': 'rgb_camera', 'task_id': task_id, 'time_limit_seconds': 1,
             'success_metric': 'pass_fail', 'outcome_rule': 'finish_before_limit'}
    with recorder_at(tmp_path) as (recorder, _, _):
        recorder.start('page-a', 'Balance test', trial=trial)
        wait_for(lambda: recorder.samples >= 3)
        recorder.stop('page-a')
        wait_for(lambda: recorder.state == 'idle')
        assert recorder.last_saved['outcome'] == 'success'
        assert not recorder.last_saved['timed_out']
        recorder.start('page-a', 'Balance test', trial=trial)
        wait_for(lambda: recorder.state == 'idle', timeout=4)
        assert recorder.last_saved['outcome'] == 'failure'
        assert recorder.last_saved['timed_out']
        saved = json.loads((tmp_path/recorder.last_saved['episode']/'episode.json').read_text())
        assert saved['outcome'] == 'failure' and saved['trial']['outcome_rule'] == 'finish_before_limit'


def test_binary_trial_manual_failure_and_discard_do_not_auto_pass(tmp_path):
    trial = {'session_id': 'binary', 'index': 1, 'total': 18, 'condition': 'fixed_rgb',
             'display_mode': 'rgb_camera', 'task_id': 'custom', 'time_limit_seconds': 75,
             'success_metric': 'pass_fail', 'outcome_rule': 'finish_before_limit'}
    with recorder_at(tmp_path) as (recorder, _, _):
        for extra, expected in [({'outcome': 'failure'}, 'failure'), ({'discard': True}, 'unrated')]:
            recorder.start('page-a', 'Balance test', trial=trial)
            wait_for(lambda: recorder.samples >= 3)
            recorder.stop('page-a', **extra)
            wait_for(lambda: recorder.state == 'idle')
            assert recorder.last_saved['outcome'] == expected


def test_count_metric_is_configurable_without_changing_task_identity(tmp_path):
    trial = {'session_id': 'counting', 'index': 1, 'total': 18, 'condition': 'fixed_rgb',
             'display_mode': 'rgb_camera', 'task_id': 'custom', 'time_limit_seconds': 75,
             'success_metric': 'block_count', 'outcome_rule': 'manual'}
    with recorder_at(tmp_path) as (recorder, _, _):
        recorder.start('page-a', 'Balance test', trial=trial)
        wait_for(lambda: recorder.samples >= 3)
        recorder.stop('page-a')
        wait_for(lambda: recorder.state == 'idle')
        assert recorder.last_saved['outcome'] == 'unrated'
        recorder.set_block_count('page-a', 0)
        assert recorder.last_saved['block_count'] == 0


@pytest.mark.parametrize('timeout', [False, True])
def test_tray_transfer_count_done_and_timeout_keep_observations_actions_and_identity(tmp_path, timeout):
    trial = {'session_id': 'tray-only', 'index': 1, 'total': 6, 'condition': 'fixed_rgb',
             'display_mode': 'rgb_camera', 'task_id': 'tray_transfer_v1', 'time_limit_seconds': 1,
             'success_metric': 'block_count', 'outcome_rule': 'manual', 'phase': 'measured', 'result_prompt': True}
    with recorder_at(tmp_path) as (recorder, _, _):
        recorder.start('page-a', 'Tray transfers: full release', trial=trial)
        if not timeout:
            wait_for(lambda: recorder.samples >= 3)
            recorder.stop('page-a')
        wait_for(lambda: recorder.state == 'idle', timeout=4)
        saved = recorder.last_saved
        assert saved['status'] == 'complete' and saved['timed_out'] is timeout
        assert saved['outcome'] == 'unrated'
        recorder.set_block_count('page-a', 0 if timeout else 7)
        folder = tmp_path/saved['episode']
        manifest = json.loads((folder/'episode.json').read_text())
        assert manifest['trial'] == trial and manifest['result_reviewed']
        assert manifest['block_count'] == (0 if timeout else 7)
        with np.load(folder/'episode.npz') as arrays:
            assert arrays['action'].shape == arrays['obs.state'].shape == (manifest['frames'], 6)


def test_result_prompt_review_is_saved_before_next_and_owned_by_recorder(tmp_path):
    trial = {'session_id': 'review', 'index': 1, 'total': 18, 'condition': 'fixed_rgb',
             'display_mode': 'rgb_camera', 'task_id': 'custom', 'time_limit_seconds': 75,
             'success_metric': 'pass_fail', 'outcome_rule': 'finish_before_limit', 'result_prompt': True}
    with recorder_at(tmp_path) as (recorder, _, _):
        recorder.start('page-a', 'Balance test', trial=trial)
        wait_for(lambda: recorder.samples >= 3)
        recorder.stop('page-a')
        wait_for(lambda: recorder.state == 'idle')
        with pytest.raises(RecordingError):recorder.set_outcome('other', 'failure')
        assert not recorder.last_saved.get('result_reviewed')
        recorder.set_outcome('page-a', 'failure')
        assert recorder.last_saved['result_reviewed'] and recorder.last_saved['outcome'] == 'failure'
        saved = json.loads((tmp_path/recorder.last_saved['episode']/'episode.json').read_text())
        assert saved['result_reviewed'] and saved['outcome'] == 'failure'
        with pytest.raises(RecordingError):recorder.resume_result_review('refreshed-page', 'wrong-session')
        recorder.resume_result_review('refreshed-page', 'review')
        recorder.set_outcome('refreshed-page', 'success')
        assert recorder.last_saved['outcome'] == 'success'


@pytest.mark.parametrize('condition', ['fixed_rgb', 'mouse_3d', 'head_3d'])
@pytest.mark.parametrize('timeout', [False, True])
def test_practice_records_observations_actions_but_never_pass_fail(tmp_path, condition, timeout):
    trial = {'session_id': 'practice', 'index': 1, 'total': 15, 'condition': condition,
             'display_mode': 'rgb_camera' if condition == 'fixed_rgb' else 'point_cloud',
             'task_id': 'single_block', 'time_limit_seconds': 1 if timeout else 60,
             'phase': 'practice', 'success_metric': 'practice', 'outcome_rule': 'manual', 'result_prompt': False}
    with recorder_at(tmp_path) as (recorder, _, _):
        recorder.start('page-a', 'Free exploration', trial=trial)
        wait_for(lambda: recorder.samples >= 3)
        if not timeout:
            recorder.stop('page-a', outcome='success')  # A stale result selector cannot score practice.
        wait_for(lambda: recorder.state == 'idle', timeout=4)
        saved = recorder.last_saved
        assert saved['status'] == 'complete' and saved['outcome'] == 'unrated'
        assert saved['timed_out'] is timeout and saved['trial'] == trial
        path = tmp_path/saved['episode']
        manifest = json.loads((path/'episode.json').read_text())
        assert manifest['trial']['phase'] == 'practice' and manifest['outcome'] == 'unrated'
        inspected, _ = inspect_episode(path)
        assert inspected['trial'] == trial
        with np.load(path/'episode.npz', allow_pickle=False) as data:
            assert len(data['action']) == len(data['obs.state']) == saved['frames']
            assert all((path/str(name)).is_file() for name in data['obs.images.camera_1'])
        with pytest.raises(RecordingError):
            recorder.set_outcome('page-a', 'success')
        with pytest.raises(RecordingError):
            recorder.set_block_count('page-a', 1)


def test_trial_preserves_actual_preview_start_settings(tmp_path):
    trial = {**TRIAL, 'point_preview_detail':40000, 'stream_preset':'latency', 'robot_overlay_enabled':True, 'control_source':'keyboard'}
    with recorder_at(tmp_path) as (recorder, _, _):
        recorder.start('page-a', 'Preview settings audit', trial=trial)
        wait_for(lambda: recorder.samples >= 3)
        recorder.stop('page-a')
        wait_for(lambda: recorder.state == 'idle')
        path = tmp_path/recorder.last_saved['episode']
        assert json.loads((path/'episode.json').read_text())['trial'] == trial
        assert inspect_episode(path)[0]['trial'] == trial


@pytest.mark.parametrize('patch', [{'point_preview_detail':True}, {'point_preview_detail':1000},
    {'stream_preset':'unknown'}, {'robot_overlay_enabled':1}, {'control_source':'unknown'}])
def test_trial_rejects_invalid_preview_settings(tmp_path, patch):
    with recorder_at(tmp_path) as (recorder, _, _):
        with pytest.raises(RecordingError):
            recorder.start('page-a', 'Invalid preview snapshot', trial={**TRIAL, **patch})
