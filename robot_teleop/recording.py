"""Host-side episode capture from existing RGB and robot services.

Like RoboCade, capture episodes first and export them with LeRobot separately.
JPEG frames go straight to disk; no camera, serial bus or motor is opened here.
Incomplete/discarded episodes remain recoverable and are excluded from export.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import http.client
import json
from pathlib import Path
import shutil
import threading
import time
from typing import Callable
from urllib.parse import urlencode
import uuid

import av
import numpy as np


class RecordingError(ValueError):
    pass


def write_json(path: Path, data: dict) -> None:
    temporary = path.with_suffix('.new')
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False), encoding='utf-8')
    temporary.replace(path)


def camera_id(value: object) -> str:
    if not isinstance(value, str) or not 1 <= len(value) <= 64 or any(
        not (c.isascii() and (c.isalnum() or c in '-_')) for c in value
    ):
        raise RecordingError('Invalid camera ID.')
    return value


@dataclass(frozen=True)
class RgbFrame:
    camera: str
    jpeg: bytes
    width: int
    height: int
    sequence: int
    epoch: str
    capture_unix_ns: int
    capture_monotonic_ns: int
    received_monotonic_ns: int


class CaptureRgbSource:
    """Subscribe to original RGB on the already-running capture service."""

    def __init__(self, host: str, port: int, reader_factory: Callable, auxiliary_hubs: dict | None = None):
        self.host, self.port, self.reader_factory = host, port, reader_factory
        self.lock = threading.Lock()
        self.frames: dict[str, deque] = {}
        self.readers: list = []
        self.threads: list[threading.Thread] = []
        self.stopping = threading.Event()
        self.error = ''
        self.auxiliary_hubs = auxiliary_hubs or {}
        self.selected_auxiliary: list[str] = []
        self.auxiliary_dimensions: dict[str, tuple[int, int]] = {}

    def list_cameras(self) -> list[dict]:
        connection = http.client.HTTPConnection(self.host, self.port, timeout=3)
        try:
            connection.request('GET', '/cameras')
            response = connection.getresponse()
            raw = response.read(65537)
            if response.status != 200 or len(raw) > 65536:
                raise RecordingError('The capture service is not ready.')
            cameras = json.loads(raw).get('cameras')
            if not isinstance(cameras, list) or not 1 <= len(cameras) <= 64:
                raise RecordingError('Connect at least one RGB camera before recording.')
            result = [{'id': camera_id(c['id']), 'name': str(c.get('name', c['id']))[:128]}
                      for c in cameras]
            for key, hub in self.auxiliary_hubs.items():
                with hub.condition:
                    if hub.process is not None and hub.process.poll() is None and hub.frame:
                        result.append({'id': 'aux_'+key, 'name': key+' (existing camera stream)'})
            return sorted(result, key=lambda c: c['id'])
        except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
            raise RecordingError('The capture camera list is unavailable.') from exc
        finally:
            connection.close()

    def start(self, cameras: list[str]) -> None:
        for name in cameras:
            if name.startswith('aux_') and name[4:] in self.auxiliary_hubs:
                self.selected_auxiliary.append(name)
                continue
            with self.lock:
                self.frames[name] = deque(maxlen=3)
            worker = threading.Thread(target=self._drain, args=(name,), daemon=True,
                                      name='recording-original-rgb')
            self.threads.append(worker)
            worker.start()

    def _drain(self, name: str) -> None:
        reader = None
        try:
            subscribed_unix_ns = time.time_ns()
            options = urlencode({'camera': name, 'w': 3840, 'q': 95})
            reader = self.reader_factory(f'http://{self.host}:{self.port}/camera-rgb-stream?{options}')
            with self.lock:
                self.readers.append(reader)
            if self.stopping.is_set():
                return
            while not self.stopping.is_set():
                headers, jpeg = reader.read_part()
                mono, unix = time.monotonic_ns(), time.time_ns()
                width, height = int(headers['x-frame-width']), int(headers['x-frame-height'])
                if 'x-source-width' not in headers or 'x-source-height' not in headers:
                    raise RecordingError('Update/reload the capture runtime before recording native RGB.')
                source_size = (int(headers['x-source-width']), int(headers['x-source-height']))
                if source_size[0] > 3840:
                    raise RecordingError('This camera exceeds the native RGB recording width supported by the capture service.')
                captured = int(float(headers['x-capture-unix-ms'])*1_000_000)
                if headers.get('x-camera-id') != name or width <= 0 or height <= 0 or width*height > 16*1024*1024:
                    raise RecordingError('Camera identity or image dimensions changed.')
                if not jpeg.startswith(b'\xff\xd8') or len(jpeg) > 16*1024*1024:
                    raise RecordingError('The capture service returned invalid RGB.')
                age = unix-captured
                # Ignore the previous consumer's cached frame while the native
                # encoder warms up for this subscription.
                if captured < subscribed_unix_ns or age > 500_000_000 or (width, height) != source_size:
                    continue
                if age < -100_000_000:
                    raise RecordingError('Camera timestamps are stale or clocks disagree.')
                frame = RgbFrame(name, jpeg, width, height, int(headers['x-frame-sequence']),
                                 str(headers.get('x-capture-epoch', ''))[:128], captured,
                                 mono-age, mono)
                with self.lock:
                    self.frames[name].append(frame)
        except Exception as exc:
            if not self.stopping.is_set():
                with self.lock:
                    self.error = str(exc) if isinstance(exc, RecordingError) else 'Original RGB stopped or its timestamp/format became invalid.'
        finally:
            if reader is not None:
                reader.close()

    def snapshot(self) -> list[RgbFrame]:
        auxiliary = []
        for name in self.selected_auxiliary:
            hub = self.auxiliary_hubs[name[4:]]
            with hub.condition:
                if hub.process is None or hub.process.poll() is not None or not hub.frame:
                    raise RecordingError('The selected auxiliary RGB stream stopped.')
                jpeg, sequence, captured, epoch = hub.frame, hub.sequence, hub.capture_unix_ms, str(id(hub.process))
            if name not in self.auxiliary_dimensions:
                decoded = av.CodecContext.create('mjpeg', 'r').decode(av.Packet(jpeg))
                if not decoded:
                    raise RecordingError('Invalid auxiliary RGB frame.')
                self.auxiliary_dimensions[name] = (decoded[-1].width, decoded[-1].height)
            width, height = self.auxiliary_dimensions[name]
            mono, unix = time.monotonic_ns(), time.time_ns()
            captured = int(captured*1_000_000)
            auxiliary.append(RgbFrame(name, jpeg, width, height, sequence, epoch, captured,
                                      mono-(unix-captured), mono))
        with self.lock:
            if self.error:
                raise RecordingError(self.error)
            if any(not frames for frames in self.frames.values()):
                return []
            anchor = next(iter(self.frames.values()))[-1].capture_monotonic_ns if self.frames else auxiliary[0].capture_monotonic_ns
            return [min(frames, key=lambda f: abs(f.capture_monotonic_ns-anchor))
                    for frames in self.frames.values()]+auxiliary

    def close(self) -> None:
        self.stopping.set()
        with self.lock:
            readers = list(self.readers)
        for reader in readers:
            reader.close()
        for worker in self.threads:
            worker.join(timeout=1)


class HostEpisodeRecorder:
    def __init__(self, root: Path, operator, source_factory: Callable,
                 minimum_free_bytes: int = 512*1024*1024):
        self.root, self.operator, self.source_factory = Path(root).resolve(), operator, source_factory
        self.minimum_free_bytes = minimum_free_bytes
        self.lock = threading.RLock()
        self.telemetry: deque = deque(maxlen=256)
        self.worker: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.owner = ''
        self.state = 'idle'
        self.message = 'Ready to record on the robot host.'
        self.episode = ''
        self.samples = 0
        self.started_ns = 0
        self.stopped_ns = 0
        self.pauses: list[dict] = []
        self.last_saved = None
        self.interruption = ''
        self.discard = False
        self.outcome = 'unrated'
        self.trial = None

    def observe_robot(self, status: dict, received_ns: int | None = None) -> None:
        """Called by the existing UDP listener; never performs file I/O."""
        sample = self.operator.recording_sample(status)
        if sample is not None:
            received_ns = received_ns or time.monotonic_ns()
            sample = {**sample, 'received_ns': received_ns,
                      'measured_ns': received_ns-int(sample['read_age_ms']*1_000_000),
                      'applied_ns': received_ns-int(sample['write_age_ms']*1_000_000)}
        with self.lock:
            # Invalid feedback must invalidate the current window immediately.
            if sample is None:
                self.telemetry.clear()
            else:
                self.telemetry.append(sample)

    def status(self, client: str = '') -> dict:
        with self.lock:
            ready = bool(self.telemetry and time.monotonic_ns()-self.telemetry[-1]['measured_ns'] <= 350_000_000)
            elapsed, _ = self._trial_clock()
            return {'supported': self.operator.recording_spec() is not None,
                    'pause_supported': True,
                    'ready': ready,
                    'state': self.state, 'message': self.message, 'episode': self.episode,
                    'frames': self.samples, 'elapsed_seconds':
                    round(elapsed, 1) if self.state in ('recording', 'paused', 'saving') else 0,
                    'is_owner': not self.owner or client == self.owner, 'last_saved': self.last_saved,
                    'trial': self.trial}

    def _trial_clock(self, at_ns: int | None = None) -> tuple[float, bool]:
        """Task time excludes pauses; capture timestamps always keep real time."""
        if not self.started_ns:
            return 0., False
        at_ns = at_ns if at_ns is not None else time.monotonic_ns()
        if self.stopped_ns:
            at_ns = min(at_ns, self.stopped_ns)
        seconds = max(0., (at_ns-self.started_ns)/1e9)
        paused = sum(max(0., min(seconds, p['end_seconds'] if p['end_seconds'] is not None else seconds)
                         - p['start_seconds']) for p in self.pauses)
        inside = any(p['start_seconds'] <= seconds and
                     (p['end_seconds'] is None or seconds < p['end_seconds']) for p in self.pauses)
        return max(0., seconds-paused), inside

    def _close_pause(self, now_ns: int) -> None:
        if self.pauses and self.pauses[-1]['end_seconds'] is None:
            self.pauses[-1]['end_seconds'] = max(0., (now_ns-self.started_ns)/1e9)

    def set_paused(self, client: str, paused: bool) -> dict:
        with self.lock:
            if self.owner != client:
                raise RecordingError('This episode belongs to another controller page.')
            if self.state not in ('recording', 'paused'):
                raise RecordingError('Pause is only available during an active recording.')
            if paused and self.state == 'recording':
                self.pauses.append({'start_seconds': (time.monotonic_ns()-self.started_ns)/1e9,
                                    'end_seconds': None})
                self.state, self.message = 'paused', 'Trial paused; original RGB and robot data are still being recorded.'
            elif not paused and self.state == 'paused':
                self._close_pause(time.monotonic_ns())
                self.state, self.message = 'recording', 'Recording original RGB and measured robot data on the host.'
            return self.status(client)

    def start(self, client: str, task: str, cameras: list[str] | None = None, fps: int = 15,
              trial: dict | None = None) -> dict:
        if not isinstance(task, str) or not task.strip() or len(task) > 500:
            raise RecordingError('Enter a task instruction (up to 500 characters).')
        if type(fps) is not int or fps not in (10, 15, 20, 30):
            raise RecordingError('Recording rate must be 10, 15, 20 or 30 Hz.')
        if cameras is not None and (not isinstance(cameras, list) or not 1 <= len(cameras) <= 8):
            raise RecordingError('Select between one and eight distinct cameras.')
        cameras = [camera_id(c) for c in cameras] if cameras is not None else None
        if cameras is not None and len(set(cameras)) != len(cameras):
            raise RecordingError('Select distinct cameras.')
        if trial is not None:
            fields = {'session_id', 'index', 'total', 'condition', 'display_mode'}
            if not isinstance(trial, dict) or set(trial) != fields:
                raise RecordingError('Invalid trial metadata fields.')
            session = trial['session_id']
            if not isinstance(session, str) or not 1 <= len(session) <= 80 or any(
                    c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in session):
                raise RecordingError('Invalid trial session identifier.')
            if (type(trial['index']) is not int or type(trial['total']) is not int
                    or not 1 <= trial['index'] <= trial['total'] <= 30):
                raise RecordingError('Invalid trial number.')
            if trial['condition'] not in ('fixed_rgb', 'mouse_3d', 'head_3d'):
                raise RecordingError('Invalid trial condition.')
            if trial['display_mode'] not in ('rgb_camera', 'point_cloud', 'shader_mesh', 'independent_mesh', 'unified_mesh', 'blendpcr'):
                raise RecordingError('Invalid trial display mode.')
            if (trial['condition'] == 'fixed_rgb') != (trial['display_mode'] == 'rgb_camera'):
                raise RecordingError('Trial condition does not match the display mode.')
            trial = dict(trial)
        with self.lock:
            if self.worker is not None and self.worker.is_alive():
                raise RecordingError('An episode is already recording or finishing.')
            if self.operator.recording_spec() is None:
                raise RecordingError('This robot module does not provide demonstration recording.')
            if not self.telemetry or time.monotonic_ns()-self.telemetry[-1]['measured_ns'] > 350_000_000:
                raise RecordingError('Fresh measured robot feedback and an applied position target are required before recording.')
            self.owner, self.state, self.samples = client, 'preparing', 0
            self.message = 'Checking original RGB and measured robot feedback…'
            self.episode = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_')+uuid.uuid4().hex[:12]
            self.stop_event = threading.Event()
            self.interruption, self.discard, self.outcome = '', False, 'unrated'
            self.trial = trial
            self.started_ns = 0
            self.stopped_ns = 0
            self.pauses = []
            self.worker = threading.Thread(target=self._record, args=(task.strip(), cameras, fps, trial),
                                           daemon=True, name='host-episode-recorder')
            self.worker.start()
            return self.status(client)

    def stop(self, client: str, *, discard: bool = False, outcome: str = 'unrated') -> dict:
        if outcome not in ('success', 'failure', 'unrated') or type(discard) is not bool:
            raise RecordingError('Choose success, failure or unrated.')
        with self.lock:
            if self.owner and client != self.owner:
                raise RecordingError('This episode belongs to another controller page.')
            if self.state not in ('preparing', 'recording', 'paused', 'saving'):
                return self.status(client)
            if self.state != 'saving':
                self.discard, self.outcome = discard, outcome
                self.stopped_ns = time.monotonic_ns()
                self._close_pause(self.stopped_ns)
                self.state, self.message = 'saving', 'Finishing the episode on the host…'
                self.stop_event.set()
            return self.status(client)

    def interrupt(self, reason: str) -> None:
        with self.lock:
            if self.state in ('preparing', 'recording', 'paused'):
                self.interruption = reason
                self.stopped_ns = time.monotonic_ns()
                self._close_pause(self.stopped_ns)
                self.state, self.message = 'saving', 'Preserving the interrupted episode…'
                self.stop_event.set()

    def close(self) -> None:
        self.interrupt('The host gateway shut down.')
        if self.worker is not None:
            self.worker.join(timeout=12)

    def _sample(self, source) -> tuple[list[RgbFrame], dict] | None:
        frames = source.snapshot()
        if not frames:
            return None
        now = time.monotonic_ns()
        anchor = frames[0].capture_monotonic_ns
        if any(not -100_000_000 <= now-f.capture_monotonic_ns <= 350_000_000 for f in frames):
            raise RecordingError('Original RGB became stale. The episode was preserved as incomplete.')
        if max(f.capture_monotonic_ns for f in frames)-min(f.capture_monotonic_ns for f in frames) > 100_000_000:
            raise RecordingError('Camera timestamps differ by more than 100 ms.')
        with self.lock:
            if not self.telemetry:
                raise RecordingError('Measured robot feedback became unavailable.')
            robot = min(self.telemetry, key=lambda s: abs(s['measured_ns']-anchor))
            if now-self.telemetry[-1]['measured_ns'] > 350_000_000 or abs(robot['measured_ns']-anchor) > 100_000_000:
                raise RecordingError('Robot feedback and camera capture differ by more than 100 ms.')
        return frames, robot

    def _record(self, task: str, cameras: list[str] | None, fps: int, trial: dict | None) -> None:
        source, destination, manifest = None, None, None
        rows, saved_bytes, failure = [], 0, ''
        try:
            source = self.source_factory()
            available = source.list_cameras()
            selected = cameras or [c['id'] for c in available]
            if not 1 <= len(selected) <= 8 or not set(selected).issubset({c['id'] for c in available}):
                raise RecordingError('Selected RGB cameras are unavailable (maximum eight).')
            source.start(selected)
            deadline = time.monotonic()+5
            sample = None
            while not self.stop_event.wait(.02) and time.monotonic() < deadline:
                sample = self._sample(source)
                if sample:
                    break
            if not sample:
                if self.stop_event.is_set():
                    return
                raise RecordingError('Original RGB did not become ready. No episode was created.')
            self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
            if shutil.disk_usage(self.root).free < self.minimum_free_bytes:
                raise RecordingError('Less than 512 MiB of free space is available for recording.')
            destination = self.root/self.episode
            frames, first_robot = sample
            # The source returns capture cameras followed by existing auxiliary
            # views. Freeze this actual order for every sample and export.
            actual_cameras = [f.camera for f in frames]
            if set(actual_cameras) != set(selected):
                raise RecordingError('The RGB source returned a different camera set.')
            dimensions = [(f.width, f.height, f.epoch) for f in frames]
            for frame in frames:
                decoder = av.CodecContext.create('mjpeg', 'r')
                decoded = decoder.decode(av.Packet(frame.jpeg))
                if not decoded or (decoded[-1].width, decoded[-1].height) != (frame.width, frame.height):
                    raise RecordingError('RGB headers do not match the encoded camera frame.')
            destination.mkdir(mode=0o700)
            manifest = {'format': 'robot-teleop-episode/v1', 'episode_id': self.episode, 'status': 'recording',
                        'task': task, 'fps': fps, 'robot': self.operator.recording_spec(),
                        'cameras': [{'key': f'camera_{i+1}', 'id': f.camera, 'width': f.width, 'height': f.height}
                                    for i, f in enumerate(frames)],
                        'created_utc': datetime.now(timezone.utc).isoformat(),
                        'timing': 'Capture Unix timestamps mapped to host monotonic time; nearest measured feedback within 100 ms.',
                        'rgb': 'Original capture JPEG, independent of browser crop, viewport and point cloud.',
                        'synthetic': bool(getattr(source, 'synthetic', False))}
            if trial is not None:
                manifest['trial'] = trial
            manifest['cameras'] = [{'key': f'camera_{i+1}', 'id': f.camera, 'width': f.width, 'height': f.height,
                                    'timestamp_source': 'host JPEG receipt' if f.camera.startswith('aux_') else 'capture service'}
                                   for i, f in enumerate(frames)]
            write_json(destination/'episode.json', manifest)
            for camera in manifest['cameras']:
                (destination/camera['key']).mkdir()
            with self.lock:
                self.started_ns = frames[0].capture_monotonic_ns
                if not self.stop_event.is_set():
                    self.state, self.message = 'recording', 'Recording original RGB and measured robot data on the host.'
            tick = time.monotonic()
            with (destination/'frames.jsonl').open('x', encoding='utf-8') as journal:
                while not self.stop_event.is_set():
                    current = self._sample(source)
                    if current is None:
                        raise RecordingError('An RGB camera stopped supplying frames.')
                    frames, robot = current
                    if [(f.width, f.height, f.epoch) for f in frames] != dimensions or robot['epoch'] != first_robot['epoch']:
                        raise RecordingError('The camera profile or robot service changed during recording.')
                    if [f.camera for f in frames] != actual_cameras:
                        raise RecordingError('The RGB source camera order changed.')
                    if rows and any(f.sequence < rows[-1]['camera_sequence'][i] or
                                    f.capture_unix_ns < rows[-1]['capture_unix_ns'][i] for i, f in enumerate(frames)):
                        raise RecordingError('Camera source timestamps or frame counters ran backwards.')
                    row = {'timestamp': (frames[0].capture_monotonic_ns-self.started_ns)/1e9,
                           'state': robot['state'], 'action': robot['action'],
                           'robot_measured_ns': robot['measured_ns'], 'robot_received_ns': robot['received_ns'],
                           'action_applied_ns': robot['applied_ns'],
                           'command_sequence': robot['sequence'], 'images': [], 'capture_unix_ns': [],
                           'capture_monotonic_ns': [], 'camera_sequence': []}
                    with self.lock:
                        row['trial_elapsed_seconds'], row['trial_paused'] = self._trial_clock(frames[0].capture_monotonic_ns)
                    # Repeated samples are explicit in source sequence/timestamps;
                    # frozen inputs are rejected by the capture-age bound above.
                    for camera, frame in zip(manifest['cameras'], frames, strict=True):
                        name = f"{camera['key']}/{len(rows):07d}.jpg"
                        with (destination/name).open('xb') as image:
                            image.write(frame.jpeg)
                        saved_bytes += len(frame.jpeg)
                        row['images'].append(name)
                        row['capture_unix_ns'].append(frame.capture_unix_ns)
                        row['capture_monotonic_ns'].append(frame.capture_monotonic_ns)
                        row['camera_sequence'].append(frame.sequence)
                    journal.write(json.dumps(row, allow_nan=False)+'\n')
                    journal.flush()
                    rows.append(row)
                    with self.lock:
                        self.samples = len(rows)
                    if time.monotonic_ns()-self.started_ns > 600*1e9 or saved_bytes > 4*1024**3:
                        raise RecordingError('The episode reached its 10-minute or 4-GiB capture limit.')
                    if len(rows) % fps == 0 and shutil.disk_usage(self.root).free < self.minimum_free_bytes:
                        raise RecordingError('Recording stopped because free disk space is low.')
                    tick += 1/fps
                    if time.monotonic()-tick > .25:
                        raise RecordingError('Disk/capture cannot keep up with the selected recording rate.')
                    self.stop_event.wait(max(0, tick-time.monotonic()))
        except Exception as exc:
            failure = str(exc) if isinstance(exc, RecordingError) else 'Host recording failed; partial files were preserved.'
        finally:
            with self.lock:
                if self.state in ('preparing', 'recording', 'paused'):
                    self.state = 'saving'
                self.stopped_ns = self.stopped_ns or time.monotonic_ns()
                self._close_pause(self.stopped_ns)
                active_duration, _ = self._trial_clock()
                pauses = [dict(p) for p in self.pauses]
            if source is not None:
                try:
                    source.close()
                except Exception:
                    failure = failure or 'The RGB subscription did not close cleanly.'
            failure = failure or self.interruption
            try:
                if destination is not None and manifest is not None:
                    if rows:
                        arrays = {'obs.state': np.asarray([r['state'] for r in rows], dtype=np.float32),
                                  'action': np.asarray([r['action'] for r in rows], dtype=np.float32),
                                  'timestamp': np.asarray([r['timestamp'] for r in rows], dtype=np.float64),
                                  'trial_elapsed_seconds': np.asarray([r['trial_elapsed_seconds'] for r in rows], dtype=np.float64),
                                  'trial_paused': np.asarray([r['trial_paused'] for r in rows], dtype=np.bool_),
                                  'obs.capture_unix_ns': np.asarray([r['capture_unix_ns'] for r in rows], dtype=np.int64),
                                  'obs.capture_monotonic_ns': np.asarray([r['capture_monotonic_ns'] for r in rows], dtype=np.int64),
                                  'obs.robot_measured_ns': np.asarray([r['robot_measured_ns'] for r in rows], dtype=np.int64),
                                  'obs.robot_received_ns': np.asarray([r['robot_received_ns'] for r in rows], dtype=np.int64),
                                  'action_applied_ns': np.asarray([r['action_applied_ns'] for r in rows], dtype=np.int64),
                                  'command_sequence': np.asarray([r['command_sequence'] for r in rows], dtype=np.int64),
                                  'camera_sequence': np.asarray([r['camera_sequence'] for r in rows], dtype=np.int64)}
                        for i, camera in enumerate(manifest['cameras']):
                            arrays['obs.images.'+camera['key']] = np.asarray([r['images'][i] for r in rows])
                        with (destination/'episode.npz').open('xb') as data:
                            np.savez_compressed(data, **arrays)
                    status = 'discarded' if self.discard else 'incomplete' if failure or len(rows) < 2 else 'complete'
                    manifest.update(status=status, outcome=self.outcome, frames=len(rows), bytes=saved_bytes,
                                    pauses=pauses, active_duration_seconds=round(active_duration, 3),
                                    error=failure, finished_utc=datetime.now(timezone.utc).isoformat())
                    write_json(destination/'episode.json', manifest)
                    self.last_saved = {'episode': self.episode, 'status': status, 'frames': len(rows),
                                       'outcome': self.outcome, 'host_folder': str(destination),
                                       'duration_seconds': round(rows[-1]['timestamp']+1/fps, 3) if rows else 0,
                                       'active_duration_seconds': round(active_duration, 3),
                                       'trial': trial}
            except Exception:
                failure = 'Episode finalization failed; preserve the host folder for recovery.'
            with self.lock:
                self.state = 'error' if failure else 'idle'
                self.message = failure or ('Episode discarded and retained locally.' if self.discard else
                                           'Episode saved on the host.' if len(rows) >= 2 else 'No complete episode was saved.')
                self.owner = ''
