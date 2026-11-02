"""Persistent native point-preview encoders. Never opens a sensor.

Each camera has one persistent encoder process and two bounded latest-packet
slots. Existing WebSocket lanes consume their original publication parity.
"""
from __future__ import annotations

import atexit
import ctypes
import hashlib
import json
import mmap
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time
import uuid
from collections import OrderedDict

ROOT = Path(__file__).resolve().parents[1]
WORK = ROOT/'.teleop/native-preview-workers'
sys.path.insert(0, str(ROOT))
SLOT_SIZE = 8 * 1024 * 1024
SLOT_HEADER = 32
CONTROL_SIZE = 64 * 1024
CONTROL_OFFSET = SLOT_SIZE * 2
SETTINGS_KEYS = {
    'workspace_stream_crop_enabled', 'display_mode', 'point_preview_enabled',
    'point_cleanup_support', 'point_cleanup_tolerance_mm',
    'point_preview_budget', 'modeled_workspace_enabled', 'modeled_workspace_center',
    'modeled_workspace_width', 'modeled_workspace_depth', 'modeled_workspace_height',
    'modeled_workspace_heading', 'modeled_workspace_clearance', '_sparse_predictor', '_mesh_profile',
}
ROBOT_PATH = Path.home()/'.local/share/godot/app_userdata/Robot Teleop Vision/robot_overlay_render_state.json'


def packet_robot(path):
    try:
        value = json.loads(Path(path).read_text())
        if Path(value.get('project_root', '')).resolve() == ROOT and 0 <= time.time()-float(value.get('updated', 0)) < 2:
            return value.get('robot', {'available': False})
    except (OSError, ValueError, TypeError):
        pass
    return {'available': False}


def write_slot(mapping, slot, version, generation, packet):
    if not 9 <= len(packet) <= SLOT_SIZE-SLOT_HEADER:
        raise ValueError('worker packet exceeds bounded slot')
    offset = slot*SLOT_SIZE
    struct.pack_into('<Q', mapping, offset, version-1)
    struct.pack_into('<I', mapping, offset+8, len(packet))
    struct.pack_into('<Q', mapping, offset+16, generation)
    mapping[offset+SLOT_HEADER:offset+SLOT_HEADER+len(packet)] = packet
    struct.pack_into('<Q', mapping, offset, version)


def read_slot(mapping, slot, previous=0):
    offset = slot*SLOT_SIZE
    version = struct.unpack_from('<Q', mapping, offset)[0]
    if not version or version & 1 or version <= previous:
        return None
    length = struct.unpack_from('<I', mapping, offset+8)[0]
    generation = struct.unpack_from('<Q', mapping, offset+16)[0]
    if not 9 <= length <= SLOT_SIZE-SLOT_HEADER:
        raise ValueError('invalid worker slot length')
    packet = mapping[offset+SLOT_HEADER:offset+SLOT_HEADER+length]
    if struct.unpack_from('<Q', mapping, offset)[0] != version:
        return None
    if packet[:5] != b'RGBD1':
        raise ValueError('invalid worker packet signature')
    return version, generation, packet


def read_control(mapping, offset=0):
    generation = struct.unpack_from('<Q', mapping, offset)[0]
    if not generation or generation & 1:
        return None
    length = struct.unpack_from('<I', mapping, offset+8)[0]
    if not 0 < length <= CONTROL_SIZE-SLOT_HEADER:
        raise ValueError('invalid preview control length')
    data = mapping[offset+SLOT_HEADER:offset+SLOT_HEADER+length]
    if struct.unpack_from('<Q', mapping, offset)[0] != generation:
        return None
    return generation, json.loads(data)


def write_control(mapping, generation, data):
    if not 0 < len(data) <= CONTROL_SIZE-SLOT_HEADER:
        raise ValueError('preview settings exceed control slot')
    struct.pack_into('<Q', mapping, 0, generation-1)
    struct.pack_into('<I', mapping, 8, len(data))
    mapping[SLOT_HEADER:SLOT_HEADER+len(data)] = data
    struct.pack_into('<Q', mapping, 0, generation)


def encode_loop(config):
    # Exit if the gateway disappears, including abnormal termination.
    if sys.platform == 'linux':
        ctypes.CDLL(None).prctl(1, signal.SIGTERM)
        if os.getppid() != config['parent_pid']:
            return
    from tools.native_sparse_stream import NativeSnapshot, encode_native_camera
    from tools.workspace_stream_crop import WorkspaceStreamCropper
    stopping = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stopping.set())
    reader = NativeSnapshot(config['camera_path'])
    cropper = WorkspaceStreamCropper()
    previous = None
    version = 0
    interval = 1/max(1, min(60, config['fps']))
    next_allowed = time.monotonic()
    with Path(config['output']).open('r+b') as stream:
        mapping = mmap.mmap(stream.fileno(), CONTROL_OFFSET+CONTROL_SIZE)
        try:
            while not stopping.wait(.001):
                if time.monotonic() < next_allowed:
                    continue
                control = read_control(mapping, CONTROL_OFFSET)
                if control is None:
                    continue
                generation, settings = control
                if settings.get('point_preview_enabled') is not True:
                    break
                snapshot = reader.read(previous)
                if snapshot is None or time.time()*1000-snapshot['stamp'] > 1000:
                    continue
                started = time.monotonic()
                encoder = encode_native_camera
                if settings.get('_mesh_profile') == 'balanced':
                    from tools.native_mesh_stream import encode_native_mesh_camera
                    encoder = encode_native_mesh_camera
                packet = encoder(snapshot, config['camera'], settings, cropper,
                                              packet_robot(config['robot_path']), config['viewer'])
                previous = snapshot['key']
                slot = (snapshot['key'][1]//2) % 2
                version += 2
                write_slot(mapping, slot, version, generation, packet)
                next_allowed = max(next_allowed+interval, started)
        finally:
            mapping.close()
            reader.close()
            Path(config['output']).unlink(missing_ok=True)
            Path(config['config_path']).unlink(missing_ok=True)


class CameraWorker:
    def __init__(self, camera, path, settings, fps, viewer, robot_path):
        self.camera_id = camera['id']
        self.condition = threading.Condition()
        self.subscribers = set()
        self.latest = [None, None]
        self.versions = [0, 0]
        self.stop_event = threading.Event()
        self.error = ''
        self.closed = False
        self.settings_signature = None
        self.generation = 0
        self.claimed_version = 0
        token = uuid.uuid4().hex
        WORK.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.config_path = WORK/f'{token}.json'
        self.output_path = Path('/dev/shm')/f'teleop_native_preview_{token}'
        fd = os.open(self.output_path, os.O_CREAT|os.O_EXCL|os.O_RDWR, 0o600)
        try:
            os.ftruncate(fd, CONTROL_OFFSET+CONTROL_SIZE)
            self.mapping = mmap.mmap(fd, SLOT_SIZE*2, access=mmap.ACCESS_READ)
            self.control = mmap.mmap(fd, CONTROL_SIZE, access=mmap.ACCESS_WRITE, offset=CONTROL_OFFSET)
        finally:
            os.close(fd)
        self.configure(settings)
        config = dict(camera=camera, camera_path=str(path), fps=fps, config_path=str(self.config_path),
                      viewer=viewer, output=str(self.output_path), parent_pid=os.getpid(), robot_path=str(robot_path))
        self.config_path.write_text(json.dumps(config))
        self.config_path.chmod(0o600)
        env = {**os.environ, 'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1',
               'MKL_NUM_THREADS': '1', 'NUMEXPR_NUM_THREADS': '1'}
        log = (WORK/'worker-errors.log').open('a')
        try:
            self.process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), '--config', str(self.config_path)],
                                            cwd=ROOT, env=env, stdout=log, stderr=log)
        except BaseException:
            self.mapping.close()
            self.control.close()
            self.output_path.unlink(missing_ok=True)
            self.config_path.unlink(missing_ok=True)
            raise
        finally:
            log.close()
        self.thread = threading.Thread(target=self.monitor, name='packet-monitor-'+camera['id'][-4:], daemon=True)
        self.thread.start()

    def configure(self, settings):
        data = json.dumps(settings, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
        with self.condition:
            if data != self.settings_signature:
                self.generation += 2
                self.claimed_version = 0
                write_control(self.control, self.generation, data)
                self.settings_signature = data
            return self.generation

    def claim(self, version, generation, session=None):
        """Assign a publication once to whichever delivery lane has credit."""
        with self.condition:
            if generation != self.generation or version <= self.claimed_version:
                return False
            if session is not None:
                ready = [peer for peer in self.subscribers if peer.adaptive_delivery and peer.delivery_waiting]
                now = time.monotonic()
                if ready and session.delivery_score(now) > min(peer.delivery_score(now) for peer in ready) + 5:
                    return False
            self.claimed_version = version
            return True

    def monitor(self):
        try:
            while not self.stop_event.wait(.001):
                if self.process.poll() is not None:
                    raise ConnectionError(f'preview worker exited: {self.process.returncode}')
                updates = []
                for slot in range(2):
                    value = read_slot(self.mapping, slot, self.versions[slot])
                    if value is not None:
                        updates.append((slot, *value))
                for slot, version, generation, packet in sorted(updates, key=lambda row: row[1]):
                    with self.condition:
                        self.versions[slot] = version
                        self.latest[slot] = generation, packet
                        subscribers = tuple(self.subscribers)
                    for session in subscribers:
                        session.publish(self, slot, version, generation, packet)
        except (OSError, ValueError, ConnectionError) as error:
            self.error = str(error)
            with self.condition:
                subscribers = tuple(self.subscribers)
            for session in subscribers:
                with session.condition:
                    session.errors[self.camera_id] = self.error
                    session.condition.notify_all()

    def subscribe(self, session):
        with self.condition:
            self.subscribers.add(session)
            initial = [(slot, version, self.latest[slot]) for slot, version in enumerate(self.versions) if self.latest[slot]]
        for slot, version, packet in sorted(initial, key=lambda row: row[1]):
            generation, packet = packet
            session.publish(self, slot, version, generation, packet)

    def close(self):
        with self.condition:
            if self.closed:
                return
            self.closed = True
        self.stop_event.set()
        self.thread.join(timeout=2)
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        self.mapping.close()
        self.control.close()
        assert self.output_path.parent == Path('/dev/shm') and self.output_path.name.startswith('teleop_native_preview_')
        self.output_path.unlink(missing_ok=True)
        self.config_path.unlink(missing_ok=True)


_registry = {}
_registry_lock = threading.RLock()


def release(worker, session):
    closing = False
    with _registry_lock:
        with worker.condition:
            worker.subscribers.discard(session)
            unused = not worker.subscribers
        if unused and _registry.get(worker.key) is worker:
            del _registry[worker.key]
            closing = True
    if closing:
        worker.close()


def close_all():
    with _registry_lock:
        workers = list(_registry.values())
        _registry.clear()
    for worker in workers:
        worker.close()


atexit.register(close_all)


class ProcessSparseSession:
    def __init__(self, cameras, settings, robot, fps=30, viewer=None, frame_lane=None, frame_lanes=1, robot_path=ROBOT_PATH, owner='local', adaptive_delivery=False):
        if frame_lanes not in (1, 2) or frame_lane is not None and frame_lane not in range(frame_lanes):
            raise ValueError('invalid preview frame lane')
        self.cameras, self.settings, self.fps, self.viewer = cameras, settings, fps, viewer
        self.frame_lane, self.frame_lanes = frame_lane, frame_lanes
        self.adaptive_delivery = bool(adaptive_delivery and frame_lanes == 2)
        self.delivery_waiting = False
        self.delivery_rtt_ms = 0.0
        self.delivery_oldest_sent = None
        self.robot_path = robot_path
        self.owner = owner
        self.condition = threading.Condition()
        self.stop_event = threading.Event()
        self.pending = OrderedDict()
        self.errors = {}
        self.workers = {}
        self.previous = {}
        self.generations = {}
        self.signature = None
        self.lifecycle_lock = threading.RLock()
        self.settings_dirty = threading.Event()

    def start(self):
        self.refresh()

    def refresh(self):
        with self.lifecycle_lock:
            self._refresh()

    def _refresh(self):
        if self.stop_event.is_set():
            return
        settings = {key: value for key, value in self.settings().items() if key in SETTINGS_KEYS}
        if settings.get('point_preview_enabled') is not True:
            self.stop_event.set()
            return
        signature = json.dumps(settings, sort_keys=True, separators=(',', ':'), allow_nan=False)
        if signature == self.signature:
            return
        with self.condition:
            self.pending.clear()
            self.previous.clear()
            self.errors.clear()
        self.signature = signature
        for camera, path in self.cameras:
            key = hashlib.sha256(json.dumps([self.owner, camera, str(path), self.fps, self.viewer, str(self.robot_path)],
                                            sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()
            with _registry_lock:
                worker = _registry.get(key)
                if worker is None:
                    worker = CameraWorker(camera, path, settings, self.fps, self.viewer, self.robot_path)
                    worker.key, worker.camera_id = key, camera['id']
                    _registry[key] = worker
                self.workers[camera['id']] = worker
                self.generations[camera['id']] = worker.configure(settings)
                worker.subscribe(self)

    def publish(self, worker, slot, version, generation, packet):
        if self.stop_event.is_set() or self.workers.get(worker.camera_id) is not worker:
            return
        if self.frame_lanes == 2 and not self.adaptive_delivery and self.frame_lane != slot:
            return
        with self.condition:
            if generation != self.generations.get(worker.camera_id):
                self.settings_dirty.set()
                self.condition.notify_all()
                return
            if version <= self.previous.get(worker.camera_id, 0):
                return
            self.previous[worker.camera_id] = version
            self.pending[worker.camera_id] = (worker, version, generation, packet) if self.adaptive_delivery else packet
            self.condition.notify_all()

    def take(self, timeout=1):
        self.delivery_waiting = True
        deadline = time.monotonic()+timeout
        while not self.stop_event.is_set():
            self.settings_dirty.clear()
            self.refresh()
            with self.condition:
                self.condition.wait_for(lambda: self.pending or self.settings_dirty.is_set() or self.stop_event.is_set()
                                        or len(self.errors) == len(self.cameras), max(0, deadline-time.monotonic()))
                if self.pending:
                    value = self.pending.popitem(last=False)[1]
                    if not self.adaptive_delivery:
                        self.delivery_waiting = False
                        return value
                    worker, version, generation, packet = value
                    if worker.claim(version, generation, self):
                        self.delivery_waiting = False
                        return packet
                    continue
                if len(self.errors) == len(self.cameras):
                    raise ConnectionError('preview worker stopped: '+str(self.errors))
                if time.monotonic() >= deadline:
                    break
        self.delivery_waiting = False
        return None

    def note_delivery_ack(self, seconds):
        sample = max(0.0, float(seconds) * 1000)
        # Back off immediately on congestion, then recover gradually.
        self.delivery_rtt_ms = sample if not self.delivery_rtt_ms else max(sample, self.delivery_rtt_ms * .9 + sample * .1)

    def note_delivery_window(self, oldest_sent):
        self.delivery_oldest_sent = oldest_sent

    def delivery_score(self, now):
        return max(self.delivery_rtt_ms, (now-self.delivery_oldest_sent)*1000 if self.delivery_oldest_sent is not None else 0)

    def close(self):
        with self.lifecycle_lock:
            self._close()

    def _close(self):
        self.delivery_waiting = False
        self.stop_event.set()
        for worker in list(self.workers.values()):
            release(worker, self)
        self.workers.clear()
        with self.condition:
            self.pending.clear()
            self.condition.notify_all()


def process_workers_supported():
    return (sys.platform == 'linux' and Path('/dev/shm').is_dir()
            and os.access('/dev/shm', os.W_OK)
            and os.environ.get('ROBOT_TELEOP_NATIVE_POINT_WORKERS', '1') != '0')


def create_native_sparse_session(cameras, settings, robot, fps=30, viewer=None, frame_lane=None, frame_lanes=1, owner='local', adaptive_delivery=False):
    arguments = dict(fps=fps, viewer=viewer, frame_lane=frame_lane, frame_lanes=frame_lanes)
    if process_workers_supported():
        return ProcessSparseSession(cameras, settings, robot, owner=owner, adaptive_delivery=adaptive_delivery, **arguments)
    from tools.native_sparse_stream import NativeSparseSession
    return NativeSparseSession(cameras, settings, robot, **arguments)


if __name__ == '__main__':
    if len(sys.argv) != 3 or sys.argv[1] != '--config':
        raise SystemExit('worker requires its preview config')
    encode_loop(json.loads(Path(sys.argv[2]).read_text()))
