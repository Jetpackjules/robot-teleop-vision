"""Authenticated WebUI bridge to the existing Godot rig calibration solver."""
from pathlib import Path
import hashlib
import json
import shutil
import threading
import time
import uuid


class CameraAlignment:
    def __init__(self, root, userdata):
        self.root = Path(root).resolve()
        self.userdata = Path(userdata)
        self.lock = threading.RLock()
        self.pending = None
        self.last_backup = None

    def status(self):
        try:
            value = json.loads((self.userdata / 'camera_alignment_web_status.json').read_text())
            fresh = (Path(value.get('project_root', '')).resolve() == self.root
                     and 0 <= time.time() - float(value.get('updated', 0)) < 5)
            if not fresh:
                raise ValueError('stale runtime')
        except (OSError, ValueError, TypeError):
            value = {'online': False, 'busy': False, 'message': 'Godot camera alignment is offline.'}
        else:
            value['online'] = True
        if self.pending:
            if value.get('request_id') == self.pending['request_id']:
                self.pending = None
            elif time.monotonic() - self.pending['sent'] < 6:
                value = {**value, 'busy': True, 'message': 'Waiting for Godot to acknowledge alignment...'}
            else:
                self.pending = None
                value = {**value, 'message': 'Godot did not acknowledge the request. No automatic retry was sent.'}
        return {**value, 'backup': self.last_backup}

    def backup(self):
        target = self.root / '.teleop' / 'camera-alignment-backups' / (time.strftime('%Y%m%dT%H%M%S') + '_' + uuid.uuid4().hex[:8])
        manifest = {}
        # Preserve the complete saved Godot configuration, including camera,
        # world placement, robot registration and visual corrections.
        files = [(p, Path('godot') / p.name) for p in self.userdata.glob('*.json')]
        files += [(p, Path('profiles') / p.name) for p in (self.root / 'robot_modules/so101/local').glob('*.json')]
        for source, relative in files:
            dest = target / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, dest)
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            if hashlib.sha256(dest.read_bytes()).hexdigest() != digest:
                raise OSError('Calibration backup verification failed')
            manifest[str(source)] = {'file': str(relative), 'sha256': digest}
        if not manifest:
            raise OSError('No saved calibration files found; alignment was not started')
        (target / 'manifest.json').write_text(json.dumps(manifest, indent=2))
        self.last_backup = str(target)
        return self.last_backup

    def start(self, mode, recording_state, robot, send):
        if mode not in ('markerless', 'refine'):
            raise ValueError('Choose Auto align or Refine current alignment.')
        with self.lock:
            status = self.status()
            if not status.get('online'):
                raise ValueError('Godot camera alignment is offline.')
            if status.get('busy'):
                raise ValueError('Camera alignment is already running.')
            if recording_state != 'idle':
                raise ValueError('Finish the current recording before aligning cameras.')
            if robot.get('armed') or robot.get('calibration_sweep_active') or robot.get('rest_return_active'):
                raise ValueError('Put the arm in Hold before aligning cameras.')
            if robot.get('follower_connected') and robot.get('state') != 'hold':
                raise ValueError('Put the arm in Hold before aligning cameras.')
            self.backup()
            request_id = uuid.uuid4().hex
            self.pending = {'request_id': request_id, 'sent': time.monotonic()}
            send(json.dumps({'type': 'camera_alignment', 'mode': mode, 'request_id': request_id,
                             'project_root': str(self.root) + '/', 'sent_unix_ms': time.time()*1000}).encode())
            return self.status()
