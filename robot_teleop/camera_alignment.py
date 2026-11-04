"""Authenticated WebUI bridge to the existing Godot rig calibration solver."""
from pathlib import Path
import math
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

    def start(self, mode, recording_state, robot, send, marker_size_mm=None, marker_id=49, board=None):
        if mode not in ('markerless', 'refine', 'aruco', 'charuco'):
            raise ValueError('Choose Auto align or Refine current alignment.')
        if mode == 'aruco':
            if type(marker_size_mm) not in (int, float) or not math.isfinite(marker_size_mm) or not 20 <= marker_size_mm <= 1000:
                raise ValueError('ArUco black square size must be 20–1000 mm.')
            if type(marker_id) is not int or not 0 <= marker_id <= 99:
                raise ValueError('ArUco DICT_4X4_100 marker ID must be 0–99.')
        if mode == 'charuco':
            if not isinstance(board, dict) or set(board) != {'squares_x','squares_y','square_mm','marker_mm','dictionary'}:
                raise ValueError('Enter the complete ChArUco board specifications.')
            for key in ['squares_x','squares_y']:
                if type(board[key]) is not int or not 3 <= board[key] <= 40:
                    raise ValueError('ChArUco square counts must be integers from 3 to 40.')
            for key in ['square_mm','marker_mm']:
                if type(board[key]) not in (int,float) or not math.isfinite(board[key]) or not .5 <= board[key] <= 1000:
                    raise ValueError('Enter physical ChArUco sizes in mm.')
            if board['square_mm'] < 1 or board['marker_mm'] >= board['square_mm']:
                raise ValueError('Each marker must be smaller than its checker square.')
            if type(board['dictionary']) is not int or not 0 <= board['dictionary'] <= 15:
                raise ValueError('Select the ChArUco marker dictionary.')
            capacity = [50,100,250,1000][board['dictionary'] % 4]
            if board['squares_x'] * board['squares_y'] // 2 > capacity:
                raise ValueError('This dictionary has too few markers for this board.')
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
                             'marker_size_mm': marker_size_mm, 'marker_id': marker_id, 'charuco_board': board,
                             'project_root': str(self.root) + '/', 'sent_unix_ms': time.time()*1000}).encode())
            return self.status()
