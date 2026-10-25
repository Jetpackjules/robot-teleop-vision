"""Export completed host episodes with the official LeRobotDataset writer.

Run this in the optional recording/export environment; capture needs no Torch.
No existing dataset is overwritten and no data is uploaded to a Hub account.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re

import av
import numpy as np

from robot_teleop.recording import RecordingError, write_json


def _image_path(root: Path, value: str) -> Path:
    path = (root/str(value)).resolve()
    if not path.is_relative_to(root.resolve()) or path.suffix != '.jpg' or not path.is_file():
        raise RecordingError('An episode contains an invalid image path.')
    return path


def inspect_episode(root: Path) -> tuple[dict, dict[str, np.ndarray]]:
    manifest = json.loads((root/'episode.json').read_text(encoding='utf-8'))
    if manifest.get('format') != 'robot-teleop-episode/v1' or manifest.get('status') != 'complete':
        raise RecordingError('Only complete episodes can be exported.')
    with np.load(root/'episode.npz', allow_pickle=False) as data:
        arrays = {name: data[name] for name in data.files}
    count = manifest.get('frames', 0)
    dimensions = len(manifest['robot']['names'])
    cameras = manifest['cameras']
    if type(count) is not int or not 2 <= count <= 100_000 or not 1 <= dimensions <= 64 or not 1 <= len(cameras) <= 8:
        raise RecordingError('Invalid episode dimensions.')
    for key in ('obs.state', 'action'):
        if arrays[key].shape != (count, dimensions) or not np.all(np.isfinite(arrays[key])):
            raise RecordingError('Invalid measured joint/action arrays.')
    times = arrays['timestamp']
    if times.shape != (count,) or not np.all(np.isfinite(times)) or np.any(np.diff(times) < 0):
        raise RecordingError('Capture timestamps are invalid or run backwards.')
    if times[-1]-times[0] < .001:
        raise RecordingError('The episode contains no advancing camera frames.')
    for camera in cameras:
        if not re.fullmatch(r'camera_[1-8]', camera['key']) or not 0 < camera['width']*camera['height'] <= 16*1024*1024:
            raise RecordingError('Invalid camera feature description.')
        paths = arrays['obs.images.'+camera['key']]
        if paths.shape != (count,):
            raise RecordingError('Image and robot sample counts differ.')
        for name in paths:
            _image_path(root, name)
    for key, shape in [('obs.capture_unix_ns', (count, len(cameras))),
                       ('obs.robot_measured_ns', (count,)), ('action_applied_ns', (count,)),
                       ('camera_sequence', (count, len(cameras)))]:
        if arrays[key].shape != shape or arrays[key].dtype.kind not in 'iu':
            raise RecordingError('Invalid source timing arrays.')
    return manifest, arrays


def resample_indices(timestamps: np.ndarray, fps: int) -> np.ndarray:
    times = timestamps-timestamps[0]
    target = np.arange(int(math.floor(times[-1]*fps))+1, dtype=np.float64)/fps
    if len(target) < 2:
        raise RecordingError('The episode is too short for the export frame rate.')
    right = np.searchsorted(times, target).clip(0, len(times)-1)
    left = np.maximum(0, right-1)
    selected = np.where(abs(times[left]-target) <= abs(times[right]-target), left, right)
    if np.max(abs(times[selected]-target)) > .1:
        raise RecordingError('A capture gap exceeds the 100-ms export alignment limit.')
    return selected


def export_episodes(source: Path, output: Path, repo_id: str = 'local/robot-teleop',
                    outcome: str = 'success', dataset_class=None) -> dict:
    source, output = Path(source).resolve(), Path(output).resolve()
    if output.exists():
        raise RecordingError('The output directory already exists; choose a new dataset path.')
    if not re.fullmatch(r'[A-Za-z0-9_-]+/[A-Za-z0-9_.-]+', repo_id):
        raise RecordingError('Use a local namespace/name dataset ID.')
    if outcome not in ('success', 'all'):
        raise RecordingError('Export outcome must be success or all.')
    candidates = [source/'episode.json'] if (source/'episode.json').is_file() else sorted(source.glob('*/episode.json'))
    selected = []
    for path in candidates:
        metadata = json.loads(path.read_text(encoding='utf-8'))
        if metadata.get('status') == 'complete' and (outcome == 'all' or metadata.get('outcome') == 'success'):
            selected.append(path.parent)
    if not selected:
        raise RecordingError('No matching complete episodes. Label successes or explicitly export --outcome all.')
    # Validate every episode before creating an output dataset. Keep images on
    # disk and load numeric arrays one episode at a time during conversion.
    first, _ = inspect_episode(selected[0])
    signature = lambda m: (m['robot'], m['cameras'], m['fps'], bool(m.get('synthetic', False)))
    for root in selected:
        manifest, arrays = inspect_episode(root)
        if signature(manifest) != signature(first):
            raise RecordingError('Camera identities, image sizes, robot schema or capture rate differ between episodes.')
        resample_indices(arrays['timestamp'], first['fps'])
    if dataset_class is None:
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as exc:
            raise RecordingError('Install the recording extra in an export environment: pip install -e ".[recording]"') from exc
        dataset_class = LeRobotDataset
    names, cameras = first['robot']['names'], first['cameras']
    features = {'observation.state': {'dtype': 'float32', 'shape': (len(names),), 'names': names},
                'action': {'dtype': 'float32', 'shape': (len(names),), 'names': names},
                'observation.capture_time_ns': {'dtype': 'int64', 'shape': (len(cameras),), 'names': [c['key'] for c in cameras]},
                'observation.robot_time_ns': {'dtype': 'int64', 'shape': (1,), 'names': ['host_monotonic_ns']},
                'observation.action_time_ns': {'dtype': 'int64', 'shape': (1,), 'names': ['host_monotonic_ns']},
                'annotation.outcome': {'dtype': 'int64', 'shape': (1,), 'names': ['failure_0_success_1_unrated_minus1']}}
    for camera in cameras:
        features['observation.images.'+camera['key']] = {'dtype': 'image',
            'shape': (camera['height'], camera['width'], 3), 'names': ['height', 'width', 'channels']}
    dataset = dataset_class.create(repo_id=repo_id, root=output, fps=first['fps'], features=features,
                                   robot_type=first['robot']['robot_type'], use_videos=False)
    summary = {'format': 'LeRobotDataset', 'repo_id': repo_id, 'robot': first['robot'], 'episodes': [],
               'synthetic': bool(first.get('synthetic', False)),
               'timing': 'Uniform export grid resampled to nearest captured RGB/state sample within 100 ms; original source timestamps retained.',
               'images': 'RGB JPEG decoded to lossless LeRobot image features at captured resolution.', 'uploaded': False}
    try:
        for root in selected:
            manifest, arrays = inspect_episode(root)
            indices = resample_indices(arrays['timestamp'], first['fps'])
            decoders = [av.CodecContext.create('mjpeg', 'r') for _ in cameras]
            for index in indices:
                frame = {'observation.state': arrays['obs.state'][index].astype(np.float32),
                         'action': arrays['action'][index].astype(np.float32),
                         'observation.capture_time_ns': arrays['obs.capture_unix_ns'][index].astype(np.int64),
                         'observation.robot_time_ns': np.array([arrays['obs.robot_measured_ns'][index]], dtype=np.int64),
                         'observation.action_time_ns': np.array([arrays['action_applied_ns'][index]], dtype=np.int64),
                         'annotation.outcome': np.array([{'success': 1, 'failure': 0}.get(manifest['outcome'], -1)], dtype=np.int64),
                         'task': manifest['task']}
                for camera, decoder in zip(cameras, decoders, strict=True):
                    key = camera['key']
                    jpeg = _image_path(root, arrays['obs.images.'+key][index]).read_bytes()
                    images = decoder.decode(av.Packet(jpeg))
                    if not images or (images[-1].width, images[-1].height) != (camera['width'], camera['height']):
                        raise RecordingError('An encoded image does not match the recorded camera dimensions.')
                    frame['observation.images.'+key] = images[-1].to_ndarray(format='rgb24')
                dataset.add_frame(frame)
            dataset.save_episode()
            summary['episodes'].append({'source_episode': manifest['episode_id'], 'frames': len(indices),
                                        'source_frames': manifest['frames'], 'outcome': manifest['outcome'],
                                        'trial': manifest.get('trial')})
    except BaseException:
        write_json(output/'capture_export_incomplete.json', summary)
        raise
    finally:
        dataset.finalize()
        if hasattr(dataset, 'stop_image_writer'):
            dataset.stop_image_writer()
    write_json(output/'capture_export.json', summary)
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('.teleop/recordings'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repo-id', default='local/robot-teleop')
    parser.add_argument('--outcome', choices=('success', 'all'), default='success')
    args = parser.parse_args(argv)
    try:
        result = export_episodes(args.source, args.output, args.repo_id, args.outcome)
    except (RecordingError, OSError, ValueError, KeyError) as exc:
        parser.exit(1, f'Export failed: {exc}\n')
    print(json.dumps({'output': str(args.output), 'episodes': len(result['episodes']), 'uploaded': False}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
