import json
import os
import struct
import sys
import threading
import time

import cv2
import numpy as np
import pytest

from test_native_sparse_stream import fixture
from tools.native_sparse_stream import NativeSnapshot, encode_native_camera
from tools.scoped_ir import CAMERA_SERIAL, FRAME_SHAPE, ScopedIrRecovery
from tools.workspace_stream_crop import WorkspaceStreamCropper


def _stereo_scene():
    rng = np.random.default_rng(12)
    left = rng.integers(0, 256, FRAME_SHAPE, dtype=np.uint8)
    disparity = 40
    right = np.zeros_like(left)
    right[:, :-disparity] = left[:, disparity:]
    depth = np.full(FRAME_SHAPE, 500, dtype=np.uint16)
    depth[166:212, 418:448] = 0
    return depth, left, right


def test_scoped_match_fills_missing_depth_and_keeps_sensor_samples():
    depth, left, right = _stereo_scene()
    untouched = depth.copy()
    # This tests only the stereo match. The packet test covers preview settings and scope.
    previous_threads = cv2.getNumThreads()
    cv2.setNumThreads(1)
    try:
        matcher = ScopedIrRecovery()
        result = matcher.recover(depth, depth, left, right, 20000)
        rejected = matcher.recover(depth, depth, left, np.zeros_like(right), 20000)
    finally:
        cv2.setNumThreads(previous_threads)
    assert result.millimetres.size > 500
    assert np.all(np.abs(result.millimetres.astype(int) - 500) < 4)
    assert np.all(depth[result.rows, result.columns] == 0)
    assert np.unique(result.rows * FRAME_SHAPE[1] + result.columns).size == result.millimetres.size
    np.testing.assert_array_equal(depth, untouched)
    assert rejected.millimetres.size == 0


def _native_pair_fixture():
    depth, left, right = _stereo_scene()
    h, w = FRAME_SHAPE
    data = bytearray(fixture(w, h))
    struct.pack_into('<I', data, 92, 2)
    data[256:256 + w*h*4] = (depth / 1000).astype('<f4').tobytes()
    return data + struct.pack('<f', 20000) + left.tobytes() + right.tobytes(), depth, left, right


def test_ir_tail_is_opt_in_and_only_b_uses_it(tmp_path):
    path = tmp_path / 'camera'
    data, depth, left, right = _native_pair_fixture()
    path.write_bytes(data)
    reader = NativeSnapshot(path)
    try:
        assert 'infrared' not in reader.read()
        snapshot = reader.read(include_ir=True)
        assert snapshot['infrared'][2] == 20000
        np.testing.assert_array_equal(snapshot['infrared'][0], left)
        np.testing.assert_array_equal(snapshot['infrared'][1], right)
    finally:
        reader.close()

    cropper = WorkspaceStreamCropper()
    settings = {'point_preview_budget': 0, 'scoped_ir_enabled': True}

    def metadata(camera_id, enabled):
        packet = encode_native_camera(snapshot, {'id': camera_id},
                                      {**settings, 'scoped_ir_enabled': enabled}, cropper)
        end = 9 + struct.unpack_from('<I', packet, 5)[0]
        return json.loads(packet[9:end])['cameras'][0]

    other = metadata('262522076048', True)
    off = metadata(CAMERA_SERIAL, False)
    on = metadata(CAMERA_SERIAL, True)
    assert other['preview_count'] == off['preview_count']
    assert 'scoped_ir_status' not in off and 'scoped_ir_status' not in other
    assert on['scoped_ir_status'] == 'active'
    assert on['preview_count'] == off['preview_count'] + on['scoped_ir_points']
    assert on['scoped_ir_points'] > 500 and not on['depth_exact']
    np.testing.assert_array_equal(snapshot['depth'], depth.astype(np.float32) / 1000)


def test_truncated_ir_tail_degrades_to_original_depth(tmp_path):
    h, w = FRAME_SHAPE
    data = bytearray(fixture(w, h))
    struct.pack_into('<I', data, 92, 2)
    path = tmp_path / 'camera'
    path.write_bytes(data)
    reader = NativeSnapshot(path)
    try:
        snapshot = reader.read(include_ir=True)
    finally:
        reader.close()
    assert 'infrared' not in snapshot
    assert snapshot['depth'].shape == (h, w)


@pytest.mark.skipif(sys.platform != 'linux', reason='Linux native preview worker')
def test_scoped_worker_switches_in_place_and_publishes_in_order(tmp_path):
    from tools.native_sparse_workers import ProcessSparseSession

    path = tmp_path / 'camera'
    data, *_ = _native_pair_fixture()
    path.write_bytes(data)
    stop = threading.Event()

    def publish():
        with path.open('r+b') as source:
            sequence = 2
            while not stop.wait(.012):
                sequence += 2
                os.pwrite(source.fileno(), struct.pack('<Q', sequence - 1), 8)
                os.pwrite(source.fileno(), struct.pack('<Q', sequence//2), 16)
                os.pwrite(source.fileno(), struct.pack('<d', time.time()*1000), 72)
                os.pwrite(source.fileno(), struct.pack('<Q', sequence), 8)

    settings = {'point_preview_enabled': True, 'point_preview_budget': 20000}
    thread = threading.Thread(target=publish)
    thread.start()
    session = ProcessSparseSession(
        [({'id': CAMERA_SERIAL}, path)], lambda: settings, lambda: {}, fps=60,
        robot_path=tmp_path/'no-robot', owner=f'ir-test-{os.getpid()}',
    )

    def frames(enabled, count):
        sequence = []
        deadline = time.monotonic() + 8
        while len(sequence) < count and time.monotonic() < deadline:
            packet = session.take(.25)
            if packet is None:
                continue
            end = 9 + struct.unpack_from('<I', packet, 5)[0]
            camera = json.loads(packet[9:end])['cameras'][0]
            if (camera.get('scoped_ir_status') == 'active') == enabled:
                sequence.append(int(camera['preview_publication']))
        assert len(sequence) == count
        return sequence

    try:
        session.start()
        process = session.workers[CAMERA_SERIAL].process
        before = frames(False, 2)
        settings = {**settings, 'scoped_ir_enabled': True}
        recovered = frames(True, 8)
        settings = {**settings, 'scoped_ir_enabled': False}
        after = frames(False, 2)
        sequence = before + recovered + after
        assert sequence == sorted(set(sequence))
        assert session.workers[CAMERA_SERIAL].process is process
    finally:
        session.close()
        stop.set()
        thread.join()
