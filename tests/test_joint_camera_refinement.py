import json
import struct
import sys
import types
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robot_teleop.joint_camera_refinement import (
    FLIP,
    Camera,
    capture_sets,
    connected,
    matrix,
    metrics,
    no_regression,
    project,
    refine,
    score,
    select,
    transform,
    updated,
)


def scene(count=3):
    rng = np.random.default_rng(1421)
    cameras = [
        SimpleNamespace(k=np.array([1300.0, 1300.0, 960.0, 540.0]), pixel_scale=1.0)
        for _ in range(count)
    ]
    true = [np.eye(4) for _ in range(count)]
    for i in range(1, count):
        true[i][:3, :3] = Rotation.from_euler("xyz", [0.01 * i, -0.12 * i, 0.02 * i]).as_matrix()
        true[i][:3, 3] = [0.18 * i, 0.025 * i, 0.01 * i]
    points = rng.uniform([-0.2, -0.12, 0.65], [0.25, 0.2, 1.15], (180, 3))
    local = [transform(np.linalg.inv(pose), points) for pose in true]
    pairs = []
    for i in range(count):
        for j in range(i + 1, count):
            pairs.append(
                {
                    "i": i,
                    "j": j,
                    "pa": local[i],
                    "pb": local[j],
                    "ua": project(cameras[i].k, local[i]),
                    "ub": project(cameras[j].k, local[j]),
                    "scores": np.ones(len(points)),
                }
            )
    delta = np.tile([0.004, -0.005, 0.003, 0.006, -0.004, 0.003], count - 1)
    return cameras, true, updated(true, delta), pairs


@pytest.mark.parametrize("count", [2, 3, 4])
def test_joint_solver_recovers_all_camera_poses_and_fixes_reference(count):
    cameras, true, initial, pairs = scene(count)
    # Disjoint image correspondences reserve a genuine held-out validation set.
    train, validation = [], []
    for pair in pairs:
        train.append({k: v[::2] if isinstance(v, np.ndarray) else v for k, v in pair.items()})
        validation.append({k: v[1::2] if isinstance(v, np.ndarray) else v for k, v in pair.items()})
    solved = refine(cameras, initial, train)
    assert np.array_equal(solved[0], initial[0])
    for expected, actual in zip(true, solved):
        np.testing.assert_allclose(actual, expected, atol=2e-5)
    before, after = metrics(cameras, initial, validation), metrics(cameras, solved, validation)
    assert score(after) < score(before) / 100
    assert no_regression(after, before)
    assert select(cameras, initial, solved, None, validation)[0] == "refined"


def test_saved_better_alignment_cannot_be_replaced_by_new_coarse_fit():
    cameras, true, initial, pairs = scene()
    selected, poses, _ = select(cameras, initial, initial, true, pairs)
    assert selected == "previous"
    assert poses is true


def test_fresh_frame_validation_rejects_candidate_that_only_fits_training():
    cameras, true, initial, pairs = scene()
    bad = updated(initial, np.tile([0.0, 0.015, 0.0, 0.012, 0.0, 0.0], 2))
    assert select(cameras, initial, bad, true, pairs)[0] == "previous"


def test_missing_overlap_must_not_partially_refine_rig():
    cameras, _, initial, pairs = scene(4)
    pairs = [p for p in pairs if p["i"] != 3 and p["j"] != 3]
    assert not connected(4, pairs)
    with pytest.raises(ValueError, match="every enabled camera"):
        refine(cameras, initial, pairs)
    assert connected(4, [{"i": 0, "j": 1}, {"i": 1, "j": 2}, {"i": 2, "j": 3}])


def test_native_rgb_depth_extrinsics_and_godot_world_roundtrip():
    _, world, _, _ = scene()
    e = np.eye(4)
    e[:3, :3] = Rotation.from_euler("xyz", [0.002, -0.004, 0.003]).as_matrix()
    e[:3, 3] = [0.015, 0.0002, -0.0001]
    reference = world[0] @ FLIP @ np.linalg.inv(e)
    for pose in world:
        color = np.linalg.inv(reference) @ pose @ FLIP @ np.linalg.inv(e)
        np.testing.assert_allclose(reference @ color @ e @ FLIP, pose, atol=1e-12)


def test_local_depth_plane_lifts_rgb_features_without_pixel_quantization():
    metadata = {"intrinsics": [150.0, 150.0, 80.0, 60.0], "color_space": "aligned_depth"}
    camera = Camera(
        metadata, np.zeros((120, 160, 3), np.uint8), np.full((120, 160), 700, np.uint16)
    )
    pixels = np.array([[60.25, 55.75], [99.8, 72.4]])
    lifted, errors = camera.lift(pixels)
    np.testing.assert_allclose(project(camera.k, lifted), pixels, atol=1e-8)
    np.testing.assert_allclose(lifted[:, 2], 0.7, atol=1e-6)
    assert np.max(errors) < 1e-6
    points, errors = camera.lift(np.empty((0, 2)))
    assert points.shape == (0, 3) and errors.shape == (0,)


def test_rejects_scaled_or_nonfinite_world_poses():
    with pytest.raises(ValueError, match="rigid camera"):
        matrix({"R": (np.eye(3) * 2).tolist(), "T": [0, 0, 0]})
    with pytest.raises(ValueError, match="rigid camera"):
        matrix({"R": np.eye(3).tolist(), "T": [0, 0, float("nan")]})


def test_capture_requires_new_frames_from_every_camera_for_both_datasets(monkeypatch, tmp_path):
    color = cv2.imencode(".jpg", np.zeros((2, 2, 3), np.uint8))[1].tobytes()
    depth = np.full((2, 2), 700, "<u2").tobytes()

    def packet(updates):
        metadata = {
            "cameras": [
                {
                    "id": serial,
                    "sequence": sequence,
                    "width": 2,
                    "height": 2,
                    "color_length": len(color),
                    "depth_length": len(depth),
                }
                for serial, sequence in updates
            ]
        }
        header = json.dumps(metadata).encode()
        return b"RGBD1" + struct.pack("<I", len(header)) + header + (color + depth) * len(updates)

    packets = iter(
        [
            packet([("A", 10), ("B", 10)]),
            packet([("A", 11)]),
            packet([("B", 11)]),
            packet([("A", 12)]),
            packet([("B", 12)]),
        ]
    )

    class Reader:
        closed = False

        def __init__(self, url):
            assert url == "http://127.0.0.1:8781/rgbd"

        def read_part(self):
            return {}, next(packets)

        def close(self):
            Reader.closed = True

    gateway = types.ModuleType("tools.lan_remote_view_server")
    gateway.MultipartRawFrameReader = Reader
    cropper = types.ModuleType("tools.workspace_stream_crop")
    cropper._decode_depth = lambda raw, camera: np.frombuffer(raw, "<u2").reshape(2, 2)
    monkeypatch.setitem(sys.modules, "tools.lan_remote_view_server", gateway)
    monkeypatch.setitem(sys.modules, "tools.workspace_stream_crop", cropper)
    frames = capture_sets("http://127.0.0.1:8781/rgbd", ["A", "B"], tmp_path)
    assert [[camera[0]["sequence"] for camera in frame] for frame in frames] == [[11, 11], [12, 12]]
    assert Reader.closed
