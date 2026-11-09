"""Compare the optional host kernels against the retained NumPy reference."""
import zlib

import numpy as np
import pytest

from tools import workspace_stream_crop as crop
from tools import workspace_stream_crop_native as native


@pytest.mark.parametrize('shape', [(1, 1), (2, 3), (19, 43), (720, 1280)])
def test_predictive_depth_preserves_wraparound_and_every_sample(shape):
    if native.LIBRARY is None:
        pytest.skip('Build tools/workspace_stream_crop_native.py to test compiled kernels')
    rng = np.random.default_rng(7301)
    values = rng.integers(0, 65536, shape, dtype=np.uint16)
    previous = np.zeros_like(values)
    previous[1:] = values[:-1]
    delta = np.subtract(values, previous, dtype=np.uint16).view(np.int16).astype(np.int32)
    zigzag = ((delta << 1) ^ (delta >> 31)).astype(np.uint16)
    raw = (zigzag & 255).astype(np.uint8).tobytes() + (zigzag >> 8).astype(np.uint8).tobytes()
    np.testing.assert_array_equal(native.decode_predictive(raw, shape[1], shape[0]), values)


@pytest.mark.parametrize('shape', [(1, 1), (2, 3), (8, 16), (9, 17), (19, 43), (480, 848), (720, 1280)])
def test_native_clipping_and_halo_are_identical_to_fallback(monkeypatch, shape):
    if native.LIBRARY is None:
        pytest.skip('Build compiled kernels first')
    rng = np.random.default_rng(98)
    h, w = shape
    depth = rng.integers(0, 2000, shape, dtype=np.uint16)
    depth[rng.random(shape) < .1] = 0
    camera = {'intrinsics': [float(w), float(w), (w-1)/2, (h-1)/2],
              'transform': np.eye(4).flatten(order='F').tolist()}
    settings = {'workspace_stream_crop_enabled':True,'modeled_workspace_enabled':True, 'modeled_workspace_center':[.05,-.2,-1.1], 'modeled_workspace_width':.5,'modeled_workspace_depth':1.,'modeled_workspace_height':.45, 'modeled_workspace_heading':25,'modeled_workspace_clearance':0}
    volumes = crop.crop_volumes(settings)
    fast_bounds = crop._depth_clip_planes(shape, camera, volumes)
    fast = crop.retained_depth(depth, camera, volumes, fast_bounds)
    with monkeypatch.context() as context:
        context.setattr(native, 'LIBRARY', None)
        reference_bounds = crop._depth_clip_planes(shape, camera, volumes)
        reference = crop.retained_depth(depth, camera, volumes, reference_bounds)
    for a, b in zip(fast_bounds[0], reference_bounds[0]):
        active = (b[1] == 1) | (b[1] == 2)
        np.testing.assert_array_equal(a[0][active], b[0][active])
        np.testing.assert_array_equal(a[1], b[1])
    np.testing.assert_array_equal(fast, reference)


@pytest.mark.parametrize('shape', [(1,1),(3,1),(1,33),(8,16),(9,17),(25,33),(480,848)])
def test_combined_crop_preserves_generic_planes_empty_cells_and_tile_edges(shape):
    if native.LIBRARY is None:
        pytest.skip('Build compiled kernels first')
    rng=np.random.default_rng(773)
    for _ in range(8):
        depth=rng.integers(0,65536,shape,dtype=np.uint16)
        depth.ravel()[::7]=0
        depth.ravel()[::11]=65535
        original=depth.copy()
        planes=[(rng.integers(0,65536,shape,dtype=np.uint16),rng.integers(0,4,shape,dtype=np.uint8)) for _ in range(6)]
        bounds=[planes]
        expected=native.retain_cells(depth,native.crop_cells(depth,bounds))
        np.testing.assert_array_equal(native.retain_depth(depth,bounds),expected)
        np.testing.assert_array_equal(depth,original)
    assert native.retain_depth(depth,[planes,planes]) is None


def test_standard_deflate_transport_is_lossless_and_bounded():
    raw = np.random.default_rng(12).integers(0, 256, 100003, dtype=np.uint8).tobytes()
    packed = native.compress_depth(raw)
    assert zlib.decompress(packed) == raw
    if native.DEFLATE is not None:
        np.testing.assert_array_equal(native.decompress_depth(packed, len(raw)), np.frombuffer(raw, dtype=np.uint8))
        with pytest.raises(zlib.error):
            native.decompress_depth(packed, len(raw)-1)
        with pytest.raises(zlib.error):
            native.decompress_depth(packed[:-8], len(raw))
