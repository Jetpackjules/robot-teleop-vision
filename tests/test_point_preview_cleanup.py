import numpy as np
import pytest

from tools.point_preview_cleanup import clean_preview_depth


def test_off_is_exact_and_source_is_never_modified():
    depth = np.full((12, 12), 900, dtype=np.uint16)
    depth[4, 5] = 1300
    original = depth.copy()
    assert clean_preview_depth(depth, {}) is depth
    result = clean_preview_depth(depth, {'point_cleanup_support': 2})
    assert result[4, 5] == 0
    np.testing.assert_array_equal(depth, original)
    assert np.all((result == 0) | (result == depth))


def test_plane_and_real_two_pixel_wide_edge_survive_mild_cleanup():
    depth = np.full((12, 12), 900, dtype=np.uint16)
    np.testing.assert_array_equal(clean_preview_depth(depth, {'point_cleanup_support': 2}), depth)
    depth[:] = 0
    depth[2:10, 5:7] = 900
    np.testing.assert_array_equal(clean_preview_depth(depth, {'point_cleanup_support': 2}), depth)


def test_disconnected_pixels_and_depth_outliers_are_removed_without_bridging():
    depth = np.zeros((12, 12), dtype=np.uint16)
    depth[2, 2] = 900
    depth[5:10, 5:10] = 900
    depth[7, 7] = 940
    result = clean_preview_depth(depth, {'point_cleanup_support': 2, 'point_cleanup_tolerance_mm': 15})
    assert result[2, 2] == result[7, 7] == 0
    assert result[6, 6] == 900
    assert clean_preview_depth(depth, {'point_cleanup_support': 2, 'point_cleanup_tolerance_mm': 50})[7, 7] == 940


@pytest.mark.parametrize('value', [-1, 1.5, 9, True, float('nan')])
def test_support_validation(value):
    with pytest.raises(ValueError):
        clean_preview_depth(np.zeros((3, 3), np.uint16), {'point_cleanup_support': value})


def test_native_cleanup_matches_reference_at_borders_and_depth_extremes(monkeypatch):
    import tools.point_preview_cleanup as cleanup
    from tools.native_sparse_kernels import LIBRARY
    if LIBRARY is None:
        pytest.skip('Optional native kernels unavailable')
    rng = np.random.default_rng(15)
    depth = rng.choice(np.array([0, 1, 900, 910, 940, 65535], np.uint16), (37, 49))
    native = cleanup.clean_depth
    for support in (1, 2, 3, 8):
        settings = {'point_cleanup_support': support, 'point_cleanup_tolerance_mm': 15.5}
        monkeypatch.setattr(cleanup, 'clean_depth', native)
        actual = cleanup.clean_preview_depth(depth, settings)
        monkeypatch.setattr(cleanup, 'clean_depth', lambda *args: None)
        np.testing.assert_array_equal(actual, cleanup.clean_preview_depth(depth, settings))
