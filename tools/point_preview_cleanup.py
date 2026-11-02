"""Preview-only depth support filter; never changes capture/recording buffers."""
from __future__ import annotations

import numpy as np

from tools.native_sparse_kernels import clean_depth


def clean_preview_depth(depth, settings):
    """Remove unsupported pixels without averaging, moving or inventing points.

    Support is counted in the eight neighbouring source depth pixels, before
    workspace cropping or point sampling. Zero disables filtering exactly.
    """
    support = settings.get('point_cleanup_support', 0)
    tolerance = settings.get('point_cleanup_tolerance_mm', 15)
    if (type(support) not in (int, float) or not np.isfinite(support)
            or support != int(support) or not 0 <= support <= 8):
        raise ValueError('point cleanup support must be an integer from 0 to 8')
    if (type(tolerance) not in (int, float) or not np.isfinite(tolerance)
            or not 1 <= tolerance <= 100):
        raise ValueError('point cleanup tolerance must be 1 to 100 mm')
    if not support:
        return depth
    native = clean_depth(depth, support, tolerance)
    if native is not None:
        return native
    values = depth.astype(np.int32)
    neighbours = np.zeros(depth.shape, dtype=np.uint8)
    height, width = depth.shape
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if not (dx or dy):
                continue
            target = (slice(max(0, dy), min(height, height+dy)),
                      slice(max(0, dx), min(width, width+dx)))
            source = (slice(max(0, -dy), min(height, height-dy)),
                      slice(max(0, -dx), min(width, width-dx)))
            other = values[source]
            neighbours[target] += ((other > 0)
                                   & (np.abs(values[target]-other) <= tolerance))
    return np.where(neighbours >= support, depth, 0).astype(depth.dtype)
