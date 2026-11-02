"""Optional cleanup of selected preview points, never capture/recording buffers."""
from __future__ import annotations

import math
import numpy as np

from tools.native_sparse_kernels import selected_point_support


def _reference_support(indices, depths, width, intrinsics, support, radius):
    """Spatial hash reference, also used without the optional native kernel."""
    y, x = np.divmod(indices, width)
    fx, fy, cx, cy = intrinsics
    points = np.column_stack(((x.astype(float)-cx)/fx*depths,
                              (y.astype(float)-cy)/fy*depths, depths))
    cells = np.floor(points/radius).astype(np.int64)
    groups = {}
    for i, cell in enumerate(cells):
        groups.setdefault(tuple(cell), []).append(i)
    keep = np.zeros(len(indices), bool)
    for i, (point, cell) in enumerate(zip(points, cells)):
        neighbours = 0
        for dz in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    for j in groups.get(tuple(cell+(dx, dy, dz)), ()):
                        if i != j and np.dot(point-points[j], point-points[j]) <= radius*radius:
                            neighbours += 1
                            if neighbours >= support: break
                    if neighbours >= support: break
                if neighbours >= support: break
            if neighbours >= support: break
        keep[i] = neighbours >= support
    return keep


def clean_selected_points(raw, count, width, intrinsics, settings):
    """Crop -> select 20k/40k/all -> support filter -> compression.

    Off returns the original bytes without decoding, copying or neighbour work.
    Retained pixel indices, depth and RGB bytes stay unchanged and in source order.
    """
    if settings.get('point_cleanup_enabled') is not True:
        return raw, count
    support = settings.get('point_cleanup_support', 3)
    radius = settings.get('point_cleanup_radius_mm', 15)
    if type(support) is not int or not 1 <= support <= 16:
        raise ValueError('point cleanup support must be an integer from 1 to 16')
    if type(radius) not in (int, float) or not math.isfinite(radius) or not 1 <= radius <= 100:
        raise ValueError('point cleanup radius must be 1 to 100 mm')
    if len(raw) != count*9 or width < 1 or len(intrinsics) != 4:
        raise ValueError('invalid selected point block')
    if not all(math.isfinite(k) for k in intrinsics) or min(intrinsics[:2]) <= 0:
        raise ValueError('invalid point intrinsics')
    indices = np.frombuffer(raw, dtype='<u4', count=count)
    depths = np.frombuffer(raw, dtype='<u2', count=count, offset=count*4)
    keep = selected_point_support(indices, depths, width, intrinsics, support, radius)
    if keep is None:
        keep = _reference_support(indices, depths, width, intrinsics, support, radius)
    kept = int(np.count_nonzero(keep))
    if kept == count:
        return raw, count
    colors = np.frombuffer(raw, dtype=np.uint8, offset=count*6).reshape(count, 3)
    return indices[keep].tobytes()+depths[keep].tobytes()+colors[keep].tobytes(), kept
