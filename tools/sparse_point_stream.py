"""Per-viewer host sampling; capture/recording packets are never modified.

Nine bytes per selected native sample: pixel index u32, depth mm u16, RGB8.
The native index/depth avoids an additional XYZ quantization step.
"""
import copy
import io
import json
import struct
import zlib
from functools import lru_cache
from concurrent.futures import ThreadPoolExecutor

import av
import numpy as np

from tools.point_preview_cleanup import clean_selected_points
from tools.workspace_stream_crop import _decode_depth, crop_volumes

ENCODING = "u16_mm_sparse_points_deflate"
_WORKERS = ThreadPoolExecutor(max_workers=3, thread_name_prefix="sparse-preview")


def _colors(data, camera, indices, depths):
    with av.open(io.BytesIO(data)) as container:
        rgb = next(container.decode(video=0)).to_ndarray(format="rgb24")
    return sample_rgb_pixels(rgb, camera, indices, depths)


def sample_rgb_pixels(rgb, camera, indices, depths):
    height, width = rgb.shape[:2]
    y, x = np.divmod(indices, int(camera["width"]))
    projection = camera.get("color_projection")
    if camera.get("color_space") != "original_rgb":
        if (height, width) != (int(camera["height"]), int(camera["width"])):
            raise ValueError("aligned sparse color dimensions do not match depth")
        result = rgb[y, x].copy()
        # Preserve the native preview's gray outside calibrated RGB coverage.
        if not projection:
            return result
    fx, fy, cx, cy = camera["intrinsics"]
    z = depths.astype(np.float64) * .001
    xyz = np.column_stack(((x-cx)*z/fx, (y-cy)*z/fy, z))
    e = np.asarray(projection["extrinsics"], dtype=np.float64)
    projected = xyz @ e[:9].reshape(3, 3, order="F").T + e[9:]
    pfx, pfy, pcx, pcy = projection["intrinsics"]
    safe_z = np.where(projected[:, 2] > 0, projected[:, 2], 1)
    u = projected[:, 0] / safe_z * pfx + pcx
    v = projected[:, 1] / safe_z * pfy + pcy
    pw, ph = projection["size"]
    valid = (projected[:, 2] > 0) & (u >= 0) & (v >= 0) & (u < pw-1) & (v < ph-1)
    if camera.get("color_space") != "original_rgb":
        result[~valid] = 140
        return result
    if (width, height) != (pw, ph):
        raise ValueError("sparse original RGB calibration dimensions mismatch")
    result = np.full((len(indices), 3), 140, dtype=np.uint8)
    a, b = np.floor(u[valid]).astype(int), np.floor(v[valid]).astype(int)
    du, dv = (u[valid]-a)[:, None], (v[valid]-b)[:, None]
    # Same calibrated bilinear lookup as the browser point shader, before RGB8 packing.
    values = (rgb[b, a]*(1-du)*(1-dv) + rgb[b, a+1]*du*(1-dv)
              + rgb[b+1, a]*(1-du)*dv + rgb[b+1, a+1]*du*dv)
    result[valid] = np.clip(np.rint(values), 0, 255).astype(np.uint8)
    return result


def _encode_camera(camera, color_bytes, depth_bytes, budget, cropper=None, volumes=None, cleanup=None,
                   *, visible_only=False):
    if cropper is None:
        return _encode_camera_uncached(camera, color_bytes, depth_bytes, budget, cropper, volumes, cleanup)
    geometry = json.dumps({k:camera.get(k) for k in ('width','height','intrinsics','transform','depth_encoding','color_encoding','color_space','color_projection','color_width','color_height')},sort_keys=True)
    volume_key = tuple((tuple(center),tuple(inverse.flat),tuple(half)) for center,inverse,half in (volumes or []))
    key = (budget,geometry,volume_key,visible_only,json.dumps(cleanup or {},sort_keys=True),color_bytes,depth_bytes)
    identity = str(camera.get('id',''))
    with cropper.cache_lock:
        cache = getattr(cropper,'sparse_camera_cache',None)
        if cache is None:
            cache = cropper.sparse_camera_cache = {}
        previous = cache.get(identity)
    if previous is not None and previous[0] == key:
        camera.update(previous[2])
        camera.pop('color_projection',None)
        return previous[1]
    encoded = _encode_camera_uncached(camera, color_bytes, depth_bytes, budget, cropper, volumes,
                                      visible_only=visible_only)
    updates = {k:camera[k] for k in ('depth_encoding','color_encoding','color_space','color_length','depth_length','color_width','color_height','preview_budget','preview_count','preview_source_points','depth_exact','preview_cleanup_enabled','preview_cleanup_kind','preview_cleanup_support','preview_cleanup_radius_mm','preview_cleanup_input_points','preview_cleanup_removed')}
    with cropper.cache_lock:
        cache[identity] = (key,encoded,updates)
        while len(cache)>32:
            del cache[next(iter(cache))]
    return encoded


def _encode_camera_uncached(camera, color_bytes, depth_bytes, budget, cropper=None, volumes=None, cleanup=None,
                            *, visible_only=False):
    depth = _decode_depth(depth_bytes, camera)
    if volumes:
        depth = cropper.point_depth(depth, camera, volumes, visible_only=visible_only)
    depth = depth.ravel()
    valid = np.flatnonzero(depth)
    total = len(valid)
    # Stable stratified sampling, matching the current browser preview.
    if budget and total > budget:
        valid = valid[np.floor((np.arange(budget)+.5)*total/budget).astype(np.int64)]
    values = depth[valid]
    colors = _colors(color_bytes, camera, valid, values)
    packed = valid.astype("<u4").tobytes() + values.astype("<u2").tobytes() + colors.tobytes()
    selected_count=len(valid)
    packed,count=clean_selected_points(packed,selected_count,camera['width'],camera['intrinsics'],cleanup or {})
    camera.update(preview_cleanup_enabled=(cleanup or {}).get('point_cleanup_enabled') is True,
                  preview_cleanup_kind='selected_radius_v1',
                  preview_cleanup_support=(cleanup or {}).get('point_cleanup_support',3),
                  preview_cleanup_radius_mm=(cleanup or {}).get('point_cleanup_radius_mm',15),
                  preview_cleanup_input_points=selected_count,preview_cleanup_removed=selected_count-count)
    encoded = zlib.compress(packed, 1)
    camera.update(depth_encoding=ENCODING, color_encoding="sparse_rgb8",
                  color_space="aligned_depth", color_length=0, depth_length=len(encoded),
                  color_width=camera["width"], color_height=camera["height"],
                  preview_budget=budget, preview_count=count, preview_source_points=total,
                  depth_exact=True)
    camera.pop("color_projection", None)
    return encoded


@lru_cache(maxsize=6)
def sparse_point_packet(packet, budget):
    return _point_packet(packet, budget)


def sparse_point_packet_for_view(packet, budget, cropper, settings):
    # Sample directly from retained native depth. Never encode a temporary
    # full RGB/depth image just to decode it again for the point stream.
    try:
        volumes = crop_volumes(settings)
    except (ValueError, TypeError, OverflowError):
        volumes = None
    cleanup = {k:settings[k] for k in ('point_cleanup_enabled','point_cleanup_support','point_cleanup_radius_mm') if k in settings}
    if not volumes and not cleanup.get('point_cleanup_enabled'):
        return sparse_point_packet(packet, budget)
    visible_only = bool(budget) and settings.get('point_preview_visible_only') is True
    return _point_packet(packet, budget, cropper, volumes, cleanup, visible_only=visible_only)


def _point_packet(packet, budget, cropper=None, volumes=None, cleanup=None, *, visible_only=False):
    if type(budget) is not int or budget not in (0, 20000, 40000):
        raise ValueError("point preview budget must be 0 (all), 20000 or 40000")
    if packet[:5] != b"RGBD1" or len(packet) < 9:
        raise ValueError("invalid sparse source packet")
    size = struct.unpack_from("<I", packet, 5)[0]
    offset = 9 + size
    metadata = copy.deepcopy(json.loads(packet[9:offset]))
    payloads = []
    for camera in metadata.get("cameras", []):
        color_end = offset + int(camera["color_length"])
        depth_end = color_end + int(camera["depth_length"])
        if depth_end > len(packet):
            raise ValueError("truncated sparse source packet")
        payloads.append(_WORKERS.submit(_encode_camera, camera, packet[offset:color_end], packet[color_end:depth_end],
                                        budget, cropper, volumes, cleanup, visible_only=visible_only))
        offset = depth_end
    if offset != len(packet):
        raise ValueError("unexpected trailing sparse source bytes")
    payloads = [future.result() for future in payloads]
    metadata["preview_transport"] = "host_sampled_points_v1"
    header = json.dumps(metadata, separators=(",", ":")).encode()
    return b"RGBD1" + struct.pack("<I", len(header)) + header + b"".join(payloads)
