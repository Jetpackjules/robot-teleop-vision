"""Optional compressed 3D preview, read directly from owned camera snapshots.

Recording, camera capture and calibration are separate from this encoder.
Retained depths stay in exact millimetres; the preview deliberately uses one
sample per 2x2 depth block and JPEG 65 with 4:2:0 chroma for camera RGB.
"""
from __future__ import annotations

import json
import struct
import time

import cv2
import numpy as np

from tools.native_sparse_kernels import depth_mm
from tools.workspace_stream_crop import crop_volumes, retained_depth
from tools.workspace_stream_crop_native import compress_depth


def encode_native_mesh_camera(snapshot, camera, settings, cropper, robot=None, viewer=None):
    started = time.perf_counter()
    depth = depth_mm(snapshot)
    if depth is None:
        values = snapshot['depth']
        valid = np.isfinite(values) & (values > 0) & (values <= snapshot['maximum'])
        depth = np.where(valid, np.minimum(65535, np.floor(
            np.where(valid, values, 0).astype(np.float64)*1000+.5)), 0).astype(np.uint16)
    metadata = dict(camera, width=snapshot['width'], height=snapshot['height'],
        intrinsics=list(snapshot['intrinsics']), transform=snapshot['transform'],
        color_space='original_rgb', color_projection=snapshot['projection'])
    volumes = crop_volumes(settings)
    if volumes:
        depth = retained_depth(depth, metadata, volumes, cropper._bounds(depth, metadata, volumes))
    source_count = int(np.count_nonzero(depth))
    # Keep dimensions stable as noisy edge pixels change the workspace crop.
    # Zeros compress cheaply and cannot create valid triangles. A moving image
    # rectangle would repeatedly recreate camera-rig textures and shaders.
    grid = np.ascontiguousarray(depth[::2, ::2], dtype='<u2')
    metadata.update(width=grid.shape[1], height=grid.shape[0],
                    intrinsics=[v/2 for v in snapshot['intrinsics']])
    rgb = snapshot['rgb']
    if snapshot['match']:
        rgb = np.floor(np.clip(rgb.astype(np.float32)*np.asarray(snapshot['gain'], np.float32)
            +255*np.asarray(snapshot['bias'], np.float32), 0, 255)+.5).astype(np.uint8)
    options = [cv2.IMWRITE_JPEG_QUALITY, 65]
    if hasattr(cv2, 'IMWRITE_JPEG_SAMPLING_FACTOR'):
        options += [cv2.IMWRITE_JPEG_SAMPLING_FACTOR, cv2.IMWRITE_JPEG_SAMPLING_FACTOR_420]
    ok, color = cv2.imencode('.jpg', np.ascontiguousarray(rgb[:, :, ::-1]), options)
    if not ok:
        raise ValueError('Compressed preview JPEG encoding failed')
    color = color.tobytes()
    packed = compress_depth(grid.tobytes())
    metadata.pop('_viewer', None)
    metadata.update(color_width=rgb.shape[1], color_height=rgb.shape[0], color_encoding='jpeg',
        depth_encoding='u16_mm_deflate', color_length=len(color), depth_length=len(packed),
        depth_exact=True, sequence=snapshot['frame'], capture_unix_ms=snapshot['stamp'],
        geometry_capture_unix_ms=snapshot['stamp'], preview_epoch=str(snapshot['key'][0]),
        preview_publication=str(snapshot['key'][1]), mesh_preview=True,
        preview_source_points=source_count, encode_ms=round((time.perf_counter()-started)*1000, 3),
        source_capture_fps=snapshot['fps'])
    envelope = dict(version=1, capture_unix_ms=snapshot['stamp'], cameras=[metadata],
        preview_transport='native_mesh_v1', preview_source='native_camera_buffers_v1',
        mesh_preview=dict(depth_stride=2, jpeg_quality=65, color_sampling='4:2:0'),
        viewer=viewer or dict(transform=metadata['transform'], fov=60, near=.05, far=100),
        stream_crop=dict(applied=bool(volumes), retained_samples=source_count, depth_pixels=grid.size))
    if robot is not None:
        envelope['robot'] = robot
    header = json.dumps(envelope, separators=(',', ':'), allow_nan=False).encode()
    return b'RGBD1'+struct.pack('<I', len(header))+header+color+packed
