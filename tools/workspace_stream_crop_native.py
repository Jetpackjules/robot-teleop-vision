"""Optional exact host kernels; build explicitly, retain NumPy when unavailable."""
from __future__ import annotations

import ctypes
import ctypes.util
import hashlib
from pathlib import Path
import os
import sys
import threading
import zlib

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'native/workspace_stream_crop/workspace_stream_crop.cpp'


def library_path():
    suffix = '.dylib' if sys.platform == 'darwin' else '.so'
    digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()[:20]
    return ROOT / '.teleop/native' / ('workspace_crop_' + digest + suffix)


def _load():
    if os.environ.get('ROBOT_TELEOP_DISABLE_NATIVE_CROP') == '1':
        return None
    try:
        lib = ctypes.CDLL(str(library_path()))
        if lib.workspace_crop_abi() != 1:
            return None
        pointer = ctypes.c_void_p
        lib.workspace_decode_predictive.argtypes = [pointer, pointer, ctypes.c_size_t, ctypes.c_size_t]
        lib.workspace_decode_predictive.restype = None
        lib.workspace_outcode_plane.argtypes = [pointer, pointer, pointer, pointer, ctypes.c_size_t, ctypes.c_uint8]
        lib.workspace_outcode_plane.restype = None
        lib.workspace_outcode_upper.argtypes = [pointer, pointer, pointer, ctypes.c_size_t, ctypes.c_uint8]
        lib.workspace_outcode_upper.restype = None
        lib.workspace_outcode_lower.argtypes = lib.workspace_outcode_plane.argtypes
        lib.workspace_outcode_lower.restype = None
        lib.workspace_quad_cells.argtypes = [pointer, pointer, pointer, ctypes.c_size_t, ctypes.c_size_t]
        lib.workspace_quad_cells.restype = None
        lib.workspace_clip_plane.argtypes = [pointer, ctypes.c_double, pointer, pointer, ctypes.c_size_t]
        lib.workspace_clip_plane.restype = None
        lib.workspace_retain_cells.argtypes = [pointer, pointer, pointer, ctypes.c_size_t, ctypes.c_size_t]
        lib.workspace_retain_cells.restype = None
        lib.workspace_sparse_retain.argtypes = [pointer, pointer, pointer, pointer, pointer, ctypes.c_size_t, ctypes.c_size_t]
        lib.workspace_sparse_retain.restype = ctypes.c_int
        return lib
    except (OSError, AttributeError):
        return None


LIBRARY = _load()


def _load_deflate():
    if os.environ.get('ROBOT_TELEOP_DISABLE_NATIVE_CROP') == '1':
        return None
    try:
        name = ctypes.util.find_library('deflate')
        if not name:
            return None
        lib = ctypes.CDLL(name)
        pointer = ctypes.c_void_p
        for kind in ('compressor', 'decompressor'):
            allocate = getattr(lib, 'libdeflate_alloc_' + kind)
            allocate.argtypes = [ctypes.c_int] if kind == 'compressor' else []
            allocate.restype = pointer
            free = getattr(lib, 'libdeflate_free_' + kind)
            free.argtypes = [pointer]
            free.restype = None
        lib.libdeflate_zlib_compress_bound.argtypes = [pointer, ctypes.c_size_t]
        lib.libdeflate_zlib_compress_bound.restype = ctypes.c_size_t
        lib.libdeflate_zlib_compress.argtypes = [pointer, pointer, ctypes.c_size_t, pointer, ctypes.c_size_t]
        lib.libdeflate_zlib_compress.restype = ctypes.c_size_t
        lib.libdeflate_zlib_decompress.argtypes = [pointer, pointer, ctypes.c_size_t, pointer, ctypes.c_size_t, ctypes.POINTER(ctypes.c_size_t)]
        lib.libdeflate_zlib_decompress.restype = ctypes.c_int
        return lib
    except (OSError, AttributeError):
        return None


DEFLATE = _load_deflate()
_codecs = threading.local()


class _Codec:
    def __init__(self, kind):
        allocate = getattr(DEFLATE, 'libdeflate_alloc_' + kind)
        self.pointer = allocate(1) if kind == 'compressor' else allocate()
        self.free = getattr(DEFLATE, 'libdeflate_free_' + kind)

    def __del__(self):
        if getattr(self, 'pointer', None):
            self.free(self.pointer)


def decompress_depth(data, size):
    if DEFLATE is None:
        return None
    if not hasattr(_codecs, 'decoder'):
        _codecs.decoder = _Codec('decompressor')
    if not _codecs.decoder.pointer:
        return None
    output = np.empty(size, dtype=np.uint8)
    source = np.frombuffer(data, dtype=np.uint8)
    actual = ctypes.c_size_t()
    result = DEFLATE.libdeflate_zlib_decompress(_codecs.decoder.pointer, source.ctypes.data, len(data),
        output.ctypes.data, size, ctypes.byref(actual))
    if result != 0 or actual.value != size:
        raise zlib.error('invalid bounded source depth')
    return output


def compress_depth(data):
    if DEFLATE is None:
        return zlib.compress(data, 1)
    if not hasattr(_codecs, 'encoder'):
        _codecs.encoder = _Codec('compressor')
    if not _codecs.encoder.pointer:
        return zlib.compress(data, 1)
    bound = DEFLATE.libdeflate_zlib_compress_bound(_codecs.encoder.pointer, len(data))
    output = np.empty(bound, dtype=np.uint8)
    source = np.frombuffer(data, dtype=np.uint8)
    size = DEFLATE.libdeflate_zlib_compress(_codecs.encoder.pointer, source.ctypes.data, len(data), output.ctypes.data, bound)
    if not size:
        return zlib.compress(data, 1)
    return output[:size].tobytes()


def decode_predictive(raw, width, height):
    if LIBRARY is None:
        return None
    if width < 1 or height < 1 or len(raw) != width*height*2:
        raise ValueError('invalid predictive depth size')
    values = np.empty((height, width), dtype=np.uint16)
    source = np.frombuffer(raw, dtype=np.uint8)
    LIBRARY.workspace_decode_predictive(source.ctypes.data, values.ctypes.data, values.size, width)
    return values


def clip_plane(coefficient, limit):
    if LIBRARY is None or coefficient.dtype != np.float32 or not coefficient.flags.c_contiguous:
        return None
    threshold = np.empty(coefficient.shape, dtype=np.uint16)
    kind = np.empty(coefficient.shape, dtype=np.uint8)
    LIBRARY.workspace_clip_plane(coefficient.ctypes.data, float(limit), threshold.ctypes.data,
        kind.ctypes.data, coefficient.size)
    plane = _Plane((threshold, kind))
    plane.upper = limit >= 0
    # Normalize only inactive/constant thresholds; the reference fallback
    # ignores these values, so it remains equivalent without extra cache RAM.
    threshold[kind == 0] = 65535 if plane.upper else 0
    threshold[kind == 3] = 0 if plane.upper else 65535
    return plane


class _Plane(tuple):
    """Retain the two reference arrays plus a vector-friendly equivalent."""


def bounds_bytes(bounds):
    return sum(a.nbytes for planes in bounds for pair in planes for a in pair)


def retain_cells(depth, cells):
    if LIBRARY is None or not depth.flags.c_contiguous or not cells.flags.c_contiguous:
        return None
    result = np.empty_like(depth)
    LIBRARY.workspace_retain_cells(depth.ctypes.data, cells.ctypes.data, result.ctypes.data,
        depth.shape[1], depth.shape[0])
    return result


def retain_depth(depth, bounds):
    """Exact common workspace-volume path; other volume sets use the reference."""
    if (LIBRARY is None or depth.dtype != np.uint16 or not depth.flags.c_contiguous
            or len(bounds) != 1 or len(bounds[0]) != 6):
        return None
    planes = bounds[0]
    if any(t.shape != depth.shape or t.dtype != np.uint16 or not t.flags.c_contiguous
           or k.shape != depth.shape or k.dtype != np.uint8 or not k.flags.c_contiguous for t,k in planes):
        return None
    pointers = ctypes.c_void_p * 6
    thresholds = pointers(*(t.ctypes.data for t,k in planes))
    kinds = pointers(*(k.ctypes.data for t,k in planes))
    modes = np.asarray([0 if getattr(p,'upper',None) is None else 1 if p.upper else 2 for p in planes],np.uint8)
    result = np.empty_like(depth)
    if not LIBRARY.workspace_sparse_retain(depth.ctypes.data,thresholds,kinds,modes.ctypes.data,
                                          result.ctypes.data,depth.shape[1],depth.shape[0]):
        return None
    return result


def crop_cells(depth, bounds):
    if LIBRARY is None or depth.dtype != np.uint16 or not depth.flags.c_contiguous:
        return None
    cells = np.ones(depth.shape, dtype=np.uint8)
    code = np.empty(depth.shape, dtype=np.uint8)
    for planes in bounds:
        code.fill(0)
        for index, plane in enumerate(planes):
            threshold, kind = plane
            if isinstance(plane, _Plane):
                if plane.upper:
                    LIBRARY.workspace_outcode_upper(depth.ctypes.data, threshold.ctypes.data, code.ctypes.data, depth.size, 1 << index)
                else:
                    LIBRARY.workspace_outcode_lower(depth.ctypes.data, threshold.ctypes.data, kind.ctypes.data,
                        code.ctypes.data, depth.size, 1 << index)
            else:
                LIBRARY.workspace_outcode_plane(depth.ctypes.data, threshold.ctypes.data,
                    kind.ctypes.data, code.ctypes.data, depth.size, 1 << index)
        LIBRARY.workspace_quad_cells(depth.ctypes.data, code.ctypes.data, cells.ctypes.data,
            depth.shape[1], depth.shape[0])
    return cells


if __name__ == '__main__':
    import subprocess
    if sys.platform not in ('linux', 'darwin'):
        raise SystemExit('Native crop build supports Linux/macOS; NumPy remains available elsewhere.')
    target = library_path()
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix('.tmp' + target.suffix)
    subprocess.run(['c++', '-O3', '-march=native', '-std=c++17', '-shared', '-fPIC', str(SOURCE), '-o', str(temporary)], check=True)
    temporary.replace(target)
    print(target)
