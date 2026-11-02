"""Optional fast camera-buffer kernels with an exact Python fallback."""
import ctypes
import hashlib
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'native/sparse_camera_stream/sparse_camera_stream.cpp'


def library_path():
    digest=hashlib.sha256(SOURCE.read_bytes()).hexdigest()[:20]
    return ROOT/'.teleop/native'/('sparse_camera_'+digest+'.so')


def _load():
    try:
        lib=ctypes.CDLL(str(library_path()))
        pointer=ctypes.c_void_p
        lib.sparse_depth_mm.argtypes=[pointer,pointer,ctypes.c_size_t,ctypes.c_double]
        lib.sparse_depth_mm.restype=None
        lib.sparse_clean_depth.argtypes=[pointer,pointer,ctypes.c_int,ctypes.c_int,ctypes.c_int,ctypes.c_double]
        lib.sparse_clean_depth.restype=None
        lib.sparse_pack.argtypes=[pointer,ctypes.c_int,ctypes.c_int,pointer,ctypes.c_int,
            ctypes.c_int,ctypes.c_int,ctypes.c_int,pointer,ctypes.c_uint32,pointer,
            ctypes.c_size_t,ctypes.POINTER(ctypes.c_uint32)]
        lib.sparse_pack.restype=ctypes.c_uint32
        return lib
    except (OSError,AttributeError):return None


LIBRARY=_load()


def clean_depth(depth, support, tolerance):
    if LIBRARY is None or depth.dtype!=np.uint16 or not depth.flags.c_contiguous:
        return None
    out=np.empty_like(depth)
    height,width=depth.shape
    LIBRARY.sparse_clean_depth(depth.ctypes.data,out.ctypes.data,width,height,int(support),tolerance)
    return out


def depth_mm(snapshot):
    if LIBRARY is None:return None
    source=snapshot['depth']
    if source.dtype!=np.dtype('<f4')or not source.flags.c_contiguous:return None
    out=np.empty(source.shape,np.uint16)
    LIBRARY.sparse_depth_mm(source.ctypes.data,out.ctypes.data,source.size,snapshot['maximum'])
    return out


def pack_points(depth,snapshot,budget):
    if LIBRARY is None or 'bgr'not in snapshot:return None
    color=snapshot['bgr']
    height,width=depth.shape
    rh,rw,bpp=color.shape
    if (depth.dtype!=np.uint16 or not depth.flags.c_contiguous or color.dtype!=np.uint8
            or color.strides[2]!=1 or color.strides[1]!=bpp):return None
    projection=snapshot['projection']
    parameters=np.asarray([*snapshot['intrinsics'],*projection['intrinsics'],
        *projection['extrinsics'],*snapshot['gain'],*snapshot['bias'],int(snapshot['match'])],np.float64)
    output=np.empty(min(budget,depth.size)*9,np.uint8)
    source_count=ctypes.c_uint32(0xffffffff)
    count=LIBRARY.sparse_pack(depth.ctypes.data,width,height,color.ctypes.data,rw,rh,
        color.strides[0],bpp,parameters.ctypes.data,budget,output.ctypes.data,output.size,ctypes.byref(source_count))
    if source_count.value==0xffffffff or count!=min(budget,source_count.value):return None
    return output[:count*9].tobytes(),source_count.value,count
