"""Read-only native camera snapshots, with independent latest-only camera jobs.

Never opens a RealSense device, changes a profile, or supplies recording frames.
"""
from __future__ import annotations

import json
import math
import mmap
import os
import re
import struct
import threading
import time
import urllib.request
import zlib
from collections import OrderedDict
from pathlib import Path

import numpy as np

from tools.sparse_point_stream import ENCODING, sample_rgb_pixels
from tools.native_sparse_kernels import depth_mm, pack_points
from tools.point_preview_cleanup import clean_selected_points
from tools.workspace_stream_crop_native import compress_depth
from tools.workspace_stream_crop import WorkspaceStreamCropper, crop_volumes, retained_depth

MAGIC = b"RSNP01\0\0"
HEADER = 256


class NativeSnapshot:
    def __init__(self, path):
        self.path = Path(path)
        self.mapping = None
        self.inode = None

    def close(self):
        if self.mapping is not None:
            self.mapping.close()
        self.mapping = None
        self.inode = None

    def read(self, previous=None, frame_lane=None, frame_lanes=1):
        stat = self.path.stat()
        if self.mapping is None or self.inode != stat.st_ino:
            self.close()
            if not HEADER <= stat.st_size <= 32*1024*1024 or stat.st_uid != getattr(os,"getuid",lambda:stat.st_uid)():
                raise ValueError("invalid native camera mapping")
            with self.path.open('rb') as stream:
                self.mapping = mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ)
            self.inode = stat.st_ino
        data = self.mapping
        for _ in range(3):
            sequence = struct.unpack_from('<Q',data,8)[0]
            if sequence & 1 or not sequence or (self.inode,sequence)==previous:
                return None
            # Stripe publications, rather than sensor frame numbers (which may
            # skip). Reject the other lane before copying the camera images.
            if frame_lane is not None and (sequence//2)%frame_lanes != frame_lane:
                return None
            header = data[:HEADER]
            if header[:8] != MAGIC:
                return None
            width,height,stride,bpp = struct.unpack_from('<4I',header,24)
            depth_size,color_size = struct.unpack_from('<2I',header,56)
            rw,rh,pitch,flags = struct.unpack_from('<4I',header,80)
            if (not 0<width<=8192 or not 0<height<=8192 or not 0<stride<=64
                    or depth_size!=width*height*4 or bpp not in (3,4)
                    or not 0<rw<=8192 or not 0<rh<=8192 or pitch<rw*bpp
                    or color_size!=pitch*rh or HEADER+depth_size+color_size>len(data)):
                raise ValueError('invalid native camera dimensions')
            # Copy to owned storage before selecting points, then check the
            # producer sequence. No SDK/UVC buffer or writable mapping escapes.
            body = data[HEADER:HEADER+depth_size+color_size]
            if struct.unpack_from('<Q',data,8)[0] != sequence:
                continue
            intrinsics = list(struct.unpack_from('<4f',header,40))
            color_k = list(struct.unpack_from('<4f',header,96))
            extrinsics = list(struct.unpack_from('<12f',header,112))
            transform = list(struct.unpack_from('<16f',header,160))
            minimum,maximum,*correction = struct.unpack_from('<8f',header,224)
            fps,stamp = struct.unpack_from('<2d',header,64)
            if (not all(math.isfinite(v) for v in [*intrinsics,*color_k,*extrinsics,*transform,minimum,maximum,fps,stamp,*correction])
                    or min(intrinsics[:2]+color_k[:2])<=0 or stamp<=0):
                raise ValueError('invalid native camera calibration metadata')
            depth = np.frombuffer(body,dtype='<f4',count=width*height).reshape(height,width)
            color = np.ndarray((rh,rw,bpp),np.uint8,buffer=body,offset=depth_size,strides=(pitch,bpp,1))
            return dict(key=(self.inode,sequence),frame=struct.unpack_from('<Q',header,16)[0],
                        width=width,height=height,intrinsics=intrinsics,transform=transform,
                        depth=depth,rgb=color[:,:,:3][:,:,::-1],bgr=color,maximum=maximum,
                        projection=dict(model='pinhole',size=[rw,rh],intrinsics=color_k,extrinsics=extrinsics),
                        fps=fps,stamp=stamp,match=bool(flags&1),gain=correction[:3],bias=correction[3:])
        return None


def encode_native_camera(snapshot, camera, settings, cropper, robot=None, viewer=None):
    started = time.perf_counter()
    budget = settings.get('point_preview_budget',20000)
    if type(budget) is not int or budget not in (0,20000,40000):
        raise ValueError('invalid native preview budget')
    depth=depth_mm(snapshot)
    if depth is None:
        values = snapshot['depth']
        valid = np.isfinite(values)&(values>0)&(values<=snapshot['maximum'])
        depth = np.where(valid,np.minimum(65535,np.floor(np.where(valid,values,0).astype(np.float64)*1000+.5)),0).astype(np.uint16)
    metadata = dict(camera,width=snapshot['width'],height=snapshot['height'],
                    intrinsics=snapshot['intrinsics'],transform=snapshot['transform'],
                    color_space='original_rgb',color_projection=snapshot['projection'])
    try:
        volumes = crop_volumes(settings)
    except (ValueError,TypeError,OverflowError):
        volumes = []
    if volumes:
        depth = retained_depth(depth,metadata,volumes,cropper._bounds(depth,metadata,volumes))
    limit=depth.size if budget==0 else budget
    native=pack_points(depth,snapshot,limit)
    if native is not None:
        raw,source_count,count=native
    else:
        indices = np.flatnonzero(depth)
        source_count = len(indices)
        if source_count>limit:
            indices=indices[np.floor((np.arange(limit)+.5)*source_count/limit).astype(np.int64)]
        selected = depth.ravel()[indices]
        colors = sample_rgb_pixels(snapshot['rgb'],metadata,indices,selected)
        if snapshot['match']:
            colors=np.floor(np.clip(colors.astype(np.float64)/255*np.asarray(snapshot['gain'])+np.asarray(snapshot['bias']),0,1)*255+.5).astype(np.uint8)
        raw=indices.astype('<u4').tobytes()+selected.astype('<u2').tobytes()+colors.tobytes()
        count=len(indices)
    selected_count=count
    raw,count=clean_selected_points(raw,count,snapshot['width'],snapshot['intrinsics'],settings)
    metadata.update(preview_cleanup_enabled=settings.get('point_cleanup_enabled') is True,
                    preview_cleanup_kind='selected_radius_v1',
                    preview_cleanup_support=settings.get('point_cleanup_support',3),
                    preview_cleanup_radius_mm=settings.get('point_cleanup_radius_mm',15),
                    preview_cleanup_input_points=selected_count,preview_cleanup_removed=selected_count-count)
    predictor=settings.get('_sparse_predictor')
    if predictor in ('delta_v1','shuffle_delta_v2'):
        # Independent, reversible differences within this frame only. Spatial
        # selection and RGB/depth precision are unchanged; no frame references.
        predicted=bytearray(raw)
        indices=np.frombuffer(predicted,dtype='<u4',count=count)
        depths=np.frombuffer(predicted,dtype='<u2',count=count,offset=count*4)
        colors=np.frombuffer(predicted,dtype=np.uint8,offset=count*6).reshape(count,3)
        indices[1:]-=indices[:-1]
        depths[1:]-=depths[:-1]
        if predictor=='shuffle_delta_v2':
            signed=depths.view(np.int16).astype(np.int32)
            depths[:]=((signed<<1)^(signed>>15)).astype(np.uint16)
            colors[:,0]-=colors[:,1]
            colors[:,2]-=colors[:,1]
        colors[1:]-=colors[:-1]
        raw=(b''.join(np.ascontiguousarray(a).tobytes() for a in (
            indices.view(np.uint8).reshape(count,4).T,depths.view(np.uint8).reshape(count,2).T,colors.T))
            if predictor=='shuffle_delta_v2' else predicted)
        metadata['preview_predictor']=predictor
    packed=compress_depth(raw)
    metadata.pop('_viewer',None)
    metadata.pop('color_projection',None)
    metadata.update(sequence=snapshot['frame'],capture_unix_ms=snapshot['stamp'],
                    geometry_capture_unix_ms=snapshot['stamp'],color_encoding='sparse_rgb8',
                    depth_encoding=ENCODING,color_space='aligned_depth',color_length=0,
                    depth_length=len(packed),preview_budget=budget,preview_count=count,
                    preview_source_points=source_count,preview_native=True,depth_exact=True,
                    encode_ms=round((time.perf_counter()-started)*1000,3),source_capture_fps=snapshot['fps'])
    if isinstance(snapshot.get('key'),tuple) and len(snapshot['key'])==2:
        metadata.update(preview_epoch=str(snapshot['key'][0]),preview_publication=str(snapshot['key'][1]))
    envelope=dict(version=1,capture_unix_ms=snapshot['stamp'],cameras=[metadata],
                  preview_transport='host_sampled_points_v1',preview_source='native_camera_buffers_v1',
                  viewer=viewer or dict(transform=metadata['transform'],fov=60,near=.05,far=100))
    if robot is not None:
        envelope['robot']=robot
    encoded=json.dumps(envelope,separators=(',',':'),allow_nan=False).encode()
    return b'RGBD1'+struct.pack('<I',len(encoded))+encoded+packed


class NativeSparseSession:
    def __init__(self, cameras, settings, robot, fps=30, viewer=None, frame_lane=None, frame_lanes=1):
        if frame_lanes not in (1,2) or (frame_lane is not None and frame_lane not in range(frame_lanes)):
            raise ValueError('invalid preview frame lane')
        self.cameras=cameras
        self.settings=settings
        self.robot=robot
        self.viewer=viewer
        self.frame_lane=frame_lane
        self.frame_lanes=frame_lanes
        self.interval=1/max(1,min(60,fps))
        self.condition=threading.Condition()
        self.stop_event=threading.Event()
        self.pending=OrderedDict()
        self.errors={}
        self.threads=[]

    def start(self):
        for camera,path in self.cameras:
            thread=threading.Thread(target=self._camera,args=(camera,path),daemon=True,name='native-points-'+camera['id'][-4:])
            self.threads.append(thread)
            thread.start()

    def _camera(self,camera,path):
        reader=NativeSnapshot(path)
        cropper=WorkspaceStreamCropper()
        previous=None
        next_allowed=time.monotonic()
        try:
            while not self.stop_event.wait(.001):
                if time.monotonic()<next_allowed:
                    continue
                snapshot=reader.read(previous) if self.frame_lane is None else reader.read(previous,self.frame_lane,self.frame_lanes)
                if snapshot is None:
                    continue
                settings=self.settings()
                if settings.get('point_preview_enabled') is not True:
                    self.stop_event.set()
                    break
                if time.time()*1000-snapshot['stamp']>1000:
                    continue
                job_started=time.monotonic()
                encoder=encode_native_camera
                if settings.get('_mesh_profile')=='balanced':
                    from tools.native_mesh_stream import encode_native_mesh_camera
                    encoder=encode_native_mesh_camera
                packet=encoder(snapshot,camera,settings,cropper,self.robot(),self.viewer)
                previous=snapshot['key']
                next_allowed=max(next_allowed+self.interval,job_started)
                with self.condition:
                    self.pending[camera['id']]=packet
                    self.condition.notify()
        except (OSError,ValueError,TypeError,KeyError,OverflowError) as error:
            self.errors[camera['id']]=str(error)
        finally:
            reader.close()
            with self.condition:self.condition.notify_all()

    def take(self,timeout=1):
        with self.condition:
            self.condition.wait_for(lambda:self.pending or self.stop_event.is_set() or len(self.errors)==len(self.cameras),timeout)
            if self.pending:
                return self.pending.popitem(last=False)[1]
            if len(self.errors)==len(self.cameras):
                raise ConnectionError('native sparse camera stopped: '+str(self.errors))
            return None

    def close(self):
        self.stop_event.set()
        with self.condition:self.condition.notify_all()
        for thread in self.threads:thread.join(timeout=2)


def discover_native_cameras(host,port):
    if host not in ('127.0.0.1','localhost','::1'):
        return []
    with urllib.request.urlopen(f'http://{host}:{port}/diagnostics',timeout=2) as response:
        diagnostics=json.load(response)
    pid=diagnostics.get('runtime',{}).get('pid')
    if type(pid) is not int or pid<=0:
        return []
    env=dict(x.split('=',1) for x in Path('/proc',str(pid),'environ').read_text().split('\0') if '='in x)
    prefix=env.get('ROBOT_TELEOP_CAPTURE_SHM_PREFIX','')
    if not re.fullmatch(r'[A-Za-z0-9_]{1,100}',prefix):return []
    with urllib.request.urlopen(f'http://{host}:{port}/cameras',timeout=2) as response:
        labels={str(c['id']):c.get('model','') for c in json.load(response).get('cameras',[])}
    cameras=[]
    for camera in diagnostics.get('cameras',[]):
        identity=str(camera.get('id',''))
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}',identity):return []
        path=Path('/dev/shm',prefix+'_'+identity+'_native')
        if not path.is_file():return []
        reader=NativeSnapshot(path)
        try:
            snapshot=None
            for _ in range(5):
                snapshot=reader.read()
                if snapshot is not None:break
                time.sleep(.005)
            if snapshot is None:return []
        finally:reader.close()
        cameras.append((dict(id=identity,model=labels.get(identity,''),priority=0 if '455' in labels.get(identity,'') else 1,_viewer=snapshot['transform']),path))
    return cameras
