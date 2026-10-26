"""Camera-free native packet, calibration, ownership and bandwidth checks."""
from pathlib import Path
import asyncio
import io
import json
import struct
import sys
import threading
from types import SimpleNamespace
import zlib

import av
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from tools.workspace_stream_crop import WorkspaceStreamCropper, crop_volumes, _decode_depth, retained_depth
from tools.lan_remote_view_server import LanRemoteServer, LanRemoteHandler
from robot_teleop.operator import load_robot_operator
import tools.lan_remote_view_server as remote


def jpeg(rgb):
    codec=av.CodecContext.create('mjpeg','w')
    codec.width,codec.height=rgb.shape[1],rgb.shape[0]
    codec.pix_fmt='yuvj444p';codec.qmin=codec.qmax=2
    return b''.join(bytes(p) for p in codec.encode(av.VideoFrame.from_ndarray(rgb,format='rgb24')))


def split(packet):
    size=struct.unpack_from('<I',packet,5)[0]
    metadata=json.loads(packet[9:9+size]);offset=9+size;parts=[]
    for camera in metadata['cameras']:
        end=offset+camera['color_length'];last=end+camera['depth_length']
        parts.append((camera,packet[offset:end],packet[end:last]));offset=last
    assert offset==len(packet)
    return metadata,parts


def packet(original_rgb=True,predictive=False,cameras=1):
    w=h=128;fx=128.;cy=63.5
    row=np.arange(h)[:,None]
    denominator=.5+np.cos(np.pi/6)*(row-cy)/fx
    z=np.broadcast_to(.6/denominator,(h,w)).copy()
    z[51:72,56:73]=.51/np.broadcast_to(denominator,(h,w))[51:72,56:73]
    depth=np.round(z*1000).astype(np.uint16)
    model=np.array([[1,0,0,0],[0,np.cos(np.pi/6),.5,.6],[0,-.5,np.cos(np.pi/6),0],[0,0,0,1]])
    cw=ch=256 if original_rgb else 128
    rgb=np.random.default_rng(45).integers(0,256,(ch,cw,3),dtype=np.uint8)
    color=jpeg(rgb)
    encoding='u16_mm_deflate'
    raw=depth.astype('<u2').tobytes()
    if predictive:
        prev=np.zeros_like(depth);prev[1:]=depth[:-1]
        delta=np.subtract(depth,prev,dtype=np.uint16).view(np.int16).astype(np.int32)
        zigzag=((delta<<1)^(delta>>31)).astype(np.uint16)
        raw=(zigzag&255).astype(np.uint8).tobytes()+(zigzag>>8).astype(np.uint8).tobytes()
        encoding='u16_mm_vpredict_shuffle_deflate'
    compressed=zlib.compress(raw,1);metadata=[];payload=[]
    for index in range(cameras):
        c={'id':str(index),'width':w,'height':h,'intrinsics':[fx,fx,cy,cy],
           'transform':model.flatten(order='F').tolist(),'sequence':1,'depth_exact':True,
           'depth_encoding':encoding,'depth_length':len(compressed),'color_length':len(color),
           'color_space':'original_rgb' if original_rgb else 'aligned_depth','color_width':cw,'color_height':ch}
        if original_rgb:
            c['color_projection']={'model':'pinhole','size':[cw,ch],'intrinsics':[256.,256.,127.5,127.5],
                                   'extrinsics':[1,0,0,0,1,0,0,0,1,0,0,0]}
        metadata.append(c);payload.extend([color,compressed])
    doc={'cameras':metadata,'robot':{'available':True,'links':{'base_link':{'origin':[0,0,-1]}}},
         'viewer':{'transform':model.flatten(order='F').tolist(),'fov':55,'near':.01,'far':10}}
    encoded=json.dumps(doc,separators=(',',':')).encode()
    return b'RGBD1'+struct.pack('<I',len(encoded))+encoded+b''.join(payload)


SETTINGS={'workspace_stream_crop_enabled':True,'modeled_workspace_enabled':True,
          'modeled_workspace_center':[0,0,-1],'modeled_workspace_width':1.2,'modeled_workspace_depth':1.2,
          'modeled_workspace_height':.5,'modeled_workspace_clearance':.006}


@pytest.mark.parametrize('shape', [(7, 15), (19, 43), (720, 1280)])
def test_cached_ray_bounds_preserve_every_measured_point_inside_rotated_volumes(shape):
    rng=np.random.default_rng(87)
    depth=rng.integers(500,2000,shape,dtype=np.uint16);depth[rng.random(shape)<.1]=0
    h,w=shape;camera={'intrinsics':[float(w),float(w),(w-1)/2,(h-1)/2],
                    'transform':np.eye(4).flatten(order='F').tolist()}
    settings={'workspace_stream_crop_enabled':True,'workspace_crop_enabled':True,
              'workspace_crop_min':[-.2,-.2,-1.6],'workspace_crop_max':[.3,.25,-.6],
              'workspace_crop_rotation':[15,25,-10]}
    volumes=crop_volumes(settings);result=retained_depth(depth,camera,volumes)
    yy,xx=np.indices(shape);z=depth*.001
    points=np.stack(((xx-camera['intrinsics'][2])*z/w,-(yy-camera['intrinsics'][3])*z/w,-z),axis=-1)
    center,inverse,half=volumes[0];inside=np.all(np.abs((points-center)@inverse.T)<half-1e-5,axis=-1)&(depth!=0)
    assert np.any(inside)
    np.testing.assert_array_equal(result[inside],depth[inside])
    assert np.all((result==0)|(result==depth))


def test_triangle_crossing_workspace_survives_when_its_vertices_are_outside():
    depth=np.full((64,128),1000,np.uint16)
    camera={'intrinsics':[100.,100.,63.5,31.5],'transform':np.eye(4).flatten(order='F').tolist()}
    settings={'workspace_stream_crop_enabled':True,'workspace_crop_enabled':True,
              'workspace_crop_min':[-.001,-.2,-1.01],'workspace_crop_max':[.001,.2,-.99]}
    result=retained_depth(depth,camera,crop_volumes(settings))
    assert result[30,63]==result[30,64]==1000


def test_calibration_change_invalidates_cached_clipping_planes():
    cropper=WorkspaceStreamCropper();source=packet()
    first=cropper.crop(source,SETTINGS)
    assert split(first)[0]['stream_crop']['retained_samples']>0
    metadata,parts=split(source)
    metadata['cameras'][0]['transform'][12]+=2.0
    header=json.dumps(metadata,separators=(',',':')).encode()
    moved=b'RGBD1'+struct.pack('<I',len(header))+header+b''.join(color+depth for _,color,depth in parts)
    result=cropper.crop(moved,SETTINGS)
    document,parts=split(result)
    assert document['stream_crop']['retained_samples']==0
    assert not _decode_depth(parts[0][2],parts[0][0]).any()


def test_large_rgb_candidate_keeps_original_jpeg_without_reencoding(monkeypatch):
    _,[(camera,_,depth)]=split(packet())
    color=jpeg(np.zeros((256,256,3),dtype=np.uint8));camera['color_length']=len(color)
    cropper=WorkspaceStreamCropper()
    monkeypatch.setattr(cropper,'_rgb',lambda *_:pytest.fail('Large low-value RGB crop must not decode/reencode'))
    output,_,_=cropper._camera(camera,color,depth,crop_volumes(SETTINGS))
    assert output==color


@pytest.mark.parametrize('original_rgb',[False,True])
@pytest.mark.parametrize('predictive',[False,True])
def test_native_roi_preserves_depth_world_positions_and_reduces_bytes(original_rgb,predictive):
    source=packet(original_rgb,predictive,cameras=2);before=bytes(source)
    cropper=WorkspaceStreamCropper();result=cropper.crop(source,SETTINGS)
    document,parts=split(result);original,source_parts=split(source)
    assert result!=source and len(result)<len(source)*.8
    assert source==before and document['robot']==original['robot']
    assert document['stream_crop']['output_bytes']==len(result)
    assert document['stream_crop']['retained_samples']<document['stream_crop']['source_samples']*.4
    for (c,color,encoded),(old,_,old_depth) in zip(parts,source_parts):
        values=_decode_depth(encoded,c);all_values=_decode_depth(old_depth,old)
        x0,y0=c.get('stream_depth_origin',[0,0]);yy,xx=np.nonzero(values)
        assert np.array_equal(values[yy,xx],all_values[yy+y0,xx+x0])
        assert c['intrinsics'][:2]==old['intrinsics'][:2]
        assert c['intrinsics'][2]==old['intrinsics'][2]-x0
        assert c['intrinsics'][3]==old['intrinsics'][3]-y0
        assert c['transform']==old['transform'] and c['depth_exact'] is True
        if c.get('color_encoding')=='png':assert color.startswith(b'\x89PNG')
    assert cropper.crop(source,SETTINGS) is result,'same immutable frame reuses its bounded encoding'
    assert cropper.crop(source,{**SETTINGS,'modeled_workspace_enabled':False}) is source
    assert cropper.crop(source,{**SETTINGS,'workspace_stream_crop_enabled':False}) is source
    assert cropper.crop(source,{**SETTINGS,'display_mode':'rgb_camera'}) is source


def test_lossless_color_roi_and_projection_are_consistent():
    source=packet();result=WorkspaceStreamCropper().crop(source,SETTINGS)
    _,[(old,old_color,_)]=split(source)
    _,[(camera,color,_)]=split(result)
    assert camera['color_encoding']=='png'
    assert camera['color_projection']['extrinsics']==old['color_projection']['extrinsics']
    cx=int(old['color_projection']['intrinsics'][2]-camera['color_projection']['intrinsics'][2])
    cy=int(old['color_projection']['intrinsics'][3]-camera['color_projection']['intrinsics'][3])
    original=av.CodecContext.create('mjpeg','r').decode(av.Packet(old_color))[0].to_ndarray(format='rgb24')
    cropped=av.CodecContext.create('png','r').decode(av.Packet(color))[0].to_ndarray(format='rgb24')
    assert np.array_equal(cropped,original[cy:cy+camera['color_height'],cx:cx+camera['color_width']])


def test_empty_workspace_sends_empty_camera_updates_instead_of_stale_objects():
    source=packet();result=WorkspaceStreamCropper().crop(source,{**SETTINGS,'modeled_workspace_center':[10,10,10]})
    document,[(c,_,depth)]=split(result)
    assert c['id']=='0' and c['width']==c['height']==2
    assert not _decode_depth(depth,c).any()
    assert document['robot']==split(source)[0]['robot']
    assert len(result)<len(source)*.1


def test_rotated_box_intersects_table_and_invalid_settings_fail_open():
    assert len(crop_volumes({**SETTINGS,'workspace_crop_enabled':True,'workspace_crop_min':[-.1,0,-1.2],
        'workspace_crop_max':[.2,.3,-.8],'workspace_crop_rotation':[15,35,5]}))==2
    source=packet();cropper=WorkspaceStreamCropper()
    assert cropper.crop(source,{**SETTINGS,'modeled_workspace_center':[float('nan'),0,0]}) is source
    assert cropper.crop(source,{**SETTINGS,'modeled_workspace_height':-1}) is source
    assert cropper.crop(b'bad',SETTINGS)==b'bad'
    assert cropper.crop(source[:-5],SETTINGS)==source[:-5]


def test_page_requests_are_typed_isolated_and_off_restores_original():
    handler=object.__new__(LanRemoteHandler)
    handler.server=SimpleNamespace(robot_operator=load_robot_operator('disabled'))
    page={**SETTINGS,'rgbd_client_id':'page-a','type':'view_settings'}
    validated=handler.validate_view_settings(page)
    for bad in ('', 'x'*65,'../../x','secret?token','ä'):
        with pytest.raises(ValueError):handler.validate_view_settings({'rgbd_client_id':bad})
    with pytest.raises(ValueError):handler.validate_view_settings({'workspace_stream_crop_enabled':'yes'})
    server=object.__new__(LanRemoteServer)
    server.stream_crop_lock=threading.Lock();server.stream_crop_clients={}
    server.udp=SimpleNamespace(sendto=lambda *_:None);server.udp_host='127.0.0.1';server.udp_port=1
    server.packet_count=0;server.last_report_sec=float('inf')
    server.forward_tracking(json.dumps(validated).encode())
    source=packet()
    assert server.crop_rgbd_packet(source,'page-a')!=source
    assert server.crop_rgbd_packet(source,'page-b') is source
    server.forward_tracking(json.dumps({'type':'view_settings','rgbd_client_id':'page-a','modeled_workspace_enabled':False}).encode())
    assert server.crop_rgbd_packet(source,'page-a') is source


def test_https_latest_applies_the_page_filter_before_writing():
    handler=object.__new__(LanRemoteHandler);source=packet();filtered=b'filtered-native-frame';seen=[]
    handler.server=SimpleNamespace(rgbd_http_slots=threading.BoundedSemaphore(1),stream_host='127.0.0.1',capture_port=8781,
        claim_rgbd_client=lambda *_:True,rgbd_latest_hub=SimpleNamespace(source=lambda _:SimpleNamespace(claim_after=lambda *_:(1,source,''))),
        crop_rgbd_packet=lambda data,page:(seen.append((data,page)) or filtered))
    handler.wfile=io.BytesIO();handler.send_response=lambda _:None;handler.send_header=lambda *_:None;handler.end_headers=lambda:None
    handler.serve_latest_rgbd('client=page-a')
    assert seen==[(source,'page-a')] and handler.wfile.getvalue()==filtered


def test_native_websocket_filters_before_sending(monkeypatch):
    source=packet(predictive=True);filtered=WorkspaceStreamCropper().crop(source,SETTINGS)
    closed=threading.Event()
    class Reader:
        def __init__(self,_):self.first=True
        def read_part(self):
            if self.first:self.first=False;return {},source
            closed.wait(2);raise OSError('closed')
        def close(self):closed.set()
    monkeypatch.setattr(remote,'MultipartRawFrameReader',Reader)
    handler=object.__new__(LanRemoteHandler)
    handler.path='/rgbd-stream?client=page-a&temporal_depth=0';handler.client_address=('127.0.0.1',1)
    seen=[]
    handler.server=SimpleNamespace(stream_host='127.0.0.1',capture_port=8781,claim_rgbd_client=lambda *_:True,
        release_rgbd_client=lambda *_:None,crop_rgbd_packet=lambda data,page:(seen.append((data,page)) or filtered))
    handler.connection=SimpleNamespace(settimeout=lambda _:None,setsockopt=lambda *_:None)
    handler.accept_websocket=lambda:True;handler.watch_stream_peer=lambda *_:None
    sent=[]
    def send(data,opcode):sent.append(data);raise BrokenPipeError('fixture complete')
    handler.send_ws_frame=send;handler.handle_rgbd_websocket()
    assert sent==[filtered] and seen[0][1]=='page-a' and closed.is_set()


def test_direct_webrtc_filters_before_sending(monkeypatch):
    source=packet(predictive=True);filtered=WorkspaceStreamCropper().crop(source,SETTINGS)
    closed=threading.Event()
    class Reader:
        def __init__(self,_):self.first=True
        def read_part(self):
            if self.first:self.first=False;return {},source
            closed.wait(2);raise OSError('closed')
        def close(self):closed.set()
    monkeypatch.setattr(remote,'MultipartRawFrameReader',Reader)
    class Channel:
        readyState='open';bufferedAmount=0
        def __init__(self):self.sent=[]
        def send(self,data):self.sent.append(data);self.readyState='closed'
    channel=Channel();seen=[]
    def filter_packet(data):seen.append(data);return filtered
    asyncio.run(remote.publish_latest_rgbd_datachannel(channel,SimpleNamespace(connectionState='connected'),'fixture',filter_packet))
    assert seen==[source] and channel.sent==[filtered] and closed.is_set()
