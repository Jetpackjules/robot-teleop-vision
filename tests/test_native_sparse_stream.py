import hashlib
import json
import os
import struct
import tempfile
import threading
import time
import unittest
import zlib
from pathlib import Path
from unittest.mock import patch

import numpy as np

from tools.native_sparse_stream import MAGIC,NativeSnapshot,NativeSparseSession,encode_native_camera
from tools.workspace_stream_crop import WorkspaceStreamCropper


def fixture(width=300,height=200,sequence=2):
    h=bytearray(256)
    h[:8]=MAGIC
    struct.pack_into('<QQ4I',h,8,sequence,sequence,width,height,1,3)
    struct.pack_into('<4f',h,40,300,300,width/2,height/2)
    struct.pack_into('<2I2d4I',h,56,width*height*4,width*height*3,30,time.time()*1000,width,height,width*3,0)
    struct.pack_into('<4f',h,96,300,300,width/2,height/2)
    struct.pack_into('<12f',h,112,1,0,0,0,1,0,0,0,1,0,0,0)
    struct.pack_into('<16f',h,160,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1)
    struct.pack_into('<8f',h,224,.0001,4.5,1,1,1,0,0,0)
    depth=np.ones((height,width),dtype='<f4')
    depth.ravel()[::17]=0
    bgr=np.empty((height,width,3),np.uint8)
    bgr[:]=[90,120,180]
    return bytes(h)+depth.tobytes()+bgr.tobytes()


class NativeSparseTests(unittest.TestCase):
    def test_rejected_native_job_falls_back_instead_of_claiming_an_empty_scene(self):
        from tools import native_sparse_kernels as kernels
        class Reject:
            def sparse_pack(self,*args):return 0
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'camera';path.write_bytes(fixture());reader=NativeSnapshot(path)
            try:snapshot=reader.read()
            finally:reader.close()
        with patch.object(kernels,'LIBRARY',Reject()):
            self.assertIsNone(kernels.pack_points(np.ones((200,300),np.uint16),snapshot,60000))

    def test_all_native_points_preserve_every_valid_index_depth_and_color(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'camera';original=fixture();path.write_bytes(original)
            reader=NativeSnapshot(path)
            try:snapshot=reader.read()
            finally:reader.close()
            packet=encode_native_camera(snapshot,{'id':'all'},{'point_preview_budget':0},WorkspaceStreamCropper())
            end=9+struct.unpack_from('<I',packet,5)[0];camera=json.loads(packet[9:end])['cameras'][0]
            count=camera['preview_count'];raw=zlib.decompress(packet[end:])
            indices=np.frombuffer(raw,dtype='<u4',count=count)
            depths=np.frombuffer(raw,dtype='<u2',count=count,offset=count*4)
            colors=np.frombuffer(raw,dtype=np.uint8,offset=count*6).reshape(count,3)
            expected=np.flatnonzero(snapshot['depth'])
            self.assertEqual(camera['preview_budget'],0)
            self.assertEqual(count,camera['preview_source_points'])
            self.assertGreater(count,40000)
            np.testing.assert_array_equal(indices,expected)
            np.testing.assert_array_equal(depths,np.full(count,1000))
            self.assertTrue(np.all((colors==[180,120,90])|(colors==140)))
            self.assertEqual(path.read_bytes(),original)

    def test_snapshot_owned_consistent_and_read_only(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'camera'
            original=fixture()
            path.write_bytes(original)
            reader=NativeSnapshot(path)
            try:
                snapshot=reader.read()
                self.assertEqual(snapshot['intrinsics'],[300,300,150,100])
                self.assertEqual(snapshot['frame'],2)
                self.assertIsNone(reader.read(snapshot['key']))
                self.assertFalse(snapshot['depth'].flags.writeable)
                with self.assertRaises(TypeError):reader.mapping[0]=0
                with path.open('r+b') as stream:
                    stream.seek(8);stream.write(struct.pack('<Q',3))
                self.assertIsNone(reader.read())
                self.assertEqual(float(snapshot['depth'][0,1]),1)
            finally:reader.close()
            self.assertTrue(path.exists(),'reader must never unlink producer storage')

    def test_frame_lanes_partition_publications_without_changing_point_bytes(self):
        with tempfile.TemporaryDirectory() as d:
            for lane_count in (2,):
                for sequence in (2,4,6,8,10,12):
                    path=Path(d)/str(sequence);path.write_bytes(fixture(sequence=sequence))
                    reader=NativeSnapshot(path)
                    try:
                        original=reader.read()
                        lane=(sequence//2)%lane_count
                        self.assertIsNone(reader.read(frame_lane=(lane+1)%lane_count,frame_lanes=lane_count))
                        selected=reader.read(frame_lane=lane,frame_lanes=lane_count)
                        self.assertEqual(selected['key'],original['key'])
                        for budget in (20000,40000):
                            settings={'point_preview_budget':budget,'_sparse_predictor':'delta_v1'}
                            packets=[encode_native_camera(snapshot,{'id':'camera'},settings,WorkspaceStreamCropper()) for snapshot in (original,selected)]
                            blocks=[]
                            for packet in packets:
                                end=9+struct.unpack_from('<I',packet,5)[0]
                                metadata=json.loads(packet[9:end])['cameras'][0]
                                self.assertEqual(metadata['preview_publication'],str(sequence))
                                blocks.append(zlib.decompress(packet[end:]))
                            self.assertEqual(blocks[0],blocks[1])
                    finally:reader.close()
    
    def test_native_caps_without_image_compression(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'camera';path.write_bytes(fixture())
            before=hashlib.sha256(path.read_bytes()).hexdigest()
            reader=NativeSnapshot(path)
            try:snapshot=reader.read()
            finally:reader.close()
            for budget in (20000,40000):
                packet=encode_native_camera(snapshot,dict(id='camera',model='D455'),dict(point_preview_budget=budget),WorkspaceStreamCropper())
                end=9+struct.unpack_from('<I',packet,5)[0]
                metadata=json.loads(packet[9:end]);camera=metadata['cameras'][0]
                self.assertEqual(len(metadata['cameras']),1)
                self.assertTrue(camera['preview_native'])
                self.assertEqual(camera['preview_count'],budget)
                self.assertEqual(camera['color_length'],0)
                raw=zlib.decompress(packet[end:]);self.assertEqual(len(raw),budget*9)
                indices=np.frombuffer(raw[:budget*4],'<u4')
                depth=np.frombuffer(raw[budget*4:budget*6],'<u2')
                np.testing.assert_array_equal(depth,np.full(budget,1000))
                self.assertEqual(len(np.unique(indices)),budget)
                self.assertTrue(np.all(snapshot['depth'].ravel()[indices]>0))
                self.assertEqual(camera['transform'],snapshot['transform'])
                self.assertEqual(camera['capture_unix_ms'],snapshot['stamp'])
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),before)

    def test_lossless_predictor_restores_identical_point_bytes(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'camera';path.write_bytes(fixture())
            reader=NativeSnapshot(path)
            try:snapshot=reader.read()
            finally:reader.close()
        rng=np.random.default_rng(56)
        snapshot['depth']=rng.uniform(.01,4.5,(200,300)).astype('<f4')
        snapshot['bgr']=rng.integers(0,256,(200,300,3),dtype=np.uint8)
        snapshot['rgb']=snapshot['bgr'][:,:,::-1]
        for budget in [0,20000,40000]:
            settings={'point_preview_budget':budget}
            raw_packet=encode_native_camera(snapshot,{'id':'camera'},settings,WorkspaceStreamCropper())
            packet=encode_native_camera(snapshot,{'id':'camera'},{**settings,'_sparse_predictor':'delta_v1'},WorkspaceStreamCropper())
            end=9+struct.unpack_from('<I',packet,5)[0]
            self.assertEqual(json.loads(packet[9:end])['cameras'][0]['preview_predictor'],'delta_v1')
            restored=bytearray(zlib.decompress(packet[end:]))
            count=json.loads(packet[9:end])['cameras'][0]['preview_count']
            for dtype,offset,shape in [('<u4',0,(count,)),('<u2',count*4,(count,)),('u1',count*6,(count,3))]:
                values=np.frombuffer(restored,dtype=dtype,offset=offset,count=int(np.prod(shape))).reshape(shape)
                values[:]=np.cumsum(values,axis=0,dtype=np.uint32)
            raw_end=9+struct.unpack_from('<I',raw_packet,5)[0]
            self.assertEqual(restored,zlib.decompress(raw_packet[raw_end:]))

    def test_invalid_mapping_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'camera';data=bytearray(fixture())
            struct.pack_into('<I',data,56,1);path.write_bytes(data)
            reader=NativeSnapshot(path)
            try:
                with self.assertRaises(ValueError):reader.read()
            finally:reader.close()

    def test_shuffled_predictor_preserves_the_original_samples(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'camera';path.write_bytes(fixture());reader=NativeSnapshot(path)
            try:snapshot=reader.read()
            finally:reader.close()
        for budget in (0,20000,40000):
            options={'point_preview_budget':budget}
            raw_packet=encode_native_camera(snapshot,{'id':'camera'},options,WorkspaceStreamCropper())
            packet=encode_native_camera(snapshot,{'id':'camera'},{**options,'_sparse_predictor':'shuffle_delta_v2'},WorkspaceStreamCropper())
            end=9+struct.unpack_from('<I',packet,5)[0];meta=json.loads(packet[9:end])['cameras'][0];count=meta['preview_count']
            self.assertEqual(meta['preview_predictor'],'shuffle_delta_v2')
            data=np.frombuffer(zlib.decompress(packet[end:]),np.uint8)
            indices=np.ascontiguousarray(data[:count*4].reshape(4,count).T).view('<u4').reshape(count)
            zigzag=np.ascontiguousarray(data[count*4:count*6].reshape(2,count).T).view('<u2').reshape(count).astype(np.int32)
            colors=np.ascontiguousarray(data[count*6:].reshape(3,count).T)
            indices=np.cumsum(indices,dtype=np.uint32)
            depths=np.cumsum((zigzag>>1)^-(zigzag&1),dtype=np.int32).astype('<u2')
            colors=np.cumsum(colors,axis=0,dtype=np.uint32).astype(np.uint8)
            colors[:,0]+=colors[:,1];colors[:,2]+=colors[:,1]
            raw_end=9+struct.unpack_from('<I',raw_packet,5)[0]
            self.assertEqual(indices.astype('<u4').tobytes()+depths.tobytes()+colors.tobytes(),zlib.decompress(raw_packet[raw_end:]))

    def test_fast_kernels_match_python_native_samples(self):
        from tools import native_sparse_kernels as kernels
        if kernels.LIBRARY is None:self.skipTest('optional native kernels unavailable')
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'camera';path.write_bytes(fixture())
            reader=NativeSnapshot(path)
            try:snapshot=reader.read()
            finally:reader.close()
        rng=np.random.default_rng(42)
        snapshot['depth']=rng.uniform(.2,4,size=(200,300)).astype('<f4')
        snapshot['depth'].ravel()[::19]=np.nan
        snapshot['bgr']=rng.integers(0,256,size=(200,300,3),dtype=np.uint8)
        snapshot['rgb']=snapshot['bgr'][:,:,::-1]
        snapshot['projection']={**snapshot['projection'],'intrinsics':[301.3,299.8,151.2,101.2],
            'extrinsics':[.99995,0,-.0099998,0,1,0,.0099998,0,.99995,.005,-.002,.001]}
        snapshot.update(match=True,gain=[1.1,.95,1.03],bias=[.01,-.02,0])
        for budget in (0,20000,40000):
            native=encode_native_camera(snapshot,dict(id='camera'),dict(point_preview_budget=budget),WorkspaceStreamCropper())
            with patch('tools.native_sparse_stream.depth_mm',return_value=None),patch('tools.native_sparse_stream.pack_points',return_value=None):
                python=encode_native_camera(snapshot,dict(id='camera'),dict(point_preview_budget=budget),WorkspaceStreamCropper())
            def payload(packet):return zlib.decompress(packet[9+struct.unpack_from('<I',packet,5)[0]:])
            self.assertEqual(payload(native),payload(python))

    def test_depth_conversion_exact_at_rounding_and_range_boundaries(self):
        from tools import native_sparse_kernels as kernels
        if kernels.LIBRARY is None:self.skipTest('optional native kernels unavailable')
        rng=np.random.default_rng(345)
        bits=rng.integers(0,2**32,size=200003,dtype=np.uint32).view(np.float32)
        half=(np.arange(65536,dtype=np.float64)+.5)/1000
        centers=half.astype(np.float32)
        values=np.concatenate([bits,centers,np.nextafter(centers,np.float32(0)),
            np.nextafter(centers,np.float32(np.inf))]).reshape(1,-1)
        for maximum in [.0005,4.5,65.535,100]:
            with np.errstate(invalid='ignore',over='ignore'):
                # The established scalar kernel promotes each source to double
                # before comparing the range and rounding millimetres.
                scalar=values.astype(np.float64)
                valid=np.isfinite(scalar)&(scalar>0)&(scalar<=maximum)
                expected=np.minimum(65535,np.floor(np.where(valid,values,0).astype(np.float64)*1000+.5)).astype(np.uint16)
            np.testing.assert_array_equal(kernels.depth_mm({'depth':values,'maximum':maximum}),expected)

    def test_slow_camera_does_not_hold_fast_camera(self):
        slow_gate=threading.Event()
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'camera';path.write_bytes(fixture(8,8))
            reader=NativeSnapshot(path)
            try:base=reader.read()
            finally:reader.close()
        class FakeReader:
            def __init__(self,path):self.slow=str(path)=='slow';self.frame=0
            def read(self,previous):
                if self.slow and not slow_gate.wait(.15):return None
                self.frame+=1
                return {**base,'key':self.frame,'frame':self.frame,'stamp':time.time()*1000}
            def close(self):pass
        with patch('tools.native_sparse_stream.NativeSnapshot',FakeReader):
            session=NativeSparseSession([(dict(id='fast'),'fast'),(dict(id='slow'),'slow')],
                       lambda:dict(point_preview_enabled=True,point_preview_budget=20000),lambda:None,fps=60)
            session.start()
            try:
                packets=[];deadline=time.monotonic()+.12
                while time.monotonic()<deadline:
                    packet=session.take(timeout=.025)
                    if packet is not None:packets.append(packet)
                self.assertGreaterEqual(len(packets),3)
                for packet in packets:
                    end=9+struct.unpack_from('<I',packet,5)[0]
                    self.assertEqual(json.loads(packet[9:end])['cameras'][0]['id'],'fast')
                self.assertLessEqual(len(session.pending),2)
            finally:
                slow_gate.set();session.close()


if __name__=='__main__':unittest.main()

def test_cleanup_runs_after_native_20k_selection(tmp_path):
    import pytest
    from tools.native_sparse_kernels import LIBRARY
    if LIBRARY is None:
        pytest.skip('Optional native kernels unavailable')
    path=tmp_path/'selected-native';path.write_bytes(fixture())
    reader=NativeSnapshot(path)
    try:snapshot=reader.read()
    finally:reader.close()
    snapshot['intrinsics']=[1000.,1000.,150.,100.]
    settings={'point_preview_budget':20000,'point_cleanup_enabled':True,
              'point_cleanup_support':3,'point_cleanup_radius_mm':1.5}
    packet=encode_native_camera(snapshot,{'id':'selected'},settings,WorkspaceStreamCropper())
    end=9+struct.unpack_from('<I',packet,5)[0]
    camera=json.loads(packet[9:end])['cameras'][0]
    assert camera['preview_source_points']>40000
    assert camera['preview_cleanup_input_points']==20000
    assert camera['preview_cleanup_removed']>0
    assert camera['preview_count']==20000-camera['preview_cleanup_removed']
    assert len(zlib.decompress(packet[end:]))==camera['preview_count']*9
