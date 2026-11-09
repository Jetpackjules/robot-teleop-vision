import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import zlib

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import numpy as np
from tools.native_sparse_workers import ProcessSparseSession,close_all,create_native_sparse_session
from tools.native_sparse_stream import MAGIC,NativeSnapshot,encode_native_camera
from tools.workspace_stream_crop import WorkspaceStreamCropper


def fixture():
    width,height=300,200
    header=bytearray(256);header[:8]=MAGIC
    struct.pack_into('<QQ4I',header,8,2,1,width,height,1,3)
    struct.pack_into('<4f',header,40,300,300,width/2,height/2)
    struct.pack_into('<2I2d4I',header,56,width*height*4,width*height*3,60,time.time()*1000,width,height,width*3,1)
    struct.pack_into('<4f',header,96,301.3,299.8,151.2,101.2)
    struct.pack_into('<12f',header,112,1,0,0,0,1,0,0,0,1,.005,-.002,.001)
    struct.pack_into('<16f',header,160,1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1)
    struct.pack_into('<8f',header,224,.0001,4.5,1.1,.95,1.03,.01,-.02,0)
    rng=np.random.default_rng(42)
    depth=rng.uniform(.2,3,(height,width)).astype('<f4');depth.ravel()[::19]=np.nan
    bgr=rng.integers(0,256,(height,width,3),dtype=np.uint8)
    return bytes(header)+depth.tobytes()+bgr.tobytes()


def metadata(packet):
    length=struct.unpack_from('<I',packet,5)[0]
    return json.loads(packet[9:9+length]),zlib.decompress(packet[9+length:])


@unittest.skipUnless(sys.platform=='linux' and Path('/dev/shm').is_dir(),'Linux camera worker transport')
class Workers(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.path=Path(self.temp.name)/'camera';self.path.write_bytes(fixture())
        self.stopping=threading.Event()
        self.sessions=[]
        def update():
            publication=2
            fd=os.open(self.path,os.O_RDWR)
            try:
                while not self.stopping.wait(.01):
                    publication+=2
                    os.pwrite(fd,struct.pack('<Q',publication-1),8)
                    os.pwrite(fd,struct.pack('<Q',publication//2),16)
                    os.pwrite(fd,struct.pack('<d',time.time()*1000),72)
                    os.pwrite(fd,struct.pack('<Q',publication),8)
            finally:os.close(fd)
        self.thread=threading.Thread(target=update);self.thread.start()
        self.settings=dict(point_preview_enabled=True,point_preview_budget=20000,_sparse_predictor='delta_v1',
                           workspace_stream_crop_enabled=True,display_mode='point_cloud',modeled_workspace_enabled=True,
                           modeled_workspace_center=[0,-2,-2],modeled_workspace_width=4,modeled_workspace_depth=4,
                           modeled_workspace_height=3,modeled_workspace_heading=0,modeled_workspace_clearance=.006)

    def tearDown(self):
        for session in self.sessions:session.close()
        close_all()
        self.stopping.set();self.thread.join();self.temp.cleanup()

    def session(self,**kwargs):
        session=ProcessSparseSession([({'id':'camera'},self.path)],lambda:self.settings,lambda:{'available':False},fps=60,
                                     robot_path=Path(self.temp.name)/'no-robot',**kwargs)
        self.sessions.append(session);session.start();return session

    def take(self,session):
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            packet=session.take(.2)
            if packet is not None:return packet
        self.fail('worker produced no packet')

    def test_identical_points_colors_calibration_20k_40k_and_reconfiguration(self):
        session=self.session()
        process=session.workers['camera'].process
        for budget in (20000,40000):
            self.settings={**self.settings,'point_preview_budget':budget}
            actual=self.take(session)
            reader=NativeSnapshot(self.path)
            try:
                snapshot=None
                for _ in range(20):
                    snapshot=reader.read()
                    if snapshot is not None:break
                    time.sleep(.001)
            finally:reader.close()
            expected=encode_native_camera(snapshot,{'id':'camera'},self.settings,WorkspaceStreamCropper(),{'available':False})
            am,ab=metadata(actual);em,eb=metadata(expected)
            self.assertEqual(ab,eb,'selected indices, depth, or RGB changed')
            for key in ('intrinsics','transform','preview_budget','preview_count','preview_source_points','depth_encoding','color_encoding','preview_predictor'):
                self.assertEqual(am['cameras'][0][key],em['cameras'][0][key],key)
            self.assertIs(session.workers['camera'].process,process,'settings changes must keep the encoder process')

    def test_two_lanes_share_one_encoder_and_close_without_leaks(self):
        a=self.session(frame_lane=0,frame_lanes=2);b=self.session(frame_lane=1,frame_lanes=2)
        worker=a.workers['camera']
        self.assertIs(worker,b.workers['camera'])
        for lane,session in enumerate((a,b)):
            packet=self.take(session);meta,_=metadata(packet)
            self.assertEqual((int(meta['cameras'][0]['preview_publication'])//2)%2,lane)
        a.close();self.assertIsNone(worker.process.poll())
        b.close();self.assertIsNotNone(worker.process.poll());self.assertFalse(worker.output_path.exists())

    def test_pending_packets_stay_bounded_when_consumer_waits(self):
        session=self.session();first=self.take(session)
        time.sleep(.4)
        self.assertLessEqual(len(session.pending),1)
        second=self.take(session)
        self.assertGreater(int(metadata(second)[0]['cameras'][0]['preview_publication']),
                           int(metadata(first)[0]['cameras'][0]['preview_publication']))

    def test_free_lane_can_deliver_both_parities_while_other_lane_waits(self):
        a=self.session(frame_lane=0,frame_lanes=2,adaptive_delivery=True)
        b=self.session(frame_lane=1,frame_lanes=2,adaptive_delivery=True)
        self.assertIs(a.workers['camera'],b.workers['camera'])
        packets=[self.take(a) for _ in range(20)]
        publications=[int(metadata(p)[0]['cameras'][0]['preview_publication']) for p in packets]
        self.assertEqual({(p//2)%2 for p in publications},{0,1})
        self.assertEqual(publications,sorted(set(publications)))
        resumed=int(metadata(self.take(b))[0]['cameras'][0]['preview_publication'])
        self.assertGreater(resumed,publications[-1])
        self.assertLessEqual(len(a.pending),1)
        self.assertLessEqual(len(b.pending),1)

    def test_adaptive_lanes_never_claim_the_same_capture_after_reconfiguration(self):
        a=self.session(frame_lane=0,frame_lanes=2,adaptive_delivery=True)
        b=self.session(frame_lane=1,frame_lanes=2,adaptive_delivery=True)
        process=a.workers['camera'].process
        for budget in (20000,40000,0):
            self.settings={**self.settings,'point_preview_budget':budget}
            publications=[]
            for session in (a,b,a,b,a,b):
                meta,_=metadata(self.take(session));camera=meta['cameras'][0]
                self.assertEqual(camera['preview_budget'],budget)
                publications.append(camera['preview_publication'])
            self.assertEqual(len(publications),len(set(publications)))
        self.assertIs(a.workers['camera'].process,process)

    def test_settings_change_updates_both_lanes_without_restarting(self):
        a=self.session(frame_lane=0,frame_lanes=2);b=self.session(frame_lane=1,frame_lanes=2)
        self.take(a);self.take(b)
        process=a.workers['camera'].process
        self.settings={**self.settings,'point_preview_budget':40000,'modeled_workspace_width':2}
        packets=[self.take(session) for session in (a,b)]
        for packet in packets:self.assertEqual(metadata(packet)[0]['cameras'][0]['preview_budget'],40000)
        self.assertIs(a.workers['camera'].process,process)
        self.assertIs(b.workers['camera'].process,process)

    def test_different_viewers_do_not_share_crop_settings(self):
        a=self.session(owner='page-a');b=self.session(owner='page-b')
        self.assertIsNot(a.workers['camera'],b.workers['camera'])
        self.take(a);self.take(b)

    def test_worker_exit_is_reported_and_cleaned_up(self):
        session=self.session();self.take(session)
        worker=session.workers['camera'];worker.process.terminate();worker.process.wait(timeout=3)
        deadline=time.monotonic()+3
        while not session.errors and time.monotonic()<deadline:time.sleep(.01)
        with session.condition:session.pending.clear()
        with self.assertRaises(ConnectionError):session.take(.1)
        session.close()
        self.assertFalse(worker.output_path.exists())
        self.assertFalse(worker.config_path.exists())

    def test_threaded_fallback_can_be_selected(self):
        from tools.native_sparse_stream import NativeSparseSession
        with patch.dict(os.environ,{'ROBOT_TELEOP_NATIVE_POINT_WORKERS':'0'}):
            session=create_native_sparse_session([({'id':'camera'},self.path)],lambda:self.settings,lambda:{'available':False})
        self.assertIsInstance(session,NativeSparseSession)


if __name__=='__main__':unittest.main(verbosity=2)
