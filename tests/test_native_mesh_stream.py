import copy
import json
import struct
import unittest
from unittest.mock import patch

import numpy as np

from tools.native_mesh_stream import encode_native_mesh_camera
from tools.workspace_stream_crop import WorkspaceStreamCropper, _decode_depth


class NativeMeshTests(unittest.TestCase):
    def test_depth_rays_pose_and_source_are_preserved_for_retained_samples(self):
        matrix=[1,0,0,0,0,1,0,0,0,0,1,0,.2,.3,-.1,1]
        depth=np.array([[.4,.5,.6,.7,.8],[1,1.1,np.nan,1.3,1.4],[2,2.1,2.2,2.3,2.4]],dtype=np.float32)
        snapshot=dict(width=5,height=3,depth=depth,rgb=np.full((6,10,3),[20,100,180],np.uint8),
            maximum=4,intrinsics=[5,6,2,1],transform=matrix,projection=dict(model='pinhole',size=[10,6],
            intrinsics=[10,12,4,2],extrinsics=[1,0,0,0,1,0,0,0,1,.01,0,0]),
            match=False,gain=[1,1,1],bias=[0,0,0],frame=7,stamp=1234.5,fps=59,key=(42,18))
        original=copy.deepcopy(snapshot)
        with patch('tools.native_mesh_stream.depth_mm',return_value=None):
            packet=encode_native_mesh_camera(snapshot,{'id':'A'},{},WorkspaceStreamCropper(),{'available':False})
        length=struct.unpack_from('<I',packet,5)[0];meta=json.loads(packet[9:9+length]);camera=meta['cameras'][0]
        values=_decode_depth(packet[9+length+camera['color_length']:],camera)
        np.testing.assert_array_equal(values,np.array([[400,600,800],[2000,2200,2400]],np.uint16))
        self.assertEqual(camera['intrinsics'],[2.5,3,1,.5])
        self.assertEqual(camera['transform'],matrix)
        self.assertEqual(camera['color_projection'],original['projection'])
        self.assertEqual(camera['preview_source_points'],14)
        self.assertEqual(meta['capture_unix_ms'],snapshot['stamp'])
        self.assertTrue(camera['mesh_preview'])
        np.testing.assert_array_equal(snapshot['depth'],original['depth'])
        np.testing.assert_array_equal(snapshot['rgb'],original['rgb'])
        for y in range(2):
            for x in range(3):
                self.assertEqual((x-camera['intrinsics'][2])/camera['intrinsics'][0],
                    (2*x-original['intrinsics'][2])/original['intrinsics'][0])

    def test_grid_dimensions_do_not_change_when_the_crop_edge_changes(self):
        # Stable dimensions prevent continuous GPU rig recreation at noisy edges.
        for size in [(848,480),(1280,720),(79,53)]:
            depth=np.zeros((size[1],size[0]),np.float32);depth[10:30,10:30]=.5
            snapshot=dict(width=size[0],height=size[1],depth=depth,rgb=np.zeros((6,10,3),np.uint8),
                maximum=4,intrinsics=[500,500,40,30],transform=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
                projection={'model':'pinhole','size':[10,6],'intrinsics':[10,10,5,3],
                    'extrinsics':[1,0,0,0,1,0,0,0,1,0,0,0]},
                match=False,frame=1,stamp=1,fps=60,key=(1,2))
            for edge in [30,31,32]:
                snapshot['depth'][10:edge,10:edge]=.5
                packet=encode_native_mesh_camera(snapshot,{'id':'A'},{},WorkspaceStreamCropper())
                n=struct.unpack_from('<I',packet,5)[0];c=json.loads(packet[9:9+n])['cameras'][0]
                self.assertEqual((c['width'],c['height']),((size[0]+1)//2,(size[1]+1)//2))

    def test_quality_capture_fits_the_bounded_worker_slot(self):
        from tools.native_sparse_workers import SLOT_SIZE,SLOT_HEADER
        rng=np.random.default_rng(93)
        snapshot=dict(width=1280,height=720,depth=rng.uniform(.1,3,(720,1280)).astype(np.float32),
            rgb=rng.integers(0,256,(1080,1920,3),dtype=np.uint8),maximum=4,
            intrinsics=[900,900,640,360],transform=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
            projection={'model':'pinhole','size':[1920,1080],'intrinsics':[1300,1300,960,540],
                'extrinsics':[1,0,0,0,1,0,0,0,1,0,0,0]},match=False,frame=1,stamp=1,fps=30,key=(1,2))
        packet=encode_native_mesh_camera(snapshot,{'id':'A'},{},WorkspaceStreamCropper())
        self.assertLess(len(packet),SLOT_SIZE-SLOT_HEADER)


if __name__=='__main__':unittest.main()
