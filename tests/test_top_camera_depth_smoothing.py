import json,struct,zlib,unittest
from unittest.mock import patch
import numpy as np
from tools.preview_depth_smoothing import smooth_depth
import importlib.util
from tools.native_sparse_stream import encode_native_camera
from tools.workspace_stream_crop import WorkspaceStreamCropper


def unpack(packet):
    end=9+struct.unpack_from('<I',packet,5)[0];meta=json.loads(packet[9:end])['cameras'][0]
    raw=zlib.decompress(packet[end:]);n=meta['preview_count']
    return meta,np.frombuffer(raw,dtype='<u4',count=n),np.frombuffer(raw,dtype='<u2',count=n,offset=n*4)


def snapshot(depth):
    h,w=depth.shape
    return dict(depth=depth,width=w,height=h,maximum=4.5,intrinsics=[500,500,w/2,h/2],
        transform=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],rgb=np.full((h,w,3),100,np.uint8),
        projection=dict(model='pinhole',size=[w,h],intrinsics=[500,500,w/2,h/2],extrinsics=[1,0,0,0,1,0,0,0,1,0,0,0]),
        match=False,frame=1,stamp=1000,fps=60)


class RoutingTests(unittest.TestCase):
    def test_b_c_and_explicit_off_bypass_smoother(self):
        s=snapshot(np.full((20,20),.6,np.float32))
        with patch('tools.native_sparse_stream.smooth_depth',side_effect=AssertionError('must bypass')):
            for id,settings in [('262522075787',{}),('262522076048',{}),('262522074294',{'top_camera_depth_smoothing_enabled':False})]:
                meta,_,depths=unpack(encode_native_camera(s,{'id':id},{'point_preview_budget':0,**settings},WorkspaceStreamCropper()))
                self.assertEqual(meta['preview_depth_smoothing'],'off');self.assertTrue(meta['depth_exact']);self.assertTrue(np.all(depths==600))

    def test_smooths_full_a_before_crop_without_mutating_source(self):
        s=snapshot(np.full((20,20),.6,np.float32));before=s['depth'].tobytes();observed=[]
        def smooth(d):observed.append(d.copy());return np.full_like(d,601)
        settings={'point_preview_budget':0,'workspace_stream_crop_enabled':True,'modeled_workspace_enabled':True,
                  'modeled_workspace_center':[0,0,-.6],'modeled_workspace_width':.2,'modeled_workspace_depth':.2,
                  'modeled_workspace_height':.8,'modeled_workspace_clearance':.006}
        with patch('tools.native_sparse_stream.smooth_depth',side_effect=smooth):
            meta,_,_=unpack(encode_native_camera(s,{'id':'realsense:262522074294'},settings,WorkspaceStreamCropper()))
        self.assertEqual(len(observed),1);self.assertTrue(np.all(observed[0]==600),'filter receives full depth, including points later cropped')
        self.assertEqual(meta['preview_depth_smoothing'],'edge_aware_median_3x3');self.assertFalse(meta['depth_exact']);self.assertEqual(s['depth'].tobytes(),before)

    def test_missing_native_filter_is_explicit_and_preserves_measurements(self):
        with patch('tools.native_sparse_stream.smooth_depth',return_value=None):
            meta,_,depths=unpack(encode_native_camera(snapshot(np.full((10,10),.6,np.float32)),{'id':'262522074294'},{'point_preview_budget':0},WorkspaceStreamCropper()))
        self.assertEqual(meta['preview_depth_smoothing'],'filter_unavailable');self.assertTrue(meta['depth_exact']);self.assertTrue(np.all(depths==600))


@unittest.skipIf(importlib.util.find_spec('cv2') is None,'OpenCV unavailable')
class SpatialSmoothingTests(unittest.TestCase):
    def test_flat_noise_reduction_and_input_immutability(self):
        rng=np.random.default_rng(3);depth=np.rint(600+rng.normal(0,4,(120,180))).astype(np.uint16);before=depth.copy();result=smooth_depth(depth)
        self.assertLess(np.std(result[3:-3,3:-3]),np.std(depth[3:-3,3:-3])*.65)
        self.assertLess(abs(float(np.mean(result))-600),.3);np.testing.assert_array_equal(depth,before)

    def test_depth_edges_missing_values_and_constant_planes(self):
        depth=np.full((50,80),600,np.uint16);depth[:,40:]=650;depth[20:25,20:25]=0;before=depth.copy()
        np.testing.assert_array_equal(smooth_depth(depth),before)
        thin=np.full((20,20),600,np.uint16);thin[10,10]=550
        np.testing.assert_array_equal(smooth_depth(thin),thin)
        np.testing.assert_array_equal(smooth_depth(np.zeros((1,1),np.uint16)),[[0]])
        np.testing.assert_array_equal(smooth_depth(np.full((3,3),65535,np.uint16)),np.full((3,3),65535))

    def test_plane_shape_and_no_temporal_drag(self):
        depth=np.tile(np.arange(600,680,dtype=np.uint16),(50,1));result=smooth_depth(depth)
        np.testing.assert_array_equal(result[3:-3,3:-3],depth[3:-3,3:-3])
        np.testing.assert_array_equal(smooth_depth(np.full((10,10),650,np.uint16)),np.full((10,10),650))
