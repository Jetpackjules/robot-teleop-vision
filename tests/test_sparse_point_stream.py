import json
import struct
import unittest
import zlib

import cv2
import numpy as np

from tools.sparse_point_stream import sparse_point_packet


class SparsePointStreamTests(unittest.TestCase):
    def source(self, original=False):
        w, h = 320, 240
        rng = np.random.default_rng(7)
        depth = rng.integers(500, 2000, size=(h, w), dtype=np.uint16)
        depth.ravel()[::13] = 0
        rgb = rng.integers(0, 256, size=(h, w, 3), dtype=np.uint8)
        color = cv2.imencode('.png', rgb[:, :, ::-1])[1].tobytes()
        packed = zlib.compress(depth.astype('<u2').tobytes())
        camera = dict(id='probe', width=w, height=h, intrinsics=[300, 300, 159, 119],
                      transform=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],
                      depth_encoding='u16_mm_deflate', color_encoding='png',
                      color_length=len(color), depth_length=len(packed))
        if original:
            camera.update(color_space='original_rgb', color_width=w, color_height=h,
                          color_projection=dict(model='pinhole', size=[w,h],
                            intrinsics=camera['intrinsics'], extrinsics=[1,0,0,0,1,0,0,0,1,0,0,0]))
        meta = json.dumps(dict(cameras=[camera], capture_unix_ms=123)).encode()
        return b'RGBD1'+struct.pack('<I',len(meta))+meta+color+packed, depth, rgb

    def unpack(self, packet):
        end = 9+struct.unpack_from('<I',packet,5)[0]
        meta = json.loads(packet[9:end])
        camera = meta['cameras'][0]
        count = camera['preview_count']
        raw = zlib.decompress(packet[end:])
        self.assertEqual(len(raw), count*9)
        return camera, np.frombuffer(raw[:count*4], '<u4'), np.frombuffer(raw[count*4:count*6], '<u2'), np.frombuffer(raw[count*6:],np.uint8).reshape(-1,3)

    def test_caps_preserve_samples_and_source(self):
        source, depth, rgb = self.source()
        original = bytes(source)
        for budget in (20000,40000):
            packet = sparse_point_packet(source,budget)
            camera, indices, values, colors = self.unpack(packet)
            self.assertEqual(len(indices),budget)
            self.assertEqual(camera['preview_source_points'],np.count_nonzero(depth))
            valid = np.flatnonzero(depth)
            expected = valid[np.floor((np.arange(budget)+.5)*len(valid)/budget).astype(int)]
            np.testing.assert_array_equal(indices,expected)
            np.testing.assert_array_equal(values,depth.ravel()[indices])
            np.testing.assert_array_equal(colors,rgb.reshape(-1,3)[indices])
            self.assertLess(len(packet),len(source))
            self.assertEqual(source,original)

    def test_original_rgb_projection(self):
        source, depth, rgb = self.source(original=True)
        camera, indices, values, colors = self.unpack(sparse_point_packet(source,20000))
        y,x=np.divmod(indices,depth.shape[1])
        interior=(x>0)&(x<depth.shape[1]-1)&(y>0)&(y<depth.shape[0]-1)
        np.testing.assert_array_equal(colors[interior],rgb[y[interior],x[interior]])
        self.assertNotIn('color_projection',camera)

    def test_empty_and_invalid(self):
        with self.assertRaises(ValueError):sparse_point_packet(b'bad',20000)
        source,_,_=self.source()
        with self.assertRaises(ValueError):sparse_point_packet(source,30000)
        with self.assertRaises(ValueError):sparse_point_packet(source[:-1],20000)

    def test_control_protocol_preserves_budget(self):
        try:
            from tools.lan_remote_view_server import LanRemoteHandler
            from robot_teleop.operator import load_robot_operator
        except ModuleNotFoundError as error:
            self.skipTest('Gateway dependencies unavailable: '+str(error))
        from types import SimpleNamespace
        handler = SimpleNamespace(server=SimpleNamespace(robot_operator=load_robot_operator('disabled')))
        for budget in (20000,40000):
            packet = LanRemoteHandler.validate_view_settings(handler,dict(point_preview_enabled=True,point_preview_budget=budget))
            self.assertIs(packet['point_preview_enabled'],True)
            self.assertEqual(packet['point_preview_budget'],budget)
        for budget in (True,30000,20000.0):
            with self.assertRaises(ValueError):
                LanRemoteHandler.validate_view_settings(handler,dict(point_preview_budget=budget))

    def test_direct_crop_sampling_matches_previous_pipeline(self):
        from tools.sparse_point_stream import sparse_point_packet_for_view
        from tools.workspace_stream_crop import WorkspaceStreamCropper
        settings = dict(workspace_stream_crop_enabled=True, display_mode='point_cloud',
                        modeled_workspace_enabled=True, modeled_workspace_center=[0,-.5,-1],
                        modeled_workspace_width=1, modeled_workspace_depth=2,
                        modeled_workspace_height=1, modeled_workspace_heading=5,
                        modeled_workspace_clearance=.006)
        for original in (False,True):
            source,_,_ = self.source(original=original)
            cropper = WorkspaceStreamCropper()
            for budget in (20000,40000):
                old = self.unpack(sparse_point_packet(cropper.crop(source,settings),budget))
                new = self.unpack(sparse_point_packet_for_view(source,budget,cropper,settings))
                def positions(result):
                    camera,indices,z,_ = result
                    fx,fy,cx,cy=camera['intrinsics']
                    y,x=np.divmod(indices,camera['width'])
                    return np.column_stack(((x-cx)*z/fx,(y-cy)*z/fy,z))
                np.testing.assert_allclose(positions(old),positions(new),rtol=0,atol=1e-7)
                np.testing.assert_array_equal(old[3],new[3])
                self.assertEqual(old[0]['preview_source_points'],new[0]['preview_source_points'])

    def test_cached_camera_keeps_current_metadata(self):
        from unittest.mock import patch
        from tools import sparse_point_stream as stream
        from tools.workspace_stream_crop import WorkspaceStreamCropper
        import copy
        source,_,_=self.source()
        end=9+struct.unpack_from('<I',source,5)[0]
        camera=json.loads(source[9:end])['cameras'][0]
        color_end=end+camera['color_length']
        cropper=WorkspaceStreamCropper()
        with patch.object(stream,'_encode_camera_uncached',wraps=stream._encode_camera_uncached) as encode:
            first=stream._encode_camera(copy.deepcopy(camera),source[end:color_end],source[color_end:],20000,cropper,[])
            current=copy.deepcopy(camera)
            current['capture_unix_ms']=999
            second=stream._encode_camera(current,source[end:color_end],source[color_end:],20000,cropper,[])
            self.assertEqual(first,second)
            self.assertEqual(current['capture_unix_ms'],999)
            self.assertEqual(encode.call_count,1)


if __name__ == '__main__':unittest.main()
