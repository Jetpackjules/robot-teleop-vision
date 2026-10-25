"""Per-viewer native RGB-D cropping, after capture and before WAN transport.

No camera settings or calibration files are changed. Source packets are immutable.
The small retained image halo supports native triangles and confidence filters.
"""
from __future__ import annotations

from collections import OrderedDict
import copy
import json
import math
import struct
import threading
import time
import zlib

import av
import numpy as np

DEPTH_ENCODINGS = {"u16_mm_deflate", "u16_mm_vpredict_shuffle_deflate"}
STREAM_SETTING_KEYS = frozenset({
    "workspace_stream_crop_enabled", "display_mode", "workspace_crop_enabled",
    "workspace_crop_min", "workspace_crop_max", "workspace_crop_rotation",
    "modeled_workspace_enabled", "modeled_workspace_center", "modeled_workspace_width",
    "modeled_workspace_depth", "modeled_workspace_height", "modeled_workspace_heading",
    "modeled_workspace_clearance",
})


def _vector(value, length):
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError("invalid crop vector")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError("nonfinite crop vector")
    return np.asarray(value, dtype=np.float64)


def _rotation(degrees):
    x, y, z = np.radians(_vector(degrees, 3))
    cx, sx, cy, sy, cz, sz = math.cos(x), math.sin(x), math.cos(y), math.sin(y), math.cos(z), math.sin(z)
    return np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                     [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx], [-sy, cy*sx, cy*cx]])


def crop_volumes(settings):
    if settings.get("workspace_stream_crop_enabled") is not True or settings.get("display_mode") == "rgb_camera":
        return []
    volumes = []
    if settings.get("workspace_crop_enabled") is True:
        lo, hi = _vector(settings.get("workspace_crop_min"), 3), _vector(settings.get("workspace_crop_max"), 3)
        if np.any(lo >= hi) or np.any(np.abs([lo, hi]) > 20):
            raise ValueError("invalid crop bounds")
        volumes.append(((lo+hi)/2, _rotation(settings.get("workspace_crop_rotation", [0, 0, 0])).T, (hi-lo)/2))
    if settings.get("modeled_workspace_enabled") is True:
        center = _vector(settings.get("modeled_workspace_center"), 3)
        width, depth, height, clearance, heading = [settings.get("modeled_workspace_"+name, default) for name, default in
            [("width", 1.2), ("depth", .8), ("height", .8), ("clearance", .006), ("heading", 0)]]
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in [width, depth, height, clearance, heading]):
            raise ValueError("invalid table volume")
        if not (.1 <= width <= 4 and .1 <= depth <= 4 and 0 <= clearance <= .03 and clearance < height <= 3 and abs(heading) <= 180 and np.all(np.abs(center) <= 20)):
            raise ValueError("invalid table volume")
        center[1] += (height+clearance)/2
        volumes.append((center, _rotation([0, heading, 0]).T, np.array([width/2, (height-clearance)/2, depth/2])))
    return volumes


def _decode_depth(data, camera):
    width, height = int(camera["width"]), int(camera["height"])
    if width < 1 or height < 1 or width*height > 16*1024*1024:
        raise ValueError("invalid source dimensions")
    decoder = zlib.decompressobj()
    raw = decoder.decompress(data, width*height*2+1)
    if len(raw) != width*height*2 or not decoder.eof or decoder.unconsumed_tail:
        raise ValueError("invalid source depth")
    if camera["depth_encoding"] == "u16_mm_deflate":
        return np.frombuffer(raw, dtype="<u2").reshape(height, width)
    shuffled = np.frombuffer(raw, dtype=np.uint8)
    count = width*height
    zigzag = shuffled[:count].astype(np.uint16) | (shuffled[count:].astype(np.uint16) << 8)
    delta = ((zigzag >> 1).astype(np.int32) ^ -(zigzag & 1).astype(np.int32)).reshape(height, width)
    return np.cumsum(delta, axis=0, dtype=np.int32).astype(np.uint16)


def _tile_extrema(values, valid):
    height, width = values.shape
    rows, columns = (height+7)//8, (width+15)//16
    low = np.full((rows*8, columns*16), np.inf, dtype=np.float32)
    high = np.full_like(low, -np.inf)
    low[:height, :width] = np.where(valid, values, np.inf)
    high[:height, :width] = np.where(valid, values, -np.inf)
    low = low.reshape(rows, 8, columns, 16).min(axis=(1, 3))
    high = high.reshape(rows, 8, columns, 16).max(axis=(1, 3))
    # Each native cell can use a vertex in the block to its right/below.
    result_lo, result_hi = low.copy(), high.copy()
    for dy, dx in [(0, 1), (1, 0), (1, 1)]:
        result_lo[:rows-dy, :columns-dx] = np.minimum(result_lo[:rows-dy, :columns-dx], low[dy:, dx:])
        result_hi[:rows-dy, :columns-dx] = np.maximum(result_hi[:rows-dy, :columns-dx], high[dy:, dx:])
    return result_lo, result_hi


def retained_depth(depth, camera, volumes):
    height, width = depth.shape
    k = _vector(camera.get("intrinsics"), 4)
    if k[0] <= 0 or k[1] <= 0:
        raise ValueError("invalid source intrinsics")
    model = _vector(camera.get("transform"), 16).reshape(4, 4, order="F")
    if not np.allclose(model[3], [0, 0, 0, 1], atol=1e-6):
        raise ValueError("invalid camera transform")
    z = depth.astype(np.float32)*.001
    x = ((np.arange(width, dtype=np.float32)-k[2])/k[0])[None, :]
    y = (-(np.arange(height, dtype=np.float32)-k[3])/k[1])[:, None]
    valid = depth != 0
    keep = np.ones(((height+7)//8, (width+15)//16), dtype=bool)
    for center, inverse, half in volumes:
        basis = inverse @ model[:3, :3]
        origin = inverse @ (model[:3, 3]-center)
        for axis in range(3):
            local = (basis[axis, 0]*x+basis[axis, 1]*y-basis[axis, 2])*z+origin[axis]
            low, high = _tile_extrema(local, valid)
            keep &= (low <= half[axis]+1e-6) & (high >= -half[axis]-1e-6)
    mask = np.repeat(np.repeat(keep, 8, axis=0), 16, axis=1)[:height, :width]
    # Two pixels of unchanged native depth support edge triangles, derivatives
    # and the confidence filter. Nothing is downsampled or depth-quantized.
    expanded = mask.copy()
    for shift in (1, 2):
        expanded[shift:] |= mask[:-shift]
        expanded[:-shift] |= mask[shift:]
    mask = expanded.copy()
    for shift in (1, 2):
        mask[:, shift:] |= expanded[:, :-shift]
        mask[:, :-shift] |= expanded[:, shift:]
    return np.where(mask, depth, 0).astype(np.uint16)


def _rectangle(mask, quantum=32):
    y, x = np.nonzero(mask)
    if not len(x):
        return 0, 0, min(2, mask.shape[1]), min(2, mask.shape[0])
    x0, y0 = int(x.min())//quantum*quantum, int(y.min())//quantum*quantum
    x1 = min(mask.shape[1], (int(x.max())//quantum+1)*quantum)
    y1 = min(mask.shape[0], (int(y.max())//quantum+1)*quantum)
    return x0, y0, x1, y1


class WorkspaceStreamCropper:
    """Bounded codecs and one immutable-packet cache per browser page."""

    def __init__(self):
        self.lock = threading.RLock()
        self.decoders = OrderedDict()
        self.encoders = OrderedDict()
        self.previous = None

    def _rgb(self, color, key):
        codec = self.decoders.get(key)
        if codec is None:
            codec = av.CodecContext.create("mjpeg", "r")
            self.decoders[key] = codec
            if len(self.decoders) > 32:
                self.decoders.popitem(last=False)
        frames = codec.decode(av.Packet(color))
        if not frames:
            raise ValueError("invalid source JPEG")
        return frames[-1].to_ndarray(format="rgb24")

    def _png(self, rgb):
        height, width = rgb.shape[:2]
        key = (width, height)
        codec = self.encoders.get(key)
        if codec is None:
            codec = av.CodecContext.create("png", "w")
            codec.width, codec.height, codec.pix_fmt = width, height, "rgb24"
            codec.options = {"compression_level": "1"}
            codec.open()
            self.encoders[key] = codec
            if len(self.encoders) > 8:
                self.encoders.popitem(last=False)
        return b"".join(bytes(p) for p in codec.encode(av.VideoFrame.from_ndarray(np.ascontiguousarray(rgb), format="rgb24")))

    def _camera(self, camera, color, encoded_depth, volumes):
        original_color=color
        original = copy.deepcopy(camera)
        depth = _decode_depth(encoded_depth, camera)
        kept = retained_depth(depth, camera, volumes)
        box = _rectangle(kept != 0)
        x0, y0, x1, y1 = box
        original_rgb = camera.get("color_space") == "original_rgb"
        pixels = None
        color_box = box
        if original_rgb:
            p = camera.get("color_projection", {})
            if p.get("model") != "pinhole":
                raise ValueError("uncalibrated original RGB")
            cw, ch = int(camera["color_width"]), int(camera["color_height"])
            if [cw, ch] != p.get("size") or cw*ch > 16*1024*1024:
                raise ValueError("invalid RGB projection dimensions")
            k = _vector(camera["intrinsics"], 4)
            ck, extrinsics = _vector(p.get("intrinsics"), 4), _vector(p.get("extrinsics"), 12)
            rotation, translation = extrinsics[:9].reshape(3, 3, order="F"), extrinsics[9:]
            yy, xx = np.nonzero(kept)
            z = kept[yy, xx].astype(np.float32)*.001
            local = np.array([(xx-k[2])*z/k[0], (yy-k[3])*z/k[1], z])
            projected = rotation @ local+translation[:, None]
            front = projected[2] > 1e-5
            uv = projected[:2, front]/projected[2:3, front]*ck[:2, None]+ck[2:, None]
            inside = (uv[0] >= 0) & (uv[0] < cw-1) & (uv[1] >= 0) & (uv[1] < ch-1)
            uv = uv[:, inside]
            if uv.shape[1]:
                color_box = (max(0, int(math.floor(uv[0].min()-2))//32*32), max(0, int(math.floor(uv[1].min()-2))//32*32),
                             min(cw, int(math.ceil((uv[0].max()+3)/32))*32), min(ch, int(math.ceil((uv[1].max()+3)/32))*32))
            else:
                color_box = (0, 0, min(cw, 2), min(ch, 2))
        else:
            cw, ch = depth.shape[1], depth.shape[0]

        cx0, cy0, cx1, cy1 = color_box
        # Decode/repack RGB only when an actual image rectangle can shrink.
        if (cx0, cy0, cx1, cy1) != (0, 0, cw, ch):
            pixels = self._rgb(color, str(camera.get("id", "")))
            if pixels.shape[:2] != (ch, cw):
                raise ValueError("JPEG dimensions do not match calibration")
            cropped_color = self._png(pixels[cy0:cy1, cx0:cx1])
            if len(cropped_color) < len(color) or not original_rgb:
                color = cropped_color
                camera["color_encoding"] = "png"
                if original_rgb:
                    camera["color_width"], camera["color_height"] = cx1-cx0, cy1-cy0
                    projection = camera["color_projection"]
                    projection["size"] = [cx1-cx0, cy1-cy0]
                    projection["intrinsics"][2] -= cx0
                    projection["intrinsics"][3] -= cy0
            else:
                color_box = (0, 0, cw, ch)
        camera["width"], camera["height"] = x1-x0, y1-y0
        camera["intrinsics"][2] -= x0
        camera["intrinsics"][3] -= y0
        camera["stream_source_size"] = [depth.shape[1], depth.shape[0]]
        camera["stream_depth_origin"] = [x0, y0]
        camera["depth_encoding"] = "u16_mm_deflate"
        raw = kept[y0:y1, x0:x1].astype("<u2").tobytes()
        packed = zlib.compress(raw, 1)
        # For legacy aligned RGB, an expensive lossless PNG can cost more than
        # the whole original JPEG. Keep that JPEG and the native grid instead.
        if not original_rgb and len(color)+len(packed) >= original["color_length"]+len(encoded_depth):
            camera.clear()
            camera.update(original)
            color = original_color
            camera["depth_encoding"] = "u16_mm_deflate"
            packed = zlib.compress(kept.astype("<u2").tobytes(), 1)
            color_box = (0, 0, cw, ch)
        camera["color_length"], camera["depth_length"] = len(color), len(packed)
        return color, packed, {"source_depth_pixels": depth.size, "depth_pixels": camera["width"]*camera["height"],
            "source_color_pixels": cw*ch, "color_pixels": (color_box[2]-color_box[0])*(color_box[3]-color_box[1]),
            "source_samples": int(np.count_nonzero(depth)), "retained_samples": int(np.count_nonzero(kept))}

    def crop(self, packet, settings):
        try:
            volumes = crop_volumes(settings)
        except (ValueError, TypeError, OverflowError):
            return packet
        if not volumes:
            return packet
        key = json.dumps({k: v for k, v in settings.items() if k in STREAM_SETTING_KEYS}, sort_keys=True)
        with self.lock:
            if self.previous is not None and self.previous[0] is packet and self.previous[1] == key:
                return self.previous[2]
            try:
                result = self._packet(packet, volumes, time.perf_counter())
            except (ValueError, TypeError, KeyError, IndexError, OverflowError, zlib.error, av.error.FFmpegError):
                # Unsupported/corrupt local data must not blank a live viewer.
                result = packet
            self.previous = (packet, key, result)
            return result

    def _packet(self, packet, volumes, started):
        if len(packet) < 9 or packet[:5] != b"RGBD1":
            return packet
        length = struct.unpack_from("<I", packet, 5)[0]
        if length > 4*1024*1024 or 9+length > len(packet):
            return packet
        metadata = json.loads(packet[9:9+length])
        cameras = metadata.get("cameras", [])
        if metadata.get("metadata_delta") or not 0 < len(cameras) <= 32 or any(c.get("depth_encoding") not in DEPTH_ENCODINGS for c in cameras):
            return packet
        offset, payloads, totals = 9+length, [], {}
        for camera in cameras:
            csize, dsize = int(camera["color_length"]), int(camera["depth_length"])
            if csize < 0 or dsize < 0 or offset+csize+dsize > len(packet):
                raise ValueError("truncated RGB-D source")
            color, depth = packet[offset:offset+csize], packet[offset+csize:offset+csize+dsize]
            offset += csize+dsize
            color, depth, counts = self._camera(camera, color, depth, volumes)
            payloads.extend([color, depth])
            for name, value in counts.items():
                totals[name] = totals.get(name, 0)+value
        if offset != len(packet):
            raise ValueError("unexpected RGB-D trailer")
        metadata["stream_crop"] = {"applied": True, "input_bytes": len(packet), "output_bytes": 0, "halo_pixels": 2,
            "processing_ms":round((time.perf_counter()-started)*1000,2), **totals}
        result = b""
        for _ in range(3):
            encoded = json.dumps(metadata, separators=(",", ":")).encode()
            result = b"RGBD1"+struct.pack("<I", len(encoded))+encoded+b"".join(payloads)
            if metadata["stream_crop"]["output_bytes"] == len(result):
                break
            metadata["stream_crop"]["output_bytes"] = len(result)
        return result
