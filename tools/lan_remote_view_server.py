#!/usr/bin/env python3
import argparse
import asyncio
import base64
import fractions
import hashlib
import hmac
import html
import json
import os
import socket
import ssl
import struct
import subprocess
import sys
import threading
import time
import uuid
import zlib
from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, quote, urlsplit

import av
import aiortc.codecs as aiortc_codecs
import jwt
import numpy as np
from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc import RTCRtpSender
from aiortc.codecs import h264, vpx
from aiortc.exceptions import InvalidStateError

from so101_arm_common import ArmProtocolError, validate_browser_arm_message


h264.MAX_BITRATE = 24_000_000
h264.DEFAULT_BITRATE = 4_000_000
vpx.MAX_BITRATE = 24_000_000


GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
MAX_WEBSOCKET_FRAME = 64 * 1024
MAX_WEBRTC_PEERS = 4
MAX_FFMPEG_PROCESSES = 2
MAX_RGBD_HTTP_REQUESTS = 48
MAX_CLAW_STREAMS = 4
MAX_REMEMBERED_RGBD_CLIENTS = 1024
MAX_ARM_TRANSPORT_AGE_MS = 250
ARM_CLOCK_REBASE_SAMPLE_COUNT = 4
ARM_CLOCK_REBASE_RANGE_MS = 40

TEMPORAL_DEPTH_MAGIC = b"DTL1"
TEMPORAL_DEPTH_HEADER = struct.Struct("<4sIHHHI")
TEMPORAL_DEPTH_ENCODING = "u16_mm_keyframe_tiles_deflate"
TEMPORAL_COLOR_MAGIC = b"CTL1"
TEMPORAL_COLOR_HEADER = struct.Struct("<4sIHHHI")
TEMPORAL_COLOR_ENCODING = "rgb8_keyframe_tiles_jpeg"


def transcode_rgbd_depth_predictive(packet: bytes) -> bytes:
    """Losslessly repack legacy uint16 depth so public RGB-D uses fewer bytes."""

    if len(packet) < 9 or packet[:5] != b"RGBD1":
        return packet
    try:
        metadata_length = struct.unpack_from("<I", packet, 5)[0]
        payload_offset = 9 + metadata_length
        if payload_offset > len(packet):
            return packet
        metadata = json.loads(packet[9:payload_offset])
        payloads: list[tuple[bytes, bytes]] = []
        changed = False
        for camera in metadata.get("cameras", []):
            color_length = int(camera.get("color_length", 0))
            depth_length = int(camera.get("depth_length", 0))
            color_end = payload_offset + color_length
            depth_end = color_end + depth_length
            if color_end > len(packet) or depth_end > len(packet):
                return packet
            color = packet[payload_offset:color_end]
            depth = packet[color_end:depth_end]
            payload_offset = depth_end
            if camera.get("depth_encoding") == "u16_mm_deflate":
                width = int(camera.get("width", 0))
                height = int(camera.get("height", 0))
                raw = zlib.decompress(depth)
                if width <= 0 or height <= 0 or len(raw) != width * height * 2:
                    return packet
                values = np.frombuffer(raw, dtype="<u2").reshape(height, width)
                previous = np.empty_like(values)
                previous[0, :] = 0
                previous[1:, :] = values[:-1, :]
                modulo_delta = np.subtract(values, previous, dtype=np.uint16)
                signed_delta = modulo_delta.view(np.int16).astype(np.int32)
                zigzag = ((signed_delta << 1) ^ (signed_delta >> 31)).astype(np.uint16)
                low = (zigzag & 0xff).astype(np.uint8).tobytes()
                high = (zigzag >> 8).astype(np.uint8).tobytes()
                depth = zlib.compress(low + high, 1)
                camera["depth_length"] = len(depth)
                camera["depth_encoding"] = "u16_mm_vpredict_shuffle_deflate"
                changed = True
            payloads.append((color, depth))
        if not changed:
            return packet
        metadata_bytes = json.dumps(metadata, separators=(",", ":")).encode("utf-8")
        rebuilt = bytearray(b"RGBD1")
        rebuilt.extend(struct.pack("<I", len(metadata_bytes)))
        rebuilt.extend(metadata_bytes)
        for color, depth in payloads:
            rebuilt.extend(color)
            rebuilt.extend(depth)
        return bytes(rebuilt)
    except (TypeError, ValueError, KeyError, json.JSONDecodeError, zlib.error):
        return packet


class TemporalRgbdDepthEncoder:
    """Repack RGB-D as temporal tile updates.

    The default recovery mode keeps independently decodable deltas against a
    periodic keyframe, preserving the latest-only transport contract.
    Persistent-reference mode instead advances the reconstructed reference
    after every encoded packet. It is substantially more efficient for a
    stationary scene, but requires every encoded packet to be delivered in
    order. The WebSocket handler enforces that contract while still collapsing
    obsolete raw capture frames before encoding.
    """

    def __init__(
        self,
        keyframe_interval: int = 30,
        tile_size: int = 8,
        stability_mm: int = 4,
        minimum_changed_pixels: int = 4,
        single_pixel_change_mm: int = 8,
        plane_stabilization: bool = True,
        motion_confirmation: bool = True,
        temporal_color: bool = True,
        color_tile_size: int = 16,
        color_quality: int = 68,
        persistent_reference: bool = False,
    ) -> None:
        self.keyframe_interval = max(1, int(keyframe_interval))
        self.tile_size = max(4, min(32, int(tile_size)))
        self.stability_mm = max(0, int(stability_mm))
        self.minimum_changed_pixels = max(1, int(minimum_changed_pixels))
        self.single_pixel_change_mm = max(
            self.stability_mm,
            int(single_pixel_change_mm),
        )
        self.plane_stabilization = bool(plane_stabilization)
        self.motion_confirmation = bool(motion_confirmation)
        self.temporal_color = bool(temporal_color)
        self.color_tile_size = max(8, min(32, int(color_tile_size)))
        self.color_quality = max(1, min(100, int(color_quality)))
        self.persistent_reference = bool(persistent_reference)
        self.frame_index = 0
        self.next_keyframe_id = 1
        self.keyframes: dict[str, tuple[int, np.ndarray, np.ndarray]] = {}
        self.change_states: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.color_keyframes: dict[str, tuple[int, np.ndarray]] = {}
        self.color_change_states: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.color_decoders: dict[str, av.CodecContext] = {}
        self.color_encoders: dict[tuple[int, int], av.CodecContext] = {}
        self.last_depth_changed_masks: dict[str, np.ndarray] = {}
        self.last_color_changed_masks: dict[str, np.ndarray] = {}
        self.persistent_metadata: dict | None = None
        self.worker_pool = (
            ThreadPoolExecutor(
                max_workers=2,
                thread_name_prefix="rgbd-temporal",
            )
            if self.temporal_color
            else None
        )
        self.plane_design = np.asarray(
            [
                (float(local_x), float(local_y), 1.0)
                for local_y in range(self.tile_size)
                for local_x in range(self.tile_size)
            ],
            dtype=np.float64,
        )

    def _next_reference_id(self) -> int:
        reference_id = self.next_keyframe_id
        self.next_keyframe_id = (self.next_keyframe_id + 1) & 0xFFFFFFFF
        if self.next_keyframe_id == 0:
            self.next_keyframe_id = 1
        return reference_id

    def _encode_persistent_metadata_delta(
        self,
        metadata: dict,
        frame_id: int,
        force_full: bool,
    ) -> dict:
        """Remove inherited JSON fields from an ordered persistent packet."""

        previous = self.persistent_metadata
        metadata["metadata_frame_id"] = frame_id
        self.persistent_metadata = metadata
        if force_full or previous is None:
            return metadata

        delta: dict = {
            "metadata_delta": "persistent_v1",
            "metadata_base_id": int(
                previous.get("metadata_frame_id", 0)
            ),
            "metadata_frame_id": frame_id,
            "temporal_reference_mode": "persistent_v1",
            "temporal_frame_id": frame_id,
        }
        for key, value in metadata.items():
            if key in (
                "cameras",
                "metadata_frame_id",
                "temporal_reference_mode",
                "temporal_frame_id",
            ):
                continue
            if previous.get(key) != value:
                delta[key] = value
        removed_top_level = [
            key
            for key in previous
            if key not in metadata
            and key not in ("cameras",)
        ]
        if removed_top_level:
            delta["metadata_removed"] = removed_top_level

        previous_cameras = previous.get("cameras", [])
        camera_deltas: list[dict] = []
        for index, camera in enumerate(metadata.get("cameras", [])):
            prior_camera = (
                previous_cameras[index]
                if index < len(previous_cameras)
                else {}
            )
            camera_delta = {
                key: value
                for key, value in camera.items()
                if prior_camera.get(key) != value
            }
            camera_delta["id"] = camera.get(
                "id",
                camera.get("serial", index),
            )
            removed_camera_fields = [
                key
                for key in prior_camera
                if key not in camera
            ]
            if removed_camera_fields:
                camera_delta["metadata_removed"] = (
                    removed_camera_fields
                )
            camera_deltas.append(camera_delta)
        delta["cameras"] = camera_deltas
        return delta

    @staticmethod
    def _apply_tile_updates(
        reference: np.ndarray,
        current: np.ndarray,
        changed: np.ndarray,
        tile: int,
    ) -> np.ndarray:
        """Return the exact reference implied by one absolute-tile delta."""

        if reference.shape != current.shape:
            raise ValueError("temporal reference dimensions changed")
        height, width = reference.shape[:2]
        rows = (height + tile - 1) // tile
        cols = (width + tile - 1) // tile
        if changed.shape != (rows * cols,):
            raise ValueError("invalid temporal changed-tile mask")
        output = reference.copy()
        for tile_index in np.flatnonzero(changed):
            tile_y = int(tile_index // cols) * tile
            tile_x = int(tile_index % cols) * tile
            output[
                tile_y : min(tile_y + tile, height),
                tile_x : min(tile_x + tile, width),
            ] = current[
                tile_y : min(tile_y + tile, height),
                tile_x : min(tile_x + tile, width),
            ]
        return output

    @staticmethod
    def _camera_key(camera: dict, index: int) -> str:
        return str(
            camera.get("id")
            or camera.get("serial")
            or camera.get("model")
            or index
        )

    def close(self) -> None:
        if self.worker_pool is not None:
            self.worker_pool.shutdown(wait=False, cancel_futures=True)
            self.worker_pool = None

    @staticmethod
    def _decode_depth(depth: bytes, camera: dict) -> np.ndarray:
        width = int(camera.get("width", 0))
        height = int(camera.get("height", 0))
        if width <= 0 or height <= 0:
            raise ValueError("invalid temporal depth dimensions")
        raw = zlib.decompress(depth)
        sample_count = width * height
        encoding = camera.get("depth_encoding")
        if encoding == "u16_mm_deflate":
            if len(raw) != sample_count * 2:
                raise ValueError("invalid raw temporal depth length")
            return np.frombuffer(raw, dtype="<u2").reshape(height, width).copy()
        if encoding != "u16_mm_vpredict_shuffle_deflate":
            raise ValueError(f"unsupported temporal depth source {encoding}")
        if len(raw) != sample_count * 2:
            raise ValueError("invalid predictive temporal depth length")
        shuffled = np.frombuffer(raw, dtype=np.uint8)
        zigzag = (
            shuffled[:sample_count].astype(np.uint16)
            | (shuffled[sample_count:].astype(np.uint16) << 8)
        )
        delta = (
            (zigzag >> 1).astype(np.int32)
            ^ -(zigzag & 1).astype(np.int32)
        ).reshape(height, width)
        # The native predictor is vertical; cumulative summation reconstructs
        # every row modulo uint16.
        return np.cumsum(delta, axis=0, dtype=np.int64).astype(np.uint16)

    def _decode_color(self, color: bytes, camera_key: str) -> np.ndarray:
        decoder = self.color_decoders.get(camera_key)
        if decoder is None:
            decoder = av.CodecContext.create("mjpeg", "r")
            self.color_decoders[camera_key] = decoder
        frames = decoder.decode(av.Packet(color))
        if not frames:
            raise ValueError("JPEG decoder produced no temporal color frame")
        return frames[-1].to_ndarray(format="rgb24")

    def _encode_color_atlas(self, atlas: np.ndarray) -> bytes:
        height, width = atlas.shape[:2]
        encoder_key = (width, height)
        encoder = self.color_encoders.get(encoder_key)
        if encoder is None:
            encoder = av.CodecContext.create("mjpeg", "w")
            encoder.width = width
            encoder.height = height
            encoder.pix_fmt = "yuvj420p"
            qscale = max(
                2,
                min(
                    20,
                    round(2 + (100 - self.color_quality) * 5 / 32),
                ),
            )
            encoder.qmin = qscale
            encoder.qmax = qscale
            encoder.open()
            self.color_encoders[encoder_key] = encoder
        frame = av.VideoFrame.from_ndarray(atlas, format="rgb24")
        return b"".join(bytes(packet) for packet in encoder.encode(frame))

    def _encode_color_delta(
        self,
        current: np.ndarray,
        camera_key: str,
        keyframe_id: int,
        keyframe: np.ndarray,
    ) -> tuple[bytes, int, int]:
        if current.shape != keyframe.shape or current.ndim != 3:
            raise ValueError("temporal color dimensions changed")
        height, width, channels = current.shape
        if channels != 3:
            raise ValueError("temporal color must be RGB")
        tile = self.color_tile_size
        tile_rows = (height + tile - 1) // tile
        tile_cols = (width + tile - 1) // tile
        padded = np.zeros((tile_rows * tile, tile_cols * tile, 3), dtype=np.uint8)
        keyframe_padded = np.zeros_like(padded)
        padded[:height, :width] = current
        keyframe_padded[:height, :width] = keyframe

        def as_tiles(values: np.ndarray) -> np.ndarray:
            return (
                values.reshape(tile_rows, tile, tile_cols, tile, 3)
                .transpose(0, 2, 1, 3, 4)
                .reshape(tile_rows * tile_cols, tile, tile, 3)
            )

        current_tiles = as_tiles(padded)
        keyframe_tiles = as_tiles(keyframe_padded)
        # Classification does not need every color sample. Complete selected
        # tiles are still transmitted, while quarter-density analysis keeps
        # temporal RGB comfortably inside a 30 FPS server budget. Full-sample
        # depth motion detection independently protects thin geometry.
        current_samples = np.ascontiguousarray(
            current_tiles[:, ::2, ::2]
        )
        keyframe_samples = np.ascontiguousarray(
            keyframe_tiles[:, ::2, ::2]
        )
        absolute = (
            np.maximum(current_samples, keyframe_samples)
            - np.minimum(current_samples, keyframe_samples)
        )
        maximum_channel_difference = np.max(absolute, axis=3)
        tile_mean_difference = np.mean(absolute, axis=(1, 2, 3))
        if self.persistent_reference:
            # Persistent RGB must not advance merely because JPEG blocks
            # shimmer. Confirm color-only changes below; confirmed geometry
            # motion is fused into this mask by encode() before packaging.
            changed = (
                (
                    np.count_nonzero(
                        maximum_channel_difference > 80,
                        axis=(1, 2),
                    )
                    >= 8
                )
                | (tile_mean_difference > 25.0)
            )
        else:
            changed = (
                (
                    np.count_nonzero(
                        maximum_channel_difference > 32,
                        axis=(1, 2),
                    )
                    >= 4
                )
                | (tile_mean_difference > 8.0)
            )

        state = self.color_change_states.get(camera_key)
        confirmation_stabilized_count = 0
        if state is not None and state[0].shape == changed.shape:
            change_runs, prior_signatures = state
            changed_indices = np.flatnonzero(changed)
            changed_maximum = maximum_channel_difference[changed_indices]
            changed_mean = tile_mean_difference[changed_indices]
            if self.persistent_reference:
                changed_immediate = (
                    (
                        np.count_nonzero(
                            changed_maximum > 140,
                            axis=(1, 2),
                        )
                        >= 16
                    )
                    | (changed_mean > 70.0)
                )
                required_change_run = 2
            else:
                changed_immediate = (
                    (
                        np.count_nonzero(
                            changed_maximum > 48,
                            axis=(1, 2),
                        )
                        >= 4
                    )
                    | (changed_mean > 18.0)
                )
                required_change_run = 2
            changed_signed = (
                current_samples[changed_indices].astype(np.int32)
                - keyframe_samples[changed_indices].astype(np.int32)
            )
            # Integer BT.601 luma weights avoid another float image.
            luma_net = (
                np.sum(changed_signed[..., 0], axis=(1, 2)) * 77
                + np.sum(changed_signed[..., 1], axis=(1, 2)) * 150
                + np.sum(changed_signed[..., 2], axis=(1, 2)) * 29
            )
            changed_signatures = np.where(
                luma_net > 0,
                1,
                np.where(luma_net < 0, 2, 0),
            ).astype(np.uint8)
            changed_consistent = (
                (changed_signatures != 0)
                & (
                    changed_signatures
                    == prior_signatures[changed_indices]
                )
            )
            change_runs[~changed] = 0
            prior_signatures[~changed] = 0
            change_runs[changed_indices] = np.where(
                changed_consistent,
                np.minimum(
                    change_runs[changed_indices].astype(np.uint16) + 1,
                    255,
                ),
                1,
            ).astype(np.uint8)
            prior_signatures[changed_indices] = changed_signatures
            changed_before_confirmation = changed.copy()
            changed[changed_indices] = (
                changed_immediate
                | (change_runs[changed_indices] >= required_change_run)
            )
            confirmation_stabilized_count = int(
                np.count_nonzero(changed_before_confirmation & ~changed)
            )

        self.last_color_changed_masks[camera_key] = changed.copy()
        return (
            self._pack_color_delta(current, keyframe_id, changed),
            int(np.count_nonzero(changed)),
            confirmation_stabilized_count,
        )

    def _pack_color_delta(
        self,
        current: np.ndarray,
        keyframe_id: int,
        changed: np.ndarray,
    ) -> bytes:
        """Package absolute RGB tiles for an already classified update mask."""

        height, width = current.shape[:2]
        tile = self.color_tile_size
        tile_rows = (height + tile - 1) // tile
        tile_cols = (width + tile - 1) // tile
        padded = np.zeros(
            (tile_rows * tile, tile_cols * tile, 3),
            dtype=np.uint8,
        )
        padded[:height, :width] = current
        current_tiles = (
            padded.reshape(tile_rows, tile, tile_cols, tile, 3)
            .transpose(0, 2, 1, 3, 4)
            .reshape(tile_rows * tile_cols, tile, tile, 3)
        )
        if changed.shape != (tile_rows * tile_cols,):
            raise ValueError("invalid temporal color update mask")

        changed_count = int(np.count_nonzero(changed))
        changed_bits = np.packbits(changed, bitorder="little").tobytes()
        atlas_jpeg = b""
        if changed_count:
            atlas_cols = min(tile_cols, changed_count)
            atlas_rows = (changed_count + atlas_cols - 1) // atlas_cols
            atlas_tiles = np.zeros(
                (atlas_rows * atlas_cols, tile, tile, 3),
                dtype=np.uint8,
            )
            atlas_tiles[:changed_count] = current_tiles[changed]
            atlas = (
                atlas_tiles.reshape(atlas_rows, atlas_cols, tile, tile, 3)
                .transpose(0, 2, 1, 3, 4)
                .reshape(atlas_rows * tile, atlas_cols * tile, 3)
            )
            atlas_jpeg = self._encode_color_atlas(atlas)

        payload = bytearray(
            TEMPORAL_COLOR_HEADER.pack(
                TEMPORAL_COLOR_MAGIC,
                keyframe_id,
                tile,
                tile_rows,
                tile_cols,
                changed_count,
            )
        )
        payload.extend(changed_bits)
        payload.extend(atlas_jpeg)
        return bytes(payload)

    @staticmethod
    def _color_mask_from_depth_motion(
        depth_changed: np.ndarray,
        depth_shape: tuple[int, int],
        depth_tile: int,
        color_shape: tuple[int, int],
        color_tile: int,
    ) -> np.ndarray:
        """Map aligned depth-motion tiles onto overlapping RGB tiles."""

        depth_height, depth_width = depth_shape
        color_height, color_width = color_shape
        depth_rows = (depth_height + depth_tile - 1) // depth_tile
        depth_cols = (depth_width + depth_tile - 1) // depth_tile
        color_rows = (color_height + color_tile - 1) // color_tile
        color_cols = (color_width + color_tile - 1) // color_tile
        if depth_changed.shape != (depth_rows * depth_cols,):
            raise ValueError("invalid depth-motion mask for RGB mapping")
        forced = np.zeros(color_rows * color_cols, dtype=bool)
        for depth_index in np.flatnonzero(depth_changed):
            depth_y = int(depth_index // depth_cols) * depth_tile
            depth_x = int(depth_index % depth_cols) * depth_tile
            color_y0 = int(np.floor(depth_y * color_height / depth_height))
            color_x0 = int(np.floor(depth_x * color_width / depth_width))
            color_y1 = int(
                np.ceil(
                    min(depth_y + depth_tile, depth_height)
                    * color_height
                    / depth_height
                )
            )
            color_x1 = int(
                np.ceil(
                    min(depth_x + depth_tile, depth_width)
                    * color_width
                    / depth_width
                )
            )
            for color_y in range(
                color_y0 // color_tile,
                (max(color_y0 + 1, color_y1) - 1) // color_tile + 1,
            ):
                for color_x in range(
                    color_x0 // color_tile,
                    (max(color_x0 + 1, color_x1) - 1) // color_tile + 1,
                ):
                    forced[color_y * color_cols + color_x] = True
        return forced

    def _mark_keyframe(
        self,
        camera: dict,
        camera_key: str,
        depth_values: np.ndarray,
        color_values: np.ndarray | None,
        reference_id: int | None = None,
    ) -> None:
        keyframe_id = (
            self._next_reference_id()
            if reference_id is None
            else int(reference_id)
        )
        plane_candidates = self._plane_candidates(depth_values)
        self.keyframes[camera_key] = (
            keyframe_id,
            depth_values.copy(),
            plane_candidates,
        )
        self.change_states[camera_key] = (
            np.zeros(plane_candidates.shape, dtype=np.uint8),
            np.zeros(plane_candidates.shape, dtype=np.uint8),
        )
        if color_values is not None:
            color_tile_rows = (
                color_values.shape[0] + self.color_tile_size - 1
            ) // self.color_tile_size
            color_tile_cols = (
                color_values.shape[1] + self.color_tile_size - 1
            ) // self.color_tile_size
            color_tile_count = color_tile_rows * color_tile_cols
            self.color_keyframes[camera_key] = (
                keyframe_id,
                color_values.copy(),
            )
            self.color_change_states[camera_key] = (
                np.zeros(color_tile_count, dtype=np.uint8),
                np.zeros(color_tile_count, dtype=np.uint8),
            )
            camera["color_temporal_codec"] = "keyframe_tiles_jpeg_v1"
            camera["color_temporal_keyframe"] = True
            camera["color_keyframe_id"] = keyframe_id
            camera["color_tile_size"] = self.color_tile_size
            if self.persistent_reference:
                camera["color_frame_id"] = keyframe_id
                camera["color_temporal_reference_mode"] = "persistent_v1"
        camera["depth_temporal_codec"] = "keyframe_tiles_v1"
        camera["depth_temporal_keyframe"] = True
        camera["depth_keyframe_id"] = keyframe_id
        camera["depth_tile_size"] = self.tile_size
        if self.persistent_reference:
            camera["depth_frame_id"] = keyframe_id
            camera["depth_temporal_reference_mode"] = "persistent_v1"
        camera["depth_stability_mm"] = self.stability_mm
        camera["depth_plane_stabilization"] = self.plane_stabilization
        camera["depth_motion_confirmation"] = self.motion_confirmation
        camera["depth_plane_candidate_tiles"] = int(
            np.count_nonzero(plane_candidates)
        )

    def _plane_candidates(self, depth_values: np.ndarray) -> np.ndarray:
        """Identify smooth partial-valid surfaces in a recovery keyframe."""

        height, width = depth_values.shape
        tile = self.tile_size
        tile_rows = (height + tile - 1) // tile
        tile_cols = (width + tile - 1) // tile
        padded = np.zeros(
            (tile_rows * tile, tile_cols * tile),
            dtype=np.uint16,
        )
        padded[:height, :width] = depth_values
        values = (
            padded.reshape(tile_rows, tile, tile_cols, tile)
            .transpose(0, 2, 1, 3)
            .reshape(tile_rows * tile_cols, tile * tile)
        )
        if not self.plane_stabilization:
            return np.zeros(values.shape[0], dtype=bool)

        valid = values > 0
        valid_count = np.count_nonzero(valid, axis=1)
        minimum_valid = max(3, int(np.ceil(tile * tile * 0.75)))
        weights = valid.astype(np.float64)
        normal = np.einsum(
            "ki,nk,kj->nij",
            self.plane_design,
            weights,
            self.plane_design,
        )
        # Invalid/sparse tiles are rejected below. A tiny ridge keeps their
        # otherwise singular normal matrices safe for the batched solve.
        diagonal = np.arange(3)
        normal[:, diagonal, diagonal] += 1e-6
        right_hand = np.einsum(
            "ki,nk,nk->ni",
            self.plane_design,
            weights,
            values.astype(np.float64),
        )
        coefficients = np.linalg.solve(
            normal,
            right_hand[..., np.newaxis],
        )[..., 0]
        predicted = coefficients @ self.plane_design.T
        residual = np.abs(values.astype(np.float64) - predicted)
        residual_outliers = np.count_nonzero(
            valid & (residual > 7.0),
            axis=1,
        )
        allowed_outliers = np.floor(valid_count * 0.05).astype(np.int32)
        return (
            (valid_count >= minimum_valid)
            & (residual_outliers <= allowed_outliers)
        )

    def _encode_delta(
        self,
        current: np.ndarray,
        camera_key: str,
        keyframe_id: int,
        keyframe: np.ndarray,
        plane_candidates: np.ndarray,
    ) -> tuple[bytes, int, int, int]:
        height, width = current.shape
        tile = self.tile_size
        tile_rows = (height + tile - 1) // tile
        tile_cols = (width + tile - 1) // tile
        padded_height = tile_rows * tile
        padded_width = tile_cols * tile
        current_padded = np.zeros((padded_height, padded_width), dtype=np.uint16)
        keyframe_padded = np.zeros_like(current_padded)
        current_padded[:height, :width] = current
        keyframe_padded[:height, :width] = keyframe

        validity_changed = (current_padded == 0) != (keyframe_padded == 0)
        both_valid = (current_padded > 0) & (keyframe_padded > 0)
        difference = np.zeros_like(current_padded, dtype=np.int32)
        difference[both_valid] = np.abs(
            current_padded[both_valid].astype(np.int32)
            - keyframe_padded[both_valid].astype(np.int32)
        )

        def as_tiles(values: np.ndarray) -> np.ndarray:
            return (
                values.reshape(tile_rows, tile, tile_cols, tile)
                .transpose(0, 2, 1, 3)
                .reshape(tile_rows * tile_cols, tile * tile)
            )

        validity_tiles = as_tiles(validity_changed)
        difference_tiles = as_tiles(difference)
        current_tiles = as_tiles(current_padded)
        keyframe_tiles = as_tiles(keyframe_padded)
        current_valid = current_tiles > 0
        keyframe_valid = keyframe_tiles > 0
        signed_difference = (
            current_tiles.astype(np.int32)
            - keyframe_tiles.astype(np.int32)
        )
        signed_difference[~(current_valid & keyframe_valid)] = 0
        validity_grid = validity_tiles.reshape(-1, tile, tile)
        if self.persistent_reference:
            validity_change_count = np.count_nonzero(
                validity_tiles,
                axis=1,
            )
            changed = (
                (validity_change_count >= 16)
                | (
                    np.count_nonzero(
                        difference_tiles > 8,
                        axis=1,
                    )
                    >= 8
                )
                | (
                    np.count_nonzero(
                        difference_tiles > 15,
                        axis=1,
                    )
                    >= 4
                )
                | (np.max(difference_tiles, axis=1) > 80)
            )
        else:
            changed = (
                np.any(validity_tiles, axis=1)
                | (
                    np.count_nonzero(
                        difference_tiles > self.stability_mm,
                        axis=1,
                    )
                    >= self.minimum_changed_pixels
                )
                | (
                    np.max(difference_tiles, axis=1)
                    > self.single_pixel_change_mm
                )
            )
        changed_before_plane_stabilization = changed.copy()
        if (
            self.plane_stabilization
            and plane_candidates.shape == changed.shape
        ):
            samples_per_tile = tile * tile
            depth_anomaly_grid = (
                difference_tiles > self.single_pixel_change_mm
            ).reshape(-1, tile, tile)
            coherent_validity_change = np.any(
                validity_grid[:, :-1, :-1]
                & validity_grid[:, 1:, :-1]
                & validity_grid[:, :-1, 1:]
                & validity_grid[:, 1:, 1:],
                axis=(1, 2),
            )
            adjacent_depth_anomaly = (
                np.any(
                    depth_anomaly_grid[:, :, :-1]
                    & depth_anomaly_grid[:, :, 1:],
                    axis=(1, 2),
                )
                | np.any(
                    depth_anomaly_grid[:, :-1, :]
                    & depth_anomaly_grid[:, 1:, :],
                    axis=(1, 2),
                )
            )
            maximum_validity_flicker = max(
                1,
                int(np.ceil(samples_per_tile * 0.0625)),
            )
            coherent_depth_pixels = max(
                1,
                int(np.ceil(samples_per_tile * 0.0625)),
            )
            coherent_direction_pixels = max(
                2,
                int(np.ceil(samples_per_tile * 0.125)),
            )
            minimum_current_valid = max(
                3,
                int(np.ceil(samples_per_tile * 0.75)),
            )
            plane_geometry_changed = (
                (
                    np.count_nonzero(current_valid, axis=1)
                    < minimum_current_valid
                )
                | (
                    np.count_nonzero(validity_tiles, axis=1)
                    > maximum_validity_flicker
                )
                | coherent_validity_change
                | (np.max(difference_tiles, axis=1) > 15)
                | (
                    np.count_nonzero(
                        difference_tiles > self.single_pixel_change_mm,
                        axis=1,
                    )
                    >= coherent_depth_pixels
                )
                | adjacent_depth_anomaly
                | (
                    np.count_nonzero(
                        signed_difference > self.stability_mm + 2,
                        axis=1,
                    )
                    >= coherent_direction_pixels
                )
                | (
                    np.count_nonzero(
                        signed_difference < -(self.stability_mm + 2),
                        axis=1,
                    )
                    >= coherent_direction_pixels
                )
            )
            # Plane awareness may only suppress an ordinary update. It never
            # creates a tile update that the base detector would have omitted.
            changed &= ~plane_candidates | plane_geometry_changed
            current_tiles_for_payload = current_tiles
        else:
            current_tiles_for_payload = current_tiles

        if self.persistent_reference:
            # Persistent references do not receive periodic recovery
            # keyframes.  Consequently, even a thin valid/invalid change at
            # a moving silhouette must remain eligible for temporal
            # confirmation.  The coarse detector above deliberately requires
            # 16 changed validity samples to reject RealSense edge shimmer;
            # leaving smaller changes ineligible, however, preserves their old
            # depth forever and produces floating points after an object has
            # moved away.  Motion confirmation below still requires four
            # frames with the same validity direction, so transient holes stay
            # suppressed while real disocclusions are eventually cleared.
            changed |= np.any(validity_tiles, axis=1)

        stabilized_count = int(
            np.count_nonzero(changed_before_plane_stabilization & ~changed)
        )
        confirmation_stabilized_count = 0
        state = self.change_states.get(camera_key)
        if (
            self.motion_confirmation
            and state is not None
            and state[0].shape == changed.shape
        ):
            change_runs, prior_signatures = state
            changed_indices = np.flatnonzero(changed)
            changed_signed = signed_difference[changed_indices]
            changed_absolute = difference_tiles[changed_indices]
            changed_validity = validity_grid[changed_indices]
            changed_current_valid = current_valid[changed_indices]
            changed_keyframe_valid = keyframe_valid[changed_indices]

            # A genuine displacement has spatial support and bypasses temporal
            # confirmation. Less structured changes must repeat with the same
            # dominant direction once (about 33 ms at 30 FPS). RealSense edge
            # shimmer usually changes direction or validity signature instead.
            coherent_2x2_validity = np.any(
                changed_validity[:, :-1, :-1]
                & changed_validity[:, 1:, :-1]
                & changed_validity[:, :-1, 1:]
                & changed_validity[:, 1:, 1:],
                axis=(1, 2),
            )
            if self.persistent_reference:
                changed_immediate = (
                    (
                        np.count_nonzero(changed_signed > 20, axis=1)
                        >= 8
                    )
                    | (
                        np.count_nonzero(changed_signed < -20, axis=1)
                        >= 8
                    )
                    | (
                        np.count_nonzero(changed_absolute > 40, axis=1)
                        >= 4
                    )
                    | (
                        np.count_nonzero(changed_absolute > 80, axis=1)
                        >= 2
                    )
                )
                required_change_run = 4
            else:
                changed_immediate = (
                    coherent_2x2_validity
                    | (
                        np.count_nonzero(changed_signed > 12, axis=1)
                        >= 8
                    )
                    | (
                        np.count_nonzero(changed_signed < -12, axis=1)
                        >= 8
                    )
                    | (
                        np.count_nonzero(changed_absolute > 25, axis=1)
                        >= 4
                    )
                    | (
                        np.count_nonzero(changed_absolute > 50, axis=1)
                        >= 2
                    )
                )
                required_change_run = 2
            depth_net = np.sum(
                changed_signed,
                axis=1,
                dtype=np.int64,
            )
            validity_net = (
                np.count_nonzero(changed_current_valid, axis=1)
                - np.count_nonzero(changed_keyframe_valid, axis=1)
            )
            changed_signatures = np.zeros(
                changed_indices.shape,
                dtype=np.uint8,
            )
            changed_signatures[depth_net > self.stability_mm] = 1
            changed_signatures[depth_net < -self.stability_mm] = 2
            no_depth_direction = changed_signatures == 0
            changed_signatures[
                no_depth_direction & (validity_net > 0)
            ] = 3
            changed_signatures[
                no_depth_direction & (validity_net < 0)
            ] = 4
            changed_consistent = (
                (changed_signatures != 0)
                & (
                    changed_signatures
                    == prior_signatures[changed_indices]
                )
            )
            change_runs[~changed] = 0
            prior_signatures[~changed] = 0
            change_runs[changed_indices] = np.where(
                changed_consistent,
                np.minimum(
                    change_runs[changed_indices].astype(np.uint16) + 1,
                    255,
                ),
                1,
            ).astype(np.uint8)
            prior_signatures[changed_indices] = changed_signatures
            changed_confirmed = (
                change_runs[changed_indices] >= required_change_run
            )
            changed_before_confirmation = changed.copy()
            changed[changed_indices] = (
                changed_immediate | changed_confirmed
            )
            confirmation_stabilized_count = int(
                np.count_nonzero(changed_before_confirmation & ~changed)
            )

        changed_count = int(np.count_nonzero(changed))
        changed_bits = np.packbits(changed, bitorder="little").tobytes()
        changed_values = current_tiles_for_payload[changed].reshape(-1)
        low = (changed_values & 0xFF).astype(np.uint8).tobytes()
        high = (changed_values >> 8).astype(np.uint8).tobytes()
        raw = bytearray(
            TEMPORAL_DEPTH_HEADER.pack(
                TEMPORAL_DEPTH_MAGIC,
                keyframe_id,
                tile,
                tile_rows,
                tile_cols,
                changed_count,
            )
        )
        raw.extend(changed_bits)
        raw.extend(low)
        raw.extend(high)
        self.last_depth_changed_masks[camera_key] = changed.copy()
        return (
            zlib.compress(raw, 1),
            changed_count,
            stabilized_count,
            confirmation_stabilized_count,
        )

    def encode(self, packet: bytes) -> tuple[bytes, bool]:
        """Return ``(packet, contains_priority_keyframe)``."""

        if len(packet) < 9 or packet[:5] != b"RGBD1":
            self.keyframes.clear()
            self.change_states.clear()
            self.color_keyframes.clear()
            self.color_change_states.clear()
            self.persistent_metadata = None
            return packet, False
        try:
            metadata_length = struct.unpack_from("<I", packet, 5)[0]
            payload_offset = 9 + metadata_length
            if payload_offset > len(packet):
                raise ValueError("truncated RGB-D metadata")
            metadata = json.loads(packet[9:payload_offset])
            payloads: list[tuple[bytes, bytes]] = []
            priority_keyframe = False
            force_periodic_keyframe = (
                not self.persistent_reference
                and self.frame_index % self.keyframe_interval == 0
            )
            persistent_result_id = (
                self._next_reference_id()
                if self.persistent_reference
                else None
            )
            if persistent_result_id is not None:
                metadata["temporal_reference_mode"] = "persistent_v1"
                metadata["temporal_frame_id"] = persistent_result_id
            for index, camera in enumerate(metadata.get("cameras", [])):
                color_length = int(camera.get("color_length", 0))
                depth_length = int(camera.get("depth_length", 0))
                color_end = payload_offset + color_length
                depth_end = color_end + depth_length
                if color_end > len(packet) or depth_end > len(packet):
                    raise ValueError("truncated RGB-D temporal payload")
                color = packet[payload_offset:color_end]
                depth = packet[color_end:depth_end]
                payload_offset = depth_end
                camera_key = self._camera_key(camera, index)
                color_decode_future: Future | None = None
                if self.temporal_color and self.worker_pool is not None:
                    color_decode_future = self.worker_pool.submit(
                        self._decode_color,
                        color,
                        camera_key,
                    )
                depth_values = self._decode_depth(depth, camera)
                color_values = (
                    color_decode_future.result()
                    if color_decode_future is not None
                    else None
                )
                prior = self.keyframes.get(camera_key)
                color_prior = self.color_keyframes.get(camera_key)
                needs_keyframe = (
                    force_periodic_keyframe
                    or prior is None
                    or prior[1].shape != depth_values.shape
                    or (
                        self.temporal_color
                        and (
                            color_prior is None
                            or color_values is None
                            or color_prior[1].shape != color_values.shape
                            or color_prior[0] != prior[0]
                        )
                    )
                )
                color_delta_future: Future | None = None
                if not needs_keyframe:
                    keyframe_id, keyframe, plane_candidates = prior
                    if (
                        self.temporal_color
                        and color_values is not None
                        and color_prior is not None
                        and self.worker_pool is not None
                    ):
                        color_delta_future = self.worker_pool.submit(
                            self._encode_color_delta,
                            color_values,
                            camera_key,
                            color_prior[0],
                            color_prior[1],
                        )
                    (
                        temporal_depth,
                        changed_count,
                        stabilized_count,
                        confirmation_stabilized_count,
                    ) = (
                        self._encode_delta(
                            depth_values,
                            camera_key,
                            keyframe_id,
                            keyframe,
                            plane_candidates,
                        )
                    )
                    if len(temporal_depth) < len(depth):
                        depth = temporal_depth
                        camera["depth_length"] = len(depth)
                        camera["depth_encoding"] = TEMPORAL_DEPTH_ENCODING
                        camera["depth_temporal_codec"] = "keyframe_tiles_v1"
                        camera["depth_temporal_keyframe"] = False
                        camera["depth_keyframe_id"] = keyframe_id
                        camera["depth_tile_size"] = self.tile_size
                        camera["depth_changed_tiles"] = changed_count
                        camera["depth_stability_mm"] = self.stability_mm
                        camera["depth_plane_stabilization"] = (
                            self.plane_stabilization
                        )
                        camera["depth_motion_confirmation"] = (
                            self.motion_confirmation
                        )
                        camera["depth_plane_stabilized_tiles"] = (
                            stabilized_count
                        )
                        camera["depth_confirmation_stabilized_tiles"] = (
                            confirmation_stabilized_count
                        )
                    else:
                        needs_keyframe = True
                if needs_keyframe:
                    if color_delta_future is not None:
                        # Let the worker finish before keyframe state replaces
                        # the delta confirmation arrays it is updating.
                        color_delta_future.result()
                    self._mark_keyframe(
                        camera,
                        camera_key,
                        depth_values,
                        color_values,
                        reference_id=persistent_result_id,
                    )
                    priority_keyframe = True
                elif self.temporal_color and color_values is not None:
                    (
                        temporal_color,
                        color_changed_count,
                        color_confirmation_stabilized_count,
                    ) = color_delta_future.result()
                    color_keyframe_id = color_prior[0]
                    color_mask = self.last_color_changed_masks.get(
                        camera_key
                    )
                    depth_mask = self.last_depth_changed_masks.get(
                        camera_key
                    )
                    if color_mask is None or depth_mask is None:
                        raise ValueError(
                            "missing temporal RGB-D update mask"
                        )
                    forced_color_mask = self._color_mask_from_depth_motion(
                        depth_mask,
                        depth_values.shape,
                        self.tile_size,
                        color_values.shape[:2],
                        self.color_tile_size,
                    )
                    combined_color_mask = color_mask | forced_color_mask
                    if np.any(combined_color_mask != color_mask):
                        temporal_color = self._pack_color_delta(
                            color_values,
                            color_keyframe_id,
                            combined_color_mask,
                        )
                        color_changed_count = int(
                            np.count_nonzero(combined_color_mask)
                        )
                        self.last_color_changed_masks[camera_key] = (
                            combined_color_mask.copy()
                        )
                    camera["color_temporal_codec"] = (
                        "keyframe_tiles_jpeg_v1"
                    )
                    camera["color_temporal_keyframe"] = False
                    camera["color_keyframe_id"] = color_keyframe_id
                    camera["color_tile_size"] = self.color_tile_size
                    camera["color_changed_tiles"] = color_changed_count
                    camera["color_confirmation_stabilized_tiles"] = (
                        color_confirmation_stabilized_count
                    )
                    if len(temporal_color) < len(color):
                        color = temporal_color
                        camera["color_length"] = len(color)
                        camera["color_encoding"] = TEMPORAL_COLOR_ENCODING
                        camera["color_temporal_fallback_full"] = False
                    else:
                        camera["color_temporal_fallback_full"] = True
                        if self.persistent_reference:
                            camera["color_temporal_keyframe"] = True
                            camera["color_keyframe_id"] = (
                                persistent_result_id
                            )
                if self.persistent_reference and not needs_keyframe:
                    depth_mask = self.last_depth_changed_masks.get(camera_key)
                    if depth_mask is None:
                        raise ValueError("missing persistent depth update mask")
                    persistent_depth = self._apply_tile_updates(
                        prior[1],
                        depth_values,
                        depth_mask,
                        self.tile_size,
                    )
                    self.keyframes[camera_key] = (
                        persistent_result_id,
                        persistent_depth,
                        prior[2],
                    )
                    camera["depth_frame_id"] = persistent_result_id
                    camera["depth_temporal_reference_mode"] = (
                        "persistent_v1"
                    )
                    depth_state = self.change_states.get(camera_key)
                    if depth_state is not None:
                        depth_state[0][depth_mask] = 0
                        depth_state[1][depth_mask] = 0

                    if self.temporal_color and color_values is not None:
                        if camera.get("color_temporal_fallback_full"):
                            persistent_color = color_values.copy()
                        else:
                            color_mask = self.last_color_changed_masks.get(
                                camera_key
                            )
                            if color_mask is None:
                                raise ValueError(
                                    "missing persistent color update mask"
                                )
                            persistent_color = self._apply_tile_updates(
                                color_prior[1],
                                color_values,
                                color_mask,
                                self.color_tile_size,
                            )
                            color_state = self.color_change_states.get(
                                camera_key
                            )
                            if color_state is not None:
                                color_state[0][color_mask] = 0
                                color_state[1][color_mask] = 0
                        self.color_keyframes[camera_key] = (
                            persistent_result_id,
                            persistent_color,
                        )
                        camera["color_frame_id"] = persistent_result_id
                        camera["color_temporal_reference_mode"] = (
                            "persistent_v1"
                        )
                payloads.append((color, depth))

            self.frame_index += 1
            wire_metadata = metadata
            if persistent_result_id is not None:
                wire_metadata = self._encode_persistent_metadata_delta(
                    metadata,
                    persistent_result_id,
                    priority_keyframe,
                )
            metadata_bytes = json.dumps(
                wire_metadata,
                separators=(",", ":"),
            ).encode("utf-8")
            rebuilt = bytearray(b"RGBD1")
            rebuilt.extend(struct.pack("<I", len(metadata_bytes)))
            rebuilt.extend(metadata_bytes)
            for color, depth in payloads:
                rebuilt.extend(color)
                rebuilt.extend(depth)
            return bytes(rebuilt), priority_keyframe
        except (
            TypeError,
            ValueError,
            KeyError,
            json.JSONDecodeError,
            zlib.error,
            av.error.FFmpegError,
        ):
            self.keyframes.clear()
            self.change_states.clear()
            self.color_keyframes.clear()
            self.color_change_states.clear()
            self.persistent_metadata = None
            self.frame_index = 0
            return packet, False


class ClawCameraHub:
    """One persistent camera capture shared by all MJPEG browser clients."""

    def __init__(self) -> None:
        self.condition = threading.Condition()
        self.process: subprocess.Popen | None = None
        self.reader_thread: threading.Thread | None = None
        self.frame = b""
        self.sequence = 0
        self.capture_unix_ms = 0
        self.config: tuple[str, int, int, int] | None = None
        self.last_error = ""
        self.stopping = False

    def ensure_running(self, device: str, width: int, height: int, fps: int) -> bool:
        with self.condition:
            if self.process is not None and self.process.poll() is None:
                return True
            self.stopping = False
            command = [
                ffmpeg_binary(),
                "-hide_banner", "-loglevel", "warning",
                "-fflags", "nobuffer", "-flags", "low_delay",
                "-f", "v4l2", "-input_format", "mjpeg",
                "-video_size", f"{width}x{height}",
                "-framerate", str(fps),
                "-i", device,
                "-an", "-c:v", "copy",
                "-f", "image2pipe", "pipe:1",
            ]
            try:
                process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
            except OSError as exc:
                self.last_error = str(exc)
                return False
            self.process = process
            self.config = (device, width, height, fps)
            self.last_error = ""
            self.reader_thread = threading.Thread(target=self._read_frames, args=(process,), daemon=True)
            self.reader_thread.start()
            return True

    def _read_frames(self, process: subprocess.Popen) -> None:
        stderr_chunks: deque[bytes] = deque(maxlen=8)

        def drain_stderr() -> None:
            if process.stderr is None:
                return
            for chunk in iter(process.stderr.readline, b""):
                stderr_chunks.append(chunk[-512:])

        threading.Thread(target=drain_stderr, daemon=True).start()
        buffer = bytearray()
        try:
            assert process.stdout is not None
            while not self.stopping:
                chunk = process.stdout.read(65536)
                if not chunk:
                    break
                buffer.extend(chunk)
                while True:
                    start = buffer.find(b"\xff\xd8")
                    if start < 0:
                        if len(buffer) > 2:
                            del buffer[:-2]
                        break
                    end = buffer.find(b"\xff\xd9", start + 2)
                    if end < 0:
                        if start > 0:
                            del buffer[:start]
                        break
                    frame = bytes(buffer[start:end + 2])
                    del buffer[:end + 2]
                    with self.condition:
                        if self.process is not process:
                            return
                        self.frame = frame
                        self.sequence += 1
                        self.capture_unix_ms = int(time.time() * 1000)
                        self.condition.notify_all()
                if len(buffer) > 8 * 1024 * 1024:
                    buffer.clear()
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
            with self.condition:
                if self.process is process:
                    self.process = None
                    if not self.stopping and stderr_chunks:
                        self.last_error = b"".join(stderr_chunks).decode("utf-8", errors="replace").strip()
                    self.condition.notify_all()

    def wait_for_frame(self, after_sequence: int, timeout: float = 2.0) -> tuple[int, bytes, int]:
        deadline = time.monotonic() + timeout
        with self.condition:
            while self.sequence <= after_sequence and time.monotonic() < deadline:
                if self.process is None:
                    break
                self.condition.wait(timeout=max(0.01, deadline - time.monotonic()))
            return self.sequence, self.frame, self.capture_unix_ms

    def stop(self) -> None:
        with self.condition:
            self.stopping = True
            process = self.process
            self.condition.notify_all()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()


class LatestRgbdSource:
    """Continuously drain one local RGB-D stream and retain only its newest packet."""

    def __init__(self, source_url: str) -> None:
        self.source_url = source_url
        self.condition = threading.Condition()
        self.packet = b""
        self.sequence = 0
        self.client_claims: dict[str, tuple[int, float]] = {}
        self.last_claim_at = time.monotonic()
        self.last_error = ""
        self.stopping = threading.Event()
        self.reader: MultipartRawFrameReader | None = None
        self.thread = threading.Thread(
            target=self._drain,
            name="rgbd-latest-source",
            daemon=True,
        )
        self.thread.start()

    def _drain(self) -> None:
        while not self.stopping.is_set():
            reader = None
            try:
                reader = MultipartRawFrameReader(self.source_url)
                self.reader = reader
                while not self.stopping.is_set():
                    _headers, packet = reader.read_part()
                    if not packet:
                        continue
                    now = time.monotonic()
                    packed_packet = transcode_rgbd_depth_predictive(packet)
                    with self.condition:
                        if now - self.last_claim_at > 60.0:
                            self.stopping.set()
                            self.condition.notify_all()
                            break
                        self.packet = packed_packet
                        self.sequence += 1
                        self.last_error = ""
                        if self.sequence % 300 == 0:
                            cutoff = now - 120.0
                            self.client_claims = {
                                client_id: claim
                                for client_id, claim in self.client_claims.items()
                                if claim[1] >= cutoff
                            }
                        self.condition.notify_all()
            except (ConnectionError, OSError, ValueError) as exc:
                with self.condition:
                    self.last_error = str(exc)
                    self.condition.notify_all()
                self.stopping.wait(0.25)
            finally:
                if reader is not None:
                    reader.close()
                if self.reader is reader:
                    self.reader = None

    def claim_after(
        self,
        client_id: str,
        after_sequence: int,
        timeout: float = 0.8,
    ) -> tuple[int, bytes, str]:
        """Assign each new source frame to at most one pending request per client."""

        deadline = time.monotonic() + timeout
        with self.condition:
            self.last_claim_at = time.monotonic()
            while not self.stopping.is_set():
                claimed_sequence = self.client_claims.get(client_id, (0, 0.0))[0]
                if (
                    self.packet
                    and self.sequence > after_sequence
                    and self.sequence > claimed_sequence
                ):
                    now = time.monotonic()
                    self.client_claims[client_id] = (self.sequence, now)
                    return self.sequence, self.packet, ""
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return self.sequence, b"", self.last_error
                self.condition.wait(timeout=remaining)
        return self.sequence, b"", "RGB-D source stopped"

    def stop(self) -> None:
        self.stopping.set()
        reader = self.reader
        if reader is not None:
            reader.close()
        with self.condition:
            self.condition.notify_all()
        self.thread.join(timeout=1.0)


class LatestRgbdHub:
    """Share latest-only local RGB-D sources across cancellable HTTP requests."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.current_source: LatestRgbdSource | None = None

    def source(self, source_url: str) -> LatestRgbdSource:
        with self.lock:
            source = self.current_source
            if (
                source is None
                or source.stopping.is_set()
                or not source.thread.is_alive()
            ):
                source = LatestRgbdSource(source_url)
                self.current_source = source
            return source

    def stop(self) -> None:
        with self.lock:
            source = self.current_source
            self.current_source = None
        if source is not None:
            source.stop()


class NvencH264Encoder(h264.H264Encoder):
    """aiortc-compatible H.264 packetizer backed by NVENC when available."""

    def __init__(self) -> None:
        super().__init__()
        self.encoder_name = "h264_nvenc"

    def _create_codec(self, frame: av.VideoFrame):
        codec = av.CodecContext.create(self.encoder_name, "w")
        if self.encoder_name == "h264_nvenc":
            codec.options = {
                "preset": "p4",
                "tune": "ull",
                "rc": "cbr",
                "delay": "0",
                "zerolatency": "1",
                "bf": "0",
                "g": "30",
                "rc-lookahead": "0",
                "spatial-aq": "1",
                "aq-strength": "8",
                "repeat-headers": "1",
                "forced-idr": "1",
                "profile": "baseline",
            }
        else:
            codec.options = {
                "preset": "ultrafast",
                "tune": "zerolatency",
                "bf": "0",
                "g": "30",
                "level": "31",
            }
            codec.profile = "Baseline"
        codec.width = frame.width
        codec.height = frame.height
        codec.bit_rate = self.target_bitrate
        codec.pix_fmt = "yuv420p"
        codec.framerate = fractions.Fraction(h264.MAX_FRAME_RATE, 1)
        codec.time_base = fractions.Fraction(1, h264.MAX_FRAME_RATE)
        return codec

    def _encode_frame(self, frame: av.VideoFrame, force_keyframe: bool):
        if self.codec and (
            frame.width != self.codec.width
            or frame.height != self.codec.height
        ):
            self.buffer_data = b""
            self.buffer_pts = None
            self.codec = None
        elif self.codec and self.codec.bit_rate != self.target_bitrate:
            # NVENC supports runtime rate reconfiguration; rebuilding it drops queued frames.
            self.codec.bit_rate = self.target_bitrate
        frame.pict_type = (
            av.video.frame.PictureType.I if force_keyframe else av.video.frame.PictureType.NONE
        )
        for _attempt in range(2):
            try:
                opened_now = self.codec is None
                if self.codec is None:
                    self.codec = self._create_codec(frame)
                data_to_send = b"".join(bytes(packet) for packet in self.codec.encode(frame))
                if data_to_send:
                    yield from self._split_bitstream(data_to_send)
                if opened_now:
                    print(
                        f"WebRTC H.264 encoder opened: {self.encoder_name} "
                        f"{frame.width}x{frame.height} {self.target_bitrate // 1000} kbps",
                        flush=True,
                    )
                return
            except Exception as exc:
                self.codec = None
                self.buffer_data = b""
                self.buffer_pts = None
                if self.encoder_name != "h264_nvenc":
                    raise
                print(f"NVENC unavailable at runtime, using ultrafast libx264: {exc}", flush=True)
                self.encoder_name = "libx264"


aiortc_codecs.H264Encoder = NvencH264Encoder


class LanRemoteServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address,
        handler,
        root: Path,
        udp_host: str,
        udp_port: int,
        stream_host: str,
        stream_port: int,
        password: str,
        arm_command_port: int = 4248,
        arm_status_port: int = 4249,
        public_arm: bool = False,
        allow_quick_tunnel_arm: bool = False,
        public_hostname: str = "",
        access_team_domain: str = "",
        access_audience: str = "",
        robot_calibration_status_port: int = 4251,
        claw_camera_device: str = "",
    ):
        super().__init__(address, handler)
        self.root = root
        self.udp_host = udp_host
        self.udp_port = udp_port
        self.stream_host = stream_host
        self.stream_port = stream_port
        self.password = password
        self.arm_command_port = arm_command_port
        self.arm_status_port = arm_status_port
        self.robot_calibration_status_port = robot_calibration_status_port
        self.public_arm = public_arm
        self.allow_quick_tunnel_arm = allow_quick_tunnel_arm
        self.public_hostname = public_hostname.strip().lower()
        self.access_team_domain = access_team_domain.strip().lower().removeprefix("https://").rstrip("/")
        self.access_audience = access_audience.strip()
        self.access_jwk_client = None
        self.session_cookie = base64.urlsafe_b64encode(os.urandom(32)).decode("ascii").rstrip("=")
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.arm_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.packet_count = 0
        self.last_report_sec = time.monotonic()
        self.last_report_count = 0
        self.peer_connections: set[RTCPeerConnection] = set()
        self.peer_lock = threading.Lock()
        self.ffmpeg_slots = threading.BoundedSemaphore(MAX_FFMPEG_PROCESSES)
        # Streaming and arm control share one newest-browser-wins owner. A
        # stable per-page ID prevents a superseded tab from stealing either
        # channel back during transport fallback or reconnect.
        self.browser_client_lock = threading.Lock()
        self.browser_client_id: str | None = None
        self.browser_client_handlers: dict[str, object | None] = {
            "rgbd": None,
            "arm": None,
        }
        self.browser_superseded_clients: set[str] = set()
        self.browser_superseded_order: deque[str] = deque()
        self.rgbd_http_slots = threading.BoundedSemaphore(MAX_RGBD_HTTP_REQUESTS)
        self.rgbd_latest_hub = LatestRgbdHub()
        self.claw_camera_device = resolve_claw_camera_device(claw_camera_device)
        self.claw_stream_slots = threading.BoundedSemaphore(MAX_CLAW_STREAMS)
        self.claw_camera_hub = ClawCameraHub()
        self.arm_controller_lock = threading.Lock()
        self.arm_controller_id: str | None = None
        self.arm_controller_peer = ""
        self.arm_controller_handler = None
        self.arm_last_command_at = 0.0
        self.arm_command_times: deque[float] = deque()
        self.arm_browser_clock_offset_ms: int | None = None
        self.arm_browser_clock_rebase_samples: deque[int] = deque(
            maxlen=ARM_CLOCK_REBASE_SAMPLE_COUNT
        )
        self.arm_stale_command_drop_count = 0
        self.arm_restart_requested_at = 0.0
        self.arm_restart_baseline_count = 0
        self.arm_status_lock = threading.Lock()
        self.arm_status_received_at = 0.0
        self.arm_status: dict = {
            "type": "arm_status",
            "follower_connected": False,
            "state": "offline",
            "armed": False,
            "torque_enabled": False,
            "fault": "follower service unavailable",
        }
        self.arm_status_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.arm_status_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.arm_status_socket.bind(("127.0.0.1", self.arm_status_port))
        self.arm_status_socket.settimeout(0.5)
        self.arm_status_running = True
        self.arm_status_thread = threading.Thread(target=self._listen_for_arm_status, daemon=True)
        self.arm_status_thread.start()
        self.robot_calibration_lock = threading.Lock()
        self.robot_calibration_received_at = 0.0
        self.robot_calibration_status: dict = {
            "type": "robot_calibration_status",
            "state": "offline",
            "progress": 0.0,
            "frames": 0,
            "confidence": 0.0,
            "message": "Waiting for Godot arm calibrator.",
        }
        self.robot_calibration_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.robot_calibration_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.robot_calibration_socket.bind(("127.0.0.1", self.robot_calibration_status_port))
        self.robot_calibration_socket.settimeout(0.5)
        self.robot_calibration_running = True
        self.robot_calibration_thread = threading.Thread(target=self._listen_for_robot_calibration_status, daemon=True)
        self.robot_calibration_thread.start()
        self.async_loop = asyncio.new_event_loop()
        self.async_thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.async_thread.start()

    def _run_async_loop(self) -> None:
        asyncio.set_event_loop(self.async_loop)
        self.async_loop.run_forever()

    def forward_tracking(self, payload: bytes) -> None:
        self.udp.sendto(payload, (self.udp_host, self.udp_port))
        self.packet_count += 1
        now = time.monotonic()
        if now - self.last_report_sec >= 1.0:
            hz = self.packet_count - self.last_report_count
            self.last_report_count = self.packet_count
            self.last_report_sec = now
            print(f"lan remote tracking wss->udp {hz}/s total={self.packet_count}", flush=True)

    def forward_arm(self, message: dict) -> None:
        if message.get("type") in ("arm_command", "arm_cartesian_velocity", "arm_joint_velocity", "arm_tool_velocity"):
            now = time.monotonic()
            self.arm_last_command_at = now
            self.arm_command_times.append(now)
            while self.arm_command_times and now - self.arm_command_times[0] > 1.0:
                self.arm_command_times.popleft()
        if message.get("type") == "arm_restart":
            with self.arm_status_lock:
                self.arm_restart_baseline_count = int(self.arm_status.get("restart_count", 0))
            self.arm_restart_requested_at = time.monotonic()
        payload = json.dumps(message, separators=(",", ":")).encode("utf-8")
        self.arm_udp.sendto(payload, ("127.0.0.1", self.arm_command_port))

    def claim_arm_controller(
        self,
        peer: str,
        client_id: str = "",
        handler=None,
    ) -> str | None:
        if client_id and not self.claim_browser_client(client_id, "arm", handler):
            return None
        previous_handler = None
        with self.arm_controller_lock:
            if not client_id and self.arm_controller_id is not None:
                return None
            previous_handler = self.arm_controller_handler
            controller_id = uuid.uuid4().hex
            self.arm_controller_id = controller_id
            self.arm_controller_peer = peer
            self.arm_controller_handler = handler
            self.arm_last_command_at = 0.0
            self.arm_command_times.clear()
            self.arm_browser_clock_offset_ms = None
            self.arm_browser_clock_rebase_samples.clear()
        if previous_handler is not None and previous_handler is not handler:
            self.forward_arm({"type": "arm_hold"})
            previous_handler.preempt_arm_websocket()
        return controller_id

    def normalize_browser_arm_timing(
        self,
        message: dict,
        bridge_ms: int | None = None,
    ) -> tuple[bool, float]:
        """Timestamp a motion command, dropping jitter without forcing Hold.

        Browser and bridge clocks can have an arbitrary offset, so transport
        age is measured relative to the best offset seen in this controller
        session. An isolated delayed packet is discarded. If four consecutive
        offsets settle tightly at a new value, the network path itself changed;
        rebase to it rather than rejecting every subsequent fresh packet.
        """

        bridge_ms = int(time.time() * 1000) if bridge_ms is None else int(bridge_ms)
        browser_ms = int(message["sent_unix_ms"])
        observed_offset = bridge_ms - browser_ms
        baseline = self.arm_browser_clock_offset_ms
        if (
            baseline is None
            or observed_offset < baseline
            or abs(observed_offset - baseline) > 5000
        ):
            baseline = observed_offset
            self.arm_browser_clock_offset_ms = baseline
            self.arm_browser_clock_rebase_samples.clear()
        transport_age_ms = float(max(0, observed_offset - baseline))
        if transport_age_ms > MAX_ARM_TRANSPORT_AGE_MS:
            samples = self.arm_browser_clock_rebase_samples
            samples.append(observed_offset)
            stable_shift = (
                len(samples) >= ARM_CLOCK_REBASE_SAMPLE_COUNT
                and max(samples) - min(samples) <= ARM_CLOCK_REBASE_RANGE_MS
            )
            if stable_shift:
                baseline = int(round(sum(samples) / len(samples)))
                self.arm_browser_clock_offset_ms = baseline
                samples.clear()
                transport_age_ms = float(max(0, observed_offset - baseline))
            else:
                self.arm_stale_command_drop_count += 1
                return False, transport_age_ms
        else:
            self.arm_browser_clock_rebase_samples.clear()
        message["browser_sent_unix_ms"] = browser_ms
        message["bridge_recv_unix_ms"] = bridge_ms
        message["transport_age_ms"] = transport_age_ms
        # The follower watchdog uses the main PC's clock, not the browser's.
        message["sent_unix_ms"] = bridge_ms
        return True, transport_age_ms

    def resume_keyboard_after_transient_watchdog(
        self,
        controller_id: str,
        message: dict,
    ) -> bool:
        """Re-arm only a same-session keyboard watchdog pause.

        Explicit Hold, contact/following error, IK/clearance guards, and
        hardware faults deliberately do not qualify.
        """

        if (
            message.get("type")
            not in (
                "arm_cartesian_velocity",
                "arm_joint_velocity",
                "arm_tool_velocity",
            )
            or message.get("deadman") is not True
        ):
            return False
        status = self.latest_arm_status(controller_id)
        recoverable_messages = (
            "command watchdog expired",
            "received keyboard command is older than watchdog limit",
        )
        if (
            status.get("state") != "hold"
            or status.get("control_session") != controller_id
            or not any(
                str(status.get("message", "")).startswith(reason)
                for reason in recoverable_messages
            )
        ):
            return False
        self.forward_arm(
            {
                "type": "arm_enable",
                "source": "keyboard",
                "control_session": controller_id,
            }
        )
        return True

    def release_arm_controller(
        self,
        controller_id: str,
        client_id: str = "",
        handler=None,
    ) -> None:
        with self.arm_controller_lock:
            if self.arm_controller_id != controller_id:
                return
            self.forward_arm({"type": "arm_hold"})
            self.arm_controller_id = None
            self.arm_controller_peer = ""
            self.arm_controller_handler = None
        if client_id and handler is not None:
            self.release_browser_client(client_id, "arm", handler)

    def _remember_superseded_browser_client(self, client_id: str) -> None:
        if not client_id or client_id in self.browser_superseded_clients:
            return
        self.browser_superseded_clients.add(client_id)
        self.browser_superseded_order.append(client_id)
        while len(self.browser_superseded_order) > MAX_REMEMBERED_RGBD_CLIENTS:
            expired = self.browser_superseded_order.popleft()
            self.browser_superseded_clients.discard(expired)

    def claim_browser_client(self, client_id: str, channel: str, handler=None) -> bool:
        """Claim one channel without disturbing another channel in the same page."""

        if channel not in self.browser_client_handlers:
            raise ValueError(f"unknown browser channel: {channel}")
        previous_handlers: list[tuple[str, object]] = []
        owner_changed = False
        with self.browser_client_lock:
            if (
                client_id in self.browser_superseded_clients
                and client_id != self.browser_client_id
            ):
                return False
            if self.browser_client_id != client_id:
                owner_changed = True
                if self.browser_client_id:
                    self._remember_superseded_browser_client(self.browser_client_id)
                previous_handlers = [
                    (name, active)
                    for name, active in self.browser_client_handlers.items()
                    if active is not None and active is not handler
                ]
                self.browser_client_id = client_id
                self.browser_client_handlers = {"rgbd": None, "arm": None}
            else:
                active = self.browser_client_handlers[channel]
                if handler is not None and active is not None and active is not handler:
                    previous_handlers.append((channel, active))
            if handler is not None:
                self.browser_client_handlers[channel] = handler
        if owner_changed and any(name == "arm" for name, _active in previous_handlers):
            self.forward_arm({"type": "arm_hold"})
        for name, active in previous_handlers:
            if name == "arm":
                active.preempt_arm_websocket()
            else:
                active.preempt_rgbd_websocket()
        return True

    def claim_rgbd_client(self, client_id: str, handler=None) -> bool:
        return self.claim_browser_client(client_id, "rgbd", handler)

    def release_browser_client(self, client_id: str, channel: str, handler) -> None:
        # Retain the owner ID across transport failure so an older superseded
        # page cannot race the legitimate owner's reconnect.
        with self.browser_client_lock:
            if (
                self.browser_client_id == client_id
                and self.browser_client_handlers.get(channel) is handler
            ):
                self.browser_client_handlers[channel] = None

    def release_rgbd_client(self, client_id: str, handler) -> None:
        self.release_browser_client(client_id, "rgbd", handler)

    def latest_arm_status(self, controller_id: str | None = None) -> dict:
        with self.arm_status_lock:
            status = dict(self.arm_status)
            status_received_at = self.arm_status_received_at
        status_age_ms = None if not status_received_at else (time.monotonic() - status_received_at) * 1000.0
        status["status_age_ms"] = status_age_ms
        status["follower_responsive"] = bool(status_age_ms is not None and status_age_ms <= 500.0)
        if status_age_ms is not None and status_age_ms > 500.0:
            status["follower_connected"] = False
            status["armed"] = False
            status["state"] = "unresponsive"
            status["fault"] = f"follower status stalled for {status_age_ms:.0f} ms"
        if self.arm_restart_requested_at:
            restart_age = time.monotonic() - self.arm_restart_requested_at
            completed = int(status.get("restart_count", 0)) > self.arm_restart_baseline_count and status.get("state") != "restarting"
            if completed:
                self.arm_restart_requested_at = 0.0
            elif restart_age <= 8.0:
                status["state"] = "restarting"
                status["armed"] = False
                status["fault"] = ""
                status["message"] = "Restarting follower motor connection..."
            else:
                self.arm_restart_requested_at = 0.0
                status["state"] = "fault"
                status["armed"] = False
                status["fault"] = "follower restart timed out; check USB power/cable and retry"
        with self.arm_controller_lock:
            status["controller_active"] = self.arm_controller_id is not None
            status["controller_is_self"] = bool(controller_id and controller_id == self.arm_controller_id)
            status["controller_peer"] = self.arm_controller_peer if self.arm_controller_id else ""
            status["leader_data_fresh"] = bool(
                self.arm_last_command_at and time.monotonic() - self.arm_last_command_at <= 0.25
            )
            status["command_rate_hz"] = len(self.arm_command_times)
        with self.robot_calibration_lock:
            calibration = dict(self.robot_calibration_status)
            calibration_received_at = self.robot_calibration_received_at
        if calibration_received_at and time.monotonic() - calibration_received_at > 2.0:
            calibration["state"] = "offline"
            calibration["message"] = "Godot arm calibrator is not responding."
        status["robot_calibration"] = calibration
        return status

    def _listen_for_arm_status(self) -> None:
        while self.arm_status_running:
            try:
                payload, address = self.arm_status_socket.recvfrom(64 * 1024)
            except socket.timeout:
                continue
            except OSError:
                break
            if address[0] != "127.0.0.1":
                continue
            try:
                status = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(status, dict) or status.get("type") != "arm_status":
                continue
            with self.arm_status_lock:
                self.arm_status = status
                self.arm_status_received_at = time.monotonic()

    def _listen_for_robot_calibration_status(self) -> None:
        while self.robot_calibration_running:
            try:
                payload, address = self.robot_calibration_socket.recvfrom(64 * 1024)
            except socket.timeout:
                continue
            except OSError:
                break
            if address[0] != "127.0.0.1":
                continue
            try:
                status = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(status, dict) or status.get("type") != "robot_calibration_status":
                continue
            with self.robot_calibration_lock:
                self.robot_calibration_status = status
                self.robot_calibration_received_at = time.monotonic()

    def verify_access_token(self, token: str) -> bool:
        if not token or not self.access_team_domain or not self.access_audience:
            return False
        try:
            if self.access_jwk_client is None:
                certs_url = f"https://{self.access_team_domain}/cdn-cgi/access/certs"
                self.access_jwk_client = jwt.PyJWKClient(certs_url, cache_keys=True)
            signing_key = self.access_jwk_client.get_signing_key_from_jwt(token)
            jwt.decode(
                token,
                signing_key.key,
                algorithms=["RS256"],
                audience=self.access_audience,
                options={"require": ["exp", "iat", "aud"]},
            )
            return True
        except Exception as exc:
            print(f"Cloudflare Access token rejected: {exc}", flush=True)
            return False

    def server_close(self):
        self.arm_status_running = False
        self.robot_calibration_running = False
        self.claw_camera_hub.stop()
        self.rgbd_latest_hub.stop()
        try:
            self.arm_status_socket.close()
        except OSError:
            pass
        try:
            self.robot_calibration_socket.close()
        except OSError:
            pass
        self.udp.close()
        self.arm_udp.close()
        try:
            future = asyncio.run_coroutine_threadsafe(self._close_peer_connections(), self.async_loop)
            future.result(timeout=2.0)
        except Exception:
            pass
        self.async_loop.call_soon_threadsafe(self.async_loop.stop)
        self.async_thread.join(timeout=2.0)
        super().server_close()

    async def _close_peer_connections(self) -> None:
        with self.peer_lock:
            peers = list(self.peer_connections)
            self.peer_connections.clear()
        for peer in peers:
            await peer.close()


class LanRemoteHandler(BaseHTTPRequestHandler):
    server_version = "GodotLanRemote/1.0"

    def handle(self):
        try:
            super().handle()
        except (ConnectionError, OSError):
            # Browsers routinely reset speculative or superseded TLS requests.
            pass
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        print("%s - %s" % (self.address_string(), fmt % args), flush=True)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        super().end_headers()

    def do_GET(self):
        parsed = urlsplit(self.path)
        if self.headers.get("Upgrade", "").lower() == "websocket":
            if not self.websocket_origin_allowed():
                self.send_error(403, "WebSocket origin rejected")
                return
            if parsed.path == "/arm-control":
                if not self.arm_authorized():
                    self.send_websocket_auth_required("arm control requires an authorized LAN session or Cloudflare Access identity")
                    return
            elif self.auth_required() and not self.is_authorized():
                self.send_websocket_auth_required()
                return
            self.handle_websocket()
            return
        if parsed.path == "/login":
            self.handle_login_page(parsed.query)
            return
        if self.auth_required() and not self.is_authorized():
            self.redirect_to_login()
            return
        if parsed.path == "/video.mp4":
            self.proxy_encoded_video(parsed.query, "mp4")
            return
        if parsed.path == "/video-av1.webm":
            self.stream_av1_video(parsed.query)
            return
        if parsed.path == "/video.webm":
            self.proxy_encoded_video(parsed.query, "webm")
            return
        if parsed.path == "/stream":
            self.proxy_stream(parsed.query)
            return
        if parsed.path == "/claw-stream":
            self.stream_claw_camera(parsed.query)
            return
        if parsed.path == "/rgbd-latest":
            self.serve_latest_rgbd(parsed.query)
            return
        if parsed.path in ("/", "/hybrid", "/hybrid/", "/hybrid/controller.html"):
            self.serve_file("controller.html")
            return
        if parsed.path in ("/frame-stream", "/frame-stream/", "/frame-stream/controller.html"):
            self.serve_file("controller.html")
            return
        if parsed.path in ("/frame-stream/advanced", "/frame-stream/webcam_tracker_test.html"):
            self.serve_file("webcam_tracker_test.html")
            return
        if parsed.path.startswith("/frame-stream/"):
            self.serve_file(parsed.path.removeprefix("/frame-stream/") or "frame_stream_controller.html")
            return
        if parsed.path.startswith("/hybrid/"):
            self.serve_file(parsed.path.removeprefix("/hybrid/") or "controller.html")
            return
        rel = parsed.path.lstrip("/")
        self.serve_file(rel)

    def serve_latest_rgbd(self, query: str) -> None:
        """Return one newly claimed packet; obsolete HTTP responses are disposable."""

        if not self.server.rgbd_http_slots.acquire(blocking=False):
            self.send_error(503, "RGB-D latest-frame request capacity reached")
            return
        try:
            params = parse_qs(query)
            width = clamp_int(params.get("w", params.get("width", ["384"]))[0], 160, 960)
            fps = clamp_int(params.get("fps", ["30"])[0], 1, 60)
            context_fps = clamp_int(params.get("context_fps", [str(fps)])[0], 1, 60)
            detail_fps = clamp_int(params.get("detail_fps", [str(fps)])[0], 1, 60)
            quality = clamp_int(params.get("q", params.get("quality", ["78"]))[0], 1, 100)
            after_sequence = clamp_int(params.get("after", ["0"])[0], 0, 2_147_483_647)
            client_id = params.get("client", [""])[0]
            if (
                not client_id
                or len(client_id) > 64
                or any(not (character.isalnum() or character in "-_") for character in client_id)
            ):
                self.send_error(400, "Invalid RGB-D latest-frame client")
                return
            if not self.server.claim_rgbd_client(client_id):
                self.send_error(409, "RGB-D page superseded by a newer browser")
                return
            source_url = (
                f"http://{self.server.stream_host}:{self.server.stream_port}"
                f"/rgbd?w={width}&fps={fps}&context_fps={context_fps}"
                f"&detail_fps={detail_fps}&q={quality}"
            )
            source = self.server.rgbd_latest_hub.source(source_url)
            sequence, packet, error = source.claim_after(client_id, after_sequence)
            if not packet:
                self.send_response(204)
                self.send_header("Content-Length", "0")
                self.send_header("X-RGBD-Sequence", str(sequence))
                if error:
                    self.send_header("X-RGBD-Source-Error", quote(error[:160], safe=""))
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/octet-stream")
            self.send_header("Content-Length", str(len(packet)))
            self.send_header("X-RGBD-Sequence", str(sequence))
            self.end_headers()
            self.wfile.write(packet)
        finally:
            self.server.rgbd_http_slots.release()

    def do_HEAD(self):
        parsed = urlsplit(self.path)
        if self.auth_required() and not self.is_authorized():
            self.send_response(401)
            self.end_headers()
            return
        if parsed.path == "/video.mp4":
            self.send_response(200)
            self.send_header("Content-Type", "video/mp4")
            self.end_headers()
            return
        if parsed.path == "/video-av1.webm":
            self.send_response(200)
            self.send_header("Content-Type", "video/webm; codecs=av01")
            self.end_headers()
            return
        if parsed.path == "/video.webm":
            self.send_response(200)
            self.send_header("Content-Type", "video/webm; codecs=vp8")
            self.end_headers()
            return
        if parsed.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=godotframe")
            self.end_headers()
            return
        if parsed.path == "/claw-stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=clawframe")
            self.end_headers()
            return
        if parsed.path in ("/", "/hybrid", "/hybrid/", "/hybrid/controller.html"):
            rel = "controller.html"
        elif parsed.path in ("/frame-stream", "/frame-stream/", "/frame-stream/controller.html"):
            rel = "controller.html"
        elif parsed.path in ("/frame-stream/advanced", "/frame-stream/webcam_tracker_test.html"):
            rel = "webcam_tracker_test.html"
        elif parsed.path.startswith("/frame-stream/"):
            rel = parsed.path.removeprefix("/frame-stream/") or "frame_stream_controller.html"
        elif parsed.path.startswith("/hybrid/"):
            rel = parsed.path.removeprefix("/hybrid/") or "controller.html"
        else:
            rel = parsed.path.lstrip("/")
        self.serve_file(rel, include_body=False)

    def do_POST(self):
        parsed = urlsplit(self.path)
        if parsed.path == "/login":
            self.handle_login_submit()
            return
        if self.auth_required() and not self.is_authorized():
            self.send_response(401)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        if parsed.path in ("/webrtc-offer", "/frame-stream/webrtc-offer", "/api/v1/webrtc-offer"):
            self.handle_webrtc_offer()
            return
        self.send_error(404, "Not found")

    def auth_required(self) -> bool:
        return bool(getattr(self.server, "password", ""))

    def is_authorized(self) -> bool:
        if self.cloudflare_access_authorized():
            return True
        if not self.auth_required():
            return True
        return self.session_cookie_authorized()

    def session_cookie_authorized(self) -> bool:
        cookie = self.headers.get("Cookie", "")
        for part in cookie.split(";"):
            if "=" not in part:
                continue
            key, value = part.strip().split("=", 1)
            if key == "godot_remote_session":
                return hmac.compare_digest(value, self.server.session_cookie)
        return False

    def is_cloudflare_request(self) -> bool:
        return bool(
            self.headers.get("Cf-Access-Jwt-Assertion")
            or self.headers.get("Cf-Ray")
            or self.headers.get("Cf-Connecting-Ip")
        )

    def cloudflare_access_authorized(self) -> bool:
        token = self.headers.get("Cf-Access-Jwt-Assertion", "")
        return self.server.verify_access_token(token)

    def arm_authorized(self) -> bool:
        if self.is_cloudflare_request():
            request_host = self.headers.get("Host", "").split(":", 1)[0].lower()
            if (
                self.server.public_arm
                and self.server.public_hostname
                and request_host == self.server.public_hostname
            ):
                return self.cloudflare_access_authorized()
            if getattr(self.server, "allow_quick_tunnel_arm", False) and request_host.endswith(".trycloudflare.com"):
                return self.session_cookie_authorized()
            return False
        return self.is_authorized()

    def websocket_origin_allowed(self) -> bool:
        origin = self.headers.get("Origin", "")
        if not origin:
            return False
        parsed = urlsplit(origin)
        if parsed.scheme not in ("http", "https") or not parsed.hostname:
            return False
        request_host = self.headers.get("Host", "").split(":", 1)[0].lower()
        return parsed.hostname.lower() == request_host

    def redirect_to_login(self):
        next_path = self.path if self.path.startswith("/") else "/"
        self.send_response(303)
        self.send_header("Location", "/login?next=" + quote(next_path, safe=""))
        self.send_header("Content-Length", "0")
        self.end_headers()

    def send_websocket_auth_required(self, message: str = "controller password required"):
        body = (message + "\n").encode("utf-8")
        response = (
            "HTTP/1.1 401 Unauthorized\r\n"
            "Content-Type: text/plain; charset=utf-8\r\n"
            f"Content-Length: {len(body)}\r\n"
            "Connection: close\r\n"
            "\r\n"
        ).encode("ascii") + body
        self.connection.sendall(response)

    def safe_login_next(self, value: str) -> str:
        parsed = urlsplit(value)
        if parsed.scheme or parsed.netloc or not parsed.path.startswith("/") or parsed.path.startswith("//"):
            return "/controller.html"
        return parsed.path + (("?" + parsed.query) if parsed.query else "")

    def handle_login_page(self, query: str, wrong_password: bool = False):
        params = parse_qs(query)
        next_path = self.safe_login_next(params.get("next", ["/controller.html"])[0])
        message = "<p class=\"error\">Wrong password.</p>" if wrong_password else ""
        body = self.login_html(next_path, message)
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def handle_login_submit(self):
        length = clamp_int(self.headers.get("Content-Length", "0"), 0, 8192)
        content_type = self.headers.get("Content-Type", "")
        if length <= 0 or not content_type.startswith("application/x-www-form-urlencoded"):
            self.send_error(400, "Invalid login request")
            return
        params = parse_qs(self.rfile.read(length).decode("utf-8", errors="replace"))
        next_path = self.safe_login_next(params.get("next", ["/controller.html"])[0])
        submitted = params.get("password", [""])[0]
        if submitted and hmac.compare_digest(submitted, self.server.password):
            self.send_response(303)
            self.send_header("Set-Cookie", f"godot_remote_session={self.server.session_cookie}; Path=/; Secure; HttpOnly; SameSite=Lax")
            self.send_header("Location", next_path)
            self.send_header("Content-Length", "0")
            self.end_headers()
            return
        body = self.login_html(next_path, "<p class=\"error\">Wrong password.</p>")
        self.send_response(401)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def login_html(self, next_path: str, message: str) -> bytes:
        return f"""<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\">
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
  <title>Godot Controller Login</title>
  <style>
    body {{ margin:0; min-height:100vh; display:grid; place-items:center; background:#111; color:white; font:18px/1.4 system-ui,sans-serif; }}
    main {{ width:min(420px, calc(100vw - 32px)); }}
    input, button {{ box-sizing:border-box; width:100%; font:inherit; padding:12px 14px; margin:8px 0; }}
    button {{ cursor:pointer; }}
    .error {{ color:#ff8b8b; }}
  </style>
</head>
<body>
  <main>
    <h1>Controller Login</h1>
    {message}
    <form method=\"post\" action=\"/login\">
      <input type=\"hidden\" name=\"next\" value=\"{html.escape(next_path, quote=True)}\">
      <input name=\"password\" type=\"password\" autocomplete=\"current-password\" autofocus placeholder=\"Password\">
      <button type=\"submit\">Enter</button>
    </form>
  </main>
</body>
</html>
""".encode("utf-8")

    def serve_file(self, rel: str, include_body: bool = True):
        root = self.server.root.resolve()
        # The operator UI is its own document root; robot models live in the
        # repository's shared assets tree and are exposed read-only here.
        if rel.startswith("assets/"):
            allowed_root = (root.parent / "assets").resolve()
            target = (root.parent / rel).resolve()
        else:
            allowed_root = root
            target = (root / rel).resolve()
        if not target.is_relative_to(allowed_root) or not target.is_file():
            self.send_error(404, "File not found")
            return
        content_type = "application/octet-stream"
        if target.suffix == ".html":
            content_type = "text/html; charset=utf-8"
        elif target.suffix == ".js":
            content_type = "text/javascript; charset=utf-8"
        elif target.suffix == ".wasm":
            content_type = "application/wasm"
        elif target.suffix == ".png":
            content_type = "image/png"
        elif target.suffix == ".ico":
            content_type = "image/x-icon"
        elif target.suffix == ".glb":
            content_type = "model/gltf-binary"
        body = target.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if include_body:
            self.wfile.write(body)

    def proxy_stream(self, query: str):
        path = "/stream" + (f"?{query}" if query else "")
        try:
            upstream = socket.create_connection((self.server.stream_host, self.server.stream_port), timeout=3.0)
            upstream.settimeout(10.0)
        except OSError as exc:
            self.send_error(503, f"Godot viewport stream unavailable: {exc}")
            return
        request = (
            f"GET {path} HTTP/1.1\r\n"
            f"Host: {self.server.stream_host}:{self.server.stream_port}\r\n"
            "Connection: close\r\n"
            "\r\n"
        ).encode("ascii")
        try:
            upstream.sendall(request)
            while True:
                chunk = upstream.recv(65536)
                if not chunk:
                    break
                self.connection.sendall(chunk)
        except OSError:
            pass
        finally:
            upstream.close()

    def stream_claw_camera(self, query: str) -> None:
        device = self.server.claw_camera_device
        if not device or not Path(device).exists():
            # The arm camera is often plugged in after the remote stack starts.
            # Re-scan here so the default-on browser preview recovers without a
            # server restart as soon as USB2.0_CAM1 appears.
            device = resolve_claw_camera_device("")
            self.server.claw_camera_device = device
            if not device or not Path(device).exists():
                self.send_error(503, "Claw camera is not connected")
                return
        if not self.server.claw_stream_slots.acquire(blocking=False):
            self.send_error(503, "Claw camera client capacity reached")
            return
        params = parse_qs(query)
        width = clamp_int(params.get("w", params.get("width", ["1280"]))[0], 320, 1920)
        height = clamp_int(params.get("h", params.get("height", ["720"]))[0], 240, 1080)
        fps = clamp_int(params.get("fps", ["30"])[0], 1, 30)
        hub = self.server.claw_camera_hub
        if not hub.ensure_running(device, width, height, fps):
            self.server.claw_stream_slots.release()
            self.send_error(503, f"Could not start claw camera: {hub.last_error or 'capture unavailable'}")
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=clawframe")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("X-Accel-Buffering", "no")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        sequence = 0
        try:
            while True:
                next_sequence, frame, capture_unix_ms = hub.wait_for_frame(sequence, timeout=2.0)
                if next_sequence <= sequence or not frame:
                    if hub.process is None:
                        if not hub.ensure_running(device, width, height, fps):
                            break
                    continue
                sequence = next_sequence
                header = (
                    "--clawframe\r\n"
                    "Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(frame)}\r\n"
                    f"X-Capture-Unix-Ms: {capture_unix_ms}\r\n"
                    f"X-Frame-Sequence: {sequence}\r\n\r\n"
                ).encode("ascii")
                self.connection.sendall(header + frame + b"\r\n")
        except (BrokenPipeError, ConnectionError, OSError):
            pass
        finally:
            self.server.claw_stream_slots.release()

    def proxy_encoded_video(self, query: str, container: str):
        if not self.server.ffmpeg_slots.acquire(blocking=False):
            self.send_error(503, "Encoded stream capacity reached")
            return
        params = parse_qs(query)
        width = clamp_int(params.get("w", params.get("width", ["640"]))[0], 160, 1920)
        fps = clamp_int(params.get("fps", ["30"])[0], 1, 60)
        jpeg_quality = clamp_int(params.get("q", params.get("quality", ["35"]))[0], 1, 100)
        bitrate = safe_bitrate(params.get("bitrate", ["2400k"])[0])
        encoder = str(params.get("encoder", ["auto"])[0]).strip().lower()
        source_url = f"http://{self.server.stream_host}:{self.server.stream_port}/stream?w={width}&q={jpeg_quality}&fps={fps}"
        raw_reader = None
        first_raw_frame = None
        raw_size = None
        if container == "mp4":
            raw_url = f"http://{self.server.stream_host}:{self.server.stream_port}/raw?w={width}&fps={fps}"
            try:
                raw_reader = MultipartRawFrameReader(raw_url)
                raw_headers, first_raw_frame = raw_reader.read_part()
                raw_width = int(raw_headers.get("x-frame-width", "0") or "0")
                raw_height = int(raw_headers.get("x-frame-height", "0") or "0")
                raw_format = raw_headers.get("x-frame-format", "rgb24")
                if raw_format != "rgb24" or raw_width <= 0 or raw_height <= 0:
                    raise ValueError(f"unsupported raw viewport frame {raw_width}x{raw_height} {raw_format}")
                expected = raw_width * raw_height * 3
                if len(first_raw_frame) != expected:
                    raise ValueError(f"bad raw viewport frame bytes={len(first_raw_frame)} expected={expected}")
                raw_size = (raw_width, raw_height)
                print(f"encoded video using raw Godot frames at {raw_width}x{raw_height}", flush=True)
            except Exception as exc:
                if raw_reader is not None:
                    raw_reader.close()
                raw_reader = None
                first_raw_frame = None
                print(f"raw encoded-video source unavailable, using MJPEG: {exc}", flush=True)
        command = self.build_encoded_video_command(source_url, fps, bitrate, container, encoder, raw_size)
        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE if raw_reader is not None else subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )
        except OSError as exc:
            if raw_reader is not None:
                raw_reader.close()
            self.server.ffmpeg_slots.release()
            self.send_error(503, f"Could not start ffmpeg: {exc}")
            return
        self.send_response(200)
        if container == "webm":
            self.send_header("Content-Type", "video/webm; codecs=vp8")
        else:
            self.send_header("Content-Type", "video/mp4; codecs=avc1.640029")
        self.send_header("Connection", "close")
        self.end_headers()
        stderr_chunks: list[bytes] = []

        def drain_stderr():
            if process.stderr is None:
                return
            while True:
                chunk = process.stderr.readline()
                if not chunk:
                    break
                stderr_chunks.append(chunk[-512:])
                if len(stderr_chunks) > 8:
                    del stderr_chunks[:-8]

        stderr_thread = threading.Thread(target=drain_stderr, daemon=True)
        stderr_thread.start()
        raw_thread = None
        if raw_reader is not None:
            def feed_raw_frames():
                frame = first_raw_frame
                try:
                    assert process.stdin is not None
                    while frame is not None and process.poll() is None:
                        process.stdin.write(frame)
                        headers, frame = raw_reader.read_part()
                        frame_width = int(headers.get("x-frame-width", "0") or "0")
                        frame_height = int(headers.get("x-frame-height", "0") or "0")
                        if (frame_width, frame_height) != raw_size or len(frame) != frame_width * frame_height * 3:
                            raise ValueError("raw viewport dimensions changed during encoded stream")
                except (BrokenPipeError, OSError, ValueError) as exc:
                    if process.poll() is None and not (isinstance(exc, OSError) and exc.errno == 9):
                        print(f"raw viewport feeder ended: {exc}", flush=True)
                finally:
                    raw_reader.close()
                    try:
                        if process.stdin is not None:
                            process.stdin.close()
                    except OSError:
                        pass

            raw_thread = threading.Thread(target=feed_raw_frames, daemon=True)
            raw_thread.start()
        try:
            assert process.stdout is not None
            while True:
                chunk = process.stdout.read(32768)
                if not chunk:
                    break
                self.connection.sendall(chunk)
        except OSError:
            pass
        finally:
            process.terminate()
            if raw_reader is not None:
                raw_reader.close()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
            if raw_thread is not None:
                raw_thread.join(timeout=1.0)
            if stderr_chunks:
                tail = b"".join(stderr_chunks).decode("utf-8", errors="replace").strip()
                if tail:
                    print(f"ffmpeg video stream ended: {tail}", flush=True)
            self.server.ffmpeg_slots.release()

    def stream_av1_video(self, query: str) -> None:
        if not self.server.ffmpeg_slots.acquire(blocking=False):
            self.send_error(503, "Encoded stream capacity reached")
            return
        params = parse_qs(query)
        width = clamp_int(params.get("w", params.get("width", ["640"]))[0], 160, 1920)
        fps = clamp_int(params.get("fps", ["30"])[0], 1, 60)
        bitrate = safe_bitrate(params.get("bitrate", ["3200k"])[0])
        bitrate_bps = int(bitrate[:-1]) * 1000
        raw_url = f"http://{self.server.stream_host}:{self.server.stream_port}/raw?w={width}&fps={fps}"
        raw_reader = None
        output = None
        writer = None
        try:
            raw_reader = MultipartRawFrameReader(raw_url)
            headers, frame_data = raw_reader.read_part()
            frame_width, frame_height = validate_raw_viewport_frame(headers, frame_data)

            self.send_response(200)
            self.send_header("Content-Type", "video/webm; codecs=av01")
            self.send_header("Transfer-Encoding", "chunked")
            self.send_header("Connection", "close")
            self.end_headers()

            writer = HttpChunkedWriteAdapter(self.connection)
            output = av.open(
                writer,
                mode="w",
                format="webm",
                options={
                    "live": "1",
                    "cluster_time_limit": "100",
                    "cluster_size_limit": "0",
                    "flush_packets": "1",
                },
            )
            stream = output.add_stream("av1_nvenc", rate=fps)
            stream.width = frame_width
            stream.height = frame_height
            stream.pix_fmt = "yuv420p"
            stream.bit_rate = bitrate_bps
            stream.codec_context.time_base = fractions.Fraction(1, fps)
            stream.codec_context.options = {
                "preset": "p4",
                "tune": "ull",
                "rc": "cbr",
                "delay": "0",
                "zerolatency": "1",
                "rc-lookahead": "0",
                "spatial-aq": "1",
                "aq-strength": "8",
                "g": str(max(1, fps)),
            }
            print(
                f"AV1 NVENC video using raw Godot frames at {frame_width}x{frame_height} "
                f"{fps}fps {bitrate}",
                flush=True,
            )

            next_pts = 0
            started_at = time.monotonic()
            previous_frame_data = frame_data
            while True:
                target_pts = max(next_pts, int((time.monotonic() - started_at) * fps))
                while next_pts < target_pts:
                    frame = raw_video_frame(previous_frame_data, frame_width, frame_height, next_pts, fps)
                    for packet in stream.encode(frame):
                        output.mux(packet)
                    next_pts += 1
                frame = raw_video_frame(frame_data, frame_width, frame_height, target_pts, fps)
                for packet in stream.encode(frame):
                    output.mux(packet)
                next_pts = target_pts + 1
                previous_frame_data = frame_data
                headers, frame_data = raw_reader.read_part()
                next_width, next_height = validate_raw_viewport_frame(headers, frame_data)
                if (next_width, next_height) != (frame_width, frame_height):
                    raise ValueError("raw viewport dimensions changed during AV1 stream")
        except (BrokenPipeError, ConnectionError, OSError):
            pass
        except Exception as exc:
            # The response may already be streaming, so a log is safer than an HTTP error body.
            print(f"AV1 NVENC stream ended: {exc}", flush=True)
        finally:
            if output is not None:
                try:
                    output.close()
                except Exception:
                    pass
            if writer is not None:
                writer.finish()
            if raw_reader is not None:
                raw_reader.close()
            self.server.ffmpeg_slots.release()

    def handle_webrtc_offer(self):
        with self.server.peer_lock:
            if len(self.server.peer_connections) >= MAX_WEBRTC_PEERS:
                self.send_error(503, "WebRTC peer capacity reached")
                return
        length = clamp_int(self.headers.get("Content-Length", "0"), 0, 2_000_000)
        if length <= 0:
            self.send_error(400, "Missing offer")
            return
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            offer = RTCSessionDescription(sdp=payload["sdp"], type=payload["type"])
        except Exception as exc:
            self.send_error(400, f"Bad offer: {exc}")
            return
        width = clamp_int(str(payload.get("width", 640)), 160, 1920)
        fps = clamp_int(str(payload.get("fps", 30)), 1, 60)
        jpeg_quality = clamp_int(str(payload.get("quality", 35)), 1, 100)
        bitrate_kbps = clamp_int(str(payload.get("bitrateKbps", 2500)), 250, 24000)
        source = str(payload.get("source", "raw")).strip().lower()
        context_fps = clamp_int(str(payload.get("contextFps", fps)), 1, 60)
        detail_fps = clamp_int(str(payload.get("detailFps", fps)), 1, 60)
        future = asyncio.run_coroutine_threadsafe(
            self.create_webrtc_answer(
                offer,
                width,
                fps,
                context_fps,
                detail_fps,
                jpeg_quality,
                bitrate_kbps,
                source,
            ),
            self.server.async_loop,
        )
        try:
            answer = future.result(timeout=15.0)
        except Exception as exc:
            self.send_error(503, f"Could not create WebRTC answer: {exc}")
            return
        body = json.dumps({"sdp": answer.sdp, "type": answer.type}).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    async def create_webrtc_answer(
        self,
        offer: RTCSessionDescription,
        width: int,
        fps: int,
        context_fps: int,
        detail_fps: int,
        jpeg_quality: int,
        bitrate_kbps: int,
        source: str,
    ) -> RTCSessionDescription:
        config = RTCConfiguration(iceServers=[RTCIceServer(urls=["stun:stun.l.google.com:19302"])])
        pc = RTCPeerConnection(configuration=config)
        with self.server.peer_lock:
            if len(self.server.peer_connections) >= MAX_WEBRTC_PEERS:
                await pc.close()
                raise RuntimeError("WebRTC peer capacity reached")
            self.server.peer_connections.add(pc)
        sender = None
        if source not in ("control_only", "rgbd_data"):
            raw_url = f"http://{self.server.stream_host}:{self.server.stream_port}/raw?w={width}&fps={fps}"
            mjpeg_url = f"http://{self.server.stream_host}:{self.server.stream_port}/stream?w={width}&q={jpeg_quality}&fps={fps}"
            if source in ("mjpeg", "quality", "legacy"):
                print(f"WebRTC using MJPEG viewport source at {width}px q={jpeg_quality} bitrate={bitrate_kbps}k", flush=True)
                sender = pc.addTrack(MjpegVideoStreamTrack(mjpeg_url))
            else:
                print(f"WebRTC using raw viewport source at {width}px bitrate={bitrate_kbps}k", flush=True)
                sender = pc.addTrack(RawViewportVideoStreamTrack(raw_url, mjpeg_url))
            prefer_video_codec(pc, "video/H264")
            configure_h264_sender(sender, bitrate_kbps)
        elif source == "control_only":
            print("WebRTC using data-only control transport", flush=True)
        else:
            print(
                f"WebRTC using latest-frame RGB-D data transport at {width}px "
                f"context={context_fps}fps detail={detail_fps}fps",
                flush=True,
            )

        peer_control_rate_window: deque[float] = deque()
        rgbd_tasks: set[asyncio.Task] = set()

        @pc.on("datachannel")
        def on_datachannel(channel):
            if channel.label == "rgbd-stream" and source == "rgbd_data":
                source_url = (
                    f"http://{self.server.stream_host}:{self.server.stream_port}"
                    f"/rgbd?w={width}&fps={fps}&context_fps={context_fps}"
                    f"&detail_fps={detail_fps}&q={jpeg_quality}"
                )
                started = False

                def start_rgbd_publisher():
                    nonlocal started
                    if started:
                        return
                    started = True
                    task = asyncio.create_task(
                        publish_latest_rgbd_datachannel(channel, pc, source_url)
                    )
                    rgbd_tasks.add(task)
                    task.add_done_callback(rgbd_tasks.discard)

                @channel.on("open")
                def on_open():
                    start_rgbd_publisher()

                if channel.readyState == "open":
                    start_rgbd_publisher()
                return
            if channel.label not in ("head-tracking", "view-settings"):
                channel.close()
                return
            print(f"WebRTC control channel open: {channel.label}", flush=True)

            @channel.on("message")
            def on_message(message):
                try:
                    if isinstance(message, bytes):
                        message = message.decode("utf-8")
                    if not isinstance(message, str) or len(message) > MAX_WEBSOCKET_FRAME:
                        raise ValueError("invalid WebRTC control message")
                    data = json.loads(message)
                    self.check_websocket_rate(peer_control_rate_window, 120)
                    packet = self.validate_remote_control_message(data)
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
                    return
                self.server.forward_tracking(json.dumps(packet, separators=(",", ":")).encode("utf-8"))

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            print(f"WebRTC connection state: {pc.connectionState}", flush=True)
            if pc.connectionState in ("failed", "closed", "disconnected"):
                for task in tuple(rgbd_tasks):
                    task.cancel()
                await pc.close()
                with self.server.peer_lock:
                    self.server.peer_connections.discard(pc)

        try:
            await pc.setRemoteDescription(offer)
            answer = await pc.createAnswer()
            if bitrate_kbps > 0:
                answer = RTCSessionDescription(
                    sdp=with_video_bitrate(answer.sdp, bitrate_kbps),
                    type=answer.type,
                )
            await pc.setLocalDescription(answer)
            return pc.localDescription
        except Exception:
            await pc.close()
            with self.server.peer_lock:
                self.server.peer_connections.discard(pc)
            raise

    def build_encoded_video_command(
        self,
        source_url: str,
        fps: int,
        bitrate: str,
        container: str,
        encoder: str = "auto",
        raw_size: tuple[int, int] | None = None,
    ) -> list[str]:
        base = [
            ffmpeg_binary(),
            "-hide_banner",
            "-loglevel", "warning",
        ]
        if raw_size is not None:
            base += [
                "-f", "rawvideo",
                "-pixel_format", "rgb24",
                "-video_size", f"{raw_size[0]}x{raw_size[1]}",
                "-framerate", str(fps),
                "-i", "pipe:0",
                "-an",
            ]
        else:
            base += [
                "-fflags", "nobuffer",
                "-flags", "low_delay",
                "-probesize", "32",
                "-analyzeduration", "0",
                "-f", "mjpeg",
                "-i", source_url,
                "-an",
            ]
        if container == "webm":
            return base + [
                "-c:v", "libvpx",
                "-deadline", "realtime",
                "-cpu-used", "8",
                "-quality", "realtime",
                "-lag-in-frames", "0",
                "-error-resilient", "1",
                "-auto-alt-ref", "0",
                "-pix_fmt", "yuv420p",
                "-r", str(fps),
                "-g", str(max(1, fps)),
                "-b:v", bitrate,
                "-maxrate", bitrate,
                "-bufsize", "600k",
                "-f", "webm",
                "-cluster_time_limit", "100",
                "-cluster_size_limit", "0",
                "pipe:1",
            ]
        use_nvenc = encoder in ("auto", "nvenc", "h264_nvenc") and ffmpeg_encoder_available("h264_nvenc")
        if use_nvenc:
            return base + [
                "-c:v", "h264_nvenc",
                "-preset", "p4",
                "-tune", "ull",
                "-rc", "cbr",
                "-rc-lookahead", "0",
                "-spatial-aq", "1",
                "-aq-strength", "8",
                "-zerolatency", "1",
                "-delay", "0",
                "-bf", "0",
                "-pix_fmt", "yuv420p",
                "-profile:v", "high",
                "-level:v", "4.1",
                "-r", str(fps),
                "-g", str(max(1, fps)),
                "-b:v", bitrate,
                "-maxrate", bitrate,
                "-bufsize", video_buffer_size(bitrate, fps),
                "-f", "mp4",
                "-movflags", "empty_moov+default_base_moof+frag_every_frame",
                "-flush_packets", "1",
                "pipe:1",
            ]
        return base + [
            "-c:v", "libx264",
            "-profile:v", "high",
            "-level", "4.1",
            "-preset", "ultrafast",
            "-tune", "zerolatency",
            "-pix_fmt", "yuv420p",
            "-r", str(fps),
            "-g", str(max(1, fps)),
            "-keyint_min", str(max(1, fps)),
            "-sc_threshold", "0",
            "-bf", "0",
            "-b:v", bitrate,
            "-maxrate", bitrate,
            "-bufsize", video_buffer_size(bitrate, fps),
            "-f", "mp4",
            "-movflags", "empty_moov+default_base_moof+frag_every_frame",
            "-flush_packets", "1",
            "pipe:1",
        ]

    def handle_websocket(self):
        path = urlsplit(self.path).path
        if path not in ("/head-tracking", "/arm-control", "/av1-stream", "/rgbd-stream"):
            self.send_error(404, "Unknown websocket path")
            return
        if path == "/arm-control":
            self.handle_arm_websocket()
            return
        if path == "/av1-stream":
            self.handle_av1_websocket()
            return
        if path == "/rgbd-stream":
            self.handle_rgbd_websocket()
            return
        self.handle_head_websocket()

    def accept_websocket(self) -> bool:
        key = self.headers.get("Sec-WebSocket-Key", "")
        if not key:
            self.send_error(400, "Missing Sec-WebSocket-Key")
            return False
        accept = base64.b64encode(hashlib.sha1((key + GUID).encode("ascii")).digest()).decode("ascii")
        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {accept}\r\n"
            "\r\n"
        )
        self.connection.sendall(response.encode("ascii"))
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.connection.settimeout(2.0)
        # BaseHTTPRequestHandler must not try to parse another HTTP request after
        # the WebSocket loop consumes its close frame.
        self.close_connection = True
        return True

    def handle_head_websocket(self):
        if not self.accept_websocket():
            return
        peer = f"{self.client_address[0]}:{self.client_address[1]}"
        print(f"lan remote head tracker connected: {peer}", flush=True)
        rate_window: deque[float] = deque()
        try:
            while True:
                frame = self.read_ws_frame()
                if frame is None:
                    break
                opcode, payload = frame
                if opcode == 0x8:
                    self.send_ws_frame(b"", 0x8)
                    break
                if opcode == 0x9:
                    self.send_ws_frame(payload, 0xA)
                    continue
                if opcode != 0x1:
                    continue
                try:
                    data = json.loads(payload.decode("utf-8", errors="replace"))
                    self.check_websocket_rate(rate_window, 120)
                    packet = self.validate_remote_control_message(data)
                except (json.JSONDecodeError, ValueError):
                    continue
                self.server.forward_tracking(json.dumps(packet, separators=(",", ":")).encode("utf-8"))
        except (ConnectionError, OSError, ValueError):
            pass
        finally:
            print(f"lan remote head tracker disconnected: {peer}", flush=True)

    def handle_rgbd_websocket(self) -> None:
        params = parse_qs(urlsplit(self.path).query)
        client_id = params.get("client", [""])[0]
        if (
            not client_id
            or len(client_id) > 64
            or any(
                not (character.isalnum() or character in "-_")
                for character in client_id
            )
        ):
            self.send_error(400, "Invalid RGB-D stream client")
            return
        width = clamp_int(params.get("w", params.get("width", ["384"]))[0], 160, 960)
        fps = clamp_int(params.get("fps", ["15"])[0], 1, 60)
        context_fps = clamp_int(params.get("context_fps", [str(fps)])[0], 1, 60)
        detail_fps = clamp_int(params.get("detail_fps", [str(fps)])[0], 1, 60)
        quality = clamp_int(params.get("q", params.get("quality", ["78"]))[0], 1, 100)
        temporal_depth = params.get("temporal_depth", ["0"])[0] in (
            "1",
            "true",
            "yes",
            "on",
        )
        temporal_color = params.get(
            "temporal_color",
            ["1" if temporal_depth else "0"],
        )[0] in ("1", "true", "yes", "on")
        persistent_reference = (
            temporal_depth
            and params.get("persistent_reference", ["0"])[0]
            in ("1", "true", "yes", "on")
        )
        source_url = (
            f"http://{self.server.stream_host}:{self.server.stream_port}"
            f"/rgbd?w={width}&fps={fps}&context_fps={context_fps}&detail_fps={detail_fps}&q={quality}"
        )
        reader = None
        reader_thread = None
        encoder_thread = None
        reader_stop = threading.Event()
        newest_raw_packet = None
        newest_raw_sequence = 0
        newest_raw_lock = threading.Condition()
        newest_packet = None
        newest_sequence = 0
        pending_keyframe = None
        persistent_packet_pending = False
        newest_lock = threading.Condition()
        reader_error = None
        temporal_encoder = (
            TemporalRgbdDepthEncoder(
                temporal_color=temporal_color,
                color_quality=quality,
                persistent_reference=persistent_reference,
            )
            if temporal_depth
            else None
        )
        peer = f"{self.client_address[0]}:{self.client_address[1]}"
        self.rgbd_preempted = threading.Event()
        self.rgbd_send_lock = threading.Lock()

        def drain_latest_packet() -> None:
            nonlocal newest_raw_packet, newest_raw_sequence, reader_error
            try:
                while not reader_stop.is_set():
                    _headers, packet = reader.read_part()
                    if not packet:
                        continue
                    with newest_raw_lock:
                        newest_raw_packet = packet
                        newest_raw_sequence += 1
                        newest_raw_lock.notify()
            except (ConnectionError, OSError, ValueError) as exc:
                reader_error = exc
            finally:
                with newest_raw_lock:
                    newest_raw_lock.notify_all()
                with newest_lock:
                    newest_lock.notify_all()

        def encode_latest_packet() -> None:
            nonlocal newest_packet, newest_sequence, pending_keyframe
            nonlocal persistent_packet_pending, reader_error
            encoded_raw_sequence = 0
            try:
                while not reader_stop.is_set():
                    if persistent_reference:
                        with newest_lock:
                            newest_lock.wait_for(
                                lambda: (
                                    not persistent_packet_pending
                                    or reader_error is not None
                                    or reader_stop.is_set()
                                ),
                                timeout=1.0,
                            )
                            if reader_error is not None or reader_stop.is_set():
                                return
                            if persistent_packet_pending:
                                continue
                    with newest_raw_lock:
                        newest_raw_lock.wait_for(
                            lambda: (
                                newest_raw_sequence != encoded_raw_sequence
                                or reader_error is not None
                                or reader_stop.is_set()
                            ),
                            timeout=1.0,
                        )
                        if reader_error is not None:
                            return
                        if reader_stop.is_set():
                            return
                        if newest_raw_sequence == encoded_raw_sequence:
                            continue
                        packet = newest_raw_packet
                        encoded_raw_sequence = newest_raw_sequence
                    if not packet:
                        continue
                    is_keyframe = False
                    if temporal_encoder is not None:
                        packet, is_keyframe = temporal_encoder.encode(packet)
                    with newest_lock:
                        if is_keyframe:
                            # A delta must never overtake the keyframe it names.
                            # Keep the newest keyframe in a priority slot while
                            # continuing to collapse ordinary frames to latest.
                            pending_keyframe = packet
                        newest_packet = packet
                        newest_sequence += 1
                        if persistent_reference:
                            persistent_packet_pending = True
                        newest_lock.notify()
            except (ConnectionError, OSError, ValueError) as exc:
                reader_error = exc
            finally:
                with newest_lock:
                    newest_lock.notify_all()

        try:
            if not self.accept_websocket():
                return
            if not self.server.claim_rgbd_client(client_id, self):
                self.send_ws_frame(
                    struct.pack("!H", 4001) + b"superseded by newer browser",
                    0x8,
                )
                return
            self.connection.settimeout(5.0)
            self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 2 * 1024 * 1024)
            reader = MultipartRawFrameReader(source_url)
            reader_thread = threading.Thread(target=drain_latest_packet, daemon=True)
            encoder_thread = threading.Thread(target=encode_latest_packet, daemon=True)
            reader_thread.start()
            encoder_thread.start()
            print(
                f"hybrid RGB-D client connected: {peer} {width}px "
                f"context={context_fps}fps detail={detail_fps}fps "
                f"temporal_depth={'on' if temporal_depth else 'off'} "
                f"temporal_color={'on' if temporal_color else 'off'} "
                f"persistent_reference={'on' if persistent_reference else 'off'}",
                flush=True,
            )
            sent_sequence = 0
            while True:
                with newest_lock:
                    newest_lock.wait_for(
                        lambda: (
                            self.rgbd_preempted.is_set()
                            or
                            pending_keyframe is not None
                            or newest_sequence != sent_sequence
                            or reader_error is not None
                        ),
                        timeout=5.0,
                    )
                    if self.rgbd_preempted.is_set():
                        raise ConnectionError("superseded by newer browser")
                    if reader_error is not None:
                        raise reader_error
                    if pending_keyframe is not None:
                        packet = pending_keyframe
                        pending_keyframe = None
                        sent_sequence = newest_sequence
                    elif newest_sequence != sent_sequence:
                        packet = newest_packet
                        sent_sequence = newest_sequence
                    else:
                        continue
                if not packet:
                    continue
                with self.rgbd_send_lock:
                    if self.rgbd_preempted.is_set():
                        raise ConnectionError("superseded by newer browser")
                    self.send_ws_frame(packet, 0x2)
                if persistent_reference:
                    with newest_lock:
                        persistent_packet_pending = False
                        newest_lock.notify_all()
        except (ConnectionError, OSError, ValueError) as exc:
            print(f"hybrid RGB-D client ended: {peer} ({exc})", flush=True)
        finally:
            reader_stop.set()
            with newest_raw_lock:
                newest_raw_lock.notify_all()
            if reader is not None:
                reader.close()
            if reader_thread is not None:
                reader_thread.join(timeout=1.0)
            if encoder_thread is not None:
                encoder_thread.join(timeout=1.0)
            if temporal_encoder is not None:
                temporal_encoder.close()
            self.server.release_rgbd_client(client_id, self)

    def preempt_rgbd_websocket(self) -> None:
        """Close this page promptly when a newer RGB-D page takes ownership."""

        preempted = getattr(self, "rgbd_preempted", None)
        if preempted is None or preempted.is_set():
            return
        preempted.set()
        send_lock = getattr(self, "rgbd_send_lock", None)
        acquired = bool(send_lock and send_lock.acquire(timeout=0.1))
        try:
            if acquired:
                try:
                    self.send_ws_frame(
                        struct.pack("!H", 4001) + b"superseded by newer browser",
                        0x8,
                    )
                except (ConnectionError, OSError, ValueError):
                    pass
        finally:
            if acquired:
                send_lock.release()
        try:
            self.connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

    def handle_av1_websocket(self) -> None:
        if not self.server.ffmpeg_slots.acquire(blocking=False):
            self.send_error(503, "Encoded stream capacity reached")
            return
        params = parse_qs(urlsplit(self.path).query)
        width = clamp_int(params.get("w", params.get("width", ["640"]))[0], 160, 1920)
        fps = clamp_int(params.get("fps", ["30"])[0], 1, 60)
        bitrate = safe_bitrate(params.get("bitrate", ["3200k"])[0])
        bitrate_bps = int(bitrate[:-1]) * 1000
        raw_url = f"http://{self.server.stream_host}:{self.server.stream_port}/raw?w={width}&fps={fps}"
        raw_reader = None
        output = None
        writer = None
        accepted = False
        try:
            if not self.accept_websocket():
                return
            accepted = True
            raw_reader = MultipartRawFrameReader(raw_url)
            headers, frame_data = raw_reader.read_part()
            frame_width, frame_height = validate_raw_viewport_frame(headers, frame_data)
            writer = WebSocketWriteAdapter(lambda data: self.send_ws_frame(data, 0x2))
            output = av.open(
                writer,
                mode="w",
                format="webm",
                options={
                    "live": "1",
                    "cluster_time_limit": "100",
                    "cluster_size_limit": "0",
                    "flush_packets": "1",
                },
            )
            stream = output.add_stream("av1_nvenc", rate=fps)
            stream.width = frame_width
            stream.height = frame_height
            stream.pix_fmt = "yuv420p"
            stream.bit_rate = bitrate_bps
            stream.codec_context.time_base = fractions.Fraction(1, fps)
            stream.codec_context.options = {
                "preset": "p4",
                "tune": "ull",
                "rc": "cbr",
                "delay": "0",
                "zerolatency": "1",
                "rc-lookahead": "0",
                "spatial-aq": "1",
                "aq-strength": "8",
                "g": str(max(1, fps)),
            }
            print(
                f"AV1 NVENC WebSocket using raw Godot frames at {frame_width}x{frame_height} "
                f"{fps}fps {bitrate}",
                flush=True,
            )

            next_pts = 0
            started_at = time.monotonic()
            previous_frame_data = frame_data
            while True:
                target_pts = max(next_pts, int((time.monotonic() - started_at) * fps))
                while next_pts < target_pts:
                    frame = raw_video_frame(previous_frame_data, frame_width, frame_height, next_pts, fps)
                    for packet in stream.encode(frame):
                        output.mux(packet)
                    next_pts += 1
                frame = raw_video_frame(frame_data, frame_width, frame_height, target_pts, fps)
                for packet in stream.encode(frame):
                    output.mux(packet)
                writer.flush()
                next_pts = target_pts + 1
                previous_frame_data = frame_data
                headers, frame_data = raw_reader.read_part()
                next_width, next_height = validate_raw_viewport_frame(headers, frame_data)
                if (next_width, next_height) != (frame_width, frame_height):
                    raise ValueError("raw viewport dimensions changed during AV1 stream")
        except (BrokenPipeError, ConnectionError, OSError):
            pass
        except Exception as exc:
            print(f"AV1 NVENC WebSocket ended: {exc}", flush=True)
        finally:
            if output is not None:
                try:
                    output.close()
                except Exception:
                    pass
            if writer is not None:
                try:
                    writer.finish()
                except OSError:
                    pass
            if raw_reader is not None:
                raw_reader.close()
            if accepted:
                try:
                    self.send_ws_frame(b"", 0x8)
                except OSError:
                    pass
            self.server.ffmpeg_slots.release()

    def handle_arm_websocket(self):
        params = parse_qs(urlsplit(self.path).query)
        client_id = params.get("client", [""])[0]
        if (
            not client_id
            or len(client_id) > 64
            or any(
                not (character.isalnum() or character in "-_")
                for character in client_id
            )
        ):
            self.send_error(400, "Invalid arm control client")
            return
        peer = f"{self.client_address[0]}:{self.client_address[1]}"
        self.arm_preempted = threading.Event()
        self.arm_send_lock = threading.Lock()
        if not self.accept_websocket():
            return
        controller_id = self.server.claim_arm_controller(peer, client_id, self)
        if controller_id is None:
            self.send_ws_frame(
                struct.pack("!H", 4001) + b"superseded by newer browser",
                0x8,
            )
            return
        print(f"SO-101 arm controller connected: {peer}", flush=True)
        rate_window: deque[float] = deque()
        self.send_arm_status(controller_id)
        try:
            while True:
                if self.arm_preempted.is_set():
                    break
                frame = self.read_ws_frame()
                if frame is None:
                    break
                opcode, payload = frame
                if opcode == 0x8:
                    self.send_ws_frame(b"", 0x8)
                    break
                if opcode == 0x9:
                    self.send_ws_frame(payload, 0xA)
                    continue
                if opcode != 0x1:
                    continue
                try:
                    self.check_websocket_rate(rate_window, 75)
                    message = validate_browser_arm_message(json.loads(payload.decode("utf-8")))
                    if message["type"] in ("arm_command", "arm_cartesian_velocity", "arm_joint_velocity", "arm_tool_velocity"):
                        accepted, _transport_age_ms = (
                            self.server.normalize_browser_arm_timing(message)
                        )
                        if not accepted:
                            self.send_arm_status(controller_id)
                            continue
                    if message["type"] != "arm_status_request":
                        message["control_session"] = controller_id
                        self.server.forward_arm(message)
                        self.server.resume_keyboard_after_transient_watchdog(
                            controller_id,
                            message,
                        )
                    self.send_arm_status(controller_id)
                except (UnicodeDecodeError, json.JSONDecodeError, ArmProtocolError, ValueError) as exc:
                    status = self.server.latest_arm_status(controller_id)
                    status["fault"] = f"browser command rejected: {exc}"
                    self.send_ws_frame(json.dumps(status, separators=(",", ":")).encode("utf-8"))
        except (ConnectionError, OSError, ValueError):
            pass
        finally:
            self.server.release_arm_controller(controller_id, client_id, self)
            print(f"SO-101 arm controller disconnected: {peer}", flush=True)

    def preempt_arm_websocket(self) -> None:
        """Hold and close this controller when a newer page takes ownership."""

        preempted = getattr(self, "arm_preempted", None)
        if preempted is None or preempted.is_set():
            return
        preempted.set()
        send_lock = getattr(self, "arm_send_lock", None)
        acquired = bool(send_lock and send_lock.acquire(timeout=0.1))
        try:
            if acquired:
                try:
                    self.send_ws_frame(
                        struct.pack("!H", 4001) + b"superseded by newer browser",
                        0x8,
                    )
                except (ConnectionError, OSError, ValueError):
                    pass
        finally:
            if acquired:
                send_lock.release()
        try:
            self.connection.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass

    def send_arm_status(self, controller_id: str) -> None:
        status = self.server.latest_arm_status(controller_id)
        payload = json.dumps(status, separators=(",", ":")).encode("utf-8")
        send_lock = getattr(self, "arm_send_lock", None)
        if send_lock is None:
            self.send_ws_frame(payload)
            return
        with send_lock:
            if not self.arm_preempted.is_set():
                self.send_ws_frame(payload)

    def check_websocket_rate(self, window: deque[float], limit: int) -> None:
        now = time.monotonic()
        while window and now - window[0] >= 1.0:
            window.popleft()
        if len(window) >= limit:
            raise ValueError("WebSocket message rate exceeded")
        window.append(now)

    def validate_head_tracking(self, data: object) -> dict:
        if not isinstance(data, dict) or data.get("type", "tracking") != "tracking":
            raise ValueError("invalid tracking message")
        active = data.get("active")
        if not isinstance(active, bool):
            raise ValueError("tracking active must be boolean")
        values = []
        for key in ("x", "y", "z"):
            value = data.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
                raise ValueError(f"tracking {key} must be finite")
            if abs(float(value)) > 1000.0:
                raise ValueError(f"tracking {key} outside accepted range")
            values.append(float(value))
        return {
            "type": "tracking",
            "source": str(data.get("source", "browser"))[:80],
            "active": active,
            "status": str(data.get("status", ""))[:160],
            "x": values[0],
            "y": values[1],
            "z": values[2],
            "sent_unix_ms": int(data.get("sent_unix_ms", int(time.time() * 1000))),
            "remote_sent_unix_ms": int(data.get("remote_sent_unix_ms", int(time.time() * 1000))),
            "bridge_recv_unix_ms": int(time.time() * 1000),
        }

    def validate_remote_control_message(self, data: object) -> dict:
        if isinstance(data, dict) and data.get("type") == "view_settings":
            return self.validate_view_settings(data)
        return self.validate_head_tracking(data)

    def validate_view_settings(self, data: object) -> dict:
        if not isinstance(data, dict):
            raise ValueError("view settings must be an object")
        bounds = {
            "yaw_gain": (0.0, 12.0),
            "pitch_gain": (0.0, 12.0),
            "max_yaw": (5.0, 180.0),
            "max_pitch": (5.0, 89.0),
            "focus_distance": (0.1, 8.0),
            "orbit_distance": (0.1, 8.0),
            "focus_vertical_offset": (-0.2, 1.0),
            "dolly_gain": (0.0, 0.08),
            "min_distance": (0.1, 8.0),
            "max_distance": (0.2, 20.0),
            "fov": (25.0, 110.0),
            "manual_wrist_flex_trim_degrees": (-90.0, 90.0),
            "manual_wrist_roll_trim_degrees": (-180.0, 180.0),
            "manual_wrist_roll_direction": (-1.0, 1.0),
            "manual_tool_x": (-0.05, 0.05),
            "manual_tool_y": (-0.05, 0.05),
            "manual_tool_z": (-0.05, 0.05),
            "manual_tool_roll": (-180.0, 180.0),
            "manual_tool_pitch": (-180.0, 180.0),
            "manual_tool_yaw": (-180.0, 180.0),
            "manual_opening_offset_degrees": (-35.0, 35.0),
            "manual_opening_scale": (0.5, 1.5),
            "manual_overlay_opacity": (0.05, 0.9),
        }
        packet: dict[str, object] = {"type": "view_settings"}
        for key, (minimum, maximum) in bounds.items():
            if key not in data:
                continue
            value = data[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
                raise ValueError(f"{key} must be finite")
            packet[key] = max(minimum, min(maximum, float(value)))
        for key in (
            "inspect_enabled",
            "dolly_enabled",
            "robot_overlay_enabled",
            "robot_overlay_mask_scanned_arm",
            "recenter",
            "calibrate_robot_position",
            "refine_robot_joint_alignment",
            "cancel_robot_position_calibration",
            "arm_measured_feedback_enabled",
            "arm_target_ghost_enabled",
            "arm_following_error_safety_enabled",
            "arm_freeze_overlay_on_stale_enabled",
            "arm_d455_visual_correction_enabled",
            "white_background_enabled",
            "manual_claw_calibration_begin",
            "manual_claw_calibration_reset",
            "manual_claw_calibration_cancel",
            "manual_claw_calibration_save",
        ):
            if key in data:
                if not isinstance(data[key], bool):
                    raise ValueError(f"{key} must be boolean")
                packet[key] = data[key]
        if "robot_overlay_style" in data:
            style = data["robot_overlay_style"]
            if not isinstance(style, str) or style not in ("alignment", "solid"):
                raise ValueError("robot_overlay_style must be alignment or solid")
            packet["robot_overlay_style"] = style
        if "focus_pick_uv" in data:
            value = data["focus_pick_uv"]
            if not isinstance(value, list) or len(value) != 2:
                raise ValueError("focus_pick_uv must contain two coordinates")
            coordinates: list[float] = []
            for coordinate in value:
                if isinstance(coordinate, bool) or not isinstance(coordinate, (int, float)) or not np.isfinite(coordinate):
                    raise ValueError("focus_pick_uv coordinates must be finite")
                coordinates.append(max(0.0, min(1.0, float(coordinate))))
            packet["focus_pick_uv"] = coordinates
        if "focus_pick_sent_unix_ms" in data:
            sent_unix_ms = data["focus_pick_sent_unix_ms"]
            if isinstance(sent_unix_ms, bool) or not isinstance(sent_unix_ms, (int, float)) or not np.isfinite(sent_unix_ms):
                raise ValueError("focus_pick_sent_unix_ms must be finite")
            packet["focus_pick_sent_unix_ms"] = int(sent_unix_ms)
        return packet

    def read_exact(self, size: int) -> bytes | None:
        data = b""
        while len(data) < size:
            chunk = self.connection.recv(size - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def read_ws_frame(self):
        header = self.read_exact(2)
        if header is None:
            return None
        first, second = header
        opcode = first & 0x0F
        masked = (second & 0x80) != 0
        if not masked:
            raise ValueError("client WebSocket frames must be masked")
        length = second & 0x7F
        if length == 126:
            extended = self.read_exact(2)
            if extended is None:
                return None
            length = struct.unpack("!H", extended)[0]
        elif length == 127:
            extended = self.read_exact(8)
            if extended is None:
                return None
            length = struct.unpack("!Q", extended)[0]
        if length > MAX_WEBSOCKET_FRAME:
            raise ValueError("WebSocket frame is too large")
        mask = self.read_exact(4) if masked else b""
        payload = self.read_exact(length)
        if payload is None:
            return None
        if masked:
            payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
        return opcode, payload

    def send_ws_frame(self, payload: bytes, opcode: int = 0x1):
        first = 0x80 | (opcode & 0x0F)
        length = len(payload)
        if length < 126:
            header = bytes([first, length])
        elif length <= 0xFFFF:
            header = bytes([first, 126]) + struct.pack("!H", length)
        else:
            header = bytes([first, 127]) + struct.pack("!Q", length)
        self.connection.sendall(header + payload)


def ensure_cert(cert: Path, key: Path, host: str):
    if cert.exists() and key.exists():
        return
    cert.parent.mkdir(parents=True, exist_ok=True)
    alt_names = ["DNS:localhost", "IP:127.0.0.1"]
    if host not in ("0.0.0.0", "::"):
        alt_names.append(f"IP:{host}")
    for ip in lan_ips():
        entry = f"IP:{ip}"
        if entry not in alt_names:
            alt_names.append(entry)
    alt_name = ",".join(alt_names)
    cmd = [
        "openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
        "-keyout", str(key), "-out", str(cert), "-days", "365",
        "-subj", "/CN=Godot LAN Remote",
        "-addext", f"subjectAltName={alt_name}",
    ]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def lan_ips() -> list[str]:
    try:
        raw = subprocess.check_output(["hostname", "-I"], text=True).strip()
    except Exception:
        return []
    return [ip for ip in raw.split() if "." in ip and not ip.startswith("127.")]


def clamp_int(value: str, minimum: int, maximum: int) -> int:
    try:
        parsed = int(float(value))
    except (TypeError, ValueError):
        parsed = minimum
    return max(minimum, min(maximum, parsed))


def safe_bitrate(value: str) -> str:
    text = str(value).strip().lower()
    if not text:
        return "2400k"
    suffix = text[-1]
    number = text[:-1] if suffix in ("k", "m") else text
    try:
        amount = float(number)
    except ValueError:
        return "2400k"
    if suffix == "m":
        kbps = int(amount * 1000)
    else:
        kbps = int(amount)
    kbps = max(250, min(24000, kbps))
    return f"{kbps}k"


def video_buffer_size(bitrate: str, fps: int) -> str:
    kbps = int(safe_bitrate(bitrate)[:-1])
    # Two frames of VBV capacity keeps NVENC responsive without crushing I-frame quality.
    buffer_kbits = max(250, min(1600, int(round(kbps * 2.0 / max(1, fps)))))
    return f"{buffer_kbits}k"


def ffmpeg_binary() -> str:
    configured = os.environ.get("GODOT_REMOTE_FFMPEG", "").strip()
    if configured:
        return configured
    system_ffmpeg = Path("/usr/bin/ffmpeg")
    if system_ffmpeg.is_file() and os.access(system_ffmpeg, os.X_OK):
        return str(system_ffmpeg)
    return "ffmpeg"


_FFMPEG_ENCODER_CACHE: dict[str, bool] = {}


def ffmpeg_encoder_available(name: str) -> bool:
    if name in _FFMPEG_ENCODER_CACHE:
        return _FFMPEG_ENCODER_CACHE[name]
    try:
        output = subprocess.check_output(
            [ffmpeg_binary(), "-hide_banner", "-encoders"],
            text=True,
            stderr=subprocess.STDOUT,
            timeout=3.0,
        )
    except Exception:
        _FFMPEG_ENCODER_CACHE[name] = False
        return False
    available = name in output
    _FFMPEG_ENCODER_CACHE[name] = available
    if available:
        print(f"ffmpeg encoder available: {name}", flush=True)
    return available


def prefer_video_codec(pc: RTCPeerConnection, mime_type: str) -> None:
    codecs = RTCRtpSender.getCapabilities("video").codecs
    preferred = [codec for codec in codecs if codec.mimeType.lower() == mime_type.lower()]
    fallback = [codec for codec in codecs if codec.mimeType.lower() != mime_type.lower()]
    if not preferred:
        return
    transceivers = pc.getTransceivers()
    if not transceivers:
        return
    transceivers[-1].setCodecPreferences(preferred + fallback)


def configure_h264_sender(sender: RTCRtpSender, bitrate_kbps: int) -> None:
    """Seed aiortc's encoder with the selected rate before the first frame."""
    encoder = NvencH264Encoder()
    encoder.target_bitrate = max(250_000, min(24_000_000, int(bitrate_kbps) * 1000))
    # aiortc creates this lazily at its fixed DEFAULT_BITRATE. Seeding the
    # private slot is currently its only per-sender startup bitrate API.
    setattr(sender, "_RTCRtpSender__encoder", encoder)
    print(f"WebRTC H.264 startup bitrate: {encoder.target_bitrate // 1000} kbps", flush=True)


def with_video_bitrate(sdp: str, bitrate_kbps: int) -> str:
    lines = sdp.splitlines()
    out: list[str] = []
    in_video = False
    inserted = False
    video_payloads: set[str] = set()
    video_codecs: set[str] = set()
    for line in lines:
        if line.startswith("m="):
            if in_video and not inserted:
                out.append(f"b=AS:{bitrate_kbps}")
            in_video = line.startswith("m=video")
            inserted = False
            if in_video:
                parts = line.split()
                video_payloads = set(parts[3:])
            out.append(line)
            continue
        if in_video and line.startswith("a=rtpmap:"):
            parts = line.split(":", 1)[1].split(None, 1)
            payload = parts[0]
            codec_name = parts[1].split("/", 1)[0].upper() if len(parts) > 1 else ""
            if payload in video_payloads and codec_name in ("VP8", "H264"):
                video_codecs.add(payload)
        if in_video and line.startswith("b=AS:"):
            if not inserted:
                out.append(f"b=AS:{bitrate_kbps}")
                inserted = True
            continue
        if in_video and line.startswith("a=fmtp:"):
            prefix, params = line.split(" ", 1) if " " in line else (line, "")
            payload = prefix.split(":", 1)[1]
            if payload in video_codecs:
                params = append_google_bitrate_params(params, bitrate_kbps)
                out.append(prefix + " " + params)
                continue
        out.append(line)
        if in_video and not inserted and line.startswith("c="):
            out.append(f"b=AS:{bitrate_kbps}")
            inserted = True
    if in_video and not inserted:
        out.append(f"b=AS:{bitrate_kbps}")
    existing_fmtp_payloads = {
        line.split(":", 1)[1].split(None, 1)[0]
        for line in out
        if line.startswith("a=fmtp:")
    }
    final: list[str] = []
    for line in out:
        final.append(line)
        if line.startswith("a=rtpmap:"):
            payload = line.split(":", 1)[1].split(None, 1)[0]
            if payload in video_codecs and payload not in existing_fmtp_payloads:
                final.append(f"a=fmtp:{payload} {google_bitrate_params(bitrate_kbps)}")
    return "\r\n".join(final) + "\r\n"


def append_google_bitrate_params(params: str, bitrate_kbps: int) -> str:
    existing = [part.strip() for part in params.split(";") if part.strip()]
    filtered = [
        part for part in existing
        if not part.startswith("x-google-start-bitrate=")
        and not part.startswith("x-google-min-bitrate=")
        and not part.startswith("x-google-max-bitrate=")
    ]
    filtered.extend(google_bitrate_params(bitrate_kbps).split(";"))
    return ";".join(filtered)


def google_bitrate_params(bitrate_kbps: int) -> str:
    start = max(250, min(bitrate_kbps, 24000))
    minimum = max(250, min(start // 2, 12000))
    return f"x-google-start-bitrate={start};x-google-min-bitrate={minimum};x-google-max-bitrate={start}"


class MjpegVideoStreamTrack(VideoStreamTrack):
    def __init__(self, source_url: str):
        super().__init__()
        self.source_url = source_url
        self.container = None
        self.decoder = None
        self.io_lock = threading.Lock()

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        loop = asyncio.get_running_loop()
        frame = await loop.run_in_executor(None, self._read_frame)
        frame.pts = pts
        frame.time_base = time_base
        return frame

    def _open(self):
        self._close()
        self.container = av.open(
            self.source_url,
            format="mjpeg",
            options={
                "fflags": "nobuffer",
                "flags": "low_delay",
                "probesize": "32",
                "analyzeduration": "0",
            },
        )
        self.decoder = self.container.decode(video=0)

    def _close(self):
        if self.container is not None:
            try:
                self.container.close()
            except Exception:
                pass
        self.container = None
        self.decoder = None

    def _read_frame(self):
        with self.io_lock:
            for _ in range(2):
                try:
                    if self.decoder is None:
                        self._open()
                    frame = next(self.decoder)
                    return frame.reformat(format="yuv420p")
                except Exception:
                    self._close()
                    time.sleep(0.02)
            image = np.zeros((360, 640, 3), dtype=np.uint8)
            return av.VideoFrame.from_ndarray(image, format="rgb24").reformat(format="yuv420p")

    def stop(self):
        with self.io_lock:
            self._close()
        super().stop()


class HttpChunkedWriteAdapter:
    def __init__(self, connection: socket.socket):
        self.connection = connection
        self.position = 0
        self.closed = False

    def write(self, data: bytes) -> int:
        if self.closed:
            raise BrokenPipeError("chunked video response is closed")
        if not data:
            return 0
        header = f"{len(data):X}\r\n".encode("ascii")
        self.connection.sendall(header + data + b"\r\n")
        self.position += len(data)
        return len(data)

    def flush(self) -> None:
        pass

    def tell(self) -> int:
        return self.position

    def finish(self) -> None:
        if self.closed:
            return
        self.closed = True
        try:
            self.connection.sendall(b"0\r\n\r\n")
        except OSError:
            pass


class WebSocketWriteAdapter:
    def __init__(self, sender, flush_threshold: int = 16 * 1024):
        self.sender = sender
        self.flush_threshold = flush_threshold
        self.buffer = bytearray()
        self.position = 0
        self.closed = False

    def write(self, data: bytes) -> int:
        if self.closed:
            raise BrokenPipeError("WebSocket video response is closed")
        if not data:
            return 0
        self.buffer.extend(data)
        self.position += len(data)
        if len(self.buffer) >= self.flush_threshold:
            self.flush()
        return len(data)

    def flush(self) -> None:
        if not self.buffer:
            return
        payload = bytes(self.buffer)
        self.buffer.clear()
        self.sender(payload)

    def tell(self) -> int:
        return self.position

    def finish(self) -> None:
        if self.closed:
            return
        self.flush()
        self.closed = True


def validate_raw_viewport_frame(headers: dict[str, str], data: bytes) -> tuple[int, int]:
    width = int(headers.get("x-frame-width", "0") or "0")
    height = int(headers.get("x-frame-height", "0") or "0")
    frame_format = headers.get("x-frame-format", "rgb24")
    if frame_format != "rgb24" or width <= 0 or height <= 0:
        raise ValueError(f"unsupported raw viewport frame {width}x{height} {frame_format}")
    expected = width * height * 3
    if len(data) != expected:
        raise ValueError(f"bad raw viewport frame bytes={len(data)} expected={expected}")
    return width, height


def raw_video_frame(data: bytes, width: int, height: int, pts: int, fps: int) -> av.VideoFrame:
    array = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
    frame = av.VideoFrame.from_ndarray(array, format="rgb24")
    frame.pts = pts
    frame.time_base = fractions.Fraction(1, fps)
    return frame


class MultipartRawFrameReader:
    def __init__(self, source_url: str):
        parsed = urlsplit(source_url)
        if parsed.scheme != "http" or not parsed.hostname:
            raise ValueError(f"Unsupported raw stream URL: {source_url}")
        self.host = parsed.hostname
        self.port = parsed.port or 80
        self.path = parsed.path or "/raw"
        if parsed.query:
            self.path += "?" + parsed.query
        self.sock: socket.socket | None = None
        self.buffer = b""
        self.boundary = b"--godotframe"
        self._open()

    def _open(self) -> None:
        self.close()
        sock = socket.create_connection((self.host, self.port), timeout=3.0)
        sock.settimeout(5.0)
        request = (
            f"GET {self.path} HTTP/1.1\r\n"
            f"Host: {self.host}:{self.port}\r\n"
            "Connection: close\r\n"
            "\r\n"
        ).encode("ascii")
        sock.sendall(request)
        self.sock = sock
        while b"\r\n\r\n" not in self.buffer:
            self._read_more()
        header_blob, self.buffer = self.buffer.split(b"\r\n\r\n", 1)
        header_text = header_blob.decode("iso-8859-1", errors="replace")
        status_line = header_text.split("\r\n", 1)[0]
        if " 200 " not in status_line:
            raise OSError(status_line)
        headers = parse_http_headers(header_text)
        content_type = headers.get("content-type", "")
        marker = "boundary="
        if marker in content_type:
            value = content_type.split(marker, 1)[1].split(";", 1)[0].strip().strip('"')
            if value:
                self.boundary = ("--" + value).encode("ascii", errors="ignore")

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None
        self.buffer = b""

    def _read_more(self) -> None:
        if self.sock is None:
            raise OSError("raw stream is closed")
        chunk = self.sock.recv(65536)
        if not chunk:
            raise OSError("raw stream ended")
        self.buffer += chunk

    def read_part(self) -> tuple[dict[str, str], bytes]:
        while True:
            boundary_index = self.buffer.find(self.boundary)
            if boundary_index < 0:
                if len(self.buffer) > 1024 * 1024:
                    self.buffer = self.buffer[-len(self.boundary):]
                self._read_more()
                continue
            if boundary_index > 0:
                self.buffer = self.buffer[boundary_index:]
            header_end = self.buffer.find(b"\r\n\r\n", len(self.boundary))
            if header_end < 0:
                self._read_more()
                continue
            header_start = len(self.boundary)
            if self.buffer[header_start:header_start + 2] == b"\r\n":
                header_start += 2
            header_text = self.buffer[header_start:header_end].decode("iso-8859-1", errors="replace")
            headers = parse_http_headers(header_text)
            length = int(headers.get("content-length", "0") or "0")
            if length <= 0:
                self.buffer = self.buffer[header_end + 4:]
                continue
            data_start = header_end + 4
            data_end = data_start + length
            while len(self.buffer) < data_end:
                self._read_more()
            data = self.buffer[data_start:data_end]
            self.buffer = self.buffer[data_end:]
            return headers, data


async def publish_latest_rgbd_datachannel(channel, pc: RTCPeerConnection, source_url: str) -> None:
    """Send only the newest local RGB-D packet over a partial-reliability channel."""

    loop = asyncio.get_running_loop()
    packet_ready = asyncio.Event()
    stop = threading.Event()
    newest_lock = threading.Lock()
    newest_packet: bytes | None = None
    newest_sequence = 0
    reader_lock = threading.Lock()
    current_reader: MultipartRawFrameReader | None = None

    def drain_local_rgbd() -> None:
        nonlocal newest_packet, newest_sequence, current_reader
        while not stop.is_set():
            reader = None
            try:
                reader = MultipartRawFrameReader(source_url)
                with reader_lock:
                    current_reader = reader
                while not stop.is_set():
                    _headers, packet = reader.read_part()
                    if not packet:
                        continue
                    with newest_lock:
                        newest_packet = packet
                        newest_sequence += 1
                    loop.call_soon_threadsafe(packet_ready.set)
            except (ConnectionError, OSError, ValueError):
                if not stop.wait(0.25):
                    continue
            finally:
                if reader is not None:
                    reader.close()
                with reader_lock:
                    if current_reader is reader:
                        current_reader = None

    reader_thread = threading.Thread(
        target=drain_local_rgbd,
        name="rgbd-datachannel-reader",
        daemon=True,
    )
    reader_thread.start()
    sent_sequence = 0
    sent_frames = 0
    dropped_frames = 0
    try:
        while channel.readyState != "closed" and pc.connectionState not in ("failed", "closed"):
            try:
                await asyncio.wait_for(packet_ready.wait(), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            packet_ready.clear()
            with newest_lock:
                packet = newest_packet
                sequence = newest_sequence
            if not packet or sequence == sent_sequence:
                continue
            if sequence > sent_sequence + 1 and sent_sequence > 0:
                dropped_frames += sequence - sent_sequence - 1
            sent_sequence = sequence
            if channel.readyState != "open":
                dropped_frames += 1
                continue
            # Never queue another complete frame behind an unsent one.
            if channel.bufferedAmount > 0:
                dropped_frames += 1
                continue
            try:
                channel.send(packet)
                sent_frames += 1
            except (ConnectionError, InvalidStateError, OSError, ValueError):
                break
            await asyncio.sleep(0)
    except asyncio.CancelledError:
        raise
    finally:
        stop.set()
        with reader_lock:
            reader = current_reader
        if reader is not None:
            reader.close()
        reader_thread.join(timeout=1.0)
        print(
            f"WebRTC RGB-D publisher ended: sent={sent_frames} dropped={dropped_frames}",
            flush=True,
        )


class RawViewportVideoStreamTrack(VideoStreamTrack):
    def __init__(self, raw_url: str, fallback_mjpeg_url: str):
        super().__init__()
        self.raw_url = raw_url
        self.fallback_mjpeg_url = fallback_mjpeg_url
        self.raw_reader: MultipartRawFrameReader | None = None
        self.mjpeg_container = None
        self.mjpeg_decoder = None
        self.last_width = 640
        self.last_height = 360
        self.using_fallback = False
        self.io_lock = threading.Lock()

    async def recv(self):
        pts, time_base = await self.next_timestamp()
        loop = asyncio.get_running_loop()
        frame = await loop.run_in_executor(None, self._read_frame)
        frame.pts = pts
        frame.time_base = time_base
        return frame

    def _close_raw(self) -> None:
        if self.raw_reader is not None:
            self.raw_reader.close()
        self.raw_reader = None

    def _close_mjpeg(self) -> None:
        if self.mjpeg_container is not None:
            try:
                self.mjpeg_container.close()
            except Exception:
                pass
        self.mjpeg_container = None
        self.mjpeg_decoder = None

    def _read_frame(self):
        with self.io_lock:
            for _ in range(2):
                try:
                    return self._read_raw_frame()
                except Exception as exc:
                    if not self.using_fallback:
                        print(f"WebRTC raw viewport stream unavailable, falling back to MJPEG: {exc}", flush=True)
                        self.using_fallback = True
                    self._close_raw()
                    time.sleep(0.01)
            try:
                return self._read_mjpeg_frame()
            except Exception:
                self._close_mjpeg()
                image = np.zeros((self.last_height, self.last_width, 3), dtype=np.uint8)
                return av.VideoFrame.from_ndarray(image, format="rgb24").reformat(format="yuv420p")

    def _read_raw_frame(self):
        if self.raw_reader is None:
            self.raw_reader = MultipartRawFrameReader(self.raw_url)
            self.using_fallback = False
            print("WebRTC using raw Godot viewport frames", flush=True)
        headers, data = self.raw_reader.read_part()
        width = int(headers.get("x-frame-width", "0") or "0")
        height = int(headers.get("x-frame-height", "0") or "0")
        frame_format = headers.get("x-frame-format", "rgb24")
        if frame_format != "rgb24":
            raise ValueError(f"unsupported raw frame format: {frame_format}")
        expected = width * height * 3
        if width <= 0 or height <= 0 or len(data) < expected:
            raise ValueError(f"bad raw frame size: {width}x{height} bytes={len(data)}")
        self.last_width = width
        self.last_height = height
        array = np.frombuffer(data[:expected], dtype=np.uint8).reshape((height, width, 3))
        return av.VideoFrame.from_ndarray(array, format="rgb24").reformat(format="yuv420p")

    def _read_mjpeg_frame(self):
        if self.mjpeg_decoder is None:
            self._close_mjpeg()
            self.mjpeg_container = av.open(
                self.fallback_mjpeg_url,
                format="mjpeg",
                options={
                    "fflags": "nobuffer",
                    "flags": "low_delay",
                    "probesize": "32",
                    "analyzeduration": "0",
                },
            )
            self.mjpeg_decoder = self.mjpeg_container.decode(video=0)
        frame = next(self.mjpeg_decoder)
        self.last_width = frame.width
        self.last_height = frame.height
        return frame.reformat(format="yuv420p")

    def stop(self):
        with self.io_lock:
            self._close_raw()
            self._close_mjpeg()
        super().stop()


def parse_http_headers(text: str) -> dict[str, str]:
    headers: dict[str, str] = {}
    for line in text.split("\r\n"):
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        headers[key.strip().lower()] = value.strip()
    return headers


def resolve_claw_camera_device(configured: str) -> str:
    requested = str(configured or "").strip()
    if requested.lower() in ("off", "none", "disabled"):
        return ""
    if requested:
        path = Path(requested).expanduser()
        return str(path.resolve()) if path.exists() else str(path)
    by_id = Path("/dev/v4l/by-id")
    if by_id.is_dir():
        matches = sorted(by_id.glob("*USB2.0_CAM1*video-index0"))
        if matches:
            return str(matches[0])
    for name_path in sorted(Path("/sys/class/video4linux").glob("video*/name")):
        try:
            if "USB2.0_CAM1" in name_path.read_text(encoding="utf-8", errors="replace"):
                return f"/dev/{name_path.parent.name}"
        except OSError:
            continue
    return ""


def main():
    parser = argparse.ArgumentParser(description="HTTPS LAN page, WSS head tracking bridge, and HTTPS viewport stream proxy.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--root", default=str(Path(__file__).resolve().parents[1] / "web"))
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=4247)
    parser.add_argument("--stream-host", default="127.0.0.1")
    parser.add_argument("--stream-port", type=int, default=8780)
    parser.add_argument("--password", default=os.environ.get("GODOT_REMOTE_PASSWORD", ""))
    parser.add_argument("--arm-command-port", type=int, default=4248)
    parser.add_argument("--arm-status-port", type=int, default=4249)
    parser.add_argument("--robot-calibration-status-port", type=int, default=4251)
    parser.add_argument(
        "--claw-camera-device",
        default=os.environ.get("GODOT_REMOTE_CLAW_CAMERA", ""),
        help="V4L2 claw-camera device; defaults to the USB2.0_CAM1 video-index0 device, or 'off'.",
    )
    parser.add_argument("--public-arm", action="store_true")
    parser.add_argument("--allow-quick-tunnel-arm", action="store_true")
    parser.add_argument("--public-hostname", default=os.environ.get("GODOT_REMOTE_PUBLIC_HOSTNAME", ""))
    parser.add_argument("--access-team-domain", default=os.environ.get("CLOUDFLARE_ACCESS_TEAM_DOMAIN", ""))
    parser.add_argument("--access-audience", default=os.environ.get("CLOUDFLARE_ACCESS_AUD", ""))
    parser.add_argument("--cert", default=str(Path(__file__).resolve().parents[1] / ".local_certs" / "lan_remote.crt"))
    parser.add_argument("--key", default=str(Path(__file__).resolve().parents[1] / ".local_certs" / "lan_remote.key"))
    args = parser.parse_args()

    if args.public_arm and not (args.public_hostname and args.access_team_domain and args.access_audience):
        parser.error("--public-arm requires --public-hostname, --access-team-domain, and --access-audience")

    cert = Path(args.cert)
    key = Path(args.key)
    try:
        ensure_cert(cert, key, args.host)
    except Exception as exc:
        print(f"Could not create TLS cert with openssl: {exc}", file=sys.stderr)
        sys.exit(1)

    server = LanRemoteServer(
        (args.host, args.port),
        LanRemoteHandler,
        Path(args.root),
        args.udp_host,
        args.udp_port,
        args.stream_host,
        args.stream_port,
        args.password,
        args.arm_command_port,
        args.arm_status_port,
        args.public_arm,
        args.allow_quick_tunnel_arm,
        args.public_hostname,
        args.access_team_domain,
        args.access_audience,
        robot_calibration_status_port=args.robot_calibration_status_port,
        claw_camera_device=args.claw_camera_device,
    )
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certfile=cert, keyfile=key)
    server.socket = context.wrap_socket(server.socket, server_side=True)
    print(f"Godot LAN remote listening on https://{args.host}:{args.port}", flush=True)
    for ip in lan_ips():
        print(f"Open from another device: https://{ip}:{args.port}/controller.html", flush=True)
    if args.password:
        print("Controller password gate is ON.", flush=True)
    else:
        print("Controller password gate is OFF. Use --password or GODOT_REMOTE_PASSWORD before exposing publicly.", flush=True)
    if args.public_arm:
        print(f"Public arm control enabled for named Access hostname: {args.public_hostname}", flush=True)
    elif args.allow_quick_tunnel_arm:
        print("WARNING: quick-tunnel arm control is ON and protected only by the controller password.", flush=True)
    else:
        print("Public arm control is OFF; /arm-control accepts authenticated direct-LAN sessions only.", flush=True)
    if server.claw_camera_device:
        print(f"Claw camera ready on /claw-stream: {server.claw_camera_device}", flush=True)
    else:
        print("Claw camera unavailable; /claw-stream will return 503.", flush=True)
    print("The first visit will show a self-signed certificate warning; accept it for LAN testing.", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Stopping Godot LAN remote...", flush=True)
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
