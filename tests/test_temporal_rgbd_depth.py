from __future__ import annotations

import json
import struct
import sys
import zlib
from pathlib import Path

import av
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from lan_remote_view_server import (  # noqa: E402
    TEMPORAL_DEPTH_ENCODING,
    TEMPORAL_DEPTH_HEADER,
    TEMPORAL_DEPTH_MAGIC,
    TEMPORAL_COLOR_HEADER,
    TEMPORAL_COLOR_MAGIC,
    TemporalRgbdDepthEncoder,
)


def make_packet(depth: np.ndarray, predictive: bool = True) -> bytes:
    depth = np.asarray(depth, dtype=np.uint16)
    if predictive:
        previous = np.zeros_like(depth)
        previous[1:] = depth[:-1]
        modulo_delta = np.subtract(depth, previous, dtype=np.uint16)
        signed_delta = modulo_delta.view(np.int16).astype(np.int32)
        zigzag = ((signed_delta << 1) ^ (signed_delta >> 31)).astype(np.uint16)
        raw = (
            (zigzag & 0xFF).astype(np.uint8).tobytes()
            + (zigzag >> 8).astype(np.uint8).tobytes()
        )
        encoding = "u16_mm_vpredict_shuffle_deflate"
    else:
        raw = depth.astype("<u2").tobytes()
        encoding = "u16_mm_deflate"
    compressed = zlib.compress(raw, 1)
    color = b"\xff\xd8test-jpeg\xff\xd9"
    camera = {
        "id": "d455",
        "width": int(depth.shape[1]),
        "height": int(depth.shape[0]),
        "color_length": len(color),
        "depth_length": len(compressed),
        "depth_encoding": encoding,
    }
    metadata = json.dumps({"cameras": [camera]}, separators=(",", ":")).encode()
    return (
        b"RGBD1"
        + struct.pack("<I", len(metadata))
        + metadata
        + color
        + compressed
    )


def split_packet(packet: bytes) -> tuple[dict, bytes]:
    metadata_length = struct.unpack_from("<I", packet, 5)[0]
    metadata = json.loads(packet[9 : 9 + metadata_length])
    offset = 9 + metadata_length
    camera = metadata["cameras"][0]
    offset += camera["color_length"]
    return camera, packet[offset : offset + camera["depth_length"]]


def parse_persistent_packet(
    packet: bytes,
    previous: dict | None = None,
) -> tuple[dict, bytes]:
    metadata_length = struct.unpack_from("<I", packet, 5)[0]
    wire = json.loads(packet[9 : 9 + metadata_length])
    metadata = wire
    if wire.get("metadata_delta") == "persistent_v1":
        assert previous is not None
        assert previous["metadata_frame_id"] == wire["metadata_base_id"]
        metadata = {**previous, **wire}
        prior_cameras = {
            str(camera.get("id", index)): camera
            for index, camera in enumerate(previous["cameras"])
        }
        metadata["cameras"] = []
        for index, camera in enumerate(wire["cameras"]):
            key = str(camera.get("id", index))
            merged = {**prior_cameras.get(key, {}), **camera}
            for field in camera.get("metadata_removed", []):
                merged.pop(field, None)
            merged.pop("metadata_removed", None)
            metadata["cameras"].append(merged)
    camera = metadata["cameras"][0]
    offset = 9 + metadata_length + camera["color_length"]
    depth = packet[offset : offset + camera["depth_length"]]
    return metadata, depth


def decode_temporal_delta(
    compressed: bytes,
    camera: dict,
    keyframe: np.ndarray,
) -> np.ndarray:
    packed = zlib.decompress(compressed)
    magic, keyframe_id, tile, rows, cols, changed_count = (
        TEMPORAL_DEPTH_HEADER.unpack_from(packed)
    )
    assert magic == TEMPORAL_DEPTH_MAGIC
    assert keyframe_id == camera["depth_keyframe_id"]
    bit_count = rows * cols
    bit_length = (bit_count + 7) // 8
    bits = packed[TEMPORAL_DEPTH_HEADER.size : TEMPORAL_DEPTH_HEADER.size + bit_length]
    values_per_tile = tile * tile
    low_offset = TEMPORAL_DEPTH_HEADER.size + bit_length
    high_offset = low_offset + changed_count * values_per_tile
    output = keyframe.copy()
    changed_index = 0
    for tile_index in range(bit_count):
        if not bits[tile_index // 8] & (1 << (tile_index % 8)):
            continue
        tile_y = tile_index // cols * tile
        tile_x = tile_index % cols * tile
        for local_y in range(tile):
            for local_x in range(tile):
                y = tile_y + local_y
                x = tile_x + local_x
                if y >= output.shape[0] or x >= output.shape[1]:
                    continue
                sample = changed_index * values_per_tile + local_y * tile + local_x
                output[y, x] = packed[low_offset + sample] | (
                    packed[high_offset + sample] << 8
                )
        changed_index += 1
    assert changed_index == changed_count
    return output


def decode_temporal_color_delta(
    packed: bytes,
    keyframe: np.ndarray,
) -> tuple[np.ndarray, int]:
    magic, _keyframe_id, tile, rows, cols, changed_count = (
        TEMPORAL_COLOR_HEADER.unpack_from(packed)
    )
    assert magic == TEMPORAL_COLOR_MAGIC
    tile_count = rows * cols
    bit_length = (tile_count + 7) // 8
    bits = packed[
        TEMPORAL_COLOR_HEADER.size : TEMPORAL_COLOR_HEADER.size + bit_length
    ]
    output = keyframe.copy()
    if changed_count == 0:
        return output, changed_count
    decoder = av.CodecContext.create("mjpeg", "r")
    jpeg = packed[TEMPORAL_COLOR_HEADER.size + bit_length :]
    atlas = decoder.decode(av.Packet(jpeg))[0].to_ndarray(format="rgb24")
    atlas_cols = min(cols, changed_count)
    changed_index = 0
    for tile_index in range(tile_count):
        if not bits[tile_index // 8] & (1 << (tile_index % 8)):
            continue
        tile_y = tile_index // cols * tile
        tile_x = tile_index % cols * tile
        atlas_y = changed_index // atlas_cols * tile
        atlas_x = changed_index % atlas_cols * tile
        copy_height = min(tile, output.shape[0] - tile_y)
        copy_width = min(tile, output.shape[1] - tile_x)
        output[
            tile_y : tile_y + copy_height,
            tile_x : tile_x + copy_width,
        ] = atlas[
            atlas_y : atlas_y + copy_height,
            atlas_x : atlas_x + copy_width,
        ]
        changed_index += 1
    assert changed_index == changed_count
    return output, changed_count


def test_temporal_delta_is_independent_and_preserves_changed_tiles():
    keyframe = np.random.default_rng(7).integers(
        1000,
        2000,
        size=(24, 24),
        dtype=np.uint16,
    )
    encoder = TemporalRgbdDepthEncoder(
        keyframe_interval=30,
        tile_size=4,
        stability_mm=4,
        minimum_changed_pixels=4,
        single_pixel_change_mm=8,
        temporal_color=False,
    )

    first_packet, first_is_keyframe = encoder.encode(make_packet(keyframe))
    first_camera, _ = split_packet(first_packet)
    assert first_is_keyframe
    assert first_camera["depth_temporal_keyframe"] is True

    second = keyframe.copy()
    second[:4, :4] += 2  # Stable sensor-scale variation remains at keyframe depth.
    second[4:8, 4:8] += 20
    second[23, 23] = 0  # A lone one-frame hole is treated as depth shimmer.
    second_packet, second_is_keyframe = encoder.encode(make_packet(second))
    second_camera, second_depth = split_packet(second_packet)
    assert not second_is_keyframe
    assert second_camera["depth_encoding"] == TEMPORAL_DEPTH_ENCODING
    reconstructed_second = decode_temporal_delta(
        second_depth,
        second_camera,
        keyframe,
    )
    np.testing.assert_array_equal(reconstructed_second[4:8, 4:8], second[4:8, 4:8])
    assert reconstructed_second[23, 23] == keyframe[23, 23]
    np.testing.assert_array_equal(reconstructed_second[:4, :4], keyframe[:4, :4])

    third = keyframe.copy()
    third[8:12, :4] += 30
    third_packet, third_is_keyframe = encoder.encode(make_packet(third))
    third_camera, third_depth = split_packet(third_packet)
    assert not third_is_keyframe
    # Decode directly against the original keyframe, without applying frame 2.
    reconstructed_third = decode_temporal_delta(
        third_depth,
        third_camera,
        keyframe,
    )
    np.testing.assert_array_equal(reconstructed_third, third)


def test_temporal_encoder_emits_periodic_recovery_keyframes():
    depth = np.full((16, 16), 1200, dtype=np.uint16)
    encoder = TemporalRgbdDepthEncoder(
        keyframe_interval=2,
        tile_size=8,
        temporal_color=False,
    )
    _, first_is_keyframe = encoder.encode(make_packet(depth, predictive=False))
    _, second_is_keyframe = encoder.encode(make_packet(depth + 20, predictive=False))
    third_packet, third_is_keyframe = encoder.encode(
        make_packet(depth + 40, predictive=False)
    )
    third_camera, _ = split_packet(third_packet)
    assert first_is_keyframe
    assert third_is_keyframe
    assert third_camera["depth_temporal_keyframe"] is True
    # Frame 2 may choose an early keyframe when a tiny synthetic delta is not
    # smaller than its already tiny full-depth payload.
    assert isinstance(second_is_keyframe, bool)


def planar_depth(shape: tuple[int, int] = (128, 128)) -> np.ndarray:
    noise = np.random.default_rng(19).integers(
        -2,
        3,
        size=shape,
        dtype=np.int16,
    )
    return (1200 + noise).astype(np.uint16)


def test_plane_stabilization_suppresses_unstructured_surface_noise():
    keyframe = planar_depth()
    encoder = TemporalRgbdDepthEncoder(
        keyframe_interval=30,
        tile_size=8,
        temporal_color=False,
    )
    encoder.encode(make_packet(keyframe))

    checkerboard = (
        (np.indices(keyframe.shape).sum(axis=0) & 1) * 12 - 6
    ).astype(np.int16)
    noisy_plane = (
        keyframe.astype(np.int32) + checkerboard.astype(np.int32)
    ).astype(np.uint16)
    packet, is_keyframe = encoder.encode(make_packet(noisy_plane))
    camera, depth = split_packet(packet)

    assert not is_keyframe
    assert camera["depth_plane_stabilization"] is True
    assert camera["depth_plane_stabilized_tiles"] > 0
    assert camera["depth_changed_tiles"] == 0
    reconstructed = decode_temporal_delta(depth, camera, keyframe)
    np.testing.assert_array_equal(reconstructed, keyframe)


def test_plane_stabilization_preserves_coherent_foreground_motion():
    keyframe = planar_depth()
    encoder = TemporalRgbdDepthEncoder(
        keyframe_interval=30,
        tile_size=8,
        temporal_color=False,
    )
    encoder.encode(make_packet(keyframe))

    foreground = keyframe.copy()
    foreground[16:20, 24:28] += 20
    packet, is_keyframe = encoder.encode(make_packet(foreground))
    camera, depth = split_packet(packet)

    assert not is_keyframe
    assert camera["depth_changed_tiles"] >= 1
    reconstructed = decode_temporal_delta(depth, camera, keyframe)
    np.testing.assert_array_equal(
        reconstructed[16:20, 24:28],
        foreground[16:20, 24:28],
    )


def test_repeated_direction_confirms_small_nonplanar_change():
    keyframe = np.random.default_rng(23).integers(
        1000,
        2000,
        size=(32, 32),
        dtype=np.uint16,
    )
    encoder = TemporalRgbdDepthEncoder(
        keyframe_interval=30,
        tile_size=8,
        temporal_color=False,
    )
    encoder.encode(make_packet(keyframe))

    changed = keyframe.copy()
    changed[10, 10] += 10
    first_packet, _ = encoder.encode(make_packet(changed))
    first_camera, first_depth = split_packet(first_packet)
    assert first_camera["depth_changed_tiles"] == 0
    assert first_camera["depth_confirmation_stabilized_tiles"] == 1
    first_reconstructed = decode_temporal_delta(
        first_depth,
        first_camera,
        keyframe,
    )
    assert first_reconstructed[10, 10] == keyframe[10, 10]

    second_packet, _ = encoder.encode(make_packet(changed))
    second_camera, second_depth = split_packet(second_packet)
    assert second_camera["depth_changed_tiles"] == 1
    second_reconstructed = decode_temporal_delta(
        second_depth,
        second_camera,
        keyframe,
    )
    assert second_reconstructed[10, 10] == changed[10, 10]


def test_changing_noise_signature_does_not_gain_confirmation():
    keyframe = np.random.default_rng(29).integers(
        1000,
        2000,
        size=(32, 32),
        dtype=np.uint16,
    )
    encoder = TemporalRgbdDepthEncoder(
        keyframe_interval=30,
        tile_size=8,
        temporal_color=False,
    )
    encoder.encode(make_packet(keyframe))

    positive = keyframe.copy()
    positive[10, 10] += 10
    encoder.encode(make_packet(positive))

    negative = keyframe.copy()
    negative[10, 10] -= 10
    packet, _ = encoder.encode(make_packet(negative))
    camera, depth = split_packet(packet)
    assert camera["depth_changed_tiles"] == 0
    reconstructed = decode_temporal_delta(depth, camera, keyframe)
    assert reconstructed[10, 10] == keyframe[10, 10]


def test_temporal_color_ignores_sensor_scale_noise():
    keyframe = np.full((32, 32, 3), (80, 100, 120), dtype=np.uint8)
    encoder = TemporalRgbdDepthEncoder(
        color_tile_size=16,
        color_quality=68,
    )
    encoder.color_change_states["d455"] = (
        np.zeros(4, dtype=np.uint8),
        np.zeros(4, dtype=np.uint8),
    )
    noisy = np.clip(
        keyframe.astype(np.int16)
        + np.random.default_rng(31).integers(-3, 4, keyframe.shape),
        0,
        255,
    ).astype(np.uint8)
    packed, changed_count, stabilized_count = encoder._encode_color_delta(
        noisy,
        "d455",
        1,
        keyframe,
    )
    reconstructed, decoded_count = decode_temporal_color_delta(
        packed,
        keyframe,
    )
    assert changed_count == decoded_count == 0
    assert stabilized_count == 0
    np.testing.assert_array_equal(reconstructed, keyframe)


def test_temporal_color_confirms_ambiguous_change_once():
    keyframe = np.full((32, 32, 3), (80, 100, 120), dtype=np.uint8)
    encoder = TemporalRgbdDepthEncoder(
        color_tile_size=16,
        color_quality=68,
    )
    encoder.color_change_states["d455"] = (
        np.zeros(4, dtype=np.uint8),
        np.zeros(4, dtype=np.uint8),
    )
    changed = keyframe.copy()
    changed[8:16, 8:16] += 40

    first, first_count, first_stabilized = encoder._encode_color_delta(
        changed,
        "d455",
        1,
        keyframe,
    )
    assert first_count == 0
    assert first_stabilized == 1
    first_reconstructed, _ = decode_temporal_color_delta(first, keyframe)
    np.testing.assert_array_equal(first_reconstructed, keyframe)

    second, second_count, _ = encoder._encode_color_delta(
        changed,
        "d455",
        1,
        keyframe,
    )
    assert second_count == 1
    second_reconstructed, _ = decode_temporal_color_delta(second, keyframe)
    mean_error = np.mean(
        np.abs(
            second_reconstructed[8:16, 8:16].astype(np.int16)
            - changed[8:16, 8:16].astype(np.int16)
        )
    )
    assert mean_error < 8


def test_temporal_color_preserves_obvious_motion_immediately():
    keyframe = np.full((32, 32, 3), (60, 80, 100), dtype=np.uint8)
    encoder = TemporalRgbdDepthEncoder(
        color_tile_size=16,
        color_quality=68,
    )
    encoder.color_change_states["d455"] = (
        np.zeros(4, dtype=np.uint8),
        np.zeros(4, dtype=np.uint8),
    )
    changed = keyframe.copy()
    changed[8:16, 8:16] += 80
    packed, changed_count, _ = encoder._encode_color_delta(
        changed,
        "d455",
        1,
        keyframe,
    )
    assert changed_count == 1
    reconstructed, _ = decode_temporal_color_delta(packed, keyframe)
    assert np.mean(reconstructed[8:16, 8:16]) > np.mean(keyframe[8:16, 8:16]) + 60


def test_depth_motion_maps_to_corresponding_color_tile():
    depth_changed = np.zeros(16, dtype=bool)
    depth_changed[5] = True
    mapped = TemporalRgbdDepthEncoder._color_mask_from_depth_motion(
        depth_changed,
        (32, 32),
        8,
        (32, 32),
        16,
    )
    np.testing.assert_array_equal(
        mapped,
        np.array([True, False, False, False]),
    )


def test_persistent_color_packages_forced_medium_contrast_motion():
    keyframe = np.full((32, 32, 3), (105, 75, 50), dtype=np.uint8)
    encoder = TemporalRgbdDepthEncoder(
        color_tile_size=16,
        color_quality=68,
        persistent_reference=True,
    )
    encoder.color_change_states["d455"] = (
        np.zeros(4, dtype=np.uint8),
        np.zeros(4, dtype=np.uint8),
    )
    changed = keyframe.copy()
    changed[:16, :16] = (170, 130, 105)

    _, changed_count, _ = encoder._encode_color_delta(
        changed,
        "d455",
        1,
        keyframe,
    )
    assert changed_count == 0
    forced = np.array([True, False, False, False])
    packed = encoder._pack_color_delta(changed, 1, forced)
    reconstructed, decoded_count = decode_temporal_color_delta(
        packed,
        keyframe,
    )

    assert decoded_count == 1
    mean_error = np.mean(
        np.abs(
            reconstructed[:16, :16].astype(np.int16)
            - changed[:16, :16].astype(np.int16)
        )
    )
    assert mean_error < 8


def test_plane_stabilization_ignores_one_hole_but_preserves_clustered_holes():
    keyframe = planar_depth()
    encoder = TemporalRgbdDepthEncoder(
        keyframe_interval=30,
        tile_size=8,
        temporal_color=False,
    )
    encoder.encode(make_packet(keyframe))

    isolated = keyframe.copy()
    isolated[8, 8] = 0
    isolated_packet, _ = encoder.encode(make_packet(isolated))
    isolated_camera, isolated_depth = split_packet(isolated_packet)
    assert isolated_camera["depth_changed_tiles"] == 0
    isolated_reconstructed = decode_temporal_delta(
        isolated_depth,
        isolated_camera,
        keyframe,
    )
    assert isolated_reconstructed[8, 8] == keyframe[8, 8]

    clustered = keyframe.copy()
    clustered[8:10, 8:10] = 0
    clustered_packet, _ = encoder.encode(make_packet(clustered))
    clustered_camera, clustered_depth = split_packet(clustered_packet)
    assert clustered_camera["depth_changed_tiles"] >= 1
    clustered_reconstructed = decode_temporal_delta(
        clustered_depth,
        clustered_camera,
        keyframe,
    )
    np.testing.assert_array_equal(
        clustered_reconstructed[8:10, 8:10],
        clustered[8:10, 8:10],
    )


def test_persistent_reference_has_no_periodic_keyframes_or_stationary_tiles():
    depth = np.full((32, 32), 1200, dtype=np.uint16)
    encoder = TemporalRgbdDepthEncoder(
        keyframe_interval=2,
        tile_size=8,
        temporal_color=False,
        persistent_reference=True,
    )
    previous_metadata = None
    reference = depth
    keyframes = 0
    for frame in range(8):
        packet, is_keyframe = encoder.encode(
            make_packet(depth, predictive=False)
        )
        metadata, packed_depth = parse_persistent_packet(
            packet,
            previous_metadata,
        )
        camera = metadata["cameras"][0]
        keyframes += int(is_keyframe)
        if frame:
            assert camera["depth_changed_tiles"] == 0
            reference = decode_temporal_delta(
                packed_depth,
                camera,
                reference,
            )
            np.testing.assert_array_equal(reference, depth)
            assert metadata["metadata_delta"] == "persistent_v1"
        previous_metadata = metadata
    assert keyframes == 1


def test_persistent_reference_keeps_a_transmitted_motion_tile():
    depth = np.full((32, 32), 1200, dtype=np.uint16)
    encoder = TemporalRgbdDepthEncoder(
        tile_size=8,
        temporal_color=False,
        persistent_reference=True,
    )
    first, _ = encoder.encode(make_packet(depth, predictive=False))
    metadata, _ = parse_persistent_packet(first)

    moved = depth.copy()
    moved[8:16, 8:16] += 100
    second, _ = encoder.encode(make_packet(moved, predictive=False))
    metadata, packed = parse_persistent_packet(second, metadata)
    camera = metadata["cameras"][0]
    assert camera["depth_changed_tiles"] == 1
    reconstructed = decode_temporal_delta(packed, camera, depth)
    np.testing.assert_array_equal(reconstructed, moved)

    third, _ = encoder.encode(make_packet(moved, predictive=False))
    metadata, packed = parse_persistent_packet(third, metadata)
    camera = metadata["cameras"][0]
    assert camera["depth_changed_tiles"] == 0
    reconstructed = decode_temporal_delta(
        packed,
        camera,
        reconstructed,
    )
    np.testing.assert_array_equal(reconstructed, moved)


def test_persistent_reference_confirms_slow_motion_before_advancing():
    depth = np.full((32, 32), 1200, dtype=np.uint16)
    encoder = TemporalRgbdDepthEncoder(
        tile_size=8,
        temporal_color=False,
        persistent_reference=True,
    )
    first, _ = encoder.encode(make_packet(depth, predictive=False))
    metadata, _ = parse_persistent_packet(first)
    changed = depth.copy()
    changed[8:10, 8:10] += 20
    reference = depth
    counts = []
    for _ in range(4):
        packet, _ = encoder.encode(
            make_packet(changed, predictive=False)
        )
        metadata, packed = parse_persistent_packet(packet, metadata)
        camera = metadata["cameras"][0]
        counts.append(camera["depth_changed_tiles"])
        reference = decode_temporal_delta(
            packed,
            camera,
            reference,
        )
    assert counts == [0, 0, 0, 1]
    np.testing.assert_array_equal(reference, changed)


def test_persistent_reference_clears_thin_stale_depth_silhouette():
    depth = np.random.default_rng(41).integers(
        1000,
        2000,
        size=(32, 32),
        dtype=np.uint16,
    )
    encoder = TemporalRgbdDepthEncoder(
        tile_size=8,
        temporal_color=False,
        persistent_reference=True,
    )
    first, _ = encoder.encode(make_packet(depth, predictive=False))
    metadata, _ = parse_persistent_packet(first)

    disoccluded = depth.copy()
    # Fewer than the persistent codec's coarse 16-pixel validity threshold:
    # this used to remain at 1200 mm forever as a floating ghost.
    disoccluded[9, 9:12] = 0
    reference = depth.copy()
    counts = []
    for _ in range(4):
        packet, _ = encoder.encode(
            make_packet(disoccluded, predictive=False)
        )
        metadata, packed = parse_persistent_packet(packet, metadata)
        camera = metadata["cameras"][0]
        counts.append(camera["depth_changed_tiles"])
        reference = decode_temporal_delta(packed, camera, reference)

    assert counts == [0, 0, 0, 1]
    np.testing.assert_array_equal(reference, disoccluded)


def test_persistent_reference_rejects_one_frame_depth_hole():
    depth = np.random.default_rng(43).integers(
        1000,
        2000,
        size=(32, 32),
        dtype=np.uint16,
    )
    encoder = TemporalRgbdDepthEncoder(
        tile_size=8,
        temporal_color=False,
        persistent_reference=True,
    )
    first, _ = encoder.encode(make_packet(depth, predictive=False))
    metadata, _ = parse_persistent_packet(first)

    transient = depth.copy()
    transient[9, 9:12] = 0
    packet, _ = encoder.encode(make_packet(transient, predictive=False))
    metadata, packed = parse_persistent_packet(packet, metadata)
    camera = metadata["cameras"][0]
    assert camera["depth_changed_tiles"] == 0
    reference = decode_temporal_delta(packed, camera, depth)

    packet, _ = encoder.encode(make_packet(depth, predictive=False))
    metadata, packed = parse_persistent_packet(packet, metadata)
    camera = metadata["cameras"][0]
    assert camera["depth_changed_tiles"] == 0
    reference = decode_temporal_delta(packed, camera, reference)
    np.testing.assert_array_equal(reference, depth)
