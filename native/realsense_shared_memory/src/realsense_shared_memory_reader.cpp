#include "realsense_shared_memory_reader.h"

#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <cstring>

using namespace godot;

RealSenseSharedMemoryReader::RealSenseSharedMemoryReader() {}

RealSenseSharedMemoryReader::~RealSenseSharedMemoryReader() {
    close();
}

void RealSenseSharedMemoryReader::_bind_methods() {
    ClassDB::bind_method(D_METHOD("open", "name"), &RealSenseSharedMemoryReader::open, DEFVAL("realsense_point_cloud_grid"));
    ClassDB::bind_method(D_METHOD("close"), &RealSenseSharedMemoryReader::close);
    ClassDB::bind_method(D_METHOD("is_open"), &RealSenseSharedMemoryReader::is_open);
    ClassDB::bind_method(D_METHOD("poll"), &RealSenseSharedMemoryReader::poll);
    ClassDB::bind_method(D_METHOD("get_sequence"), &RealSenseSharedMemoryReader::get_sequence);
    ClassDB::bind_method(D_METHOD("get_frame_id"), &RealSenseSharedMemoryReader::get_frame_id);
    ClassDB::bind_method(D_METHOD("get_width"), &RealSenseSharedMemoryReader::get_width);
    ClassDB::bind_method(D_METHOD("get_height"), &RealSenseSharedMemoryReader::get_height);
    ClassDB::bind_method(D_METHOD("get_stride"), &RealSenseSharedMemoryReader::get_stride);
    ClassDB::bind_method(D_METHOD("get_intrinsics"), &RealSenseSharedMemoryReader::get_intrinsics);
    ClassDB::bind_method(D_METHOD("get_depth_image"), &RealSenseSharedMemoryReader::get_depth_image);
    ClassDB::bind_method(D_METHOD("get_color_image"), &RealSenseSharedMemoryReader::get_color_image);
    ClassDB::bind_method(D_METHOD("get_frame"), &RealSenseSharedMemoryReader::get_frame);
}

bool RealSenseSharedMemoryReader::open(const String &p_name) {
    close();
#ifdef _WIN32
    CharString name_utf8 = p_name.utf8();
    mapping_handle = OpenFileMappingA(FILE_MAP_READ, FALSE, name_utf8.get_data());
    if (mapping_handle == nullptr) {
        UtilityFunctions::push_warning(String("Could not open RealSense shared memory: ") + p_name);
        return false;
    }
    view = static_cast<const uint8_t *>(MapViewOfFile(mapping_handle, FILE_MAP_READ, 0, 0, 0));
    if (view == nullptr) {
        CloseHandle(mapping_handle);
        mapping_handle = nullptr;
        UtilityFunctions::push_warning(String("Could not map RealSense shared memory: ") + p_name);
        return false;
    }
    MEMORY_BASIC_INFORMATION info;
    if (VirtualQuery(view, &info, sizeof(info)) != 0) {
        view_size = static_cast<size_t>(info.RegionSize);
    }
    return true;
#else
    return false;
#endif
}

void RealSenseSharedMemoryReader::close() {
#ifdef _WIN32
    if (view != nullptr) {
        UnmapViewOfFile(view);
        view = nullptr;
    }
    if (mapping_handle != nullptr) {
        CloseHandle(mapping_handle);
        mapping_handle = nullptr;
    }
#endif
    view_size = 0;
}

bool RealSenseSharedMemoryReader::is_open() const {
    return view != nullptr;
}

bool RealSenseSharedMemoryReader::poll() {
    if (view == nullptr || view_size < HEADER_SIZE) {
        return false;
    }
    if (memcmp(view, MAGIC, sizeof(MAGIC)) != 0) {
        return false;
    }

    uint64_t seq_before = read_le<uint64_t>(view, 8);
    if ((seq_before & 1ULL) != 0ULL || seq_before == sequence) {
        return false;
    }

    uint64_t next_frame_id = read_le<uint64_t>(view, 16);
    uint32_t next_width = read_le<uint32_t>(view, 24);
    uint32_t next_height = read_le<uint32_t>(view, 28);
    uint32_t next_stride = read_le<uint32_t>(view, 32);
    uint32_t next_color_format = read_le<uint32_t>(view, 36);
    float next_fx = read_le<float>(view, 40);
    float next_fy = read_le<float>(view, 44);
    float next_ppx = read_le<float>(view, 48);
    float next_ppy = read_le<float>(view, 52);
    uint32_t depth_size = read_le<uint32_t>(view, 56);
    uint32_t color_size = read_le<uint32_t>(view, 60);

    if (next_width == 0 || next_height == 0) {
        return false;
    }
    size_t expected_depth = static_cast<size_t>(next_width) * static_cast<size_t>(next_height) * 4;
    int source_color_channels = 4;
    if (next_color_format == 2 || next_color_format == 3) {
        source_color_channels = 3;
    } else {
        next_color_format = 1;
    }
    size_t expected_color = static_cast<size_t>(next_width) * static_cast<size_t>(next_height) * static_cast<size_t>(source_color_channels);
    if (depth_size != expected_depth || color_size != expected_color) {
        return false;
    }
    size_t total = HEADER_SIZE + expected_depth + expected_color;
    if (total > view_size) {
        return false;
    }

    PackedByteArray next_depth;
    PackedByteArray source_color;
    next_depth.resize(static_cast<int64_t>(expected_depth));
    source_color.resize(static_cast<int64_t>(expected_color));
    memcpy(next_depth.ptrw(), view + HEADER_SIZE, expected_depth);
    memcpy(source_color.ptrw(), view + HEADER_SIZE + expected_depth, expected_color);

    uint64_t seq_after = read_le<uint64_t>(view, 8);
    if (seq_before != seq_after || (seq_after & 1ULL) != 0ULL) {
        return false;
    }

    sequence = seq_after;
    frame_id = next_frame_id;
    width = static_cast<int>(next_width);
    height = static_cast<int>(next_height);
    stride = static_cast<int>(next_stride);
    color_format = static_cast<int>(next_color_format);
    fx = next_fx;
    fy = next_fy;
    ppx = next_ppx;
    ppy = next_ppy;
    depth_bytes = next_depth;
    if (source_color_channels == 4) {
        color_bytes = source_color;
    } else {
        const int64_t pixel_count = static_cast<int64_t>(next_width) * static_cast<int64_t>(next_height);
        PackedByteArray next_color;
        next_color.resize(pixel_count * 4);
        const uint8_t *src = source_color.ptr();
        uint8_t *dst = next_color.ptrw();
        for (int64_t i = 0; i < pixel_count; ++i) {
            const uint8_t c0 = src[i * 3 + 0];
            const uint8_t c1 = src[i * 3 + 1];
            const uint8_t c2 = src[i * 3 + 2];
            if (next_color_format == 3) {
                dst[i * 4 + 0] = c2;
                dst[i * 4 + 1] = c1;
                dst[i * 4 + 2] = c0;
            } else {
                dst[i * 4 + 0] = c0;
                dst[i * 4 + 1] = c1;
                dst[i * 4 + 2] = c2;
            }
            dst[i * 4 + 3] = 255;
        }
        color_bytes = next_color;
    }
    depth_image = Image::create_from_data(width, height, false, Image::FORMAT_RF, depth_bytes);
    color_image = Image::create_from_data(width, height, false, Image::FORMAT_RGBA8, color_bytes);
    return true;
}

uint64_t RealSenseSharedMemoryReader::get_sequence() const { return sequence; }
uint64_t RealSenseSharedMemoryReader::get_frame_id() const { return frame_id; }
int RealSenseSharedMemoryReader::get_width() const { return width; }
int RealSenseSharedMemoryReader::get_height() const { return height; }
int RealSenseSharedMemoryReader::get_stride() const { return stride; }
Vector4 RealSenseSharedMemoryReader::get_intrinsics() const { return Vector4(fx, fy, ppx, ppy); }
Ref<Image> RealSenseSharedMemoryReader::get_depth_image() const { return depth_image; }
Ref<Image> RealSenseSharedMemoryReader::get_color_image() const { return color_image; }

Dictionary RealSenseSharedMemoryReader::get_frame() const {
    Dictionary frame;
    frame["sequence"] = sequence;
    frame["frame_id"] = frame_id;
    frame["width"] = width;
    frame["height"] = height;
    frame["stride"] = stride;
    frame["intrinsics"] = get_intrinsics();
    frame["depth_image"] = depth_image;
    frame["color_image"] = color_image;
    return frame;
}
