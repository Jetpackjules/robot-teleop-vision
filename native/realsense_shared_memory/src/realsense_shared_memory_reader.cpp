#include "realsense_shared_memory_reader.h"

#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <cstring>
#include <atomic>
#include <cmath>
#ifndef _WIN32
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

using namespace godot;

namespace {
bool valid_mapping_name(const String &name) {
    if (name.is_empty() || name.length() > 180) return false;
    for (int i = 0; i < name.length(); ++i) {
        const char32_t c = name[i];
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '_' || c == '-')) return false;
    }
    return true;
}
uint64_t mapping_sequence(const uint8_t *view) {
    return reinterpret_cast<const std::atomic<uint64_t> *>(view + 8)->load(std::memory_order_acquire);
}
}

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
    ClassDB::bind_method(D_METHOD("get_capture_fps"), &RealSenseSharedMemoryReader::get_capture_fps);
    ClassDB::bind_method(D_METHOD("get_capture_unix_ms"), &RealSenseSharedMemoryReader::get_capture_unix_ms);
}

bool RealSenseSharedMemoryReader::open(const String &p_name) {
    close();
    if (!valid_mapping_name(p_name)) return false;
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
    const CharString name_utf8 = (String("/") + p_name).utf8();
    int fd = shm_open(name_utf8.get_data(), O_RDONLY | O_CLOEXEC, 0);
    if (fd < 0) return false;
    struct stat info;
    if (fstat(fd, &info) != 0 || info.st_size < HEADER_SIZE || info.st_size > 64 * 1024 * 1024) {
        ::close(fd);
        return false;
    }
    view_size = static_cast<size_t>(info.st_size);
    void *mapped = mmap(nullptr, view_size, PROT_READ, MAP_SHARED, fd, 0);
    ::close(fd);
    if (mapped == MAP_FAILED) { view_size = 0; return false; }
    view = static_cast<const uint8_t *>(mapped);
    return true;
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
#else
    if (view != nullptr) munmap(const_cast<uint8_t *>(view), view_size);
#endif
    view = nullptr;
    view_size = 0;
    sequence = 0;
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

    uint64_t seq_before = mapping_sequence(view);
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

    if (next_width == 0 || next_height == 0 || next_width > 8192 || next_height > 8192 ||
            next_stride == 0 || next_stride > 64 || !std::isfinite(next_fx) || !std::isfinite(next_fy) ||
            next_fx <= 0 || next_fy <= 0 || !std::isfinite(next_ppx) || !std::isfinite(next_ppy)) {
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

    const double next_capture_fps = read_le<double>(view, 64);
    const double next_capture_unix_ms = read_le<double>(view, 72);
    std::atomic_thread_fence(std::memory_order_acquire);
    uint64_t seq_after = mapping_sequence(view);
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
    capture_fps = std::isfinite(next_capture_fps) && next_capture_fps >= 0 && next_capture_fps <= 1000 ? next_capture_fps : 0;
    capture_unix_ms = std::isfinite(next_capture_unix_ms) && next_capture_unix_ms >= 0 ? next_capture_unix_ms : 0;
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

double RealSenseSharedMemoryReader::get_capture_fps() const { return capture_fps; }
double RealSenseSharedMemoryReader::get_capture_unix_ms() const { return capture_unix_ms; }

RealSenseSharedMemoryWriter::~RealSenseSharedMemoryWriter() { close(); }

bool RealSenseSharedMemoryWriter::open(const String &p_name) {
    close();
    if (!valid_mapping_name(p_name)) return false;
#ifdef _WIN32
    const CharString utf8 = p_name.utf8();
    mapping_handle = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, CAPACITY, utf8.get_data());
    if (!mapping_handle || GetLastError() == ERROR_ALREADY_EXISTS) { close(); return false; }
    view = static_cast<uint8_t *>(MapViewOfFile(mapping_handle, FILE_MAP_WRITE, 0, 0, CAPACITY));
#else
    const CharString utf8 = (String("/") + p_name).utf8();
    const int fd = shm_open(utf8.get_data(), O_CREAT | O_EXCL | O_RDWR | O_CLOEXEC, 0600);
    if (fd < 0) return false;
    if (ftruncate(fd, CAPACITY) != 0) { ::close(fd); shm_unlink(utf8.get_data()); return false; }
    void *mapped = mmap(nullptr, CAPACITY, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    ::close(fd);
    if (mapped == MAP_FAILED) { shm_unlink(utf8.get_data()); return false; }
    view = static_cast<uint8_t *>(mapped);
#endif
    if (!view) { close(); return false; }
    name = p_name;
    memset(view, 0, 128);
    return true;
}

void RealSenseSharedMemoryWriter::close() {
#ifdef _WIN32
    if (view) UnmapViewOfFile(view);
    if (mapping_handle) CloseHandle(mapping_handle);
    mapping_handle = nullptr;
#else
    if (view) munmap(view, CAPACITY);
    if (!name.is_empty()) shm_unlink((String("/") + name).utf8().get_data());
#endif
    view = nullptr;
    name = String();
    sequence = 0;
}

bool RealSenseSharedMemoryWriter::publish(uint64_t p_frame_id, int p_stride, const Vector4 &p_intrinsics,
        const Ref<Image> &p_depth, const Ref<Image> &p_color, double p_capture_fps, double p_capture_unix_ms) {
    if (!view || p_depth.is_null() || p_color.is_null() || p_depth->get_format() != Image::FORMAT_RF ||
            p_color->get_format() != Image::FORMAT_RGBA8 || p_depth->get_size() != p_color->get_size()) return false;
    const PackedByteArray depth = p_depth->get_data(), color = p_color->get_data();
    if (128 + size_t(depth.size()) + size_t(color.size()) > CAPACITY) return false;
    auto *counter = reinterpret_cast<std::atomic<uint64_t> *>(view + 8);
    counter->store(++sequence, std::memory_order_seq_cst); // odd: a reader must retry
    const uint8_t magic[8] = {'R','S','P','G','0','1',0,0};
    memcpy(view, magic, 8);
    memcpy(view + 16, &p_frame_id, 8);
    uint32_t dims[4] = {uint32_t(p_depth->get_width()), uint32_t(p_depth->get_height()), uint32_t(p_stride), 1};
    memcpy(view + 24, dims, 16);
    float intrinsics[4] = {float(p_intrinsics.x), float(p_intrinsics.y), float(p_intrinsics.z), float(p_intrinsics.w)};
    memcpy(view + 40, intrinsics, 16);
    uint32_t sizes[2] = {uint32_t(depth.size()), uint32_t(color.size())};
    memcpy(view + 56, sizes, 8);
    memcpy(view + 64, &p_capture_fps, 8);
    memcpy(view + 72, &p_capture_unix_ms, 8);
    memcpy(view + 128, depth.ptr(), depth.size());
    memcpy(view + 128 + depth.size(), color.ptr(), color.size());
    counter->store(++sequence, std::memory_order_release);
    return true;
}
