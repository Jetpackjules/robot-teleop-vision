#pragma once

#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/core/binder_common.hpp>
#include <godot_cpp/templates/vector.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/string.hpp>

#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif

class RealSenseSharedMemoryReader : public godot::RefCounted {
    GDCLASS(RealSenseSharedMemoryReader, godot::RefCounted)

public:
    RealSenseSharedMemoryReader();
    ~RealSenseSharedMemoryReader();

    bool open(const godot::String &p_name = "realsense_point_cloud_grid");
    void close();
    bool is_open() const;
    bool poll();

    uint64_t get_sequence() const;
    uint64_t get_frame_id() const;
    int get_width() const;
    int get_height() const;
    int get_stride() const;
    godot::Vector4 get_intrinsics() const;
    godot::Ref<godot::Image> get_depth_image() const;
    godot::Ref<godot::Image> get_color_image() const;
    godot::Dictionary get_frame() const;

protected:
    static void _bind_methods();

private:
    static constexpr uint32_t HEADER_SIZE = 128;
    static constexpr char MAGIC[8] = {'R', 'S', 'P', 'G', '0', '1', '\0', '\0'};

#ifdef _WIN32
    HANDLE mapping_handle = nullptr;
#endif
    const uint8_t *view = nullptr;
    size_t view_size = 0;

    uint64_t sequence = 0;
    uint64_t frame_id = 0;
    int width = 0;
    int height = 0;
    int stride = 1;
    int color_format = 1;
    float fx = 0.0f;
    float fy = 0.0f;
    float ppx = 0.0f;
    float ppy = 0.0f;
    godot::PackedByteArray depth_bytes;
    godot::PackedByteArray color_bytes;
    godot::Ref<godot::Image> depth_image;
    godot::Ref<godot::Image> color_image;

    template <typename T>
    static T read_le(const uint8_t *p_data, size_t p_offset) {
        T value;
        memcpy(&value, p_data + p_offset, sizeof(T));
        return value;
    }
};
