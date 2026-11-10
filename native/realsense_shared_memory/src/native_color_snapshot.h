#pragma once

#include <cstdint>
#include <memory>
#include <vector>

// Owned sensor bytes, never an SDK/UVC frame. Capture replaces shared storage
// before writing again, so queued encoders can read this acquisition unchanged.
struct NativeColorSnapshot {
    std::shared_ptr<const std::vector<uint8_t>> pixels;
    int width = 0;
    int height = 0;
    int bytes_per_pixel = 0;
    int pitch = 0;

    bool valid() const {
        return pixels && width > 0 && height > 0 &&
            (bytes_per_pixel == 3 || bytes_per_pixel == 4) &&
            int64_t(pitch) >= int64_t(width) * bytes_per_pixel &&
            uint64_t(pitch) * uint64_t(height) <= pixels->size();
    }
};

struct NativeInfraredPair {
    static constexpr int WIDTH = 848;
    static constexpr int HEIGHT = 480;
    static constexpr int FPS = 60;
    static constexpr const char *CAMERA_SERIAL = "262522075787";

    std::vector<uint8_t> left;
    std::vector<uint8_t> right;
    float disparity_scale_mm = 0.0f;
};
