#pragma once
#include <algorithm>
#include <cstdint>

namespace realsense_capture {
// SDK framesets may advance depth while retaining the same color exposure.
// Track exposure identity, rather than the number of frameset publications.
struct RgbFrameFreshness {
    bool seen = false;
    uint64_t number = 0;
    double sensor_timestamp_ms = 0.0;
    double progressed_at = 0.0;
    uint64_t advances = 0;

    void reset() { *this = RgbFrameFreshness(); }
    void observe(uint64_t next_number, double next_timestamp_ms, double now) {
        if (!seen || next_number != number || next_timestamp_ms != sensor_timestamp_ms) {
            seen = true;
            number = next_number;
            sensor_timestamp_ms = next_timestamp_ms;
            progressed_at = now;
            ++advances;
        }
    }
    double age_ms(double now) const {
        return seen ? std::max(0.0, (now - progressed_at) * 1000.0) : -1.0;
    }
};
}
