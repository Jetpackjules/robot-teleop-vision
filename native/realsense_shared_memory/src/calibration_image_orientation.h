#pragma once

#include <array>

// Undo an image-only CCW rotation before sampling depth or deprojecting with
// the original color intrinsics. Camera axes and saved extrinsics stay fixed.
inline std::array<double, 2> calibration_original_pixel(
    double x, double y, int width, int height, int rotation_ccw_deg
) {
    switch (rotation_ccw_deg) {
        case 90: return {double(width - 1) - y, x};
        case 180: return {double(width - 1) - x, double(height - 1) - y};
        case 270: return {y, double(height - 1) - x};
        default: return {x, y};
    }
}
