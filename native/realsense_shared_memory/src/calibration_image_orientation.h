#pragma once

#include <array>
#include <algorithm>
#include <cmath>

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

// Fit without stretching: a rotated portrait camera must retain the same
// proportions as the landscape reference seen by the feature detector.
struct CalibrationImageFit {
    int width;
    int height;
    int left;
    int top;
    double source_per_x;
    double source_per_y;

    bool original_pixel(double x, double y, std::array<double, 2> &pixel) const {
        if (x < left || y < top || x >= left + width || y >= top + height) return false;
        // OpenCV resize maps pixel centres, rather than the image corners.
        pixel = {(x - left + 0.5) * source_per_x - 0.5,
                 (y - top + 0.5) * source_per_y - 0.5};
        return true;
    }
};

inline CalibrationImageFit calibration_image_fit(int source_width, int source_height, int input_width, int input_height) {
    const double scale = std::min(double(input_width) / source_width, double(input_height) / source_height);
    const int width = std::clamp(int(std::lround(source_width * scale)), 1, input_width);
    const int height = std::clamp(int(std::lround(source_height * scale)), 1, input_height);
    return {width, height, (input_width - width) / 2, (input_height - height) / 2,
            double(source_width) / width, double(source_height) / height};
}
