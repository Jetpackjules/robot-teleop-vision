#pragma once

#include <algorithm>
#include <cstdint>
#include <tuple>
#include <vector>

namespace realsense_capture {

struct Mode {
    int width;
    int height;
    int fps;
    bool operator==(const Mode &other) const {
        return std::tie(width, height, fps) == std::tie(other.width, other.height, other.fps);
    }
};

struct Pair { Mode depth; Mode color; };

// Preserve the requested common cadence first, then maximize both native grids.
// Fall back only to advertised modes at or below the target cadence.
inline std::vector<Pair> ranked_modes(const std::vector<Mode> &depth,
        const std::vector<Mode> &color, int target_fps, bool with_color) {
    std::vector<Pair> result;
    for (const auto &d : depth) {
        if (d.width < 2 || d.height < 2 || d.fps < 1 || d.fps > target_fps) continue;
        if (!with_color) result.push_back({d, {0, 0, d.fps}});
        else for (const auto &c : color) {
            if (c.fps == d.fps && c.width > 1 && c.height > 1) result.push_back({d, c});
        }
    }
    const auto rank = [&](const Pair &p) {
        return std::make_tuple(p.depth.fps, int64_t(p.depth.width) * p.depth.height,
                int64_t(p.color.width) * p.color.height, p.depth.width, p.depth.height,
                p.color.width, p.color.height);
    };
    std::sort(result.begin(), result.end(), [&](const Pair &a, const Pair &b) { return rank(a) > rank(b); });
    result.erase(std::unique(result.begin(), result.end(), [](const Pair &a, const Pair &b) {
        return a.depth == b.depth && a.color == b.color;
    }), result.end());
    return result;
}

} // namespace realsense_capture
