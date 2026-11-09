#include "rgb_frame_freshness.h"
#include <cassert>

int main() {
    realsense_capture::RgbFrameFreshness rgb;
    assert(rgb.age_ms(100.0) == -1.0);
    rgb.observe(10, 1234.0, 100.0);
    // Depth publication may continue; repeating one RGB exposure stays stale.
    for (int i = 1; i <= 6; ++i) rgb.observe(10, 1234.0, 100.0 + i);
    assert(rgb.advances == 1 && rgb.age_ms(106.0) == 6000.0);
    // An unchanged scene is healthy when the sensor delivers new exposures.
    rgb.observe(11, 1267.0, 106.0);
    assert(rgb.age_ms(106.0) == 0.0 && rgb.advances == 2);
    // Timestamp changes and counter resets both count as new exposures.
    rgb.observe(11, 1300.0, 107.0);
    rgb.observe(1, 33.0, 108.0);
    assert(rgb.advances == 4 && rgb.age_ms(109.0) == 1000.0);
    rgb.reset();
    assert(!rgb.seen && rgb.advances == 0 && rgb.age_ms(109.0) == -1.0);
}
