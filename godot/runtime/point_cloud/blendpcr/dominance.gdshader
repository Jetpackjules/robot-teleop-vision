// Godot adaptation of BlendPCR screen-space passes.
// Original: Andre Mühlenbrock, CGVR, 2024. SPDX-License-Identifier: EUPL-1.2
shader_type canvas_item;
render_mode blend_disabled;
#include "camera_buffers.gdshaderinc"
void fragment() {
    float nearest = depth_span;
    for (int i = 0; i < camera_count; i++) {
        vec4 m = sample_meta(i, UV);
        if (camera_active(i) && valid_meta(m)) nearest = min(nearest, eye_depth(m));
    }
    float best = -1.0;
    int owner = -1;
    for (int i = 0; i < camera_count; i++) {
        vec4 m = sample_meta(i, UV);
        if (camera_active(i) && valid_meta(m) && abs(eye_depth(m) - nearest) <= surface_tolerance && m.b > best) {
            best = m.b;
            owner = i;
        }
    }
    COLOR = vec4(float(owner + 1) / 255.0, 0.0, 0.0, 1.0);
}
