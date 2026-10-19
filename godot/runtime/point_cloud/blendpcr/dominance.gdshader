// Godot adaptation of BlendPCR screen-space passes.
// Original: Andre Mühlenbrock, CGVR, 2024. SPDX-License-Identifier: EUPL-1.2
shader_type canvas_item;
render_mode blend_disabled;
#include "camera_buffers.gdshaderinc"
void fragment() {
    float nearest = depth_span;
    float depths[4];
    float qualities[4];
    for (int i = 0; i < camera_count; i++) {
        vec4 m = sample_meta(i, UV);
        float q=m.b,z=packed_value(m,i);
        bool valid=camera_active(i) && valid_meta(m,i);
        if(valid && native_camera(i)) {
            vec2 rgb_uv;float edge;
            valid=native_sample(i,UV,z,rgb_uv,q,z,edge);
        }
        depths[i]=z;
        qualities[i]=valid?q:-1.0;
        if(valid)nearest=min(nearest,z);
    }
    float best = -1.0;
    int owner = -1;
    for (int i = 0; i < camera_count; i++) {
        float q=qualities[i];
        if (q<0.0 || abs(depths[i]-nearest)>agreement_tolerance(nearest))continue;
        if (q > best) {
            best = q;
            owner = i;
        }
    }
    COLOR = vec4(float(owner + 1) / 255.0, 0.0, 0.0, 1.0);
}
