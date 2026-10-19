// Godot adaptation of BlendPCR screen-space passes.
// Original: Andre Mühlenbrock, CGVR, 2024. SPDX-License-Identifier: EUPL-1.2
shader_type canvas_item;
render_mode blend_disabled;
uniform sampler2D dominance : filter_nearest, repeat_disable;
uniform int first_camera = 0;
uniform int seam_radius = 3;
void fragment() {
    vec2 texel = 1.0 / vec2(textureSize(dominance, 0));
    vec4 weights = vec4(0.0);
    for (int x = -seam_radius; x <= seam_radius; x++) {
        int owner = int(round(texture(dominance, UV + vec2(float(x),0.0) * texel).r * 255.0)) - 1;
        if (owner >= first_camera && owner < first_camera + 4) weights[owner - first_camera] += 1.0;
    }
    // Store integer counts exactly in RGBA8. The vertical pass reconstructs
    // them before dividing, avoiding an intermediate rounding/quality loss.
    COLOR = weights / 255.0;
}
