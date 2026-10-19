// Separable form of BlendPCR's original square box filter. SPDX-License-Identifier: EUPL-1.2
shader_type canvas_item;
render_mode blend_disabled;
uniform sampler2D horizontal_counts : filter_nearest, repeat_disable;
uniform int seam_radius = 3;
void fragment() {
    vec2 texel = 1.0 / vec2(textureSize(horizontal_counts, 0));
    vec4 counts = vec4(0.0);
    for (int y = -seam_radius; y <= seam_radius; y++) {
        counts += round(texture(horizontal_counts, UV + vec2(0.0,float(y)) * texel).rgba * 255.0);
    }
    float width = float(2 * seam_radius + 1);
    COLOR = counts / (width * width);
}
