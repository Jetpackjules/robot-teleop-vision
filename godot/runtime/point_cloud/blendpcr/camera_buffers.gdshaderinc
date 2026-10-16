// Godot adaptation of BlendPCR screen-space passes.
// Original: Andre Mühlenbrock, CGVR, 2024. SPDX-License-Identifier: EUPL-1.2
uniform int camera_count = 0;
uniform int active_mask = 0;
uniform float depth_span = 100.0;
uniform float surface_tolerance = 0.035;
uniform sampler2D camera_color_0 : source_color, filter_nearest, repeat_disable;
uniform sampler2D camera_meta_0 : filter_nearest, repeat_disable;
uniform sampler2D camera_color_1 : source_color, filter_nearest, repeat_disable;
uniform sampler2D camera_meta_1 : filter_nearest, repeat_disable;
uniform sampler2D camera_color_2 : source_color, filter_nearest, repeat_disable;
uniform sampler2D camera_meta_2 : filter_nearest, repeat_disable;
uniform sampler2D camera_color_3 : source_color, filter_nearest, repeat_disable;
uniform sampler2D camera_meta_3 : filter_nearest, repeat_disable;
vec4 sample_meta(int i, vec2 uv) {
    if (i == 0) return texture(camera_meta_0, uv);
    if (i == 1) return texture(camera_meta_1, uv);
    if (i == 2) return texture(camera_meta_2, uv);
    if (i == 3) return texture(camera_meta_3, uv);
    return vec4(0.0);
}
vec4 sample_color(int i, vec2 uv) {
    if (i == 0) return texture(camera_color_0, uv);
    if (i == 1) return texture(camera_color_1, uv);
    if (i == 2) return texture(camera_color_2, uv);
    if (i == 3) return texture(camera_color_3, uv);
    return vec4(0.0);
}
bool camera_active(int i) { return (active_mask & (1 << i)) != 0; }
float eye_depth(vec4 meta) { return (meta.r * 255.0 * 256.0 + meta.g * 255.0) / 65535.0 * depth_span; }
bool valid_meta(vec4 meta) { return meta.a > 0.5 && eye_depth(meta) > 0.0; }
