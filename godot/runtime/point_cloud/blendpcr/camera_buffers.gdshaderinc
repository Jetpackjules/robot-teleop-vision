// Godot adaptation of BlendPCR screen-space passes.
// Original: Andre Mühlenbrock, CGVR, 2024. SPDX-License-Identifier: EUPL-1.2
uniform int camera_count = 0;
uniform int active_mask = 0;
uniform float depth_span = 100.0;
uniform float surface_tolerance = 0.035;
uniform int native_mask = 0;
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
bool native_camera(int i) { return (native_mask & (1 << i)) != 0; }
float packed_value(vec4 meta,int i) {
    vec3 bytes=round(meta.rgb*255.0);
    return native_camera(i)?dot(bytes,vec3(65536,256,1)):
        dot(bytes.rg,vec2(256,1))/65535.0*depth_span;
}
bool valid_meta(vec4 meta,int i) { return meta.a > 0.5 && packed_value(meta,i) > 0.0; }
float agreement_tolerance(float depth) {
    // Sensor noise grows with range. The user's tolerance is an upper bound,
    // not permission to average a near cube edge into the floor behind it.
    return min(surface_tolerance, max(0.004, 0.005 * depth * depth));
}
#include "native_samples.gdshaderinc"
