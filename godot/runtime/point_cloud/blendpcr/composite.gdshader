// Godot adaptation of BlendPCR screen-space passes.
// Original: Andre Mühlenbrock, CGVR, 2024. SPDX-License-Identifier: EUPL-1.2
shader_type spatial;
render_mode unshaded, cull_disabled, fog_disabled;
#include "camera_buffers.gdshaderinc"
uniform sampler2D weights_a : filter_linear, repeat_disable;
void vertex() { POSITION = vec4(VERTEX.xy, 1.0, 1.0); }
void fragment() {
    vec2 uv = SCREEN_UV;
    float nearest = depth_span;
    for (int i = 0; i < camera_count; i++) {
        vec4 m = sample_meta(i, uv);
        if (camera_active(i) && valid_meta(m)) nearest = min(nearest, eye_depth(m));
    }
    vec4 wa = texture(weights_a, uv);
    float total = 0.0;
    float sum_depth = 0.0;
    vec3 rgb = vec3(0.0);
    vec4 fallback = vec4(0.0);
    float fallback_quality = -1.0;
    float fallback_depth = nearest;
    for (int i = 0; i < camera_count; i++) {
        vec4 m = sample_meta(i, uv);
        if (!camera_active(i) || !valid_meta(m) || abs(eye_depth(m) - nearest) > surface_tolerance) continue;
        vec4 c = sample_color(i, uv);
        if (c.a < 0.5) continue;
        float w = wa[i];
        // The source mesh already rejects invalid triangles. Give remaining
        // silhouette pixels a floor so seam smoothing cannot erase thin parts.
        w *= max(m.b, 1.0 / 255.0);
        rgb += c.rgb * w;
        sum_depth += eye_depth(m) * w;
        total += w;
        if (m.b > fallback_quality) {
            fallback = c;
            fallback_quality = m.b;
            fallback_depth = eye_depth(m);
        }
    }
    if (fallback_quality < 0.0) discard;
    float z = total > 0.00001 ? sum_depth / total : fallback_depth;
    ALBEDO = total > 0.00001 ? rgb / total : fallback.rgb;
    vec4 ray = INV_PROJECTION_MATRIX * vec4(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, 1.0, 1.0);
    vec3 point = ray.xyz / ray.w;
    point *= z / max(-point.z, 0.000001);
    vec4 clip = PROJECTION_MATRIX * vec4(point, 1.0);
    float device_z = clip.z / clip.w;
    DEPTH = CLIP_SPACE_FAR < 0.0 ? device_z * 0.5 + 0.5 : device_z;
}
