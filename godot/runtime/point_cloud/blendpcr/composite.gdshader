// Godot adaptation of BlendPCR screen-space passes.
// Original: Andre Mühlenbrock, CGVR, 2024. SPDX-License-Identifier: EUPL-1.2
shader_type spatial;
render_mode unshaded, cull_disabled, fog_disabled;
#include "camera_buffers.gdshaderinc"
uniform sampler2D weights_a : filter_linear, repeat_disable;
uniform bool workspace_crop_enabled = false;
uniform vec3 workspace_crop_min = vec3(-0.6, -0.45, -0.6);
uniform vec3 workspace_crop_max = vec3(0.6, 0.45, 0.6);
uniform mat3 workspace_crop_inverse = mat3(1.0);
void vertex() { POSITION = vec4(VERTEX.xy, 1.0, 1.0); }
void fragment() {
    vec2 uv = SCREEN_UV;
    float nearest = depth_span;
    float depths[4];
    float qualities[4];
    vec4 colors[4];
    for (int i = 0; i < camera_count; i++) {
        vec4 m = sample_meta(i, uv);
        float q=m.b,z=packed_value(m,i);
        vec4 c=vec4(0.0);
        bool valid=camera_active(i) && valid_meta(m,i);
        if(valid && native_camera(i)) {
            vec2 rgb_uv;float edge;
            valid=native_sample(i,uv,z,rgb_uv,q,z,edge);
            if(valid)c=native_color(i,rgb_uv,edge);
        } else if(valid)c=sample_color(i,uv);
        valid=valid && c.a>=0.5;
        depths[i]=z;
        qualities[i]=valid?q:-1.0;
        colors[i]=c;
        if(valid)nearest=min(nearest,z);
    }
    vec4 wa = texture(weights_a, uv);
    float total = 0.0;
    vec3 rgb = vec3(0.0);
    vec4 fallback = vec4(0.0);
    float fallback_quality = -1.0;
    float fallback_depth = nearest;
    for (int i = 0; i < camera_count; i++) {
        float q=qualities[i],z=depths[i];
        if(q<0.0 || abs(z-nearest)>agreement_tolerance(nearest))continue;
        vec4 c=colors[i];
        float w = wa[i];
        // The source mesh already rejects invalid triangles. Give remaining
        // silhouette pixels a floor so seam smoothing cannot erase thin parts.
        w *= max(q, 1.0 / 255.0);
        rgb += c.rgb * w;
        total += w;
        if (q > fallback_quality) {
            fallback = c;
            fallback_quality = q;
            fallback_depth = z;
        }
    }
    if (fallback_quality < 0.0) discard;
    // Keep the best camera's measured geometry. Only color is smoothed: depth
    // averaging invents intermediate surfaces at cube/table silhouettes.
    float z = fallback_depth;
    ALBEDO = total > 0.00001 ? rgb / total : fallback.rgb;
    vec4 ray = INV_PROJECTION_MATRIX * vec4(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, 1.0, 1.0);
    vec3 point = ray.xyz / ray.w;
    point *= z / max(-point.z, 0.000001);
    vec3 world = (INV_VIEW_MATRIX * vec4(point, 1.0)).xyz;
    vec3 crop_center = (workspace_crop_min + workspace_crop_max) * 0.5;
    vec3 crop_local = crop_center + workspace_crop_inverse * (world - crop_center);
    if (workspace_crop_enabled && (any(lessThan(crop_local, workspace_crop_min)) || any(greaterThan(crop_local, workspace_crop_max)))) discard;
    vec4 clip = PROJECTION_MATRIX * vec4(point, 1.0);
    float device_z = clip.z / clip.w;
    DEPTH = CLIP_SPACE_FAR < 0.0 ? device_z * 0.5 + 0.5 : device_z;
}
