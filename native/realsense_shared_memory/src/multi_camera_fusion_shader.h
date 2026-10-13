#pragma once

// Texture layers and metadata scale with the rig, not a fixed sampler array.
// A stable priority order chooses one surface; independent/shader-mesh remain
// separate modes. Coverage checks mirror the static mesh's actual triangles.
static constexpr const char *MULTI_CAMERA_FUSION_SHADER = R"(
uniform bool rig_fusion_enabled = false;
uniform sampler2DArray rig_depth : filter_nearest, repeat_disable;
uniform sampler2DArray rig_color : filter_nearest, repeat_disable;
uniform sampler2D rig_cameras : filter_nearest, repeat_disable;
uniform int rig_camera_index = 0;
varying vec3 rig_world_position;

vec4 rig_data(int camera, int column) {
    return texelFetch(rig_cameras, ivec2(column, camera), 0);
}

float rig_z(int camera, vec2 pixel) {
    vec2 size = rig_data(camera, 4).xy;
    if (any(lessThan(pixel, vec2(0.0))) || any(greaterThanEqual(pixel, size))) return 0.0;
    return texelFetch(rig_depth, ivec3(ivec2(pixel), camera), 0).r;
}

bool rig_in_range(float z, vec4 limits) {
    return z >= limits.x && z <= limits.y;
}

float rig_neighbors(int camera, vec2 pixel, float z, float delta, vec4 limits) {
    vec4 neighbors = vec4(rig_z(camera, pixel + vec2(-1, 0)), rig_z(camera, pixel + vec2(1, 0)),
                          rig_z(camera, pixel + vec2(0, -1)), rig_z(camera, pixel + vec2(0, 1)));
    float count = 0.0;
    for (int i = 0; i < 4; i++) {
        if (rig_in_range(neighbors[i], limits) && abs(neighbors[i] - z) <= delta) count += 1.0;
    }
    return count;
}

vec3 rig_project(vec2 pixel, float z, vec4 peer_k, float stride) {
    return vec3((pixel.x * stride - peer_k.z) * z / peer_k.x,
                -(pixel.y * stride - peer_k.w) * z / peer_k.y, -z);
}

bool rig_covers(int camera, vec3 point) {
    vec4 color_info = rig_data(camera, 6);
    if (color_info.w < 0.5 || point.z >= -0.001) return false;
    vec4 peer_k = rig_data(camera, 3);
    vec4 grid = rig_data(camera, 4);
    vec4 limits = rig_data(camera, 5);
    float z = -point.z;
    vec2 pixel = vec2(point.x * peer_k.x / z + peer_k.z,
                     -point.y * peer_k.y / z + peer_k.w) / grid.z;
    if (any(lessThan(pixel, vec2(0.0))) || any(greaterThanEqual(pixel, grid.xy - vec2(1.0)))) return false;
    vec2 center = floor(pixel + vec2(0.5));
    float depth = rig_z(camera, center);
    if (!rig_in_range(depth, limits) || abs(depth - z) > fusion_depth_tolerance) return false;

    vec2 cell = floor(pixel);
    vec2 a = cell;
    vec2 b = cell + vec2(1.0, 0.0);
    vec2 c = cell + vec2(0.0, 1.0);
    if (fract(pixel.x) + fract(pixel.y) > 1.0) a = cell + vec2(1.0);
    float da = rig_z(camera, a);
    float db = rig_z(camera, b);
    float dc = rig_z(camera, c);
    if (!rig_in_range(da, limits) || !rig_in_range(db, limits) || !rig_in_range(dc, limits)) return false;
    if (abs(da-db) > limits.z * max(da,db) || abs(db-dc) > limits.z * max(db,dc) || abs(dc-da) > limits.z * max(dc,da)) return false;
    if (rig_neighbors(camera, a, da, max(0.002, limits.z * da * 0.5), limits) < 2.0 ||
        rig_neighbors(camera, b, db, max(0.002, limits.z * db * 0.5), limits) < 2.0 ||
        rig_neighbors(camera, c, dc, max(0.002, limits.z * dc * 0.5), limits) < 2.0) return false;
    vec3 pa = rig_project(a, da, peer_k, grid.z);
    vec3 pb = rig_project(b, db, peer_k, grid.z);
    vec3 pc = rig_project(c, dc, peer_k, grid.z);
    if (distance(pa,pb) > limits.w || distance(pb,pc) > limits.w || distance(pc,pa) > limits.w) return false;
    vec4 cleanup = rig_data(camera, 7);
    if (cleanup.x > 0.5 && rig_neighbors(camera, center, depth, cleanup.y, limits) < cleanup.z) return false;
    if (color_info.z > 0.5) {
        vec2 uv = ((pixel + vec2(0.5)) / grid.xy) * color_info.xy / vec2(textureSize(rig_color, 0).xy);
        if (texture(rig_color, vec3(uv, float(camera))).a < 0.5) return false;
    }
    return true;
}

bool rig_overlap() {
    if (!rig_fusion_enabled) return false;
    vec4 world = vec4(rig_world_position, 1.0);
    // No camera-count constant: this loop visits every higher-priority layer.
    for (int camera = 0; camera < rig_camera_index; camera++) {
        vec3 point = vec3(dot(rig_data(camera, 0), world), dot(rig_data(camera, 1), world), dot(rig_data(camera, 2), world));
        if (rig_covers(camera, point)) return true;
    }
    return false;
}
)";
