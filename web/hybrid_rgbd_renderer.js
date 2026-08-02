const state = {
  canvas: null,
  gl: null,
  program: null,
  uniforms: null,
  robotProgram: null,
  robotUniforms: null,
  robotVao: null,
  robotMeshes: {},
  robotTransforms: {},
  robotTransformsUpdatedMs: 0,
  manualRgbCanvas: null,
  manualCropCanvas: null,
  manualRgbBaseCanvas: null,
  manualOverlayCanvas: null,
  manualOverlaySignature: "",
  manualOverlayBounds: null,
  robotLoadToken: 0,
  dataChannel: null,
  dataChannelHandlers: null,
  dataChannelFallbackTimer: 0,
  rgbdClientId: "",
  httpClientId: "",
  httpGeneration: 0,
  httpControllers: new Set(),
  httpWorkers: 0,
  httpSequence: 0,
  httpExpectedSequence: 0,
  httpReorderPackets: new Map(),
  httpReorderTimer: 0,
  httpReorderGapStartedMs: 0,
  socket: null,
  socketOptions: null,
  reconnectTimer: 0,
  reconnectAttempt: 0,
  socketOpenedMs: 0,
  lastPacketMs: 0,
  visibilityHandler: null,
  animationFrame: 0,
  lastRenderMs: 0,
  pendingFrameQueue: [],
  temporalDepthKeyframes: new Map(),
  temporalColorKeyframes: new Map(),
  persistentMetadata: null,
  decoding: false,
  running: false,
  cameras: [],
  baseViewer: null,
  currentViewer: null,
  focusWorld: null,
  focusPicked: false,
  orbitAnchorDirection: null,
  orbitAnchorHeadDelta: null,
  lastHeadDelta: [0, 0, 0],
  headBaseline: null,
  lastHeadActive: false,
  manualOrbitYaw: 0,
  manualOrbitPitch: 0,
  manualDolly: 0,
  manualPan: [0, 0, 0],
  pointerNavigation: null,
  settings: {
    inspect_enabled: false,
    yaw_gain: 2.5,
    pitch_gain: 1.825,
    orbit_pitch_offset: 0,
    max_yaw: 80,
    max_pitch: 45,
    focus_distance: 0.45,
    orbit_distance: 0.35,
    focus_vertical_offset: 0.35,
    dolly_enabled: true,
    dolly_gain: 0.008,
    min_distance: 0.2,
    max_distance: 3.0,
    fov: 55,
    splat_fill: 1.3,
    geometry_mode: "points",
    mesh_depth_delta: 0.035,
    mesh_max_edge: 0.055,
    point_underlay: false,
    robot_overlay_enabled: true,
    robot_overlay_style: "alignment",
    robot_overlay_mask_scanned_arm: true,
    white_background_enabled: false,
  },
  stats: {
    status: "idle",
    connected: false,
    sourceFps: 0,
    bitrateKbps: 0,
    received: 0,
    decoded: 0,
    dropped: 0,
    bytes: 0,
    cameras: 0,
    width: 0,
    height: 0,
    captureAgeMs: -1,
    captureUnixMs: 0,
    decodeMs: 0,
    renderFps: 0,
    renderFrames: 0,
    cameraFrames: {},
    cameraFps: {},
    cameraEncodeMs: {},
    robotModelsLoaded: 0,
    robotModelsExpected: 8,
    robotTransforms: 0,
    robotLoadError: "",
    transport: "--",
  },
  statsWindow: { timestamp: performance.now(), decoded: 0, bytes: 0, renderFrames: 0, cameraFrames: {} },
};

const VERTEX_SHADER = `#version 300 es
precision highp float;
precision highp int;
precision highp usampler2D;

uniform usampler2D u_depth;
uniform sampler2D u_color;
uniform mat4 u_model;
uniform mat4 u_view_projection;
uniform vec4 u_intrinsics;
uniform ivec2 u_size;
uniform float u_point_size;
uniform highp int u_render_mode;
uniform float u_mesh_depth_delta;
uniform float u_mesh_max_edge;
uniform highp int u_robot_mask_enabled;
uniform vec4 u_mask_capsule_a[6];
uniform vec4 u_mask_capsule_b[6];
uniform highp int u_fusion_enabled;
uniform usampler2D u_fusion_depth;
uniform sampler2D u_fusion_color;
uniform mat4 u_world_to_fusion;
uniform vec4 u_fusion_intrinsics;
uniform ivec2 u_fusion_size;
uniform float u_fusion_depth_tolerance;

out vec3 v_color;
out float v_valid;

bool depth_supported(ivec2 pixel, uint depth_mm) {
  float tolerance_mm = max(2.0, u_mesh_depth_delta * float(depth_mm) * 0.5);
  int support = 0;
  const ivec2 offsets[4] = ivec2[4](ivec2(-1, 0), ivec2(1, 0), ivec2(0, -1), ivec2(0, 1));
  for (int index = 0; index < 4; index++) {
    ivec2 neighbor_pixel = pixel + offsets[index];
    if (any(lessThan(neighbor_pixel, ivec2(0))) || any(greaterThanEqual(neighbor_pixel, u_size))) continue;
    uint neighbor_depth = texelFetch(u_depth, neighbor_pixel, 0).r;
    if (neighbor_depth > uint(0) && abs(float(neighbor_depth) - float(depth_mm)) <= tolerance_mm) support += 1;
  }
  return support >= 2;
}

void main() {
  ivec2 pixel_coord;
  bool triangle_valid = true;
  if (u_render_mode == 1) {
    int cells_x = u_size.x - 1;
    int vertex_in_cell = gl_VertexID % 6;
    int cell_index = gl_VertexID / 6;
    int cell_x = cell_index % cells_x;
    int cell_y = cell_index / cells_x;
    ivec2 top_left = ivec2(cell_x, cell_y);
    ivec2 top_right = top_left + ivec2(1, 0);
    ivec2 bottom_left = top_left + ivec2(0, 1);
    ivec2 bottom_right = top_left + ivec2(1, 1);
    ivec2 triangle_a;
    ivec2 triangle_b;
    ivec2 triangle_c;
    if (vertex_in_cell < 3) {
      triangle_a = top_left;
      triangle_b = top_right;
      triangle_c = bottom_left;
      pixel_coord = vertex_in_cell == 0 ? triangle_a : (vertex_in_cell == 1 ? triangle_b : triangle_c);
    } else {
      triangle_a = top_right;
      triangle_b = bottom_right;
      triangle_c = bottom_left;
      pixel_coord = vertex_in_cell == 3 ? triangle_a : (vertex_in_cell == 4 ? triangle_b : triangle_c);
    }
    uint depth_a = texelFetch(u_depth, triangle_a, 0).r;
    uint depth_b = texelFetch(u_depth, triangle_b, 0).r;
    uint depth_c = texelFetch(u_depth, triangle_c, 0).r;
    uint minimum_depth = min(depth_a, min(depth_b, depth_c));
    uint maximum_depth = max(depth_a, max(depth_b, depth_c));
    float za = float(depth_a) * 0.001;
    float zb = float(depth_b) * 0.001;
    float zc = float(depth_c) * 0.001;
    vec3 pa = vec3((float(triangle_a.x) - u_intrinsics.z) * za / u_intrinsics.x, -(float(triangle_a.y) - u_intrinsics.w) * za / u_intrinsics.y, -za);
    vec3 pb = vec3((float(triangle_b.x) - u_intrinsics.z) * zb / u_intrinsics.x, -(float(triangle_b.y) - u_intrinsics.w) * zb / u_intrinsics.y, -zb);
    vec3 pc = vec3((float(triangle_c.x) - u_intrinsics.z) * zc / u_intrinsics.x, -(float(triangle_c.y) - u_intrinsics.w) * zc / u_intrinsics.y, -zc);
    float maximum_edge_squared = u_mesh_max_edge * u_mesh_max_edge;
    triangle_valid = minimum_depth > uint(0)
      && float(maximum_depth - minimum_depth) <= u_mesh_depth_delta * float(maximum_depth)
      && dot(pa - pb, pa - pb) <= maximum_edge_squared
      && dot(pb - pc, pb - pc) <= maximum_edge_squared
      && dot(pc - pa, pc - pa) <= maximum_edge_squared
      && depth_supported(triangle_a, depth_a)
      && depth_supported(triangle_b, depth_b)
      && depth_supported(triangle_c, depth_c);
  } else {
    pixel_coord = ivec2(gl_VertexID % u_size.x, gl_VertexID / u_size.x);
  }
  int x = pixel_coord.x;
  int y = pixel_coord.y;
  uint depth_mm = texelFetch(u_depth, ivec2(x, y), 0).r;
  bool reject_vertex = depth_mm == uint(0)
    || (u_render_mode == 1 && !triangle_valid);
  if (reject_vertex) {
    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
    gl_PointSize = 0.0;
    v_color = vec3(0.0);
    v_valid = 0.0;
    return;
  }
  float z = float(depth_mm) * 0.001;
  vec3 local_position = vec3(
    (float(x) - u_intrinsics.z) * z / u_intrinsics.x,
    -(float(y) - u_intrinsics.w) * z / u_intrinsics.y,
    -z
  );
  vec3 world_position = (u_model * vec4(local_position, 1.0)).xyz;
  if (u_robot_mask_enabled == 1) {
    for (int index = 0; index < 6; index++) {
      vec3 segment = u_mask_capsule_b[index].xyz - u_mask_capsule_a[index].xyz;
      float segment_length_squared = dot(segment, segment);
      float along = segment_length_squared > 0.000001
        ? clamp(dot(world_position - u_mask_capsule_a[index].xyz, segment) / segment_length_squared, 0.0, 1.0)
        : 0.0;
      vec3 closest = u_mask_capsule_a[index].xyz + segment * along;
      float radius = u_mask_capsule_a[index].w;
      if (dot(world_position - closest, world_position - closest) <= radius * radius) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        gl_PointSize = 0.0;
        v_color = vec3(0.0);
        v_valid = 0.0;
        return;
      }
    }
  }
  gl_Position = u_view_projection * vec4(world_position, 1.0);
  gl_PointSize = u_point_size;
  v_color = texelFetch(u_color, ivec2(x, y), 0).rgb;
  if (u_fusion_enabled == 1) {
    vec3 fusion_position = (u_world_to_fusion * vec4(world_position, 1.0)).xyz;
    float fusion_projected_depth = -fusion_position.z;
    if (fusion_projected_depth > 0.001) {
      ivec2 fusion_pixel = ivec2(round(vec2(
        fusion_position.x * u_fusion_intrinsics.x / fusion_projected_depth + u_fusion_intrinsics.z,
        -fusion_position.y * u_fusion_intrinsics.y / fusion_projected_depth + u_fusion_intrinsics.w
      )));
      if (all(greaterThanEqual(fusion_pixel, ivec2(0))) && all(lessThan(fusion_pixel, u_fusion_size))) {
        uint fusion_depth_mm = texelFetch(u_fusion_depth, fusion_pixel, 0).r;
        float fusion_depth_m = float(fusion_depth_mm) * 0.001;
        if (fusion_depth_mm > uint(0) && abs(fusion_depth_m - fusion_projected_depth) <= u_fusion_depth_tolerance) {
          v_color = texelFetch(u_fusion_color, fusion_pixel, 0).rgb;
        }
      }
    }
  }
  v_valid = 1.0;
}
`;

const FRAGMENT_SHADER = `#version 300 es
precision highp float;

in vec3 v_color;
in float v_valid;
uniform highp int u_render_mode;
out vec4 out_color;

void main() {
  if (v_valid < 0.9999) discard;
  // A surface triangle is only usable when all three vertices survived the
  // depth-continuity test. Interpolation makes v_valid less than one if even
  // one corner was rejected, preventing the long screen-spanning streaks
  // produced by triangles connected across depth holes.
  if (u_render_mode != 1) {
    vec2 centered = gl_PointCoord * 2.0 - 1.0;
    if (dot(centered, centered) > 1.0) discard;
  }
  out_color = vec4(v_color, 1.0);
}
`;

const ROBOT_VERTEX_SHADER = `#version 300 es
precision highp float;

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
uniform mat4 u_view_projection;
uniform mat4 u_model;
uniform vec2 u_viewport;
uniform float u_outline_pixels;

void main() {
  vec4 world = u_model * vec4(a_position, 1.0);
  vec4 clip = u_view_projection * world;
  if (u_outline_pixels > 0.0 && dot(a_normal, a_normal) > 0.01) {
    mat3 normal_matrix = transpose(inverse(mat3(u_model)));
    vec3 world_normal = normalize(normal_matrix * a_normal);
    vec4 offset_clip = u_view_projection * vec4(world.xyz + world_normal * 0.001, 1.0);
    vec2 projected = offset_clip.xy / max(abs(offset_clip.w), 0.00001) - clip.xy / max(abs(clip.w), 0.00001);
    float projected_length = length(projected);
    if (projected_length > 0.000001) {
      clip.xy += projected / projected_length * (2.0 * u_outline_pixels / u_viewport) * clip.w;
    }
  }
  gl_Position = clip;
}
`;

const ROBOT_FRAGMENT_SHADER = `#version 300 es
precision highp float;

uniform vec4 u_color;
out vec4 out_color;

void main() {
  out_color = u_color;
}
`;

const ROBOT_MODEL_URLS = {
  base_link: "/assets/robots/so101/base_link.glb",
  shoulder_link: "/assets/robots/so101/shoulder_link.glb",
  upper_arm_link: "/assets/robots/so101/upper_arm_link.glb",
  lower_arm_link: "/assets/robots/so101/lower_arm_link.glb",
  wrist_link: "/assets/robots/so101/wrist_link.glb",
  gripper_link: "/assets/robots/so101/gripper_link.glb",
  moving_jaw_so101_v1_link: "/assets/robots/so101/moving_jaw_so101_v1_link.glb",
};

const ROBOT_LINK_COLORS = {
  base_link: [0.91, 0.76, 0.18, 1],
  shoulder_link: [0.78, 0.64, 0.12, 1],
  upper_arm_link: [0.95, 0.81, 0.24, 1],
  lower_arm_link: [0.78, 0.64, 0.12, 1],
  wrist_link: [0.95, 0.81, 0.24, 1],
  gripper_link: [0.78, 0.64, 0.12, 1],
  moving_jaw_so101_v1_link: [0.95, 0.81, 0.24, 1],
};

function clamp(value, minimum, maximum) {
  return Math.max(minimum, Math.min(maximum, value));
}

function normalize(vector) {
  const length = Math.hypot(vector[0], vector[1], vector[2]) || 1;
  return [vector[0] / length, vector[1] / length, vector[2] / length];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function add(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function scale(vector, amount) {
  return [vector[0] * amount, vector[1] * amount, vector[2] * amount];
}

function rotateAroundAxis(vector, axis, radians) {
  const unitAxis = normalize(axis);
  const cosine = Math.cos(radians);
  const sine = Math.sin(radians);
  return add(
    add(scale(vector, cosine), scale(cross(unitAxis, vector), sine)),
    scale(unitAxis, dot(unitAxis, vector) * (1 - cosine)),
  );
}

function multiplyMatrix(a, b) {
  const out = new Float32Array(16);
  for (let column = 0; column < 4; column += 1) {
    for (let row = 0; row < 4; row += 1) {
      out[column * 4 + row] =
        a[row] * b[column * 4]
        + a[4 + row] * b[column * 4 + 1]
        + a[8 + row] * b[column * 4 + 2]
        + a[12 + row] * b[column * 4 + 3];
    }
  }
  return out;
}

function transformPoint(matrix, point) {
  const x = point[0];
  const y = point[1];
  const z = point[2];
  return [
    matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12],
    matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13],
    matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14],
  ];
}

function transformPoint4(matrix, point) {
  const x = point[0];
  const y = point[1];
  const z = point[2];
  const w = point[3] ?? 1;
  return [
    matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12] * w,
    matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13] * w,
    matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14] * w,
    matrix[3] * x + matrix[7] * y + matrix[11] * z + matrix[15] * w,
  ];
}

function inverseRigidTransform(matrix) {
  const out = new Float32Array(16);
  out[0] = matrix[0]; out[1] = matrix[4]; out[2] = matrix[8]; out[3] = 0;
  out[4] = matrix[1]; out[5] = matrix[5]; out[6] = matrix[9]; out[7] = 0;
  out[8] = matrix[2]; out[9] = matrix[6]; out[10] = matrix[10]; out[11] = 0;
  out[15] = 1;
  const origin = [matrix[12], matrix[13], matrix[14]];
  out[12] = -dot([out[0], out[4], out[8]], origin);
  out[13] = -dot([out[1], out[5], out[9]], origin);
  out[14] = -dot([out[2], out[6], out[10]], origin);
  return out;
}

function perspectiveMatrix(fovDegrees, aspect, near, far) {
  const f = 1 / Math.tan((fovDegrees * Math.PI) / 360);
  const out = new Float32Array(16);
  out[0] = f / Math.max(0.001, aspect);
  out[5] = f;
  out[10] = (far + near) / (near - far);
  out[11] = -1;
  out[14] = (2 * far * near) / (near - far);
  return out;
}

function lookAtTransform(position, target, upHint) {
  const back = normalize([
    position[0] - target[0],
    position[1] - target[1],
    position[2] - target[2],
  ]);
  let right = normalize(cross(upHint, back));
  if (Math.hypot(...right) < 0.001) right = [1, 0, 0];
  const up = normalize(cross(back, right));
  return new Float32Array([
    right[0], right[1], right[2], 0,
    up[0], up[1], up[2], 0,
    back[0], back[1], back[2], 0,
    position[0], position[1], position[2], 1,
  ]);
}

function compileShader(gl, type, source) {
  const shader = gl.createShader(type);
  gl.shaderSource(shader, source);
  gl.compileShader(shader);
  if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
    const message = gl.getShaderInfoLog(shader) || "shader compilation failed";
    gl.deleteShader(shader);
    throw new Error(message);
  }
  return shader;
}

function createProgram(gl, vertexSource = VERTEX_SHADER, fragmentSource = FRAGMENT_SHADER) {
  const vertex = compileShader(gl, gl.VERTEX_SHADER, vertexSource);
  const fragment = compileShader(gl, gl.FRAGMENT_SHADER, fragmentSource);
  const program = gl.createProgram();
  gl.attachShader(program, vertex);
  gl.attachShader(program, fragment);
  gl.linkProgram(program);
  gl.deleteShader(vertex);
  gl.deleteShader(fragment);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
    const message = gl.getProgramInfoLog(program) || "shader linking failed";
    gl.deleteProgram(program);
    throw new Error(message);
  }
  return program;
}

function findRobotUniforms(gl, program) {
  return {
    viewProjection: gl.getUniformLocation(program, "u_view_projection"),
    model: gl.getUniformLocation(program, "u_model"),
    color: gl.getUniformLocation(program, "u_color"),
    viewport: gl.getUniformLocation(program, "u_viewport"),
    outlinePixels: gl.getUniformLocation(program, "u_outline_pixels"),
  };
}

function identityMatrix() {
  return new Float32Array([
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ]);
}

function transformDictionaryMatrix(value) {
  if (!value || !Array.isArray(value.basis_x) || !Array.isArray(value.origin)) return null;
  return new Float32Array([
    Number(value.basis_x[0]), Number(value.basis_x[1]), Number(value.basis_x[2]), 0,
    Number(value.basis_y[0]), Number(value.basis_y[1]), Number(value.basis_y[2]), 0,
    Number(value.basis_z[0]), Number(value.basis_z[1]), Number(value.basis_z[2]), 0,
    Number(value.origin[0]), Number(value.origin[1]), Number(value.origin[2]), 1,
  ]);
}

function nodeLocalMatrix(node) {
  if (Array.isArray(node.matrix) && node.matrix.length === 16) return new Float32Array(node.matrix);
  const translation = node.translation || [0, 0, 0];
  const rotation = node.rotation || [0, 0, 0, 1];
  const scaling = node.scale || [1, 1, 1];
  const [x, y, z, w] = rotation.map(Number);
  const x2 = x + x;
  const y2 = y + y;
  const z2 = z + z;
  const xx = x * x2;
  const xy = x * y2;
  const xz = x * z2;
  const yy = y * y2;
  const yz = y * z2;
  const zz = z * z2;
  const wx = w * x2;
  const wy = w * y2;
  const wz = w * z2;
  return new Float32Array([
    (1 - (yy + zz)) * scaling[0], (xy + wz) * scaling[0], (xz - wy) * scaling[0], 0,
    (xy - wz) * scaling[1], (1 - (xx + zz)) * scaling[1], (yz + wx) * scaling[1], 0,
    (xz + wy) * scaling[2], (yz - wx) * scaling[2], (1 - (xx + yy)) * scaling[2], 0,
    Number(translation[0]), Number(translation[1]), Number(translation[2]), 1,
  ]);
}

function parseGlb(arrayBuffer) {
  const view = new DataView(arrayBuffer);
  if (view.byteLength < 20 || view.getUint32(0, true) !== 0x46546c67 || view.getUint32(4, true) !== 2) {
    throw new Error("invalid GLB asset");
  }
  let offset = 12;
  let gltf = null;
  let binary = null;
  while (offset + 8 <= view.byteLength) {
    const chunkLength = view.getUint32(offset, true);
    const chunkType = view.getUint32(offset + 4, true);
    offset += 8;
    if (offset + chunkLength > view.byteLength) throw new Error("truncated GLB chunk");
    if (chunkType === 0x4e4f534a) {
      const text = new TextDecoder().decode(new Uint8Array(arrayBuffer, offset, chunkLength)).replace(/[\0\s]+$/, "");
      gltf = JSON.parse(text);
    } else if (chunkType === 0x004e4942) {
      binary = arrayBuffer.slice(offset, offset + chunkLength);
    }
    offset += chunkLength;
  }
  if (!gltf || !binary) throw new Error("GLB is missing JSON or binary data");
  return { gltf, binary };
}

function accessorValues(gltf, binary, accessorIndex) {
  const accessor = gltf.accessors[accessorIndex];
  const bufferView = gltf.bufferViews[accessor.bufferView];
  const componentCounts = { SCALAR: 1, VEC2: 2, VEC3: 3, VEC4: 4, MAT4: 16 };
  const componentBytes = { 5120: 1, 5121: 1, 5122: 2, 5123: 2, 5125: 4, 5126: 4 };
  const readers = {
    5120: (view, offset) => view.getInt8(offset),
    5121: (view, offset) => view.getUint8(offset),
    5122: (view, offset) => view.getInt16(offset, true),
    5123: (view, offset) => view.getUint16(offset, true),
    5125: (view, offset) => view.getUint32(offset, true),
    5126: (view, offset) => view.getFloat32(offset, true),
  };
  const componentCount = componentCounts[accessor.type];
  const bytes = componentBytes[accessor.componentType];
  const reader = readers[accessor.componentType];
  if (!componentCount || !bytes || !reader) throw new Error(`unsupported GLB accessor ${accessor.type}/${accessor.componentType}`);
  const start = Number(bufferView.byteOffset || 0) + Number(accessor.byteOffset || 0);
  const stride = Number(bufferView.byteStride || componentCount * bytes);
  const view = new DataView(binary);
  const values = new Float64Array(accessor.count * componentCount);
  for (let index = 0; index < accessor.count; index += 1) {
    for (let component = 0; component < componentCount; component += 1) {
      values[index * componentCount + component] = reader(view, start + index * stride + component * bytes);
    }
  }
  return { values, count: accessor.count, componentCount };
}

function glbPrimitives(gltf, binary) {
  const output = [];
  const scene = gltf.scenes[(gltf.scene ?? 0)] || { nodes: [] };
  const visit = (nodeIndex, parentMatrix) => {
    const node = gltf.nodes[nodeIndex];
    const localMatrix = multiplyMatrix(parentMatrix, nodeLocalMatrix(node));
    if (Number.isInteger(node.mesh)) {
      for (const primitive of gltf.meshes[node.mesh].primitives || []) {
        if (!primitive.attributes || !Number.isInteger(primitive.attributes.POSITION)) continue;
        const positions = accessorValues(gltf, binary, primitive.attributes.POSITION);
        const normals = Number.isInteger(primitive.attributes.NORMAL)
          ? accessorValues(gltf, binary, primitive.attributes.NORMAL)
          : null;
        let indices = null;
        if (Number.isInteger(primitive.indices)) {
          const source = accessorValues(gltf, binary, primitive.indices).values;
          indices = new Uint32Array(source.length);
          for (let index = 0; index < source.length; index += 1) indices[index] = source[index];
        }
        output.push({
          positions: new Float32Array(positions.values),
          normals: normals ? new Float32Array(normals.values) : new Float32Array(positions.count * 3),
          count: indices ? indices.length : positions.count,
          indices,
          mode: Number(primitive.mode ?? 4),
          localMatrix,
        });
      }
    }
    for (const child of node.children || []) visit(child, localMatrix);
  };
  for (const rootNode of scene.nodes || []) visit(rootNode, identityMatrix());
  return output;
}

function robotDrawMode(gl, mode) {
  return ({ 0: gl.POINTS, 1: gl.LINES, 2: gl.LINE_LOOP, 3: gl.LINE_STRIP, 4: gl.TRIANGLES, 5: gl.TRIANGLE_STRIP, 6: gl.TRIANGLE_FAN })[mode] || gl.TRIANGLES;
}

async function loadRobotMeshes(gl, token) {
  deleteRobotMeshes(gl, state.robotMeshes);
  state.robotMeshes = {};
  state.stats.robotModelsLoaded = 0;
  state.stats.robotModelsExpected = Object.keys(ROBOT_MODEL_URLS).length;
  state.stats.robotLoadError = "";
  const failures = [];
  await Promise.all(Object.entries(ROBOT_MODEL_URLS).map(async ([name, url]) => {
    let uploaded = [];
    try {
      const response = await fetch(url, { cache: "no-cache" });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      const { gltf, binary } = parseGlb(await response.arrayBuffer());
      const primitives = glbPrimitives(gltf, binary);
      if (!primitives.length) throw new Error("model has no drawable primitives");
      if (!state.running || state.gl !== gl || state.robotLoadToken !== token) return;
      uploaded = primitives.map((primitive) => {
        const positionBuffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, primitive.positions, gl.STATIC_DRAW);
        const normalBuffer = gl.createBuffer();
        gl.bindBuffer(gl.ARRAY_BUFFER, normalBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, primitive.normals, gl.STATIC_DRAW);
        let indexBuffer = null;
        if (primitive.indices) {
          indexBuffer = gl.createBuffer();
          gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, indexBuffer);
          gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, primitive.indices, gl.STATIC_DRAW);
        }
        const keepForCalibration = ["wrist_link", "gripper_link", "moving_jaw_so101_v1_link"].includes(name);
        return {
          ...primitive,
          cpuPositions: keepForCalibration ? primitive.positions : null,
          cpuIndices: keepForCalibration ? primitive.indices : null,
          positions: null,
          normals: null,
          indices: null,
          positionBuffer,
          normalBuffer,
          indexBuffer,
        };
      });
      if (!state.running || state.gl !== gl || state.robotLoadToken !== token) {
        deleteRobotMeshes(gl, { [name]: uploaded });
        return;
      }
      state.robotMeshes[name] = uploaded;
      state.stats.robotModelsLoaded = Object.keys(state.robotMeshes).length;
    } catch (error) {
      if (uploaded.length) deleteRobotMeshes(gl, { [name]: uploaded });
      const message = `${name}: ${error && error.message ? error.message : error}`;
      failures.push(message);
      console.warn("Hybrid robot link unavailable:", message);
    }
  }));
  if (state.running && state.gl === gl && state.robotLoadToken === token) {
    state.stats.robotLoadError = failures.join("; ");
    console.info(`Hybrid robot overlay loaded ${state.stats.robotModelsLoaded}/${state.stats.robotModelsExpected} links`);
  }
}

function deleteRobotMeshes(gl, meshes = state.robotMeshes) {
  if (!gl) return;
  for (const primitives of Object.values(meshes || {})) {
    for (const primitive of primitives) {
      if (primitive.positionBuffer) gl.deleteBuffer(primitive.positionBuffer);
      if (primitive.normalBuffer) gl.deleteBuffer(primitive.normalBuffer);
      if (primitive.indexBuffer) gl.deleteBuffer(primitive.indexBuffer);
    }
  }
  if (meshes === state.robotMeshes) state.robotMeshes = {};
}

function drawRobotOverlay(viewProjection) {
  if (!state.settings.robot_overlay_enabled || !state.robotProgram || !state.robotVao) return;
  const gl = state.gl;
  gl.useProgram(state.robotProgram);
  gl.bindVertexArray(state.robotVao);
  gl.uniformMatrix4fv(state.robotUniforms.viewProjection, false, viewProjection);
  gl.uniform2f(state.robotUniforms.viewport, Math.max(gl.canvas.width, 1), Math.max(gl.canvas.height, 1));
  gl.enable(gl.DEPTH_TEST);
  gl.depthFunc(gl.LEQUAL);
  const alignmentStyle = state.settings.robot_overlay_style === "alignment";
  const drawLinks = (outline) => {
    gl.uniform1f(state.robotUniforms.outlinePixels, outline ? 1.75 : 0.0);
    for (const [name, primitives] of Object.entries(state.robotMeshes)) {
      const globalMatrix = transformDictionaryMatrix(state.robotTransforms[name]);
      if (!globalMatrix) continue;
      const movingJaw = name === "moving_jaw_so101_v1_link";
      const color = alignmentStyle
        ? movingJaw
          ? [0.0, 0.88, 1.0, outline ? 0.95 : 0.24]
          : [1.0, 0.82, 0.0, outline ? 0.95 : 0.20]
        : ROBOT_LINK_COLORS[name] || [0.9, 0.75, 0.18, 1];
      gl.uniform4fv(state.robotUniforms.color, color);
      for (const primitive of primitives) {
      gl.bindBuffer(gl.ARRAY_BUFFER, primitive.positionBuffer);
      gl.enableVertexAttribArray(0);
      gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
      gl.bindBuffer(gl.ARRAY_BUFFER, primitive.normalBuffer);
      gl.enableVertexAttribArray(1);
      gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 0, 0);
      gl.uniformMatrix4fv(state.robotUniforms.model, false, multiplyMatrix(globalMatrix, primitive.localMatrix));
      if (primitive.indexBuffer) {
        gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, primitive.indexBuffer);
        gl.drawElements(robotDrawMode(gl, primitive.mode), primitive.count, gl.UNSIGNED_INT, 0);
      } else {
        gl.drawArrays(robotDrawMode(gl, primitive.mode), 0, primitive.count);
      }
    }
    }
  };
  if (alignmentStyle) {
    gl.depthMask(false);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    gl.enable(gl.CULL_FACE);
    gl.cullFace(gl.FRONT);
    drawLinks(true);
    gl.cullFace(gl.BACK);
    drawLinks(false);
    gl.depthMask(true);
    gl.disable(gl.BLEND);
    gl.disable(gl.CULL_FACE);
  } else {
    gl.disable(gl.BLEND);
    gl.disable(gl.CULL_FACE);
    gl.depthMask(true);
    drawLinks(false);
  }
  gl.bindVertexArray(null);
}

function robotMaskCapsules() {
  const names = ["base_link", "shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link", "moving_jaw_so101_v1_link"];
  const radii = [0.065, 0.060, 0.050, 0.047, 0.043, 0.040];
  const starts = new Float32Array(24);
  const ends = new Float32Array(24);
  for (let index = 0; index < radii.length; index += 1) {
    const start = transformDictionaryMatrix(state.robotTransforms[names[index]]);
    const end = transformDictionaryMatrix(state.robotTransforms[names[index + 1]]);
    if (!start || !end) return null;
    starts.set([start[12], start[13], start[14], radii[index]], index * 4);
    ends.set([end[12], end[13], end[14], radii[index]], index * 4);
  }
  return { starts, ends };
}

function findUniforms(gl, program) {
  return {
    viewProjection: gl.getUniformLocation(program, "u_view_projection"),
    depth: gl.getUniformLocation(program, "u_depth"),
    color: gl.getUniformLocation(program, "u_color"),
    model: gl.getUniformLocation(program, "u_model"),
    intrinsics: gl.getUniformLocation(program, "u_intrinsics"),
    size: gl.getUniformLocation(program, "u_size"),
    pointSize: gl.getUniformLocation(program, "u_point_size"),
    renderMode: gl.getUniformLocation(program, "u_render_mode"),
    meshDepthDelta: gl.getUniformLocation(program, "u_mesh_depth_delta"),
    meshMaxEdge: gl.getUniformLocation(program, "u_mesh_max_edge"),
    robotMaskEnabled: gl.getUniformLocation(program, "u_robot_mask_enabled"),
    maskCapsuleA: gl.getUniformLocation(program, "u_mask_capsule_a[0]"),
    maskCapsuleB: gl.getUniformLocation(program, "u_mask_capsule_b[0]"),
    fusionEnabled: gl.getUniformLocation(program, "u_fusion_enabled"),
    fusionDepth: gl.getUniformLocation(program, "u_fusion_depth"),
    fusionColor: gl.getUniformLocation(program, "u_fusion_color"),
    worldToFusion: gl.getUniformLocation(program, "u_world_to_fusion"),
    fusionIntrinsics: gl.getUniformLocation(program, "u_fusion_intrinsics"),
    fusionSize: gl.getUniformLocation(program, "u_fusion_size"),
    fusionDepthTolerance: gl.getUniformLocation(program, "u_fusion_depth_tolerance"),
  };
}

function parsePacket(arrayBuffer) {
  const bytes = new Uint8Array(arrayBuffer);
  if (bytes.length < 9 || new TextDecoder().decode(bytes.subarray(0, 5)) !== "RGBD1") {
    throw new Error("invalid RGB-D packet header");
  }
  const metadataLength = new DataView(arrayBuffer, 5, 4).getUint32(0, true);
  const metadataStart = 9;
  const metadataEnd = metadataStart + metadataLength;
  if (metadataEnd > bytes.length) throw new Error("truncated RGB-D metadata");
  const wireMetadata = JSON.parse(new TextDecoder().decode(bytes.subarray(metadataStart, metadataEnd)));
  let metadata = wireMetadata;
  if (wireMetadata.metadata_delta === "persistent_v1") {
    const previous = state.persistentMetadata;
    if (
      !previous
      || Number(previous.metadata_frame_id || 0)
        !== Number(wireMetadata.metadata_base_id || 0)
    ) {
      throw new Error(
        `missing persistent metadata reference ${wireMetadata.metadata_base_id || 0}`,
      );
    }
    const previousCameras = new Map(
      (previous.cameras || []).map((camera, index) => [
        String(camera.id || camera.serial || index),
        camera,
      ]),
    );
    metadata = { ...previous, ...wireMetadata };
    for (const key of wireMetadata.metadata_removed || []) {
      delete metadata[key];
    }
    delete metadata.metadata_removed;
    metadata.cameras = (wireMetadata.cameras || []).map((camera, index) => {
      const key = String(camera.id || camera.serial || index);
      const merged = { ...(previousCameras.get(key) || {}), ...camera };
      for (const field of camera.metadata_removed || []) {
        delete merged[field];
      }
      delete merged.metadata_removed;
      return merged;
    });
  }
  if (
    metadata.temporal_reference_mode === "persistent_v1"
    && Number(metadata.metadata_frame_id || 0) > 0
  ) {
    state.persistentMetadata = metadata;
  }
  let offset = metadataEnd;
  const payloads = [];
  for (const camera of metadata.cameras || []) {
    const colorEnd = offset + Number(camera.color_length || 0);
    const depthEnd = colorEnd + Number(camera.depth_length || 0);
    if (colorEnd > bytes.length || depthEnd > bytes.length) throw new Error("truncated RGB-D payload");
    payloads.push({
      metadata: camera,
      color: bytes.slice(offset, colorEnd),
      depth: bytes.slice(colorEnd, depthEnd),
    });
    offset = depthEnd;
  }
  return { metadata, payloads };
}

function cameraPayloadKey(payload, index = 0) {
  const metadata = payload && payload.metadata ? payload.metadata : {};
  return String(metadata.id || metadata.serial || metadata.model || index);
}

async function inflateDepth(bytes, metadata = {}) {
  if (typeof DecompressionStream !== "function") {
    throw new Error("this browser does not support deflate decompression");
  }
  const stream = new Blob([bytes]).stream().pipeThrough(new DecompressionStream("deflate"));
  const buffer = await new Response(stream).arrayBuffer();
  const width = Number(metadata.width || 0);
  const height = Number(metadata.height || 0);
  const sampleCount = width * height;
  const cameraKey = String(metadata.id || metadata.serial || metadata.model || "camera");
  if (metadata.depth_encoding === "u16_mm_keyframe_tiles_deflate") {
    const packed = new Uint8Array(buffer);
    if (
      packed.length < 18
      || packed[0] !== 68
      || packed[1] !== 84
      || packed[2] !== 76
      || packed[3] !== 49
    ) {
      throw new Error("invalid temporal depth header");
    }
    const view = new DataView(
      packed.buffer,
      packed.byteOffset,
      packed.byteLength,
    );
    const keyframeId = view.getUint32(4, true);
    const tileSize = view.getUint16(8, true);
    const tileRows = view.getUint16(10, true);
    const tileCols = view.getUint16(12, true);
    const changedCount = view.getUint32(14, true);
    const keyframe = state.temporalDepthKeyframes.get(cameraKey);
    if (
      !keyframe
      || keyframe.id !== keyframeId
      || keyframe.width !== width
      || keyframe.height !== height
    ) {
      throw new Error(`missing temporal depth keyframe ${keyframeId}`);
    }
    if (
      tileSize < 4
      || tileSize > 32
      || tileRows !== Math.ceil(height / tileSize)
      || tileCols !== Math.ceil(width / tileSize)
    ) {
      throw new Error("invalid temporal depth tile geometry");
    }
    const tileCount = tileRows * tileCols;
    const changedBitsLength = Math.ceil(tileCount / 8);
    const valuesPerTile = tileSize * tileSize;
    const changedSamples = changedCount * valuesPerTile;
    const lowOffset = 18 + changedBitsLength;
    const highOffset = lowOffset + changedSamples;
    if (highOffset + changedSamples > packed.length) {
      throw new Error("truncated temporal depth tiles");
    }
    const depth = new Uint16Array(keyframe.depth);
    let changedIndex = 0;
    for (let tileIndex = 0; tileIndex < tileCount; tileIndex += 1) {
      if ((packed[18 + (tileIndex >> 3)] & (1 << (tileIndex & 7))) === 0) continue;
      if (changedIndex >= changedCount) {
        throw new Error("temporal depth tile count overflow");
      }
      const tileY = Math.floor(tileIndex / tileCols) * tileSize;
      const tileX = (tileIndex % tileCols) * tileSize;
      const sampleBase = changedIndex * valuesPerTile;
      for (let localY = 0; localY < tileSize; localY += 1) {
        const y = tileY + localY;
        if (y >= height) break;
        for (let localX = 0; localX < tileSize; localX += 1) {
          const x = tileX + localX;
          if (x >= width) break;
          const sample = sampleBase + localY * tileSize + localX;
          depth[y * width + x] = packed[lowOffset + sample]
            | (packed[highOffset + sample] << 8);
        }
      }
      changedIndex += 1;
    }
    if (changedIndex !== changedCount) {
      throw new Error("temporal depth tile count mismatch");
    }
    if (
      metadata.depth_temporal_reference_mode === "persistent_v1"
      && Number(metadata.depth_frame_id || 0) > 0
    ) {
      state.temporalDepthKeyframes.set(cameraKey, {
        id: Number(metadata.depth_frame_id),
        width,
        height,
        depth: new Uint16Array(depth),
      });
    }
    return depth;
  }

  let depth;
  if (metadata.depth_encoding !== "u16_mm_vpredict_shuffle_deflate") {
    depth = new Uint16Array(buffer);
  } else {
    const predicted = new Uint8Array(buffer);
    if (width <= 0 || height <= 0 || predicted.length < sampleCount * 2) {
      throw new Error(`invalid predictive depth dimensions ${width}x${height}`);
    }
    depth = new Uint16Array(sampleCount);
    for (let index = 0; index < sampleCount; index += 1) {
      const zigzag = predicted[index] | (predicted[index + sampleCount] << 8);
      const delta = (zigzag >>> 1) ^ -(zigzag & 1);
      const previous = index >= width ? depth[index - width] : 0;
      depth[index] = (previous + delta) & 0xffff;
    }
  }
  if (
    metadata.depth_temporal_keyframe === true
    && Number(metadata.depth_keyframe_id || 0) > 0
  ) {
    state.temporalDepthKeyframes.set(cameraKey, {
      id: Number(metadata.depth_keyframe_id),
      width,
      height,
      depth: new Uint16Array(depth),
    });
  }
  return depth;
}

function colorCanvas(width, height) {
  if (typeof OffscreenCanvas === "function") {
    return new OffscreenCanvas(width, height);
  }
  const canvas = document.createElement("canvas");
  canvas.width = width;
  canvas.height = height;
  return canvas;
}

function rgbaToRgb(rgba) {
  const rgb = new Uint8Array((rgba.length / 4) * 3);
  for (let source = 0, target = 0; source < rgba.length; source += 4, target += 3) {
    rgb[target] = rgba[source];
    rgb[target + 1] = rgba[source + 1];
    rgb[target + 2] = rgba[source + 2];
  }
  return rgb;
}

async function decodeColor(bytes, metadata = {}) {
  const width = Number(metadata.width || 0);
  const height = Number(metadata.height || 0);
  const cameraKey = String(metadata.id || metadata.serial || metadata.model || "camera");
  if (metadata.color_encoding !== "rgb8_keyframe_tiles_jpeg") {
    const bitmap = await createImageBitmap(new Blob([bytes], { type: "image/jpeg" }));
    if (
      metadata.color_temporal_keyframe === true
      && Number(metadata.color_keyframe_id || 0) > 0
    ) {
      const canvas = colorCanvas(width, height);
      const context = canvas.getContext("2d", { willReadFrequently: true });
      context.drawImage(bitmap, 0, 0, width, height);
      const rgba = context.getImageData(0, 0, width, height).data;
      state.temporalColorKeyframes.set(cameraKey, {
        id: Number(metadata.color_keyframe_id),
        width,
        height,
        rgb: rgbaToRgb(rgba),
      });
    }
    return { bitmap, pixels: null };
  }

  const packed = bytes;
  if (
    packed.length < 18
    || packed[0] !== 67
    || packed[1] !== 84
    || packed[2] !== 76
    || packed[3] !== 49
  ) {
    throw new Error("invalid temporal color header");
  }
  const view = new DataView(
    packed.buffer,
    packed.byteOffset,
    packed.byteLength,
  );
  const keyframeId = view.getUint32(4, true);
  const tileSize = view.getUint16(8, true);
  const tileRows = view.getUint16(10, true);
  const tileCols = view.getUint16(12, true);
  const changedCount = view.getUint32(14, true);
  const keyframe = state.temporalColorKeyframes.get(cameraKey);
  if (
    !keyframe
    || keyframe.id !== keyframeId
    || keyframe.width !== width
    || keyframe.height !== height
  ) {
    throw new Error(`missing temporal color keyframe ${keyframeId}`);
  }
  if (
    tileSize < 8
    || tileSize > 32
    || tileRows !== Math.ceil(height / tileSize)
    || tileCols !== Math.ceil(width / tileSize)
  ) {
    throw new Error("invalid temporal color tile geometry");
  }
  const tileCount = tileRows * tileCols;
  const changedBitsLength = Math.ceil(tileCount / 8);
  const jpegOffset = 18 + changedBitsLength;
  if (changedCount > tileCount || jpegOffset > packed.length) {
    throw new Error("truncated temporal color tiles");
  }
  const persistentColor = (
    metadata.color_temporal_reference_mode === "persistent_v1"
    && Number(metadata.color_frame_id || 0) > 0
  );
  if (changedCount === 0) {
    if (persistentColor) {
      state.temporalColorKeyframes.set(cameraKey, {
        id: Number(metadata.color_frame_id),
        width,
        height,
        rgb: null,
      });
      return { bitmap: null, pixels: null, tileUpdates: [] };
    }
    const rgb = new Uint8Array(keyframe.rgb);
    return { bitmap: null, pixels: rgb };
  }
  if (jpegOffset === packed.length) {
    throw new Error("missing temporal color atlas");
  }

  const atlasCols = Math.min(tileCols, changedCount);
  const atlasRows = Math.ceil(changedCount / atlasCols);
  const atlasBitmap = await createImageBitmap(
    new Blob([packed.slice(jpegOffset)], { type: "image/jpeg" }),
  );
  if (
    atlasBitmap.width !== atlasCols * tileSize
    || atlasBitmap.height !== atlasRows * tileSize
  ) {
    atlasBitmap.close();
    throw new Error("invalid temporal color atlas dimensions");
  }
  const atlasCanvas = colorCanvas(atlasBitmap.width, atlasBitmap.height);
  const atlasContext = atlasCanvas.getContext("2d", { willReadFrequently: true });
  atlasContext.drawImage(atlasBitmap, 0, 0);
  atlasBitmap.close();
  const atlas = atlasContext.getImageData(
    0,
    0,
    atlasCanvas.width,
    atlasCanvas.height,
  ).data;
  const rgb = persistentColor ? null : new Uint8Array(keyframe.rgb);
  const tileUpdates = persistentColor ? [] : null;
  let changedIndex = 0;
  for (let tileIndex = 0; tileIndex < tileCount; tileIndex += 1) {
    if ((packed[18 + (tileIndex >> 3)] & (1 << (tileIndex & 7))) === 0) continue;
    if (changedIndex >= changedCount) {
      throw new Error("temporal color tile count overflow");
    }
    const targetY = Math.floor(tileIndex / tileCols) * tileSize;
    const targetX = (tileIndex % tileCols) * tileSize;
    const sourceY = Math.floor(changedIndex / atlasCols) * tileSize;
    const sourceX = (changedIndex % atlasCols) * tileSize;
    const copyHeight = Math.min(tileSize, height - targetY);
    const copyWidth = Math.min(tileSize, width - targetX);
    const updatePixels = persistentColor
      ? new Uint8Array(copyWidth * copyHeight * 3)
      : null;
    for (let localY = 0; localY < copyHeight; localY += 1) {
      for (let localX = 0; localX < copyWidth; localX += 1) {
        const source = (
          (sourceY + localY) * atlasCanvas.width + sourceX + localX
        ) * 4;
        const target = persistentColor
          ? (localY * copyWidth + localX) * 3
          : ((targetY + localY) * width + targetX + localX) * 3;
        const destination = persistentColor ? updatePixels : rgb;
        destination[target] = atlas[source];
        destination[target + 1] = atlas[source + 1];
        destination[target + 2] = atlas[source + 2];
      }
    }
    if (persistentColor) {
      tileUpdates.push({
        x: targetX,
        y: targetY,
        width: copyWidth,
        height: copyHeight,
        pixels: updatePixels,
      });
    }
    changedIndex += 1;
  }
  if (changedIndex !== changedCount) {
    throw new Error("temporal color tile count mismatch");
  }
  if (persistentColor) {
    state.temporalColorKeyframes.set(cameraKey, {
      id: Number(metadata.color_frame_id),
      width,
      height,
      rgb: null,
    });
    return { bitmap: null, pixels: null, tileUpdates };
  }
  return { bitmap: null, pixels: rgb };
}

  function repairSmallDepthHoles(depth, width, height) {
  // Preserve thin geometry lost during RGB-D downsampling without bridging
  // broad gaps. A hole is filled only when opposite neighbors agree in depth.
    const repaired = new Uint16Array(depth);
    const rowEnd = height - 1;
    const colEnd = width - 1;
    const pairOffsets = [
      -1, 1,
      -width, width,
      -width - 1, width + 1,
      -width + 1, width - 1,
    ];
    for (let y = 1; y < rowEnd; y += 1) {
      const row = y * width;
      for (let x = 1; x < colEnd; x += 1) {
        const index = row + x;
        if (depth[index] !== 0) continue;

        let sum = 0;
        let votes = 0;
        for (let pair = 0; pair < pairOffsets.length; pair += 2) {
          const a = depth[index + pairOffsets[pair]];
          const b = depth[index + pairOffsets[pair + 1]];
          if (a === 0 || b === 0) continue;
          const toleranceMm = Math.max(45, Math.round((a + b) * 0.0125));
        if (Math.abs(a - b) > toleranceMm) continue;
        sum += (a + b) * 0.5;
        votes += 1;
      }
      if (votes > 0) repaired[index] = Math.round(sum / votes);
    }
  }
  return repaired;
}

async function decodeCamera(payload) {
  const [color, rawDepth] = await Promise.all([
    decodeColor(payload.color, payload.metadata),
    inflateDepth(payload.depth, payload.metadata),
  ]);
  const width = Number(payload.metadata.width || 0);
  const height = Number(payload.metadata.height || 0);
  if (width <= 0 || height <= 0 || rawDepth.length < width * height) {
    if (color.bitmap) color.bitmap.close();
    throw new Error(`invalid depth dimensions ${width}x${height}`);
  }
  const depth = repairSmallDepthHoles(rawDepth, width, height);
  return {
    metadata: payload.metadata,
    width,
    height,
    depth,
    bitmap: color.bitmap,
    colorPixels: color.pixels,
    colorTileUpdates: color.tileUpdates,
    modelMatrix: new Float32Array(payload.metadata.transform),
  };
}

function deleteCameraTextures(camera) {
  if (!state.gl) return;
  if (camera.depthTexture) state.gl.deleteTexture(camera.depthTexture);
  if (camera.colorTexture) state.gl.deleteTexture(camera.colorTexture);
  if (camera.meshIndexBuffer) state.gl.deleteBuffer(camera.meshIndexBuffer);
}

function createMeshIndexBuffer(gl, width, height) {
  const indices = new Uint32Array((width - 1) * (height - 1) * 6);
  let offset = 0;
  for (let y = 0; y < height - 1; y += 1) {
    for (let x = 0; x < width - 1; x += 1) {
      const topLeft = y * width + x;
      const topRight = topLeft + 1;
      const bottomLeft = topLeft + width;
      const bottomRight = bottomLeft + 1;
      indices[offset++] = topLeft;
      indices[offset++] = topRight;
      indices[offset++] = bottomLeft;
      indices[offset++] = topRight;
      indices[offset++] = bottomRight;
      indices[offset++] = bottomLeft;
    }
  }
  const buffer = gl.createBuffer();
  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, buffer);
  gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, indices, gl.STATIC_DRAW);
  return { buffer, count: indices.length };
}

function ensureMeshIndexBuffer(camera) {
  if (camera.meshIndexBuffer || !state.gl || camera.width <= 1 || camera.height <= 1) return;
  const mesh = createMeshIndexBuffer(state.gl, camera.width, camera.height);
  camera.meshIndexBuffer = mesh.buffer;
  camera.meshIndexCount = mesh.count;
}

function uploadCamera(decoded) {
  const gl = state.gl;
  const previous = state.cameras.find((camera) => camera.metadata.id === decoded.metadata.id);
  const camera = previous || {};
  if (!previous || previous.width !== decoded.width || previous.height !== decoded.height) {
    deleteCameraTextures(camera);
    camera.depthTexture = gl.createTexture();
    camera.colorTexture = gl.createTexture();
    camera.meshIndexBuffer = null;
    camera.meshIndexCount = 0;

    gl.bindTexture(gl.TEXTURE_2D, camera.depthTexture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texStorage2D(gl.TEXTURE_2D, 1, gl.R16UI, decoded.width, decoded.height);

    gl.bindTexture(gl.TEXTURE_2D, camera.colorTexture);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.texStorage2D(gl.TEXTURE_2D, 1, gl.RGB8, decoded.width, decoded.height);
  }
  Object.assign(camera, decoded);
  camera.lastUpdateMs = performance.now();
  camera.intrinsicsArray = new Float32Array(camera.metadata.intrinsics);

  gl.bindTexture(gl.TEXTURE_2D, camera.depthTexture);
  gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, camera.width, camera.height, gl.RED_INTEGER, gl.UNSIGNED_SHORT, camera.depth);

  gl.bindTexture(gl.TEXTURE_2D, camera.colorTexture);
  gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);
  gl.pixelStorei(gl.UNPACK_ALIGNMENT, 1);
  updateManualRgbBase(camera);
  if (camera.bitmap) {
    gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, gl.RGB, gl.UNSIGNED_BYTE, camera.bitmap);
    camera.bitmap.close();
  } else if (Array.isArray(camera.colorTileUpdates)) {
    for (const update of camera.colorTileUpdates) {
      gl.texSubImage2D(
        gl.TEXTURE_2D,
        0,
        update.x,
        update.y,
        update.width,
        update.height,
        gl.RGB,
        gl.UNSIGNED_BYTE,
        update.pixels,
      );
    }
  } else if (camera.colorPixels) {
    gl.texSubImage2D(
      gl.TEXTURE_2D,
      0,
      0,
      0,
      camera.width,
      camera.height,
      gl.RGB,
      gl.UNSIGNED_BYTE,
      camera.colorPixels,
    );
  }
  delete camera.bitmap;
  delete camera.colorPixels;
  delete camera.colorTileUpdates;
  renderManualCalibrationPreview(camera);
  return camera;
}

function rgbToRgba(rgb) {
  const rgba = new Uint8ClampedArray(Math.floor(rgb.length / 3) * 4);
  for (let source = 0, target = 0; source < rgb.length; source += 3, target += 4) {
    rgba[target] = rgb[source];
    rgba[target + 1] = rgb[source + 1];
    rgba[target + 2] = rgb[source + 2];
    rgba[target + 3] = 255;
  }
  return rgba;
}

function updateManualRgbBase(camera) {
  const visible = state.manualRgbCanvas;
  if (!(visible instanceof HTMLCanvasElement)) return;
  const model = String(camera.metadata.model || "").toLowerCase();
  if (state.cameras.length > 0 && !model.includes("455")) return;
  let base = state.manualRgbBaseCanvas;
  if (!(base instanceof HTMLCanvasElement)) {
    base = document.createElement("canvas");
    state.manualRgbBaseCanvas = base;
  }
  if (base.width !== camera.width || base.height !== camera.height) {
    base.width = camera.width;
    base.height = camera.height;
  }
  const context = base.getContext("2d", { alpha: false });
  if (camera.bitmap) {
    context.drawImage(camera.bitmap, 0, 0, camera.width, camera.height);
  } else if (Array.isArray(camera.colorTileUpdates)) {
    for (const update of camera.colorTileUpdates) {
      context.putImageData(
        new ImageData(rgbToRgba(update.pixels), update.width, update.height),
        update.x,
        update.y,
      );
    }
  } else if (camera.colorPixels) {
    context.putImageData(
      new ImageData(rgbToRgba(camera.colorPixels), camera.width, camera.height),
      0,
      0,
    );
  }
}

function renderManualCalibrationPreview(camera) {
  const visible = state.manualRgbCanvas;
  const crop = state.manualCropCanvas;
  const base = state.manualRgbBaseCanvas;
  if (!(visible instanceof HTMLCanvasElement) || !(base instanceof HTMLCanvasElement)) return;
  if (visible.closest("[hidden]")) return;
  const model = String(camera.metadata.model || "").toLowerCase();
  if (state.cameras.length > 0 && !model.includes("455")) return;
  visible.width = camera.width;
  visible.height = camera.height;
  const context = visible.getContext("2d", { alpha: false });
  context.drawImage(base, 0, 0);
  let overlay = state.manualOverlayCanvas;
  if (!(overlay instanceof HTMLCanvasElement)) {
    overlay = document.createElement("canvas");
    state.manualOverlayCanvas = overlay;
  }
  if (overlay.width !== camera.width || overlay.height !== camera.height) {
    overlay.width = camera.width;
    overlay.height = camera.height;
  }
  const cameraMatrix = Array.isArray(camera.metadata.transform)
    && camera.metadata.transform.length === 16
    ? new Float32Array(camera.metadata.transform)
    : null;
  if (!cameraMatrix || !Array.isArray(camera.metadata.intrinsics)) return;
  const opacity = Math.max(0.05, Math.min(0.9, Number(state.settings.manual_overlay_opacity ?? 0.3)));
  const distalNames = ["wrist_link", "gripper_link", "moving_jaw_so101_v1_link"];
  const signature = JSON.stringify([
    camera.width,
    camera.height,
    camera.metadata.transform,
    camera.metadata.intrinsics,
    ...distalNames.map((name) => state.robotTransforms[name] || null),
  ]);
  let projectedBounds = state.manualOverlayBounds;
  if (signature !== state.manualOverlaySignature) {
    const overlayContext = overlay.getContext("2d");
    overlayContext.clearRect(0, 0, overlay.width, overlay.height);
    const worldToCamera = inverseRigidTransform(cameraMatrix);
    const [fx, fy, cx, cy] = camera.metadata.intrinsics.map(Number);
    projectedBounds = { minX: camera.width, minY: camera.height, maxX: 0, maxY: 0, count: 0 };
    const project = (matrix, positions, index) => {
      const local = [positions[index * 3], positions[index * 3 + 1], positions[index * 3 + 2]];
      const point = transformPoint(worldToCamera, transformPoint(matrix, local));
      const depth = -point[2];
      if (depth <= 0.01) return null;
      return [fx * point[0] / depth + cx, -fy * point[1] / depth + cy];
    };
    for (const name of distalNames) {
      const primitives = state.robotMeshes[name] || [];
      const globalMatrix = transformDictionaryMatrix(state.robotTransforms[name]);
      if (!globalMatrix) continue;
      overlayContext.fillStyle = name === "moving_jaw_so101_v1_link" ? "#00e1ff" : "#ffd200";
      for (const primitive of primitives) {
        const positions = primitive.cpuPositions;
        if (!positions) continue;
        const matrix = multiplyMatrix(globalMatrix, primitive.localMatrix);
        const indices = primitive.cpuIndices;
        const triangleCount = Math.floor((indices ? indices.length : positions.length / 3) / 3);
        // This is a calibration view: preserve every modeled triangle so the
        // silhouette remains continuous and can be compared precisely.
        for (let triangle = 0; triangle < triangleCount; triangle += 1) {
          const vertex = triangle * 3;
          const a = project(matrix, positions, indices ? indices[vertex] : vertex);
          const b = project(matrix, positions, indices ? indices[vertex + 1] : vertex + 1);
          const c = project(matrix, positions, indices ? indices[vertex + 2] : vertex + 2);
          if (!a || !b || !c) continue;
          overlayContext.beginPath();
          overlayContext.moveTo(a[0], a[1]); overlayContext.lineTo(b[0], b[1]); overlayContext.lineTo(c[0], c[1]); overlayContext.closePath();
          overlayContext.fill();
          if (name !== "wrist_link") {
            for (const point of [a, b, c]) {
              projectedBounds.minX = Math.min(projectedBounds.minX, point[0]);
              projectedBounds.minY = Math.min(projectedBounds.minY, point[1]);
              projectedBounds.maxX = Math.max(projectedBounds.maxX, point[0]);
              projectedBounds.maxY = Math.max(projectedBounds.maxY, point[1]);
              projectedBounds.count += 1;
            }
          }
        }
      }
    }
    state.manualOverlaySignature = signature;
    state.manualOverlayBounds = projectedBounds;
  }
  context.globalAlpha = opacity;
  context.drawImage(overlay, 0, 0);
  context.globalAlpha = 1.0;
  if (!(crop instanceof HTMLCanvasElement) || projectedBounds.count === 0) return;
  const centerX = (projectedBounds.minX + projectedBounds.maxX) * 0.5;
  const centerY = (projectedBounds.minY + projectedBounds.maxY) * 0.5;
  const extent = Math.max(projectedBounds.maxX - projectedBounds.minX, projectedBounds.maxY - projectedBounds.minY, 80) * 1.5;
  const sourceX = Math.max(0, Math.min(camera.width - extent, centerX - extent * 0.5));
  const sourceY = Math.max(0, Math.min(camera.height - extent, centerY - extent * 0.5));
  crop.width = 420;
  crop.height = 420;
  const cropContext = crop.getContext("2d", { alpha: false });
  cropContext.imageSmoothingEnabled = false;
  cropContext.drawImage(visible, sourceX, sourceY, Math.min(extent, camera.width), Math.min(extent, camera.height), 0, 0, crop.width, crop.height);
}

async function decodeNewestPackets() {
  if (state.decoding) return;
  state.decoding = true;
  let persistentFrame = false;
  try {
    if (!state.running || state.pendingFrameQueue.length === 0) return;
    const frame = state.pendingFrameQueue.shift();
    const payloads = frame.payloads;
    const metadata = frame.metadata;
    persistentFrame = (
      metadata.temporal_reference_mode === "persistent_v1"
    );
    const decodeStarted = performance.now();
    const robot = metadata.robot;
    if (robot && robot.available !== false && robot.links && typeof robot.links === "object") {
      state.robotTransforms = robot.links;
      state.robotTransformsUpdatedMs = performance.now();
    } else if (robot && robot.available === false
               && performance.now() - state.robotTransformsUpdatedMs > 3000) {
      state.robotTransforms = {};
    }
    state.stats.robotTransforms = Object.keys(state.robotTransforms).length;
    const decoded = await Promise.all(payloads.map(decodeCamera));
    if (!state.running) {
      for (const camera of decoded) {
        if (camera.bitmap) camera.bitmap.close();
      }
      return;
    }
    const updatedCameras = decoded.map(uploadCamera);
    for (const camera of updatedCameras) {
      const id = String(camera.metadata.id || camera.metadata.model || "camera");
      state.stats.cameraFrames[id] = Number(state.stats.cameraFrames[id] || 0) + 1;
      state.stats.cameraEncodeMs[id] = Number(camera.metadata.encode_ms || 0);
      if (!state.cameras.includes(camera)) state.cameras.push(camera);
    }
    const staleBefore = performance.now() - 5000;
    state.cameras = state.cameras.filter((camera) => {
      if (Number(camera.lastUpdateMs || 0) >= staleBefore) return true;
      deleteCameraTextures(camera);
      return false;
    });
    state.cameras.sort((left, right) => Number(left.metadata.priority || 0) - Number(right.metadata.priority || 0));
    const viewerTransform = metadata.viewer && metadata.viewer.transform;
    if (!state.baseViewer && Array.isArray(viewerTransform) && viewerTransform.length === 16) {
      state.baseViewer = {
        transform: new Float32Array(viewerTransform),
        fov: Number(metadata.viewer.fov || 60),
        near: Math.max(0.01, Number(metadata.viewer.near || 0.05)),
        far: Math.max(2, Number(metadata.viewer.far || 100)),
      };
      const origin = state.baseViewer.transform.slice(12, 15);
      const back = state.baseViewer.transform.slice(8, 11);
      state.focusWorld = add(origin, scale(back, -state.settings.focus_distance));
    }
    state.stats.decoded += 1;
    state.stats.cameras = state.cameras.length;
    state.stats.width = Math.max(...state.cameras.map((camera) => camera.width));
    state.stats.height = Math.max(...state.cameras.map((camera) => camera.height));
    state.stats.captureUnixMs = Number(metadata.capture_unix_ms || Date.now());
    state.stats.captureAgeMs = Math.max(0, Date.now() - state.stats.captureUnixMs);
    const decodeElapsed = performance.now() - decodeStarted;
    state.stats.decodeMs = state.stats.decodeMs > 0
      ? state.stats.decodeMs * 0.8 + decodeElapsed * 0.2
      : decodeElapsed;
  } catch (error) {
    state.stats.status = `decode error: ${error && error.message ? error.message : error}`;
    if (persistentFrame && state.socket) {
      const options = state.socketOptions || {};
      state.pendingFrameQueue.length = 0;
      state.temporalDepthKeyframes.clear();
      state.temporalColorKeyframes.clear();
      state.persistentMetadata = null;
      state.stats.connected = false;
      state.stats.status = "persistent RGB-D reference lost; recovering";
      closeRgbdSocket();
      state.reconnectTimer = window.setTimeout(() => {
        state.reconnectTimer = 0;
        startConfiguredRgbdFallback(options);
      }, 100);
    }
  } finally {
    state.decoding = false;
    if (state.pendingFrameQueue.length > 0 && state.running) {
      setTimeout(decodeNewestPackets, 0);
    }
  }
}

function currentHeadDelta() {
  const latest = window.godotWebcamTrackerLatest || {};
  if (latest.active !== true) {
    state.lastHeadActive = false;
    return [0, 0, 0];
  }
  if (!state.headBaseline || !state.lastHeadActive) {
    state.headBaseline = { x: Number(latest.x || 0), y: Number(latest.y || 0), z: Number(latest.z || 0) };
  }
  state.lastHeadActive = true;
  return [
    (Number(latest.x || 0) - state.headBaseline.x) * 0.01,
    -(Number(latest.y || 0) - state.headBaseline.y) * 0.01,
    (Number(latest.z || 0) - state.headBaseline.z) * 0.01,
  ];
}

function translatedViewer(base, delta) {
  const result = new Float32Array(base);
  for (let row = 0; row < 3; row += 1) {
    result[12 + row] = base[12 + row]
      + base[row] * delta[0]
      + base[4 + row] * delta[1]
      + base[8 + row] * delta[2];
  }
  return result;
}

function robotBasePosition() {
  const matrix = transformDictionaryMatrix(state.robotTransforms.base_link);
  return matrix ? [matrix[12], matrix[13], matrix[14]] : null;
}

function verticalLiftForFocus(focus) {
  const robotBase = robotBasePosition();
  if (!robotBase) return Number(state.settings.focus_vertical_offset || 0);
  return robotBase[1] + Number(state.settings.focus_vertical_offset || 0) - focus[1];
}

function orbitViewer(base, headDelta) {
  const settings = state.settings;
  const baseBack = normalize([base[8], base[9], base[10]]);
  const worldUp = [0, 1, 0];
  const projectedBack = add(baseBack, scale(worldUp, -dot(baseBack, worldUp)));
  const levelBack = Math.hypot(...projectedBack) > 0.001 ? normalize(projectedBack) : [0, 0, 1];
  const levelRight = normalize(cross(worldUp, levelBack));
  const anchorDelta = state.orbitAnchorHeadDelta || [0, 0, 0];
  const relativeHead = state.orbitAnchorDirection
    ? [headDelta[0] - anchorDelta[0], headDelta[1] - anchorDelta[1], headDelta[2] - anchorDelta[2]]
    : headDelta;
  const baseDirection = state.orbitAnchorDirection || levelBack;
  const baseRight = normalize(cross(worldUp, baseDirection));
  const yaw = (
    clamp(relativeHead[0] * 100 * settings.yaw_gain, -settings.max_yaw, settings.max_yaw)
    + state.manualOrbitYaw
  ) * Math.PI / 180;
  const trackedPitch = clamp(
    relativeHead[1] * 100 * settings.pitch_gain,
    -settings.max_pitch,
    settings.max_pitch,
  );
  const pitch = clamp(
    Number(settings.orbit_pitch_offset || 0) + trackedPitch + state.manualOrbitPitch,
    -80,
    80,
  ) * Math.PI / 180;
  let direction = rotateAroundAxis(baseDirection, worldUp, -yaw);
  const pitchAxis = rotateAroundAxis(baseRight, worldUp, -yaw);
  direction = normalize(rotateAroundAxis(direction, pitchAxis, pitch));
  let distance = Number(settings.orbit_distance || 0.65);
  if (settings.dolly_enabled) distance += (relativeHead[2] * 100) * Number(settings.dolly_gain || 0);
  distance += state.manualDolly;
  distance = clamp(distance, Number(settings.min_distance || 0.2), Number(settings.max_distance || 3));
  const robotBase = robotBasePosition();
  const robotFocus = robotBase ? [robotBase[0], robotBase[1] + 0.12, robotBase[2]] : null;
  const baseFocus = state.focusPicked && state.focusWorld
    ? state.focusWorld
    : robotFocus || state.focusWorld || add(base.slice(12, 15), scale(levelBack, -settings.focus_distance));
  const focus = add(baseFocus, state.manualPan);
  const virtualHeadOffset = scale(worldUp, verticalLiftForFocus(focus));
  const cameraPosition = add(add(focus, scale(direction, distance)), virtualHeadOffset);
  return lookAtTransform(cameraPosition, focus, worldUp);
}

function attachPointerNavigation() {
  const canvas = state.canvas;
  if (!canvas || state.pointerNavigation) return;
  const drag = { active: false, id: -1, x: 0, y: 0, mode: "orbit" };
  const pointerDown = (event) => {
    if (![0, 1, 2].includes(event.button)) return;
    drag.active = true;
    drag.id = event.pointerId;
    drag.x = event.clientX;
    drag.y = event.clientY;
    drag.mode = event.button !== 0 || event.shiftKey ? "pan" : "orbit";
    state.settings.inspect_enabled = true;
    canvas.setPointerCapture(event.pointerId);
    event.preventDefault();
  };
  const pointerMove = (event) => {
    if (!drag.active || event.pointerId !== drag.id) return;
    const dx = event.clientX - drag.x;
    const dy = event.clientY - drag.y;
    drag.x = event.clientX;
    drag.y = event.clientY;
    if (drag.mode === "orbit") {
      state.manualOrbitYaw = ((state.manualOrbitYaw + dx * 0.25 + 180) % 360) - 180;
      state.manualOrbitPitch = clamp(state.manualOrbitPitch + dy * 0.2, -80, 80);
    } else {
      const viewer = state.currentViewer || (state.baseViewer && state.baseViewer.transform);
      if (viewer) {
        const distance = clamp(
          Number(state.settings.orbit_distance || 0.65) + state.manualDolly,
          Number(state.settings.min_distance || 0.2),
          Number(state.settings.max_distance || 3),
        );
        const metersPerPixel = 2 * distance
          * Math.tan(Number(state.settings.fov || 60) * Math.PI / 360)
          / Math.max(1, canvas.clientHeight);
        const right = normalize([viewer[0], viewer[1], viewer[2]]);
        const up = normalize([viewer[4], viewer[5], viewer[6]]);
        state.manualPan = add(
          state.manualPan,
          add(scale(right, -dx * metersPerPixel), scale(up, dy * metersPerPixel)),
        );
      }
    }
    event.preventDefault();
  };
  const pointerUp = (event) => {
    if (event.pointerId !== drag.id) return;
    drag.active = false;
    drag.id = -1;
    if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
    event.preventDefault();
  };
  const wheel = (event) => {
    state.settings.inspect_enabled = true;
    const distance = Math.max(0.1, Number(state.settings.orbit_distance || 0.65) + state.manualDolly);
    state.manualDolly += event.deltaY * distance * 0.0015;
    state.manualDolly = clamp(
      state.manualDolly,
      Number(state.settings.min_distance || 0.2) - Number(state.settings.orbit_distance || 0.65),
      Number(state.settings.max_distance || 3) - Number(state.settings.orbit_distance || 0.65),
    );
    event.preventDefault();
  };
  const contextMenu = (event) => event.preventDefault();
  const doubleClick = (event) => {
    recenterGodotHybridRenderer();
    event.preventDefault();
  };
  canvas.addEventListener("pointerdown", pointerDown);
  canvas.addEventListener("pointermove", pointerMove);
  canvas.addEventListener("pointerup", pointerUp);
  canvas.addEventListener("pointercancel", pointerUp);
  canvas.addEventListener("wheel", wheel, { passive: false });
  canvas.addEventListener("contextmenu", contextMenu);
  canvas.addEventListener("dblclick", doubleClick);
  state.pointerNavigation = { pointerDown, pointerMove, pointerUp, wheel, contextMenu, doubleClick };
}

function detachPointerNavigation() {
  const canvas = state.canvas;
  const handlers = state.pointerNavigation;
  if (!canvas || !handlers) return;
  canvas.removeEventListener("pointerdown", handlers.pointerDown);
  canvas.removeEventListener("pointermove", handlers.pointerMove);
  canvas.removeEventListener("pointerup", handlers.pointerUp);
  canvas.removeEventListener("pointercancel", handlers.pointerUp);
  canvas.removeEventListener("wheel", handlers.wheel);
  canvas.removeEventListener("contextmenu", handlers.contextMenu);
  canvas.removeEventListener("dblclick", handlers.doubleClick);
  state.pointerNavigation = null;
}

function resizeCanvas() {
  const canvas = state.canvas;
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const width = Math.max(1, Math.round(canvas.clientWidth * dpr));
  const height = Math.max(1, Math.round(canvas.clientHeight * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  return { width, height, dpr };
}

function render() {
  if (!state.running || !state.gl || !state.baseViewer) return;
  const gl = state.gl;
  const dimensions = resizeCanvas();
  const headDelta = currentHeadDelta();
  state.lastHeadDelta = headDelta;
  const viewerTransform = state.settings.inspect_enabled
    ? orbitViewer(state.baseViewer.transform, headDelta)
    : translatedViewer(state.baseViewer.transform, headDelta);
  state.currentViewer = viewerTransform;
  const fov = Number(state.settings.fov || state.baseViewer.fov || 60);
  const projection = perspectiveMatrix(fov, dimensions.width / dimensions.height, state.baseViewer.near, state.baseViewer.far);
  const viewProjection = multiplyMatrix(projection, inverseRigidTransform(viewerTransform));
  state.viewProjection = viewProjection;

  gl.viewport(0, 0, dimensions.width, dimensions.height);
  if (state.settings.white_background_enabled) {
    gl.clearColor(1, 1, 1, 1);
  } else {
    gl.clearColor(0.03, 0.035, 0.04, 1);
  }
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  gl.enable(gl.DEPTH_TEST);
  gl.depthFunc(gl.LEQUAL);
  gl.disable(gl.BLEND);
  gl.useProgram(state.program);
  gl.uniformMatrix4fv(state.uniforms.viewProjection, false, viewProjection);
  const maskCapsules = state.settings.robot_overlay_enabled && state.settings.robot_overlay_mask_scanned_arm
    ? robotMaskCapsules()
    : null;
  gl.uniform1i(state.uniforms.robotMaskEnabled, maskCapsules ? 1 : 0);
  if (maskCapsules) {
    gl.uniform4fv(state.uniforms.maskCapsuleA, maskCapsules.starts);
    gl.uniform4fv(state.uniforms.maskCapsuleB, maskCapsules.ends);
  }
  const meshMode = state.settings.geometry_mode === "mesh";
  gl.uniform1f(state.uniforms.meshDepthDelta, Number(state.settings.mesh_depth_delta || 0.045));
  gl.uniform1f(state.uniforms.meshMaxEdge, Number(state.settings.mesh_max_edge || 0.08));
  const fusionReference = state.cameras.find((camera) => String(camera.metadata.model || "").includes("435")) || null;

  for (const camera of state.cameras) {
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, camera.depthTexture);
    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_2D, camera.colorTexture);
    const fuseColor = fusionReference && fusionReference !== camera;
    gl.uniform1i(state.uniforms.fusionEnabled, fuseColor ? 1 : 0);
    if (fuseColor) {
      gl.activeTexture(gl.TEXTURE2);
      gl.bindTexture(gl.TEXTURE_2D, fusionReference.depthTexture);
      gl.activeTexture(gl.TEXTURE3);
      gl.bindTexture(gl.TEXTURE_2D, fusionReference.colorTexture);
      gl.uniformMatrix4fv(state.uniforms.worldToFusion, false, inverseRigidTransform(fusionReference.modelMatrix));
      gl.uniform4fv(state.uniforms.fusionIntrinsics, fusionReference.intrinsicsArray);
      gl.uniform2i(state.uniforms.fusionSize, fusionReference.width, fusionReference.height);
      gl.uniform1f(state.uniforms.fusionDepthTolerance, 0.05);
    }
    gl.uniformMatrix4fv(state.uniforms.model, false, camera.modelMatrix);
    gl.uniform4fv(state.uniforms.intrinsics, camera.intrinsicsArray);
    gl.uniform2i(state.uniforms.size, camera.width, camera.height);
    const detailBoost = String(camera.metadata.model || "").includes("435") ? 1.15 : 1.0;
    const sampleFootprint = Math.max(
      dimensions.width / Math.max(1, camera.width),
      dimensions.height / Math.max(1, camera.height),
    );
    const pointSize = sampleFootprint * Number(state.settings.splat_fill || 1.15) * detailBoost;
    if (meshMode && camera.width > 1 && camera.height > 1) {
      gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, null);
      if (state.settings.point_underlay) {
        gl.uniform1i(state.uniforms.renderMode, 2);
        gl.uniform1f(state.uniforms.pointSize, clamp(pointSize * 0.85, 1, 10));
        gl.drawArrays(gl.POINTS, 0, camera.width * camera.height);
      }
      // Validate each emitted triangle from its actual three depth samples, matching Godot's mesh rule.
      gl.uniform1i(state.uniforms.renderMode, 1);
      gl.uniform1f(state.uniforms.pointSize, clamp(pointSize, 1, 10));
      gl.drawArrays(gl.TRIANGLES, 0, (camera.width - 1) * (camera.height - 1) * 6);
    } else {
      gl.uniform1i(state.uniforms.renderMode, 0);
      gl.uniform1f(state.uniforms.pointSize, clamp(pointSize, 1, 10));
      gl.drawArrays(gl.POINTS, 0, camera.width * camera.height);
    }
  }
  drawRobotOverlay(viewProjection);
  state.stats.renderFrames += 1;
}

function updateStats() {
  const now = performance.now();
  const elapsed = now - state.statsWindow.timestamp;
  if (elapsed < 500) return;
  state.stats.sourceFps = ((state.stats.decoded - state.statsWindow.decoded) * 1000) / elapsed;
  state.stats.renderFps = ((state.stats.renderFrames - state.statsWindow.renderFrames) * 1000) / elapsed;
  state.stats.bitrateKbps = ((state.stats.bytes - state.statsWindow.bytes) * 8) / elapsed;
  const cameraFps = {};
  for (const [id, count] of Object.entries(state.stats.cameraFrames)) {
    cameraFps[id] = ((Number(count) - Number(state.statsWindow.cameraFrames[id] || 0)) * 1000) / elapsed;
  }
  state.stats.cameraFps = cameraFps;
  if (state.stats.captureUnixMs > 0) {
    state.stats.captureAgeMs = Math.max(0, Date.now() - state.stats.captureUnixMs);
  }
  state.statsWindow = {
    timestamp: now,
    decoded: state.stats.decoded,
    bytes: state.stats.bytes,
    renderFrames: state.stats.renderFrames,
    cameraFrames: { ...state.stats.cameraFrames },
  };
}

function animationLoop(now) {
  if (!state.running) return;
  if (
    state.dataChannel
    && state.dataChannel.readyState === "open"
    && state.lastPacketMs > 0
    && now - state.lastPacketMs > 1500
  ) {
    state.stats.status = "RGB-D WebRTC stale; switching transport";
    try {
      state.dataChannel.close();
    } catch (_error) {
      startConfiguredRgbdFallback(state.socketOptions || {});
    }
  }
  if (
    state.socket
    && state.socket.readyState === WebSocket.OPEN
    && state.lastPacketMs > 0
    && now - state.lastPacketMs > 1500
  ) {
    const options = state.socketOptions || {};
    state.stats.connected = false;
    state.stats.status = "RGB-D WebSocket stale; reconnecting";
    closeRgbdSocket();
    state.reconnectTimer = window.setTimeout(() => {
      state.reconnectTimer = 0;
      startConfiguredRgbdFallback(options);
    }, 100);
  }
  // High-refresh displays were rendering hundreds of thousands of points at
  // 120-165 Hz and starving RGB-D decode/upload work on the same main thread.
  if (!state.lastRenderMs || now - state.lastRenderMs >= 15.5) {
    state.lastRenderMs = now;
    render();
  }
  updateStats();
  state.animationFrame = requestAnimationFrame(animationLoop);
}

function latestRgbdUrl(options, afterSequence, worker) {
  const query = new URLSearchParams({
    w: String(options.width || 384),
    fps: String(options.fps || 60),
    context_fps: String(options.contextFps || options.fps || 30),
    detail_fps: String(options.detailFps || options.fps || 60),
    q: String(options.quality || 78),
    client: state.httpClientId,
    after: String(Math.max(0, afterSequence || 0)),
    worker: String(worker),
  });
  return `/rgbd-latest?${query}`;
}

function waitMilliseconds(milliseconds) {
  return new Promise((resolve) => window.setTimeout(resolve, milliseconds));
}

function handleRgbdPacket(arrayBuffer, transport) {
  if (!(arrayBuffer instanceof ArrayBuffer)) return;
  state.lastPacketMs = performance.now();
  state.reconnectAttempt = 0;
  state.stats.received += 1;
  state.stats.bytes += arrayBuffer.byteLength;
  state.stats.transport = transport;
  let parsed;
  try {
    parsed = parsePacket(arrayBuffer);
  } catch (error) {
    state.stats.status = `packet error: ${error.message || error}`;
    return;
  }
  // Parallel HTTPS requests can complete in small bursts. Preserve complete
  // chronological frames instead of overwriting one while its predecessor is
  // decoding. The hard cap still prevents latency from accumulating.
  state.pendingFrameQueue.push(parsed);
  if (
    parsed.metadata.temporal_reference_mode === "persistent_v1"
    && state.pendingFrameQueue.length > 8
  ) {
    const options = state.socketOptions || {};
    state.pendingFrameQueue.length = 0;
    state.stats.dropped += 1;
    state.stats.status = "persistent RGB-D decode fell behind; recovering";
    closeRgbdSocket();
    state.reconnectTimer = window.setTimeout(() => {
      state.reconnectTimer = 0;
      startConfiguredRgbdFallback(options);
    }, 100);
    return;
  }
  if (
    parsed.metadata.temporal_reference_mode !== "persistent_v1"
    && state.pendingFrameQueue.length > 4
  ) {
    state.pendingFrameQueue.shift();
    state.stats.dropped += 1;
  }
  decodeNewestPackets();
}

function clearRgbdHttpReorder() {
  if (state.httpReorderTimer) clearTimeout(state.httpReorderTimer);
  state.httpReorderTimer = 0;
  state.httpExpectedSequence = 0;
  state.httpReorderPackets.clear();
  state.httpReorderGapStartedMs = 0;
}

function drainRgbdHttpReorder(forceGap = false) {
  if (!state.running || state.httpReorderPackets.size === 0) return;
  if (state.httpExpectedSequence <= 0) {
    if (!forceGap) return;
    state.httpExpectedSequence = Math.min(...state.httpReorderPackets.keys());
  }

  if (forceGap && !state.httpReorderPackets.has(state.httpExpectedSequence)) {
    const nextSequence = Math.min(...state.httpReorderPackets.keys());
    if (nextSequence > state.httpExpectedSequence) {
      state.stats.dropped += nextSequence - state.httpExpectedSequence;
      state.httpExpectedSequence = nextSequence;
    }
  }

  let emitted = 0;
  while (state.httpReorderPackets.has(state.httpExpectedSequence)) {
    const entry = state.httpReorderPackets.get(state.httpExpectedSequence);
    state.httpReorderPackets.delete(state.httpExpectedSequence);
    state.httpExpectedSequence += 1;
    handleRgbdPacket(entry.packet, "HTTPS latest-frame fallback");
    emitted += 1;
  }
  if (emitted > 0) state.httpReorderGapStartedMs = 0;

  if (state.httpReorderPackets.size === 0 || state.httpReorderTimer) return;
  if (!state.httpReorderGapStartedMs) state.httpReorderGapStartedMs = performance.now();
  const elapsed = performance.now() - state.httpReorderGapStartedMs;
  state.httpReorderTimer = window.setTimeout(() => {
    state.httpReorderTimer = 0;
    drainRgbdHttpReorder(true);
  }, Math.max(0, 20 - elapsed));
}

function enqueueRgbdHttpPacket(sequence, packet) {
  if (!Number.isFinite(sequence) || sequence <= 0) {
    handleRgbdPacket(packet, "HTTPS latest-frame fallback");
    return;
  }
  if (state.httpExpectedSequence > 0 && sequence < state.httpExpectedSequence) {
    state.stats.dropped += 1;
    return;
  }
  if (state.httpReorderPackets.has(sequence)) {
    state.stats.dropped += 1;
    return;
  }
  state.httpReorderPackets.set(sequence, { packet, receivedMs: performance.now() });
  drainRgbdHttpReorder(false);
  if (!state.httpReorderTimer) {
    state.httpReorderGapStartedMs = performance.now();
    state.httpReorderTimer = window.setTimeout(() => {
      state.httpReorderTimer = 0;
      drainRgbdHttpReorder(true);
    }, 20);
  }
}

function closeRgbdSocket() {
  if (state.reconnectTimer) clearTimeout(state.reconnectTimer);
  state.reconnectTimer = 0;
  const socket = state.socket;
  state.socket = null;
  if (!socket) return;
  socket.onopen = null;
  socket.onmessage = null;
  socket.onerror = null;
  socket.onclose = null;
  try {
    socket.close(1000, "newer RGB-D transport active");
  } catch (_error) {
    // A connection attempt can be between states.
  }
}

function rgbdSocketUrl(options) {
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  const query = new URLSearchParams({
    w: String(options.width || 384),
    fps: String(options.fps || 30),
    context_fps: String(options.contextFps || options.fps || 30),
    detail_fps: String(options.detailFps || options.fps || 30),
    q: String(options.quality || 78),
    temporal_depth: options.temporalDepth === false ? "0" : "1",
    temporal_color: options.temporalColor === false ? "0" : "1",
    persistent_reference: options.persistentReference === true ? "1" : "0",
    client: state.rgbdClientId,
  });
  return `${scheme}://${window.location.host}/rgbd-stream?${query}`;
}

function startRgbdWebSocket(options) {
  if (!state.running || document.hidden) return;
  stopRgbdHttpLatest();
  closeRgbdSocket();
  const socket = new WebSocket(rgbdSocketUrl(options));
  state.socket = socket;
  socket.binaryType = "arraybuffer";
  state.stats.connected = false;
  state.stats.status = "RGB-D WebSocket connecting";
  state.stats.transport = "WebSocket latest-frame";
  socket.onopen = () => {
    if (!state.running || state.socket !== socket) return;
    const now = performance.now();
    state.socketOpenedMs = now;
    state.lastPacketMs = now;
    state.stats.connected = true;
    state.stats.status = "RGB-D WebSocket connected";
  };
  socket.onmessage = (event) => {
    if (!state.running || state.socket !== socket || document.hidden) return;
    handleRgbdPacket(event.data, "WebSocket latest-frame");
  };
  socket.onerror = () => {
    if (state.socket === socket) state.stats.status = "RGB-D WebSocket error";
  };
  socket.onclose = (event) => {
    if (!state.running || state.socket !== socket) return;
    state.socket = null;
    state.stats.connected = false;
    if (event.code === 4001) {
      state.stats.status = "RGB-D superseded by a newer browser";
      state.stats.transport = "Superseded";
      return;
    }
    state.stats.status = "RGB-D WebSocket unavailable; using HTTPS latest-frame";
    state.stats.transport = "HTTPS latest-frame fallback";
    window.setTimeout(() => startRgbdHttpLatest(options), 100);
  };
}

function startConfiguredRgbdFallback(options) {
  if ((options || {}).transport === "websocket") {
    startRgbdWebSocket(options || {});
  } else {
    startRgbdHttpLatest(options || {});
  }
}

function stopRgbdHttpLatest() {
  state.httpGeneration += 1;
  for (const controller of state.httpControllers) controller.abort();
  state.httpControllers.clear();
  state.httpWorkers = 0;
  clearRgbdHttpReorder();
}

async function runRgbdHttpWorker(options, worker, generation) {
  try {
    while (
      state.running
      && generation === state.httpGeneration
      && !(state.dataChannel && state.dataChannel.readyState === "open")
    ) {
      if (document.hidden) {
        await waitMilliseconds(250);
        continue;
      }
      const controller = new AbortController();
      state.httpControllers.add(controller);
      const timeout = window.setTimeout(() => controller.abort(), 1500);
      try {
        const response = await fetch(
          latestRgbdUrl(options, state.httpSequence, worker),
          {
            method: "GET",
            credentials: "same-origin",
            cache: "no-store",
            signal: controller.signal,
          },
        );
        if (generation !== state.httpGeneration || !state.running) return;
        if (response.status === 204) {
          continue;
        }
        if (response.status === 409) {
          state.httpGeneration += 1;
          state.stats.connected = false;
          state.stats.status = "RGB-D superseded by a newer browser";
          state.stats.transport = "Superseded";
          return;
        }
        if (!response.ok) throw new Error(`HTTP ${response.status}`);
        const sequence = Number(response.headers.get("X-RGBD-Sequence") || 0);
        const packet = await response.arrayBuffer();
        if (generation !== state.httpGeneration || !state.running) return;
        state.httpSequence = Math.max(state.httpSequence, sequence);
        state.stats.connected = true;
        state.stats.status = "RGB-D HTTPS latest-frame connected";
        enqueueRgbdHttpPacket(sequence, packet);
      } catch (error) {
        if (
          generation !== state.httpGeneration
          || !state.running
          || (state.dataChannel && state.dataChannel.readyState === "open")
        ) {
          return;
        }
        if (error && error.name !== "AbortError") {
          state.stats.status = `RGB-D HTTPS retrying: ${error.message || error}`;
          await waitMilliseconds(100);
        }
      } finally {
        window.clearTimeout(timeout);
        state.httpControllers.delete(controller);
      }
    }
  } finally {
    if (generation === state.httpGeneration) {
      state.httpWorkers = Math.max(0, state.httpWorkers - 1);
    }
  }
}

function startRgbdHttpLatest(options) {
  if (!state.running) return;
  if (state.dataChannel && state.dataChannel.readyState === "open") return;
  if (state.httpWorkers > 0) return;
  if (document.hidden) {
    state.stats.connected = false;
    state.stats.status = "RGB-D paused while tab is hidden";
    return;
  }
  closeRgbdSocket();
  state.httpGeneration += 1;
  const generation = state.httpGeneration;
  state.httpClientId = state.rgbdClientId;
  state.httpSequence = 0;
  clearRgbdHttpReorder();
  // Several independent requests hide Cloudflare round-trip latency. Each
  // request can contain only one source frame, so increasing this pipeline
  // depth does not create an ordered frame backlog.
  // Six requests is the measured sweet spot with native predictive depth:
  // it sustains the full 30 Hz source cadence without the response reordering
  // and sharply higher tail latency seen with the former ten-worker pool.
  state.httpWorkers = 6;
  state.stats.connected = false;
  state.stats.status = "RGB-D HTTPS latest-frame connecting";
  state.stats.transport = "HTTPS latest-frame fallback";
  for (let worker = 0; worker < state.httpWorkers; worker += 1) {
    runRgbdHttpWorker(options, worker, generation);
  }
}

function attachRgbdDataChannel(channel, options) {
  if (!channel) return false;
  state.dataChannel = channel;
  channel.binaryType = "arraybuffer";
  const onOpen = () => {
    if (!state.running || state.dataChannel !== channel) return;
    if (state.dataChannelFallbackTimer) clearTimeout(state.dataChannelFallbackTimer);
    state.dataChannelFallbackTimer = 0;
    stopRgbdHttpLatest();
    closeRgbdSocket();
    const now = performance.now();
    state.socketOpenedMs = now;
    state.lastPacketMs = now;
    state.stats.connected = true;
    state.stats.status = "RGB-D WebRTC connected";
    state.stats.transport = "WebRTC latest-frame";
  };
  const onMessage = (event) => {
    if (!state.running || state.dataChannel !== channel || document.hidden) return;
    handleRgbdPacket(event.data, "WebRTC latest-frame");
  };
  const onClose = () => {
    if (!state.running || state.dataChannel !== channel) return;
    state.dataChannel = null;
    state.stats.connected = false;
    state.stats.status = "RGB-D WebRTC unavailable; switching transport";
    startConfiguredRgbdFallback(options);
  };
  const onError = () => {
    if (state.dataChannel === channel) {
      state.stats.status = "RGB-D WebRTC channel error";
    }
  };
  state.dataChannelHandlers = { onOpen, onMessage, onClose, onError };
  channel.addEventListener("open", onOpen);
  channel.addEventListener("message", onMessage);
  channel.addEventListener("close", onClose);
  channel.addEventListener("error", onError);
  if (channel.readyState === "open") onOpen();
  return true;
}

function handleRgbdVisibilityChange() {
  if (!state.running) return;
  if (document.hidden) {
    stopRgbdHttpLatest();
    closeRgbdSocket();
    state.stats.connected = false;
    state.stats.status = "RGB-D paused while tab is hidden";
    state.reconnectAttempt = 0;
    state.socketOpenedMs = 0;
    state.lastPacketMs = 0;
    return;
  }
  if (state.dataChannel && state.dataChannel.readyState === "open") {
    state.stats.connected = true;
    state.stats.status = "RGB-D WebRTC connected";
    state.stats.transport = "WebRTC latest-frame";
    state.lastPacketMs = performance.now();
    return;
  }
  state.reconnectAttempt = 0;
  state.stats.status = "RGB-D reconnecting";
  startConfiguredRgbdFallback(state.socketOptions || {});
}

export async function startGodotHybridRenderer(options = {}) {
  stopGodotHybridRenderer();
  const canvas = typeof options.canvas === "string" ? document.getElementById(options.canvas) : options.canvas;
  if (!(canvas instanceof HTMLCanvasElement)) throw new Error("hybrid renderer canvas is missing");
  const gl = canvas.getContext("webgl2", { alpha: false, antialias: false, depth: true, powerPreference: "high-performance" });
  if (!gl) throw new Error("WebGL2 is required for the hybrid RGB-D renderer");
  state.canvas = canvas;
  attachPointerNavigation();
  state.manualRgbCanvas = typeof options.manualRgbCanvas === "string"
    ? document.getElementById(options.manualRgbCanvas) : options.manualRgbCanvas;
  state.manualCropCanvas = typeof options.manualCropCanvas === "string"
    ? document.getElementById(options.manualCropCanvas) : options.manualCropCanvas;
  state.gl = gl;
  state.program = createProgram(gl);
  state.uniforms = findUniforms(gl, state.program);
  state.robotProgram = createProgram(gl, ROBOT_VERTEX_SHADER, ROBOT_FRAGMENT_SHADER);
  state.robotUniforms = findRobotUniforms(gl, state.robotProgram);
  state.robotVao = gl.createVertexArray();
  gl.useProgram(state.program);
  gl.uniform1i(state.uniforms.depth, 0);
  gl.uniform1i(state.uniforms.color, 1);
  gl.uniform1i(state.uniforms.fusionDepth, 2);
  gl.uniform1i(state.uniforms.fusionColor, 3);
  state.running = true;
  state.robotLoadToken += 1;
  const robotLoadToken = state.robotLoadToken;
  loadRobotMeshes(gl, robotLoadToken).catch((error) => {
    if (state.running && state.robotLoadToken === robotLoadToken) {
      console.warn("Hybrid robot overlay unavailable:", error);
    }
  });
  state.lastRenderMs = 0;
  state.stats = {
    ...state.stats,
    status: "connecting",
    connected: false,
    received: 0,
    decoded: 0,
    dropped: 0,
    bytes: 0,
    captureUnixMs: 0,
    captureAgeMs: -1,
    decodeMs: 0,
    renderFrames: 0,
    renderFps: 0,
    cameraFrames: {},
    cameraFps: {},
    cameraEncodeMs: {},
    robotModelsLoaded: 0,
    robotModelsExpected: Object.keys(ROBOT_MODEL_URLS).length,
    robotTransforms: 0,
    robotLoadError: "",
    transport: "--",
  };
  state.statsWindow = { timestamp: performance.now(), decoded: 0, bytes: 0, renderFrames: 0, cameraFrames: {} };
  state.socketOptions = { ...options };
  state.rgbdClientId = String(window.robotTeleopPageId || "").replace(/[^A-Za-z0-9_-]/g, "");
  if (!state.rgbdClientId) throw new Error("browser page identity is missing");
  state.reconnectAttempt = 0;
  state.socketOpenedMs = 0;
  state.lastPacketMs = 0;
  state.visibilityHandler = handleRgbdVisibilityChange;
  document.addEventListener("visibilitychange", state.visibilityHandler);
  if (attachRgbdDataChannel(options.dataChannel, state.socketOptions)) {
    state.dataChannelFallbackTimer = window.setTimeout(() => {
      state.dataChannelFallbackTimer = 0;
      if (!state.dataChannel || state.dataChannel.readyState !== "open") {
        startConfiguredRgbdFallback(state.socketOptions || {});
      }
    }, 1500);
  } else {
    startConfiguredRgbdFallback(state.socketOptions);
  }
  state.animationFrame = requestAnimationFrame(animationLoop);
  return true;
}

export function stopGodotHybridRenderer() {
  state.running = false;
  detachPointerNavigation();
  state.robotLoadToken += 1;
  if (state.reconnectTimer) clearTimeout(state.reconnectTimer);
  state.reconnectTimer = 0;
  if (state.dataChannelFallbackTimer) clearTimeout(state.dataChannelFallbackTimer);
  state.dataChannelFallbackTimer = 0;
  if (state.animationFrame) cancelAnimationFrame(state.animationFrame);
  state.animationFrame = 0;
  stopRgbdHttpLatest();
  closeRgbdSocket();
  if (state.dataChannel) {
    const handlers = state.dataChannelHandlers;
    if (handlers) {
      state.dataChannel.removeEventListener("open", handlers.onOpen);
      state.dataChannel.removeEventListener("message", handlers.onMessage);
      state.dataChannel.removeEventListener("close", handlers.onClose);
      state.dataChannel.removeEventListener("error", handlers.onError);
    }
    try {
      state.dataChannel.close();
    } catch (_error) {
      // Its owning peer may already be closed.
    }
  }
  state.dataChannel = null;
  state.dataChannelHandlers = null;
  state.socketOptions = null;
  state.reconnectAttempt = 0;
  state.socketOpenedMs = 0;
  state.lastPacketMs = 0;
  state.httpClientId = "";
  state.rgbdClientId = "";
  state.httpSequence = 0;
  if (state.visibilityHandler) {
    document.removeEventListener("visibilitychange", state.visibilityHandler);
    state.visibilityHandler = null;
  }
  state.pendingFrameQueue.length = 0;
  state.temporalDepthKeyframes.clear();
  state.temporalColorKeyframes.clear();
  state.persistentMetadata = null;
  for (const camera of state.cameras) deleteCameraTextures(camera);
  state.cameras = [];
  const gl = state.gl;
  deleteRobotMeshes(gl);
  if (gl && state.robotProgram) gl.deleteProgram(state.robotProgram);
  if (gl && state.robotVao) gl.deleteVertexArray(state.robotVao);
  if (gl && state.program) gl.deleteProgram(state.program);
  state.gl = null;
  state.program = null;
  state.uniforms = null;
  state.robotProgram = null;
  state.robotUniforms = null;
  state.robotVao = null;
  state.robotTransforms = {};
  state.baseViewer = null;
  state.currentViewer = null;
  state.focusWorld = null;
  state.focusPicked = false;
  state.orbitAnchorDirection = null;
  state.orbitAnchorHeadDelta = null;
  state.lastHeadDelta = [0, 0, 0];
  state.headBaseline = null;
  state.lastHeadActive = false;
  state.stats.connected = false;
}

export function setGodotHybridViewSettings(settings = {}) {
  const overlayStyle = settings.robot_overlay_style;
  state.settings = {
    ...state.settings,
    ...settings,
    robot_overlay_style: overlayStyle === "solid" ? "solid" : overlayStyle === "alignment"
      ? "alignment"
      : state.settings.robot_overlay_style,
  };
  if (settings.recenter) recenterGodotHybridRenderer();
}

export function recenterGodotHybridRenderer() {
  state.headBaseline = null;
  state.lastHeadActive = false;
  state.orbitAnchorDirection = null;
  state.orbitAnchorHeadDelta = null;
  state.focusPicked = false;
  state.manualOrbitYaw = 0;
  state.manualOrbitPitch = 0;
  state.manualDolly = 0;
  state.manualPan = [0, 0, 0];
}

export function focusGodotHybridRendererAt(clientX, clientY) {
  if (!state.canvas || !state.viewProjection || state.cameras.length === 0) return false;
  const rect = state.canvas.getBoundingClientRect();
  if (rect.width <= 0 || rect.height <= 0) return false;
  const targetX = ((clientX - rect.left) / rect.width) * 2 - 1;
  const targetY = 1 - ((clientY - rect.top) / rect.height) * 2;
  let bestPoint = null;
  let bestScore = 0.03 * 0.03;
  for (const camera of state.cameras) {
    const intrinsics = camera.metadata.intrinsics;
    const stride = Math.max(1, Math.floor(Math.max(camera.width, camera.height) / 180));
    const combined = multiplyMatrix(state.viewProjection, camera.modelMatrix);
    for (let y = 0; y < camera.height; y += stride) {
      for (let x = 0; x < camera.width; x += stride) {
        const depthMm = camera.depth[y * camera.width + x];
        if (!depthMm) continue;
        const z = depthMm * 0.001;
        const local = [
          (x - intrinsics[2]) * z / intrinsics[0],
          -(y - intrinsics[3]) * z / intrinsics[1],
          -z,
        ];
        const clip = transformPoint4(combined, [...local, 1]);
        if (clip[3] <= 0) continue;
        const ndcX = clip[0] / clip[3];
        const ndcY = clip[1] / clip[3];
        const score = (ndcX - targetX) ** 2 + (ndcY - targetY) ** 2;
        if (score < bestScore) {
          bestScore = score;
          bestPoint = transformPoint(camera.modelMatrix, local);
        }
      }
    }
  }
  if (!bestPoint) return false;
  // Only move the target. The orbit inputs remain unchanged, so the complete
  // camera rig translates without changing its viewing angle.
  state.focusWorld = bestPoint;
  state.focusPicked = true;
  state.settings.inspect_enabled = true;
  return true;
}

export function getGodotHybridRendererStatus() {
  const captureAgeMs = state.stats.captureUnixMs > 0
    ? Math.max(0, Date.now() - state.stats.captureUnixMs)
    : -1;
  const packetAgeMs = state.lastPacketMs > 0
    ? Math.max(0, performance.now() - state.lastPacketMs)
    : -1;
  const stalled = state.stats.connected === true && packetAgeMs > 1500;
  return { ...state.stats, captureAgeMs, packetAgeMs, stalled };
}

window.startGodotHybridRenderer = startGodotHybridRenderer;
window.stopGodotHybridRenderer = stopGodotHybridRenderer;
window.setGodotHybridViewSettings = setGodotHybridViewSettings;
window.recenterGodotHybridRenderer = recenterGodotHybridRenderer;
window.focusGodotHybridRendererAt = focusGodotHybridRendererAt;
window.getGodotHybridRendererStatus = getGodotHybridRendererStatus;
