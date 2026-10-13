// Shared by the live renderer and the camera-free GPU regression test.
export function cameraIsFresh(camera, now = performance.now()) {
  return camera.width > 1 && camera.height > 1 && now - camera.lastUpdateMs < 1000;
}

export function orderedCameras(cameras) {
  return [...cameras].sort((a, b) => Number(a.metadata.priority || 0) - Number(b.metadata.priority || 0)
    || String(a.metadata.id).localeCompare(String(b.metadata.id)));
}

export function chooseColorReference(cameras, now = performance.now()) {
  const live = orderedCameras(cameras).filter(camera => cameraIsFresh(camera, now));
  return live.find(camera => String(camera.metadata.model || "").includes("435")) || live[0] || null;
}

export function pointPixelScale(height, fov, intrinsics, fill) {
  const focal = height / (2 * Math.tan(fov * Math.PI / 360));
  return focal * fill / Math.max(1, Math.min(intrinsics[0], intrinsics[1]));
}

export class MultiCameraFusion {
  constructor(gl) { this.gl = gl; this.signature = ""; this.depth = null; this.data = null; this.uploads = []; }

  clear() {
    if (this.depth) this.gl.deleteTexture(this.depth);
    if (this.data) this.gl.deleteTexture(this.data);
    this.depth = null; this.data = null; this.signature = ""; this.uploads = [];
  }

  update(cameras, inverseTransform, now = performance.now()) {
    const gl = this.gl;
    const width = Math.max(...cameras.map(camera => camera.width));
    const height = Math.max(...cameras.map(camera => camera.height));
    if (cameras.length > gl.getParameter(gl.MAX_ARRAY_TEXTURE_LAYERS)
      || Math.max(width, height, cameras.length) > gl.getParameter(gl.MAX_TEXTURE_SIZE)) {
      throw new Error("Unified mesh exceeds this GPU's camera texture limits");
    }
    const signature = JSON.stringify(cameras.map(camera => [camera.metadata.id, camera.width, camera.height]));
    if (signature !== this.signature) {
      this.clear();
      this.depth = gl.createTexture();
      gl.activeTexture(gl.TEXTURE4);
      gl.bindTexture(gl.TEXTURE_2D_ARRAY, this.depth);
      gl.texParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
      gl.texParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
      gl.texParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
      gl.texParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
      gl.texStorage3D(gl.TEXTURE_2D_ARRAY, 1, gl.R16UI, width, height, cameras.length);
      this.data = gl.createTexture();
      gl.activeTexture(gl.TEXTURE5);
      gl.bindTexture(gl.TEXTURE_2D, this.data);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
      gl.texStorage2D(gl.TEXTURE_2D, 1, gl.RGBA32F, 5, cameras.length);
      if (gl.getError() !== gl.NO_ERROR) {
        this.clear();
        throw new Error("Unified mesh could not allocate camera textures on this GPU");
      }
      this.signature = signature;
    }
    const metadata = new Float32Array(cameras.length * 20);
    gl.activeTexture(gl.TEXTURE4);
    gl.bindTexture(gl.TEXTURE_2D_ARRAY, this.depth);
    gl.pixelStorei(gl.UNPACK_ALIGNMENT, 1);
    cameras.forEach((camera, index) => {
      if (this.uploads[index] !== camera.lastUpdateMs) {
        gl.texSubImage3D(gl.TEXTURE_2D_ARRAY, 0, 0, 0, index, camera.width, camera.height, 1,
          gl.RED_INTEGER, gl.UNSIGNED_SHORT, camera.depth);
        this.uploads[index] = camera.lastUpdateMs;
      }
      const m = inverseTransform(camera.modelMatrix);
      metadata.set([m[0], m[4], m[8], m[12], m[1], m[5], m[9], m[13], m[2], m[6], m[10], m[14],
        ...camera.intrinsicsArray, camera.width, camera.height, cameraIsFresh(camera, now) ? 1 : 0, 0], index * 20);
    });
    gl.activeTexture(gl.TEXTURE5);
    gl.bindTexture(gl.TEXTURE_2D, this.data);
    gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, 5, cameras.length, gl.RGBA, gl.FLOAT, metadata);
  }
}

export const FUSION_FRAGMENT_SHADER = `
uniform highp usampler2DArray u_rig_depth;
uniform highp sampler2D u_rig_data;
uniform highp int u_rig_enabled;
uniform highp int u_rig_index;
uniform float u_mesh_depth_delta;
uniform float u_mesh_max_edge;
uniform float u_fusion_depth_tolerance;
in vec3 v_world_position;

vec4 rig_data(int camera, int column) { return texelFetch(u_rig_data, ivec2(column, camera), 0); }
float rig_z(int camera, ivec2 pixel) {
  if (any(lessThan(pixel, ivec2(0))) || any(greaterThanEqual(pixel, ivec2(rig_data(camera, 4).xy)))) return 0.0;
  return float(texelFetch(u_rig_depth, ivec3(pixel, camera), 0).r) * 0.001;
}
bool rig_supported(int camera, ivec2 p, float z) {
  float tolerance = max(0.002, u_mesh_depth_delta * z * 0.5);
  const ivec2 offsets[4] = ivec2[4](ivec2(-1,0), ivec2(1,0), ivec2(0,-1), ivec2(0,1));
  int count = 0;
  for (int i = 0; i < 4; i++) {
    float neighbor = rig_z(camera, p + offsets[i]);
    if (neighbor > 0.0 && abs(neighbor - z) <= tolerance) count++;
  }
  return count >= 2;
}
vec3 rig_point(ivec2 pixel, float z, vec4 k) {
  return vec3((float(pixel.x) - k.z) * z / k.x, -(float(pixel.y) - k.w) * z / k.y, -z);
}
bool rig_covers(int camera, vec3 point) {
  vec4 info = rig_data(camera, 4);
  if (info.z < 0.5 || point.z >= -0.001) return false;
  vec4 k = rig_data(camera, 3);
  float z = -point.z;
  vec2 pixel = vec2(point.x * k.x / z + k.z, -point.y * k.y / z + k.w);
  if (any(lessThan(pixel, vec2(0))) || any(greaterThanEqual(pixel, info.xy - vec2(1)))) return false;
  float depth = rig_z(camera, ivec2(floor(pixel + vec2(0.5))));
  if (depth <= 0.0 || abs(depth - z) > u_fusion_depth_tolerance) return false;
  ivec2 cell = ivec2(floor(pixel));
  ivec2 a = fract(pixel.x) + fract(pixel.y) > 1.0 ? cell + ivec2(1) : cell;
  ivec2 b = cell + ivec2(1,0);
  ivec2 c = cell + ivec2(0,1);
  float za = rig_z(camera,a), zb = rig_z(camera,b), zc = rig_z(camera,c);
  if (min(za,min(zb,zc)) <= 0.0 || max(za,max(zb,zc)) - min(za,min(zb,zc)) > u_mesh_depth_delta * max(za,max(zb,zc))) return false;
  if (!rig_supported(camera,a,za) || !rig_supported(camera,b,zb) || !rig_supported(camera,c,zc)) return false;
  vec3 pa = rig_point(a,za,k), pb = rig_point(b,zb,k), pc = rig_point(c,zc,k);
  return max(distance(pa,pb), max(distance(pb,pc), distance(pc,pa))) <= u_mesh_max_edge;
}
bool rig_overlap() {
  if (u_rig_enabled == 0) return false;
  vec4 world = vec4(v_world_position,1);
  for (int camera = 0; camera < u_rig_index; camera++) {
    vec3 point = vec3(dot(rig_data(camera,0),world), dot(rig_data(camera,1),world), dot(rig_data(camera,2),world));
    if (rig_covers(camera,point)) return true;
  }
  return false;
}
`;
