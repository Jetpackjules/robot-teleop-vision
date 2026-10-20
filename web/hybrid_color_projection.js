// Color coverage comes from calibration, never from how dark a JPEG pixel is.
export const COLOR_PROJECTION_SHADER = `
uniform int u_color_projection_enabled;
uniform mat3 u_depth_to_color_rotation;
uniform vec3 u_depth_to_color_translation;
uniform vec4 u_raw_color_intrinsics;
uniform vec2 u_raw_color_size;
bool camera_has_color(vec3 local) {
  if(u_color_projection_enabled==0)return true;
  vec3 p=u_depth_to_color_rotation*(local*vec3(1.0,-1.0,-1.0))+u_depth_to_color_translation;
  if(p.z<=0.0)return false;
  vec2 pixel=p.xy/p.z*u_raw_color_intrinsics.xy+u_raw_color_intrinsics.zw;
  return all(greaterThanEqual(pixel,vec2(0)))&&all(lessThan(pixel,u_raw_color_size-vec2(1)));
}`;

const names = ['u_color_projection_enabled', 'u_depth_to_color_rotation',
  'u_depth_to_color_translation', 'u_raw_color_intrinsics', 'u_raw_color_size'];
const bindings = new WeakMap();
export function bindColorProjection(gl, program, camera) {
  let u = bindings.get(program);
  if (!u) {u = Object.fromEntries(names.map(name => [name, gl.getUniformLocation(program, name)])); bindings.set(program, u);}
  const p = camera.metadata.color_projection;
  const vector = (value, length) => Array.isArray(value) && value.length === length && value.every(Number.isFinite);
  const valid = p?.model === 'pinhole' && vector(p.size, 2) && p.size.every(v => v > 1)
    && vector(p.intrinsics, 4) && p.intrinsics[0] > 0 && p.intrinsics[1] > 0 && vector(p.extrinsics, 12);
  gl.uniform1i(u.u_color_projection_enabled, valid ? 1 : 0);
  if (!valid) return;
  gl.uniformMatrix3fv(u.u_depth_to_color_rotation, false, p.extrinsics.slice(0, 9));
  gl.uniform3fv(u.u_depth_to_color_translation, p.extrinsics.slice(9));
  gl.uniform4fv(u.u_raw_color_intrinsics, p.intrinsics);
  gl.uniform2fv(u.u_raw_color_size, p.size);
}
