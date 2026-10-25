// Color coverage comes from calibration, never from how dark a JPEG pixel is.
export const COLOR_PROJECTION_SHADER = `
uniform int u_color_projection_enabled;
uniform int u_original_rgb_enabled;
uniform vec2 u_color_image_size;
uniform mat3 u_depth_to_color_rotation;
uniform vec3 u_depth_to_color_translation;
uniform vec4 u_raw_color_intrinsics;
uniform vec2 u_raw_color_size;
bool camera_color_pixel(vec3 local,out vec2 pixel) {
  pixel=vec2(0);
  if(u_color_projection_enabled==0)return false;
  vec3 p=u_depth_to_color_rotation*(local*vec3(1.0,-1.0,-1.0))+u_depth_to_color_translation;
  if(p.z<=0.0)return false;
  pixel=p.xy/p.z*u_raw_color_intrinsics.xy+u_raw_color_intrinsics.zw;
  return all(greaterThanEqual(pixel,vec2(0)))&&all(lessThan(pixel,u_raw_color_size-vec2(1)));
}
bool camera_has_color(vec3 local) {
  if(u_color_projection_enabled==0)return true;
  vec2 pixel;return camera_color_pixel(local,pixel);
}
vec4 camera_sample_color(sampler2D image,vec3 local,ivec2 depth_pixel) {
  if(u_original_rgb_enabled==0)return vec4(texelFetch(image,depth_pixel,0).rgb,camera_has_color(local)?1.0:0.0);
  vec2 pixel;
  if(!camera_color_pixel(local,pixel))return vec4(0);
  return vec4(texture(image,(pixel+vec2(0.5))/u_color_image_size).rgb,1.0);
}`;

const names = ['u_color_projection_enabled', 'u_original_rgb_enabled','u_color_image_size', 'u_depth_to_color_rotation',
  'u_depth_to_color_translation', 'u_raw_color_intrinsics', 'u_raw_color_size'];
const bindings = new WeakMap();
export function bindColorProjection(gl, program, camera) {
  let u = bindings.get(program);
  if (!u) {u = Object.fromEntries(names.map(name => [name, gl.getUniformLocation(program, name)])); bindings.set(program, u);}
  const p = validatedColorProjection(camera.metadata);
  const valid=!!p;
  gl.uniform1i(u.u_original_rgb_enabled,camera.metadata.color_space==='original_rgb'&&valid?1:0);
  gl.uniform2f(u.u_color_image_size,camera.colorWidth||camera.width,camera.colorHeight||camera.height);
  gl.uniform1i(u.u_color_projection_enabled, valid ? 1 : 0);
  if (!valid) return;
  gl.uniformMatrix3fv(u.u_depth_to_color_rotation, false, p.extrinsics.slice(0, 9));
  gl.uniform3fv(u.u_depth_to_color_translation, p.extrinsics.slice(9));
  gl.uniform4fv(u.u_raw_color_intrinsics, p.intrinsics);
  gl.uniform2fv(u.u_raw_color_size, p.size);
}

export function validatedColorProjection(metadata) {
  const p=metadata?.color_projection;
  const vector=(value,length)=>(Array.isArray(value)||ArrayBuffer.isView(value))&&value.length===length&&Array.from(value).every(Number.isFinite);
  return p?.model==='pinhole'&&vector(p.size,2)&&p.size.every(v=>v>1)
    &&vector(p.intrinsics,4)&&p.intrinsics[0]>0&&p.intrinsics[1]>0&&vector(p.extrinsics,12)?p:null;
}
export function colorFrameDimensions(metadata) {
  const raw=metadata.color_space==='original_rgb';
  const width=Number(raw?metadata.color_width:metadata.width),height=Number(raw?metadata.color_height:metadata.height);
  if(!Number.isInteger(width)||!Number.isInteger(height)||width<1||height<1||width*height>16*1024*1024)
    throw new Error('invalid color dimensions');
  if(raw) {
    const p=validatedColorProjection(metadata);
    if(!p||p.size[0]!==width||p.size[1]!==height)throw new Error('original RGB requires matching calibrated color dimensions');
  }
  return [width,height];
}
