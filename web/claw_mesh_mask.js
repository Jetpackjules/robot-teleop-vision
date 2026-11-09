// Link-local CAD distance fields: inflate the surfaces, not the wrist pivot.
// Unit 9 is free in the surface/mask passes (counts use it in a separate pass).
export const CLAW_MASK_SHADER = `
uniform highp sampler3D u_claw_sdf;
uniform highp int u_claw_mesh_enabled;
uniform highp mat4 u_world_to_claw[2];
uniform highp vec3 u_claw_min[2];
uniform highp vec3 u_claw_max[2];
uniform highp float u_claw_buffer;
uniform highp float u_claw_grid_size;
bool claw_mesh_masked(vec3 world) {
  if(u_claw_mesh_enabled==0)return false;
  for(int link=0;link<2;link++) {
    vec3 p=(u_world_to_claw[link]*vec4(world,1)).xyz;
    vec3 uv=(p-u_claw_min[link])/(u_claw_max[link]-u_claw_min[link]);
    if(any(lessThan(uv,vec3(0)))||any(greaterThan(uv,vec3(1))))continue;
    uv=uv*(u_claw_grid_size-1.)/u_claw_grid_size+.5/u_claw_grid_size;
    vec2 distance=textureLod(u_claw_sdf,uv,0.).rg;
    if((link==0?distance.r:distance.g)<=u_claw_buffer)return true;
  }
  return false;
}
`;

let asset;
const contexts=new WeakMap();
function context(gl) {
  let state=contexts.get(gl);
  if(state)return state;
  const texture=gl.createTexture(),previous=gl.getParameter(gl.ACTIVE_TEXTURE);
  gl.activeTexture(gl.TEXTURE9);gl.bindTexture(gl.TEXTURE_3D,texture);
  for(const key of [gl.TEXTURE_MIN_FILTER,gl.TEXTURE_MAG_FILTER])gl.texParameteri(gl.TEXTURE_3D,key,gl.LINEAR);
  for(const key of [gl.TEXTURE_WRAP_S,gl.TEXTURE_WRAP_T,gl.TEXTURE_WRAP_R])gl.texParameteri(gl.TEXTURE_3D,key,gl.CLAMP_TO_EDGE);
  gl.texImage3D(gl.TEXTURE_3D,0,gl.RG16F,1,1,1,0,gl.RG,gl.HALF_FLOAT,new Uint16Array(2));
  gl.activeTexture(previous);
  state={texture,ready:false,enabled:false,bindings:new WeakMap()};contexts.set(gl,state);
  asset??=Promise.all(['json','bin'].map(async ext=>{
    const response=await fetch(`/robot-modules/so101/assets/claw-mask-sdf.${ext}`);
    if(!response.ok)throw Error(`Claw mask asset HTTP ${response.status}`);
    return ext==='json'?response.json():response.arrayBuffer();
  }));
  state.loading=asset.then(([meta,bytes])=>{
    if(meta.links.length!==2||bytes.byteLength!==meta.size**3*4)throw Error('Invalid claw distance field');
    const previous=gl.getParameter(gl.ACTIVE_TEXTURE);
    gl.activeTexture(gl.TEXTURE9);gl.bindTexture(gl.TEXTURE_3D,texture);
    gl.texImage3D(gl.TEXTURE_3D,0,gl.RG16F,meta.size,meta.size,meta.size,0,gl.RG,gl.HALF_FLOAT,new Uint16Array(bytes));
    gl.activeTexture(previous);
    state.meta=meta;state.ready=true;
  }).catch(error=>{state.error=error.message;console.warn('Claw mesh mask unavailable; retaining capsule mask.',error.message);});
  return state;
}
export async function loadClawMeshMask(gl) {await context(gl).loading;return context(gl).ready;}

export function updateClawMeshMask(gl,worldToLinks,buffer,enabled=true) {
  const state=context(gl);
  state.enabled=Boolean(enabled&&state.ready&&worldToLinks?.every(Boolean));
  if(!state.enabled)return '';
  state.matrices=new Float32Array(worldToLinks.flatMap(matrix=>Array.from(matrix)));
  state.buffer=Math.max(0,Math.min(.05,Number(buffer)||0));
  return `${state.buffer}:${state.matrices}`;
}

export function bindClawMeshMask(gl,program) {
  const state=context(gl);let u=state.bindings.get(program);
  if(!u){
    u={};for(const name of ['u_claw_sdf','u_claw_mesh_enabled','u_world_to_claw[0]','u_claw_min[0]','u_claw_max[0]','u_claw_buffer','u_claw_grid_size'])u[name]=gl.getUniformLocation(program,name);
    state.bindings.set(program,u);
  }
  if(u.u_claw_sdf===null)return;
  const previous=gl.getParameter(gl.ACTIVE_TEXTURE);
  gl.activeTexture(gl.TEXTURE9);gl.bindTexture(gl.TEXTURE_3D,state.texture);gl.activeTexture(previous);
  gl.uniform1i(u.u_claw_sdf,9);gl.uniform1i(u.u_claw_mesh_enabled,state.enabled?1:0);
  if(state.enabled){
    gl.uniformMatrix4fv(u['u_world_to_claw[0]'],false,state.matrices);
    gl.uniform3fv(u['u_claw_min[0]'],state.meta.links.flatMap(link=>link.min));
    gl.uniform3fv(u['u_claw_max[0]'],state.meta.links.flatMap(link=>link.max));
    gl.uniform1f(u.u_claw_buffer,state.buffer);gl.uniform1f(u.u_claw_grid_size,state.meta.size);
  }
}
