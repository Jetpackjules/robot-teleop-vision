// GPU camera buffers and local surface confidence for the browser renderer.
// Adapted from this project's Godot/RealSense BlendPCR port.
// SPDX-License-Identifier: EUPL-1.2

export const FULLSCREEN_VERTEX_SHADER = `#version 300 es
precision highp float;
void main() {
  vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}`;

export function compileGpuProgram(gl, vertex, fragment) {
  const shaders = [];
  const program = gl.createProgram();
  try {
    for (const [type, source] of [[gl.VERTEX_SHADER, vertex], [gl.FRAGMENT_SHADER, fragment]]) {
      const shader = gl.createShader(type);
      shaders.push(shader);
      gl.shaderSource(shader, source);
      gl.compileShader(shader);
      if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(shader));
      gl.attachShader(program, shader);
    }
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));
    return program;
  } catch (error) {
    gl.deleteProgram(program);
    throw error;
  } finally {
    for (const shader of shaders) gl.deleteShader(shader);
  }
}

export function gpuTexture(gl, target, unit, format, width, height, layers = 0, linear = false) {
  const texture = gl.createTexture();
  gl.activeTexture(gl.TEXTURE0 + unit);
  gl.bindTexture(target, texture);
  gl.texParameteri(target, gl.TEXTURE_MIN_FILTER, linear ? gl.LINEAR : gl.NEAREST);
  gl.texParameteri(target, gl.TEXTURE_MAG_FILTER, linear ? gl.LINEAR : gl.NEAREST);
  gl.texParameteri(target, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(target, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  if (layers) gl.texStorage3D(target, 1, format, width, height, layers);
  else gl.texStorage2D(target, 1, format, width, height);
  return texture;
}

const QUALITY_FRAGMENT_SHADER = `#version 300 es
precision highp float;
precision highp int;
precision highp usampler2D;
uniform usampler2D u_depth;
uniform sampler2D u_color;
uniform ivec2 u_size;
uniform vec4 u_intrinsics;
uniform float u_mesh_depth_delta;
uniform float u_mesh_max_edge;
layout(location=0) out vec4 out_quality;
layout(location=1) out vec4 out_color;
uint depth_mm(ivec2 p) {
  if (any(lessThan(p, ivec2(0))) || any(greaterThanEqual(p, u_size))) return 0u;
  return texelFetch(u_depth,p,0).r;
}
vec3 point(ivec2 p, float z) {
  return vec3((float(p.x)-u_intrinsics.z)*z/u_intrinsics.x,
    -(float(p.y)-u_intrinsics.w)*z/u_intrinsics.y,-z);
}
vec3 support(ivec2 p,uint z,float noise_a,float noise_b) {
  const ivec2 offsets[4]=ivec2[4](ivec2(-1,0),ivec2(1,0),ivec2(0,-1),ivec2(0,1));
  vec3 n=vec3(0);
  float range=float(z)*0.001;
  float footprint=range/max(min(u_intrinsics.x,u_intrinsics.y),1.0);
  float valid_delta=min(u_mesh_depth_delta*float(z),max(4.0,6.0*range*range+2000.0*footprint));
  for(int i=0;i<4;i++) {
    uint neighbor=depth_mm(p+offsets[i]);
    if(neighbor==0u)continue;
    float delta=abs(float(neighbor)-float(z));
    if(delta<=valid_delta)n.x+=1.0;
    if(delta<=noise_a)n.y+=1.0;
    if(delta<=noise_b)n.z+=1.0;
  }
  return n;
}
bool geometry(uint za,uint zb,uint zc,vec3 pa,vec3 pb,vec3 pc) {
  uint lo=min(za,min(zb,zc)),hi=max(za,max(zb,zc));
  float range=float(hi)*0.001;
  float footprint=range/max(min(u_intrinsics.x,u_intrinsics.y),1.0);
  float limit_mm=min(u_mesh_depth_delta*float(hi),max(4.0,6.0*range*range+2000.0*footprint));
  if(lo==0u || float(hi-lo)>limit_mm)return false;
  float limit=u_mesh_max_edge*u_mesh_max_edge;
  return max(dot(pa-pb,pa-pb),max(dot(pb-pc,pb-pc),dot(pc-pa,pc-pa)))<=limit;
}
void main() {
  ivec2 p=ivec2(gl_FragCoord.xy);
  out_quality=vec4(0);
  out_color=vec4(texelFetch(u_color,p,0).rgb,1.0);
  if(any(greaterThanEqual(p,u_size-ivec2(1))))return;
  ivec2 b=p+ivec2(1,0),c=p+ivec2(0,1),d=p+ivec2(1,1);
  uint za=depth_mm(p),zb=depth_mm(b),zc=depth_mm(c),zd=depth_mm(d);
  vec3 pa=point(p,float(za)*0.001),pb=point(b,float(zb)*0.001);
  vec3 pc=point(c,float(zc)*0.001),pd=point(d,float(zd)*0.001);
  bool first=geometry(za,zb,zc,pa,pb,pc),second=geometry(zb,zd,zc,pb,pd,pc);
  if(!first&&!second)return;
  float noise_a=max(3.0,0.006*pow(float(max(za,max(zb,zc)))*0.001,2.0)*1000.0);
  float noise_b=max(3.0,0.006*pow(float(max(zb,max(zd,zc)))*0.001,2.0)*1000.0);
  // Evaluate each corner once. Both native validity and each triangle's exact
  // noise threshold share the same four neighbor reads (16 instead of 48).
  vec3 sa=first?support(p,za,noise_a,noise_b):vec3(0);
  vec3 sd=second?support(d,zd,noise_a,noise_b):vec3(0);
  vec3 sb=support(b,zb,noise_a,noise_b),sc=support(c,zc,noise_a,noise_b);
  first=first&&min(sa.x,min(sb.x,sc.x))>=2.0;
  second=second&&min(sb.x,min(sd.x,sc.x))>=2.0;
  float a=min(sa.y,min(sb.y,sc.y))*0.25,bn=min(sb.z,min(sd.z,sc.z))*0.25;
  out_quality=vec4(first?1.0:0.0,second?1.0:0.0,first?0.25+0.75*a*a:0.0,second?0.25+0.75*bn*bn:0.0);

}`;

// Shared by Unified mesh, shared-color points/mesh and BlendPCR. Arrays allow
// every camera in the rig to participate, including rigs with more than four.
export const RIG_SAMPLING_SHADER = `
uniform highp usampler2DArray u_rig_depth;
uniform highp sampler2DArray u_rig_quality;
uniform highp sampler2DArray u_rig_color;
uniform highp sampler2D u_rig_data;
uniform highp int u_rig_count;
uniform float u_fusion_depth_tolerance;
uniform highp int u_robot_mask_enabled;
uniform vec4 u_mask_capsule_a[6];
uniform vec4 u_mask_capsule_b[6];
vec4 rig_data(int camera,int column) {return texelFetch(u_rig_data,ivec2(column,camera),0);}
vec3 rig_local(int camera,vec3 p) {
  vec4 h=vec4(p,1);
  return vec3(dot(rig_data(camera,0),h),dot(rig_data(camera,1),h),dot(rig_data(camera,2),h));
}
vec3 rig_world(int camera,vec3 p) {
  vec4 h=vec4(p,1);
  return vec3(dot(rig_data(camera,5),h),dot(rig_data(camera,6),h),dot(rig_data(camera,7),h));
}
vec3 rig_direction(int camera,vec3 d) {
  return vec3(dot(rig_data(camera,0).xyz,d),dot(rig_data(camera,1).xyz,d),dot(rig_data(camera,2).xyz,d));
}
float rig_z(int camera,ivec2 pixel) {
  if(any(lessThan(pixel,ivec2(0)))||any(greaterThanEqual(pixel,ivec2(rig_data(camera,4).xy))))return 0.0;
  return float(texelFetch(u_rig_depth,ivec3(pixel,camera),0).r)*0.001;
}
vec3 rig_point(ivec2 pixel,float z,vec4 k) {
  return vec3((float(pixel.x)-k.z)*z/k.x,-(float(pixel.y)-k.w)*z/k.y,-z);
}
bool rig_masked(vec3 world) {
  for(int i=0;i<6;i++) {
    vec3 segment=u_mask_capsule_b[i].xyz-u_mask_capsule_a[i].xyz;
    float length_sq=dot(segment,segment);
    float along=length_sq>0.000001?clamp(dot(world-u_mask_capsule_a[i].xyz,segment)/length_sq,0.0,1.0):0.0;
    vec3 offset=world-u_mask_capsule_a[i].xyz-segment*along;
    if(dot(offset,offset)<=u_mask_capsule_a[i].w*u_mask_capsule_a[i].w)return true;
  }
  return false;
}
struct RigHit {vec3 local;vec3 world;vec3 normal;vec3 color;float confidence;float score;};
bool rig_triangle(int camera,uint id,vec3 origin,vec3 ray,out RigHit hit) {
  if(id==0u||rig_data(camera,4).z<0.5)return false;
  int primitive=int(id-1u),width=int(rig_data(camera,4).x)-1;
  int cell=primitive/2;
  ivec2 p=ivec2(cell%width,cell/width);
  if(any(greaterThanEqual(p,ivec2(rig_data(camera,4).xy)-ivec2(1))))return false;
  bool second=(primitive%2)==1;
  vec4 cache=texelFetch(u_rig_quality,ivec3(p,camera),0);
  if((second?cache.g:cache.r)<0.5)return false;
  ivec2 a=p,b=p+ivec2(1,0),c=p+ivec2(0,1);
  if(second){a=b;b=p+ivec2(1,1);}
  vec4 k=rig_data(camera,3);
  vec3 pa=rig_point(a,rig_z(camera,a),k),pb=rig_point(b,rig_z(camera,b),k),pc=rig_point(c,rig_z(camera,c),k);
  if(u_robot_mask_enabled==1&&(rig_masked(rig_world(camera,pa))||rig_masked(rig_world(camera,pb))||rig_masked(rig_world(camera,pc))))return false;
  vec3 v0=pb-pa,v1=pc-pa,n=cross(v0,v1);
  float denominator=dot(n,ray);
  if(abs(denominator)<1e-15)return false;
  float t=dot(n,pa-origin)/denominator;
  if(t<=0.0)return false;
  vec3 p3=origin+ray*t,v2=p3-pa;
  float d00=dot(v0,v0),d01=dot(v0,v1),d11=dot(v1,v1),det=d00*d11-d01*d01;
  if(det<=1e-25)return false;
  float v=(d11*dot(v2,v0)-d01*dot(v2,v1))/det,w=(d00*dot(v2,v1)-d01*dot(v2,v0))/det;
  hit.local=p3;hit.world=rig_world(camera,p3);hit.normal=normalize(n);
  // Match Godot's nearest RGB texture sampling at the reconstructed UV.
  ivec2 color_pixel=clamp(ivec2(floor((1.0-v-w)*vec2(a)+v*vec2(b)+w*vec2(c)+0.5)),ivec2(0),ivec2(rig_data(camera,4).xy)-1);
  hit.color=texelFetch(u_rig_color,ivec3(color_pixel,camera),0).rgb;
  hit.confidence=second?cache.a:cache.b;
  float range_sq=max(dot(p3,p3),0.0001);
  hit.score=k.x*k.y*1e-6*hit.confidence*abs(dot(hit.normal,p3/sqrt(range_sq)))/range_sq;
  return true;
}
bool rig_surface(int camera,vec3 world,out RigHit hit) {
  vec3 p=rig_local(camera,world);
  float z=-p.z;
  if(z<=0.001||rig_data(camera,4).z<0.5)return false;
  vec4 k=rig_data(camera,3),info=rig_data(camera,4);
  vec2 pixel=vec2(p.x*k.x/z+k.z,-p.y*k.y/z+k.w);
  if(any(lessThan(pixel,vec2(0)))||any(greaterThanEqual(pixel,info.xy-vec2(1))))return false;
  ivec2 cell=ivec2(floor(pixel));
  uint id=uint((cell.y*(int(info.x)-1)+cell.x)*2+(fract(pixel.x)+fract(pixel.y)>1.0?1:0)+1);
  return rig_triangle(camera,id,vec3(0),p,hit);
}
float rig_tolerance(float z) {return min(u_fusion_depth_tolerance,max(0.004,0.005*z*z));}
float rig_rank(float quality) {return floor(log(max(quality,1e-20))*50.4983498);}
bool rig_better(float q,int index,float other,int other_index) {
  float a=rig_rank(q),b=rig_rank(other);
  return a>b || (a==b&&index<other_index);
}
bool rig_free_space(RigHit own,int peer,vec3 world,RigHit other) {
  // A nearer peer means occlusion, never evidence that this surface is false.
  float expected=-rig_local(peer,world).z;
  return own.confidence<0.95&&other.confidence>0.95&&other.score>own.score*1.15
    &&-other.local.z>expected+rig_tolerance(expected);
}
bool rig_veto_hit(int own_index,vec3 world,RigHit own) {
  if(own.confidence>=0.95)return false;
  for(int peer=0;peer<u_rig_count;peer++) {
    RigHit other;
    if(peer!=own_index&&rig_surface(peer,world,other)&&rig_free_space(own,peer,world,other))return true;
  }
  return false;
}
bool rig_veto(int own_index,vec3 world) {
  RigHit own;
  return rig_surface(own_index,world,own)&&rig_veto_hit(own_index,world,own);
}
`;

export class GpuCameraRig {
  constructor(gl) {
    this.gl = gl;
    this.signature = "";
    this.uploads = [];
    this.rebuilds = 0;
    this.program = null;
    this.metadataSignature = "";
    this.count = 0;
    this.bindings = new WeakMap();
    this.failedSignature = "";
    this.version = 0;
  }

  clear() {
    const gl = this.gl;
    for (const key of ["depth", "data", "quality", "color"]) {
      if (this[key]) gl.deleteTexture(this[key]);
      this[key] = null;
    }
    if (this.framebuffer) gl.deleteFramebuffer(this.framebuffer);
    if (this.vao) gl.deleteVertexArray(this.vao);
    if (this.program) gl.deleteProgram(this.program);
    this.framebuffer = this.vao = this.program = null;
    this.signature = this.metadataSignature = "";
    this.uploads = [];
    this.count = 0;
    this.failedSignature = "";
  }

  bind(program) {
    const gl = this.gl;
    let locations = this.bindings.get(program);
    if (!locations) {
      locations = {};
      for (const name of ["depth","data","quality","color","count"]) locations[name] = gl.getUniformLocation(program,`u_rig_${name}`);
      this.bindings.set(program,locations);
    }
    for (const [key, unit, target] of [["depth",4,gl.TEXTURE_2D_ARRAY],["data",5,gl.TEXTURE_2D],
      ["quality",6,gl.TEXTURE_2D_ARRAY],["color",7,gl.TEXTURE_2D_ARRAY]]) {
      gl.activeTexture(gl.TEXTURE0 + unit);
      gl.bindTexture(target,this[key]);
      gl.uniform1i(locations[key],unit);
    }
    gl.uniform1i(locations.count,this.count);
  }

  update(cameras, inverseTransform, now = performance.now(), settings = {}) {
    const gl = this.gl;
    if (!cameras.length) {this.clear();return;}
    const width = Math.max(...cameras.map(c => c.width)), height = Math.max(...cameras.map(c => c.height));
    if (cameras.length > gl.getParameter(gl.MAX_ARRAY_TEXTURE_LAYERS)
      || Math.max(width,height,cameras.length) > gl.getParameter(gl.MAX_TEXTURE_SIZE)) {
      throw new Error("Camera rig exceeds this GPU's texture limits; using independent mesh");
    }
    const signature = JSON.stringify(cameras.map(c => [c.metadata.id,c.width,c.height]));
    if (signature === this.failedSignature) throw new Error("GPU camera cache unavailable");
    if (signature !== this.signature) {
      this.clear();
      try {
        this.depth = gpuTexture(gl,gl.TEXTURE_2D_ARRAY,4,gl.R16UI,width,height,cameras.length);
        this.data = gpuTexture(gl,gl.TEXTURE_2D,5,gl.RGBA32F,8,cameras.length);
        this.quality = gpuTexture(gl,gl.TEXTURE_2D_ARRAY,6,gl.RGBA8,width,height,cameras.length);
        this.color = gpuTexture(gl,gl.TEXTURE_2D_ARRAY,7,gl.RGBA8,width,height,cameras.length);
        this.framebuffer = gl.createFramebuffer();
        this.vao = gl.createVertexArray();
        this.program = compileGpuProgram(gl,FULLSCREEN_VERTEX_SHADER,QUALITY_FRAGMENT_SHADER);
        this.uniforms = {};
        for (const name of ["u_depth","u_color","u_size","u_intrinsics","u_mesh_depth_delta","u_mesh_max_edge"])
          this.uniforms[name] = gl.getUniformLocation(this.program,name);
        if (gl.getError() !== gl.NO_ERROR) throw new Error("Camera cache allocation failed");
        this.signature = signature;
        this.width = width;this.height = height;
      } catch (error) {this.clear();this.failedSignature=signature;throw error;}
    }
    this.count = cameras.length;
    const delta = Number(settings.mesh_depth_delta || 0.045), edge = Number(settings.mesh_max_edge || 0.08);
    const metadata = new Float32Array(cameras.length * 32);
    const changed = [];
    cameras.forEach((camera,index) => {
      const m = inverseTransform(camera.modelMatrix), w = camera.modelMatrix;
      metadata.set([m[0],m[4],m[8],m[12],m[1],m[5],m[9],m[13],m[2],m[6],m[10],m[14],
        ...camera.intrinsicsArray,camera.width,camera.height,now-camera.lastUpdateMs<1000?1:0,0,
        w[0],w[4],w[8],w[12],w[1],w[5],w[9],w[13],w[2],w[6],w[10],w[14]],index*32);
      const revision = `${camera.uploadRevision ?? camera.lastUpdateMs}:${delta}:${edge}:${camera.intrinsicsArray}`;
      const previous = this.uploads[index];
      if (!previous || previous.camera !== camera || previous.revision !== revision) changed.push({camera,index,revision});
    });
    const key = metadata.toString();
    if (!changed.length && key === this.metadataSignature) return;
    const oldFramebuffer = gl.getParameter(gl.FRAMEBUFFER_BINDING), oldVao = gl.getParameter(gl.VERTEX_ARRAY_BINDING);
    const viewport = gl.getParameter(gl.VIEWPORT);
    const u = this.uniforms;
    try {
      gl.useProgram(this.program);
      gl.bindVertexArray(this.vao);
      gl.disable(gl.DEPTH_TEST);gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);
      gl.uniform1i(u.u_depth,0);
      gl.uniform1i(u.u_color,1);
      gl.uniform1f(u.u_mesh_depth_delta,delta);
      gl.uniform1f(u.u_mesh_max_edge,edge);
      gl.pixelStorei(gl.UNPACK_ALIGNMENT,1);
      changed.forEach(({camera,index,revision}) => {
        // Upload revision is monotonic even when two packets share a timer tick.
        gl.activeTexture(gl.TEXTURE4);gl.bindTexture(gl.TEXTURE_2D_ARRAY,this.depth);
        gl.texSubImage3D(gl.TEXTURE_2D_ARRAY,0,0,0,index,camera.width,camera.height,1,gl.RED_INTEGER,gl.UNSIGNED_SHORT,camera.depth);
        gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,camera.depthTexture);
        gl.activeTexture(gl.TEXTURE1);gl.bindTexture(gl.TEXTURE_2D,camera.colorTexture);
        gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,this.quality,0,index);
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT1,this.color,0,index);
        gl.drawBuffers([gl.COLOR_ATTACHMENT0,gl.COLOR_ATTACHMENT1]);
        if (gl.checkFramebufferStatus(gl.FRAMEBUFFER) !== gl.FRAMEBUFFER_COMPLETE) throw new Error("Camera cache framebuffer unavailable");
        gl.viewport(0,0,camera.width,camera.height);
        gl.uniform2i(u.u_size,camera.width,camera.height);
        gl.uniform4fv(u.u_intrinsics,camera.intrinsicsArray);
        gl.drawArrays(gl.TRIANGLES,0,3);
        this.uploads[index] = {camera,revision};this.rebuilds++;
      });
      // Transforms/freshness can change without a new depth capture.
      if (key !== this.metadataSignature) {
        gl.activeTexture(gl.TEXTURE5);gl.bindTexture(gl.TEXTURE_2D,this.data);
        gl.texSubImage2D(gl.TEXTURE_2D,0,0,0,8,cameras.length,gl.RGBA,gl.FLOAT,metadata);
        this.metadataSignature = key;
      }
      this.version++;
    } catch (error) {
      this.failedSignature=signature;
      throw error;
    } finally {
      gl.bindFramebuffer(gl.FRAMEBUFFER,oldFramebuffer);
      gl.bindVertexArray(oldVao);
      gl.viewport(...viewport);gl.enable(gl.DEPTH_TEST);
    }
  }
}
