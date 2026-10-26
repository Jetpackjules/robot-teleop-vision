// GPU camera buffers and local surface confidence for the browser renderer.
// Adapted from this project's Godot/RealSense BlendPCR port.
// SPDX-License-Identifier: EUPL-1.2
import { COLOR_PROJECTION_SHADER, bindColorProjection, validatedColorProjection } from './hybrid_color_projection.js?v=20261021_original_rgb';

import {RgbVisibility} from './hybrid_rgb_visibility.js?v=20261021_original_rgb';

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
${COLOR_PROJECTION_SHADER}
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
  out_color=camera_sample_color(u_color,point(p,float(depth_mm(p))*0.001),p);
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
#ifndef RIG_CAMERA_CAPACITY
#define RIG_CAMERA_CAPACITY 32
#endif
uniform highp usampler2DArray u_rig_depth;
uniform highp sampler2DArray u_rig_quality;
uniform highp sampler2DArray u_rig_color;
uniform highp sampler2DArray u_rig_rgb;
uniform highp sampler2DArray u_rig_visibility;
#ifdef RIG_CACHED_MASK
uniform highp usampler2DArray u_rig_mask;
#endif
uniform highp ivec4 u_rig_visibility_layers[RIG_CAMERA_CAPACITY];
uniform highp int u_rig_original_rgb_enabled;
#ifdef RIG_UNIFORM_DATA
uniform highp vec4 u_rig_uniform_data[RIG_CAMERA_CAPACITY*13];
#else
uniform highp sampler2D u_rig_data;
#endif
uniform highp int u_rig_count;
uniform float u_fusion_depth_tolerance;
uniform highp int u_robot_mask_enabled;
uniform vec4 u_mask_capsule_a[6];
uniform vec4 u_mask_capsule_b[6];
vec4 rig_data(int camera,int column) {
#ifdef RIG_UNIFORM_DATA
  return u_rig_uniform_data[camera*13+column];
#else
  return texelFetch(u_rig_data,ivec2(column,camera),0);
#endif
}
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
struct RigHit {vec3 local;vec3 world;vec3 normal;vec3 color;bool color_valid;float confidence;float score;};
bool rig_triangle_checked(int camera,uint id,vec3 origin,vec3 ray,bool check_mask,out RigHit hit) {
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
  if(check_mask&&u_robot_mask_enabled==1){
#ifdef RIG_CACHED_MASK
    if(texelFetch(u_rig_mask,ivec3(a,camera),0).r!=0u||texelFetch(u_rig_mask,ivec3(b,camera),0).r!=0u||texelFetch(u_rig_mask,ivec3(c,camera),0).r!=0u)return false;
#else
    if(rig_masked(rig_world(camera,pa))||rig_masked(rig_world(camera,pb))||rig_masked(rig_world(camera,pc)))return false;
#endif
  }
  vec3 v0=pb-pa,v1=pc-pa,n=cross(v0,v1);
  float denominator=dot(n,ray);
  if(abs(denominator)<1e-15)return false;
  float t=dot(n,pa-origin)/denominator;
  if(t<=0.0)return false;
  vec3 p3=origin+ray*t,v2=p3-pa;
  float d00=dot(v0,v0),d01=dot(v0,v1),d11=dot(v1,v1),det=d00*d11-d01*d01;
  if(det<=1e-25)return false;
  hit.local=p3;hit.world=rig_world(camera,p3);hit.normal=normalize(n);
#ifdef RIG_NATIVE_RGB_ONLY
  hit.color=vec3(0);hit.color_valid=false;
#else
  float v=(d11*dot(v2,v0)-d01*dot(v2,v1))/det,w=(d00*dot(v2,v1)-d01*dot(v2,v0))/det;
  // Match Godot's nearest RGB texture sampling at the reconstructed UV.
  ivec2 color_pixel=clamp(ivec2(floor((1.0-v-w)*vec2(a)+v*vec2(b)+w*vec2(c)+0.5)),ivec2(0),ivec2(rig_data(camera,4).xy)-1);
  vec4 sample_color=texelFetch(u_rig_color,ivec3(color_pixel,camera),0);
  hit.color=sample_color.rgb;hit.color_valid=sample_color.a>0.5;
#endif
  hit.confidence=second?cache.a:cache.b;
  float range_sq=max(dot(p3,p3),0.0001);
  hit.score=k.x*k.y*1e-6*hit.confidence*abs(dot(hit.normal,p3/sqrt(range_sq)))/range_sq;
  return true;
}
bool rig_triangle(int camera,uint id,vec3 origin,vec3 ray,out RigHit hit) {
  return rig_triangle_checked(camera,id,origin,ray,true,hit);
}
bool rig_metadata_triangle(int camera,uint id,vec3 origin,vec3 ray,out RigHit hit) {
  // Metadata already rejected the complete masked native triangle. Peer
  // surface queries still use rig_triangle and retain their own mask check.
  return rig_triangle_checked(camera,id,origin,ray,false,hit);
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
vec3 rig_world_normal(int camera,vec3 normal) {
  return normalize(rig_world(camera,normal)-rig_world(camera,vec3(0)));
}
// Color can use an agreeing planar RGB view when millimetre stereo error
// places its measured surface just beyond the stricter geometry-owner test.
// This never moves geometry, averages depth, or colors across a sharp edge.
bool rig_color_agrees(int camera,vec3 world,vec3 normal,RigHit peer) {
  if(!peer.color_valid||peer.confidence<0.95)return false;
  float expected=-rig_local(camera,world).z;
  return abs(-peer.local.z-expected)<=max(rig_tolerance(expected),0.012)
    &&abs(dot(normal,rig_world_normal(camera,peer.normal)))>=0.985;
}
float rig_rgb_visible_depth(ivec2 q,ivec2 end,int layer) {
  float observed=min(min(texelFetch(u_rig_visibility,ivec3(q,layer),0).r,
      texelFetch(u_rig_visibility,ivec3(min(q+ivec2(1,0),end),layer),0).r),
    min(texelFetch(u_rig_visibility,ivec3(min(q+ivec2(0,1),end),layer),0).r,
      texelFetch(u_rig_visibility,ivec3(min(q+ivec2(1),end),layer),0).r));
  return (0.01*65.535)/(65.535-observed*(65.535-0.01));
}
bool rig_rgb_same_surface(int camera,vec3 world,vec3 normal) {
  RigHit measured;
  if(!rig_surface(camera,world,measured)||measured.confidence<0.95)return false;
  vec3 measured_normal=rig_world_normal(camera,measured.normal);
  // Require supported, parallel local geometry as well as a small normal
  // separation. This allowance is for uncertain depth, never geometry fusion.
  return abs(dot(normal,measured_normal))>=0.985
    &&abs(dot(world-measured.world,measured_normal))<=0.012
    &&abs(rig_local(camera,world).z-measured.local.z)<=0.012;
}
// Color projects the *same world point* into the independent RGB sensor.
// Visibility is rendered from ALL measured cameras, so a peer depth hole or
// a different triangulation cannot suppress an otherwise visible RGB image.
bool rig_rgb_sample(int camera,vec3 world,vec3 normal,out vec3 color,out float score) {
  if(rig_data(camera,4).z<0.5||rig_data(camera,12).z<0.5)return false;
  vec3 local=rig_local(camera,world)*vec3(1.0,-1.0,-1.0);
  vec4 h=vec4(local,1.0);
  vec3 p=vec3(dot(rig_data(camera,8),h),dot(rig_data(camera,9),h),dot(rig_data(camera,10),h));
  vec4 k=rig_data(camera,11);vec2 size=rig_data(camera,12).xy;
  if(p.z<=0.01)return false;
  vec2 pixel=p.xy/p.z*k.xy+k.zw;
  if(any(lessThan(pixel,vec2(0)))||any(greaterThan(pixel,size-vec2(1))))return false;
  ivec2 q=ivec2(floor(pixel)),end=ivec2(size)-1;
  // Bilinear RGB touches four pixels. Actual self-occlusion is always tested
  // against this camera's native surface, before considering peer uncertainty.
  float strict_margin=max(0.004,0.004*p.z*p.z);
  ivec4 layers=u_rig_visibility_layers[camera];
  float own_z=rig_rgb_visible_depth(q,end,layers.y);
  if(own_z<p.z-strict_margin)return false;
  float visible_z=min(own_z,rig_rgb_visible_depth(q,end,layers.x));
  float gap=p.z-visible_z;
  if(gap>strict_margin) {
    if(!rig_rgb_same_surface(camera,world,normal))return false;
    // Retain measured color through small disagreement between coincident
    // planes. One previous observation may bridge only the narrow 12-16 mm
    // boundary band; a clear occluder is rejected on its first frame.
    if(gap>0.012) {
      if(gap>0.016)return false;
      float previous_z=min(rig_rgb_visible_depth(q,end,layers.z),rig_rgb_visible_depth(q,end,layers.w));
      if(p.z-previous_z>0.012)return false;
    }
  }
  vec3 origin=rig_world(camera,vec3(0));vec3 ray=world-origin;
  float range_sq=max(dot(ray,ray),0.0001);
  float incidence=abs(dot(normal,ray*inversesqrt(range_sq)));
  float edge=min(min(pixel.x,pixel.y),min(size.x-1.0-pixel.x,size.y-1.0-pixel.y));
  score=k.x*k.y*1e-6*max(incidence,0.02)/range_sq*smoothstep(0.0,8.0,edge);
  if(score<=0.0)return false;
  color=textureLod(u_rig_rgb,vec3((pixel+vec2(0.5))/vec2(textureSize(u_rig_rgb,0).xy),float(camera)),0.0).rgb;
  return true;
}
vec3 rig_rgb_linear(vec3 c){return mix(c/12.92,pow(max((c+.055)/1.055,vec3(0)),vec3(2.4)),step(vec3(.04045),c));}
vec3 rig_rgb_srgb(vec3 c){return mix(c*12.92,1.055*pow(max(c,vec3(0)),vec3(1.0/2.4))-.055,step(vec3(.0031308),c));}
vec3 rig_projected_color(vec3 world,vec3 normal,vec3 fallback,int source_camera) {
  vec3 color=vec3(0),source_color=fallback;float total=0.0;
  bool source_verified=false;
  for(int camera=0;camera<u_rig_count;camera++) {
    vec3 rgb;float score;
    bool valid=rig_rgb_sample(camera,world,normal,rgb,score);
    if(!valid)continue;
    if(camera==source_camera){source_color=rgb;source_verified=true;}
    // Squared projected pixel density favors close, detailed views while
    // continuously blending their calibrated coverage edges in linear light.
    float weight=score*score;
    color+=rig_rgb_linear(rgb)*weight;total+=weight;
  }
  // A verified native source must survive tiny projected weights (far range,
  // grazing view or coverage edge), rather than unexpectedly turning gray.
  return total>1e-12?rig_rgb_srgb(color/total):source_verified?source_color:fallback;
}
vec3 rig_projected_color(vec3 world,vec3 normal,vec3 fallback) {
  return rig_projected_color(world,normal,fallback,-1);
}
float rig_free_space_margin(float confidence,float expected) {
  bool strong_support=confidence>=0.95;
  return max(strong_support?0.012:0.008,rig_tolerance(expected)*(strong_support?3.0:2.0));
}
bool rig_may_free_space(int peer,vec3 p,float margin) {
  vec4 info=rig_data(peer,4),k=rig_data(peer,3);
  if(info.z<0.5||p.z>=-0.001)return false;
  vec2 pixel=vec2(p.x*k.x/-p.z+k.z,-p.y*k.y/-p.z+k.w);
  if(any(lessThan(pixel,vec2(0)))||any(greaterThanEqual(pixel,info.xy-vec2(1))))return false;
  ivec2 cell=ivec2(floor(pixel)),a=cell,b=cell+ivec2(1,0),c=cell+ivec2(0,1);
  if(fract(pixel.x)+fract(pixel.y)>1.0)a+=ivec2(1);
  // A ray inside a projected triangle cannot meet it beyond its farthest
  // vertex. Agreeing/nearer surfaces need no normal, color or support reads.
  return max(rig_z(peer,a),max(rig_z(peer,b),rig_z(peer,c)))>-p.z+margin;
}
bool rig_free_space(RigHit own,int peer,vec3 world,RigHit other) {
  // A nearer peer means occlusion, never evidence that this surface is false.
  float expected=-rig_local(peer,world).z;
  // A smooth stereo error can have perfect local neighbor support. Allow a
  // substantially better view to reject it, with a wider calibration margin.
  bool strong_support=own.confidence>=0.95;
  float ratio=strong_support?1.5:1.15;
  float margin=rig_free_space_margin(own.confidence,expected);
  return other.confidence>0.95&&other.score>own.score*ratio&&-other.local.z>expected+margin;
}
bool rig_veto_hit(int own_index,vec3 world,RigHit own) {
  float required=own.score*(own.confidence>=0.95?1.5:1.15);
  for(int peer=0;peer<u_rig_count;peer++) {
    if(peer==own_index)continue;
    vec3 p=rig_local(peer,world);vec4 k=rig_data(peer,3);
    // Confidence and incidence cannot exceed one. This exact upper bound
    // avoids depth/triangle reads for peers that cannot overturn this hit.
    if(p.z>=-0.001||k.x*k.y*1e-6/max(dot(p,p),0.0001)<=required)continue;
    if(!rig_may_free_space(peer,p,rig_free_space_margin(own.confidence,-p.z)))continue;
    RigHit other;
    if(rig_surface(peer,world,other)&&rig_free_space(own,peer,world,other))return true;
  }
  return false;
}
bool rig_veto(int own_index,vec3 world) {
  RigHit own;
  return rig_surface(own_index,world,own)&&rig_veto_hit(own_index,world,own);
}
`;

export class GpuCameraRig {
  constructor(gl, {copyDepthOnGpu = true} = {}) {
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
    this.metadataValues = null;
    this.maskTexture=null;
    this.maxLayers = gl.getParameter(gl.MAX_ARRAY_TEXTURE_LAYERS);
    this.maxTextureSize = gl.getParameter(gl.MAX_TEXTURE_SIZE);
    this.copyDepthOnGpu = copyDepthOnGpu;
    this.visibility=new RgbVisibility(gl,compileGpuProgram,gpuTexture);
  }

  clear() {
    const gl = this.gl;
    this.visibility.clear();
    if(this.rgb&&this.rgb!==this.color)gl.deleteTexture(this.rgb);this.rgb=null;
    for (const key of ["depth", "data", "quality", "color"]) {
      if (this[key]) gl.deleteTexture(this[key]);
      this[key] = null;
    }
    if (this.framebuffer) gl.deleteFramebuffer(this.framebuffer);
    if (this.depthCopyFramebuffer) gl.deleteFramebuffer(this.depthCopyFramebuffer);
    if (this.colorCopyFramebuffer)gl.deleteFramebuffer(this.colorCopyFramebuffer);this.colorCopyFramebuffer=null;
    if (this.vao) gl.deleteVertexArray(this.vao);
    if (this.program) gl.deleteProgram(this.program);
    this.depthCopyFramebuffer = this.framebuffer = this.vao = this.program = null;
    this.signature = this.metadataSignature = "";
    this.uploads = [];
    this.count = 0;
    this.metadataValues = null;
    this.failedSignature = "";
  }

  bind(program) {
    const gl = this.gl;
    let locations = this.bindings.get(program);
    if (!locations) {
      locations = {};
      for (const name of ["depth","data","quality","color","rgb","visibility","count","original_rgb_enabled"]) locations[name] = gl.getUniformLocation(program,`u_rig_${name}`);
      locations.visibilityLayers=gl.getUniformLocation(program,"u_rig_visibility_layers[0]");
      locations.uniformData=gl.getUniformLocation(program,"u_rig_uniform_data[0]");
      locations.mask=gl.getUniformLocation(program,'u_rig_mask');
      this.bindings.set(program,locations);
    }
    for (const [key, unit, target] of [["depth",4,gl.TEXTURE_2D_ARRAY],["data",5,gl.TEXTURE_2D],
      ["quality",6,gl.TEXTURE_2D_ARRAY],["color",7,gl.TEXTURE_2D_ARRAY],["rgb",14,gl.TEXTURE_2D_ARRAY],["visibility",15,gl.TEXTURE_2D_ARRAY]]) {
      gl.activeTexture(gl.TEXTURE0 + unit);
      gl.bindTexture(target,key==="visibility"?this.visibility.texture:this[key]);
      gl.uniform1i(locations[key],unit);
    }
    gl.uniform1i(locations.count,this.count);
    gl.uniform1i(locations.original_rgb_enabled,this.originalRgb?1:0);
    if(this.visibility.layers.length)gl.uniform4iv(locations.visibilityLayers,this.visibility.layers);
    if(locations.uniformData!==null&&this.metadataValues)gl.uniform4fv(locations.uniformData,this.metadataValues);
    if(locations.mask!==null){gl.activeTexture(gl.TEXTURE3);gl.bindTexture(gl.TEXTURE_2D_ARRAY,this.maskTexture||this.depth);gl.uniform1i(locations.mask,3);}
  }

  update(cameras, inverseTransform, now = performance.now(), settings = {}, restoreState = null) {
    const gl = this.gl;
    if (!cameras.length) {this.clear();return;}
    const width = Math.max(...cameras.map(c => c.width)), height = Math.max(...cameras.map(c => c.height));
    const colorWidth=Math.max(...cameras.map(c=>c.colorWidth||c.width)),colorHeight=Math.max(...cameras.map(c=>c.colorHeight||c.height));
    const originalRgb=cameras.some(c=>c.metadata.color_space==='original_rgb');
    if (cameras.length > this.maxLayers
      || Math.max(width,height,colorWidth,colorHeight,cameras.length) > this.maxTextureSize) {
      throw new Error("Camera rig exceeds this GPU's texture limits; using independent mesh");
    }
    const signature = JSON.stringify(cameras.map(c => [c.metadata.id,c.width,c.height,c.colorWidth,c.colorHeight,c.metadata.color_space]));
    if (signature === this.failedSignature) throw new Error("GPU camera cache unavailable");
    if (signature !== this.signature) {
      this.clear();
      try {
        this.depth = gpuTexture(gl,gl.TEXTURE_2D_ARRAY,4,gl.R16UI,width,height,cameras.length);
        this.data = gpuTexture(gl,gl.TEXTURE_2D,5,gl.RGBA32F,13,cameras.length);
        this.quality = gpuTexture(gl,gl.TEXTURE_2D_ARRAY,6,gl.RGBA8,width,height,cameras.length);
        this.color = gpuTexture(gl,gl.TEXTURE_2D_ARRAY,7,gl.RGBA8,width,height,cameras.length);
        this.rgb=originalRgb?gpuTexture(gl,gl.TEXTURE_2D_ARRAY,14,gl.RGBA8,colorWidth,colorHeight,cameras.length,true):this.color;
        this.colorCopyFramebuffer=gl.createFramebuffer();
        this.framebuffer = gl.createFramebuffer();
        if(this.copyDepthOnGpu)this.depthCopyFramebuffer = gl.createFramebuffer();
        this.vao = gl.createVertexArray();
        this.program = compileGpuProgram(gl,FULLSCREEN_VERTEX_SHADER,QUALITY_FRAGMENT_SHADER);
        this.uniforms = {};
        for (const name of ["u_depth","u_color","u_size","u_intrinsics","u_mesh_depth_delta","u_mesh_max_edge"])
          this.uniforms[name] = gl.getUniformLocation(this.program,name);
        // The immutable attachments have identical dimensions at every layer.
        // Validate once, rather than synchronizing the GPU on every capture.
        const parent = restoreState ? restoreState.framebuffer : gl.getParameter(gl.FRAMEBUFFER_BINDING);
        gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,this.quality,0,0);
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT1,this.color,0,0);
        gl.drawBuffers([gl.COLOR_ATTACHMENT0,gl.COLOR_ATTACHMENT1]);
        const complete = gl.checkFramebufferStatus(gl.FRAMEBUFFER) === gl.FRAMEBUFFER_COMPLETE;
        gl.bindFramebuffer(gl.FRAMEBUFFER,parent);
        if (!complete) throw new Error("Camera cache framebuffer unavailable");
        if (gl.getError() !== gl.NO_ERROR) throw new Error("Camera cache allocation failed");
        this.signature = signature;
        this.width = width;this.height = height;
      } catch (error) {this.clear();this.failedSignature=signature;throw error;}
    }
    this.count = cameras.length;
    this.originalRgb=originalRgb;
    const delta = Number(settings.mesh_depth_delta || 0.045), edge = Number(settings.mesh_max_edge || 0.08);
    const metadata = new Float32Array(cameras.length * 52);
    const changed = [];
    cameras.forEach((camera,index) => {
      const m = inverseTransform(camera.modelMatrix), w = camera.modelMatrix;
      metadata.set([m[0],m[4],m[8],m[12],m[1],m[5],m[9],m[13],m[2],m[6],m[10],m[14],
        ...camera.intrinsicsArray,camera.width,camera.height,now-camera.lastUpdateMs<1000?1:0,0,
        w[0],w[4],w[8],w[12],w[1],w[5],w[9],w[13],w[2],w[6],w[10],w[14]],index*52);
      const p=validatedColorProjection(camera.metadata);
      if(p){const e=p.extrinsics;
        metadata.set([e[0],e[3],e[6],e[9],e[1],e[4],e[7],e[10],e[2],e[5],e[8],e[11],
          ...p.intrinsics,...p.size,camera.metadata.color_space==='original_rgb'?1:0,1],index*52+32);
      }
      const revision = `${camera.uploadRevision ?? camera.lastUpdateMs}:${delta}:${edge}:${camera.intrinsicsArray}`;
      const previous = this.uploads[index];
      if (!previous || previous.camera !== camera || previous.revision !== revision) changed.push({camera,index,revision});
    });
    const key = metadata.toString()+":"+(settings.depth_only_enabled===true?"gray":"color");
    this.metadataValues = metadata;
    if (!changed.length && key === this.metadataSignature) return;
    const oldFramebuffer = restoreState ? restoreState.framebuffer : gl.getParameter(gl.FRAMEBUFFER_BINDING);
    const oldVao = restoreState ? restoreState.vertexArray : gl.getParameter(gl.VERTEX_ARRAY_BINDING);
    const viewport = restoreState ? restoreState.viewport : gl.getParameter(gl.VIEWPORT);
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
        if(this.copyDepthOnGpu) {
          // Reuse the exact R16UI upload already made by uploadCamera. This
          // GPU copy avoids transferring the same native depth array twice.
          gl.bindFramebuffer(gl.READ_FRAMEBUFFER,this.depthCopyFramebuffer);
          gl.framebufferTexture2D(gl.READ_FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,camera.depthTexture,0);
          gl.readBuffer(gl.COLOR_ATTACHMENT0);
          gl.copyTexSubImage3D(gl.TEXTURE_2D_ARRAY,0,0,0,index,0,0,camera.width,camera.height);
        } else gl.texSubImage3D(gl.TEXTURE_2D_ARRAY,0,0,0,index,camera.width,camera.height,1,gl.RED_INTEGER,gl.UNSIGNED_SHORT,camera.depth);
        gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,camera.depthTexture);
        gl.activeTexture(gl.TEXTURE1);gl.bindTexture(gl.TEXTURE_2D,camera.colorTexture);
        gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,this.quality,0,index);
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT1,this.color,0,index);
        gl.viewport(0,0,camera.width,camera.height);
        gl.uniform2i(u.u_size,camera.width,camera.height);
        gl.uniform4fv(u.u_intrinsics,camera.intrinsicsArray);
        bindColorProjection(gl,this.program,camera);
        gl.drawArrays(gl.TRIANGLES,0,3);
        if(originalRgb) {
          // Copy the original RGB texture on-GPU. Never resample onto depth UVs.
          gl.bindFramebuffer(gl.READ_FRAMEBUFFER,this.colorCopyFramebuffer);
          gl.framebufferTexture2D(gl.READ_FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,camera.colorTexture,0);
          gl.readBuffer(gl.COLOR_ATTACHMENT0);
          gl.activeTexture(gl.TEXTURE14);gl.bindTexture(gl.TEXTURE_2D_ARRAY,this.rgb);
          gl.copyTexSubImage3D(gl.TEXTURE_2D_ARRAY,0,0,0,index,0,0,camera.colorWidth||camera.width,camera.colorHeight||camera.height);
        }
        this.uploads[index] = {camera,revision};this.rebuilds++;
      });
      // Transforms/freshness can change without a new depth capture.
      if (key !== this.metadataSignature) {
        gl.activeTexture(gl.TEXTURE5);gl.bindTexture(gl.TEXTURE_2D,this.data);
        gl.texSubImage2D(gl.TEXTURE_2D,0,0,0,13,cameras.length,gl.RGBA,gl.FLOAT,metadata);
        this.metadataSignature = key;
      }
      this.visibility.update(cameras,this.quality,inverseTransform,originalRgb,now,settings.depth_only_enabled!==true,this.uploads.map(entry=>entry.revision));
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
