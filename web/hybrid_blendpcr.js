// Browser/RealSense adaptation of BlendPCR's dominance and spatial blending.
// Original approach: Andre Muehlenbrock, CGVR, University of Bremen, 2024.
// Adapted from godot/runtime/point_cloud/blendpcr in this repository.
// SPDX-License-Identifier: EUPL-1.2
import { FULLSCREEN_VERTEX_SHADER, RIG_SAMPLING_SHADER, compileGpuProgram, gpuTexture } from "./hybrid_camera_quality.js?v=20261020_native_asyncgl";

const METADATA_FRAGMENT_SHADER = `#version 300 es
precision highp float;
precision highp int;
in float v_valid;
flat in uint v_triangle_id;
layout(location=0) out highp uint out_primitive;
void main() {
  if(v_valid<0.9999)discard;
  out_primitive=v_triangle_id;
}`;

// Keep one unique provoking vertex per triangle for its exact integer ID and
// validity. The other corners use shared grid vertices. No samples, triangles
// or output pixels are removed. Masked robot triangles retain the old path.
const INDEXED_METADATA_VERTEX_SHADER = `#version 300 es
precision highp float;
precision highp int;
precision highp usampler2D;
uniform usampler2D u_depth;
uniform highp sampler2DArray u_rig_quality;
uniform highp int u_rig_index;
uniform ivec2 u_size;
uniform vec4 u_intrinsics;
uniform mat4 u_model;
uniform mat4 u_view_projection;
flat out float v_valid;
flat out uint v_triangle_id;
void main() {
  int grid_count=u_size.x*u_size.y;
  ivec2 pixel;
  v_valid=1.0;v_triangle_id=0u;
  if(gl_VertexID<grid_count)pixel=ivec2(gl_VertexID%u_size.x,gl_VertexID/u_size.x);
  else {
    int primitive=gl_VertexID-grid_count,cell=primitive/2;
    ivec2 top_left=ivec2(cell%(u_size.x-1),cell/(u_size.x-1));
    vec4 quality=texelFetch(u_rig_quality,ivec3(top_left,u_rig_index),0);
    v_valid=((primitive%2)==0?quality.r:quality.g)>0.5?1.0:0.0;
    v_triangle_id=uint(primitive+1);
    // Invalid triangles must be degenerate before rasterization. Moving only
    // their provoking corner off-screen makes enormous discarded triangles
    // around depth holes, wasting fill rate despite identical final pixels.
    pixel=v_valid>0.5?top_left+ivec2(0,1):top_left+ivec2((primitive%2)==0?0:1,0);
  }
  uint mm=texelFetch(u_depth,pixel,0).r;
  if(mm==0u){gl_Position=vec4(2.0,2.0,2.0,1.0);return;}
  float z=float(mm)*0.001;
  vec3 local_position=vec3((float(pixel.x)-u_intrinsics.z)*z/u_intrinsics.x,
    -(float(pixel.y)-u_intrinsics.w)*z/u_intrinsics.y,-z);
  vec3 world_position=(u_model*vec4(local_position,1.0)).xyz;
  gl_Position=u_view_projection*vec4(world_position,1.0);
}`;

const CAMERA_SAMPLES_SHADER = `
${RIG_SAMPLING_SHADER}
uniform highp usampler2DArray u_metadata;
uniform ivec2 u_output_size;
uniform mat4 u_viewer;
uniform mat4 u_view;
uniform mat4 u_projection;
bool sample_camera(int camera,vec2 uv,out RigHit hit,out float view_z) {
  ivec2 pixel=clamp(ivec2(uv*vec2(u_output_size)),ivec2(0),u_output_size-ivec2(1));
  uint id=texelFetch(u_metadata,ivec3(pixel,camera),0).r;
  if(id==0u)return false;
  // The triangle ID identifies this metadata texel, including in the smaller
  // dominance pass. Reconstruct its ray, rather than an adjacent pixel's ray.
  vec2 screen=(vec2(pixel)+vec2(0.5))/vec2(u_output_size)*2.0-1.0;
  vec3 ray=mat3(u_viewer)*vec3(screen.x/u_projection[0][0],screen.y/u_projection[1][1],-1.0);
  vec3 origin=rig_local(camera,u_viewer[3].xyz);
  if(!rig_triangle(camera,id,origin,rig_direction(camera,ray),hit))return false;
  view_z=-(u_view*vec4(hit.world,1.0)).z;
  return view_z>0.0;
}
bool sample_usable(int camera,vec2 uv,out RigHit hit,out float z) {
  return sample_camera(camera,uv,hit,z)&&!rig_veto_hit(camera,hit.world,hit);
}
`;

const DOMINANCE_FRAGMENT_SHADER = `#version 300 es
precision highp float;
precision highp int;
${CAMERA_SAMPLES_SHADER}
uniform ivec2 u_pass_size;
layout(location=0) out highp uint out_owner;
void main() {
  vec2 uv=gl_FragCoord.xy/vec2(u_pass_size);
  float nearest=1e30;
  float depths[RIG_CAMERA_CAPACITY];
  float qualities[RIG_CAMERA_CAPACITY];
  for(int camera=0;camera<u_rig_count;camera++) {
    RigHit hit;float z;
    bool valid=sample_usable(camera,uv,hit,z);
    depths[camera]=valid?z:1e30;qualities[camera]=valid?hit.score:-1.0;
    if(valid)nearest=min(nearest,z);
  }
  int owner=-1;float best=-1.0;
  for(int camera=0;camera<u_rig_count;camera++) {
    float q=qualities[camera];
    if(q<0.0||abs(depths[camera]-nearest)>rig_tolerance(nearest))continue;
    if(owner<0||rig_better(q,camera,best,owner)){best=q;owner=camera;}
  }
  out_owner=uint(owner+1);
}`;

const HORIZONTAL_FRAGMENT_SHADER = `#version 300 es
precision highp float;
precision highp int;
uniform highp usampler2D u_owner;
uniform ivec2 u_pass_size;
uniform int u_group;
layout(location=0) out highp uvec4 out_counts;
void main() {
  ivec2 p=ivec2(gl_FragCoord.xy);
  uvec4 counts=uvec4(0);
  for(int x=-3;x<=3;x++) {
    uint owner=texelFetch(u_owner,clamp(p+ivec2(x,0),ivec2(0),u_pass_size-ivec2(1)),0).r;
    for(int channel=0;channel<4;channel++)if(owner==uint(u_group*4+channel+1))counts[channel]++;
  }
  out_counts=counts;
}`;

const VERTICAL_FRAGMENT_SHADER = `#version 300 es
precision highp float;
precision highp int;
uniform highp usampler2DArray u_counts;
uniform ivec2 u_pass_size;
uniform int u_group;
layout(location=0) out vec4 out_weights;
void main() {
  ivec2 p=ivec2(gl_FragCoord.xy);
  uvec4 counts=uvec4(0);
  for(int y=-3;y<=3;y++)counts+=texelFetch(u_counts,ivec3(clamp(p+ivec2(0,y),ivec2(0),u_pass_size-ivec2(1)),u_group),0);
  out_weights=vec4(counts)/49.0;
}`;

const COMPOSITE_FRAGMENT_SHADER = `#version 300 es
precision highp float;
precision highp int;
${CAMERA_SAMPLES_SHADER}
uniform highp sampler2DArray u_weights;
out vec4 out_color;
vec3 to_linear(vec3 c) {return mix(c/12.92,pow(max((c+0.055)/1.055,vec3(0)),vec3(2.4)),step(vec3(0.04045),c));}
vec3 to_srgb(vec3 c) {return mix(c*12.92,1.055*pow(max(c,vec3(0)),vec3(1.0/2.4))-0.055,step(vec3(0.0031308),c));}
void main() {
  vec2 uv=gl_FragCoord.xy/vec2(u_output_size);
  float nearest=1e30;
  RigHit samples[RIG_CAMERA_CAPACITY];
  float depths[RIG_CAMERA_CAPACITY];
  bool valid[RIG_CAMERA_CAPACITY];
  for(int camera=0;camera<u_rig_count;camera++) {
    RigHit hit;float z;
    valid[camera]=sample_usable(camera,uv,hit,z);
    samples[camera]=hit;depths[camera]=z;
    if(valid[camera])nearest=min(nearest,z);
  }
  int owner=-1;float quality=-1.0;
  vec3 color=vec3(0),best_color=vec3(0),best_world=vec3(0);
  float total=0.0;
  for(int camera=0;camera<u_rig_count;camera++) {
    if(!valid[camera]||abs(depths[camera]-nearest)>rig_tolerance(nearest))continue;
    RigHit hit=samples[camera];
    if(owner<0||rig_better(hit.score,camera,quality,owner)) {
      owner=camera;quality=hit.score;best_color=hit.color;best_world=hit.world;
    }
    float weight=texture(u_weights,vec3(uv,float(camera/4)))[camera%4];
    color+=to_linear(hit.color)*weight;
    total+=weight;
  }
  if(owner<0)discard;
  // Thin surfaces can disappear from the small dominance grid. They still
  // retain their full-resolution coverage and their best local source color.
  out_color=vec4(total>0.00001?to_srgb(color/total):best_color,1.0);
  vec4 clip=u_projection*u_view*vec4(best_world,1.0);
  gl_FragDepth=(clip.z/clip.w)*0.5+0.5;
}`;

const PRESENT_FRAGMENT_SHADER = `#version 300 es
precision highp float;
uniform sampler2D u_present_color;
uniform sampler2D u_present_depth;
out vec4 out_color;
void main() {
  ivec2 p=ivec2(gl_FragCoord.xy);
  out_color=texelFetch(u_present_color,p,0);
  gl_FragDepth=texelFetch(u_present_depth,p,0).r;
}`;

function uniforms(gl, program) {
  const result = {};
  const count = gl.getProgramParameter(program,gl.ACTIVE_UNIFORMS);
  for (let i=0;i<count;i++) {
    const name = gl.getActiveUniform(program,i).name.replace(/\[0\]$/,"");
    result[name] = gl.getUniformLocation(program,name);
  }
  return new Proxy(result,{get:(target,key)=>target[key]??null});
}

export class BrowserBlendPCR {
  constructor(gl, vertexSource, {indexedMetadata = true, staticCameraLoops = true, uniformCameraData = true} = {}) {
    this.gl = gl;
    this.vertexSource = vertexSource;
    this.signature = "";
    this.programs = null;
    this.failedSignature = "";
    this.compositions = 0;
    this.metadataPasses = 0;
    this.indexedMetadata = indexedMetadata;
    this.staticCameraLoops = staticCameraLoops;
    this.uniformCameraData = uniformCameraData;
    this.gridBuffers = new Map();
  }

  clearTargets() {
    const gl = this.gl;
    for (const key of ["metadata","owner","counts","weights","compositeColor","compositeDepth"]) {
      if(this[key])gl.deleteTexture(this[key]);this[key]=null;
    }
    if(this.depth)gl.deleteRenderbuffer(this.depth);
    if(this.framebuffer)gl.deleteFramebuffer(this.framebuffer);
    this.depth=this.framebuffer=null;this.signature="";
    this.cacheKey="";
    this.metadataLayerKeys=[];
    this.metadataLayerCameras=[];
    for(const entry of this.gridBuffers.values())gl.deleteBuffer(entry.buffer);
    this.gridBuffers.clear();
  }

  dispose() {
    this.clearTargets();
    if(this.programs)for(const value of Object.values(this.programs))this.gl.deleteProgram(value.program);
    if(this.vao)this.gl.deleteVertexArray(this.vao);
    this.vao=null;this.programs=null;this.failedSignature="";
  }

  prepare(width,height,count) {
    const gl = this.gl;
    const signature = `${width}:${height}:${count}`;
    if(this.failedSignature===signature)throw new Error("BlendPCR unavailable on this GPU; using independent mesh");
    if(!this.programs || this.capacity!==count) {
      this.dispose();
      this.programs={};
      try {
        for(const [name,vertex,fragment] of [
          ["metadata",this.vertexSource,METADATA_FRAGMENT_SHADER],
          ["metadataIndexed",INDEXED_METADATA_VERTEX_SHADER,METADATA_FRAGMENT_SHADER.replace("in float v_valid;","flat in float v_valid;")],
          ["dominance",FULLSCREEN_VERTEX_SHADER,DOMINANCE_FRAGMENT_SHADER],
          ["horizontal",FULLSCREEN_VERTEX_SHADER,HORIZONTAL_FRAGMENT_SHADER],
          ["vertical",FULLSCREEN_VERTEX_SHADER,VERTICAL_FRAGMENT_SHADER],
          ["composite",FULLSCREEN_VERTEX_SHADER,COMPOSITE_FRAGMENT_SHADER],
          ["present",FULLSCREEN_VERTEX_SHADER,PRESENT_FRAGMENT_SHADER]]) {
          // Specialize only array capacity, never resolution or sampling. Each
          // camera triangle is reconstructed once per output pixel per pass.
          let source=fragment.replace('#version 300 es',`#version 300 es\n#define RIG_CAMERA_CAPACITY ${count}`);
          // Camera transforms/intrinsics are constant across the frame. Load
          // their exact float32 values from uniforms instead of issuing many
          // repeated texture fetches per output pixel. Large rigs retain the
          // texture path when they exceed the guaranteed uniform budget.
          if(this.uniformCameraData&&count*8+64<=gl.getParameter(gl.MAX_FRAGMENT_UNIFORM_VECTORS))
            source=source.replace('#version 300 es','#version 300 es\n#define RIG_UNIFORM_DATA');
          // Capacity is the exact current rig size and programs rebuild when
          // it changes. Constant loop bounds let ANGLE unroll camera/peer
          // sampling and scalarize the per-camera hit arrays.
          if(this.staticCameraLoops)source=source.replaceAll('camera<u_rig_count','camera<RIG_CAMERA_CAPACITY')
            .replaceAll('peer<u_rig_count','peer<RIG_CAMERA_CAPACITY');
          const program=compileGpuProgram(gl,vertex,source);
          this.programs[name]={program,uniforms:uniforms(gl,program)};
        }
        this.vao=gl.createVertexArray();
        this.capacity=count;
      } catch(error) {this.dispose();this.failedSignature=signature;throw error;}
    }
    if(this.signature===signature)return;
    this.clearTargets();
    try {
      if(count<1||count>gl.getParameter(gl.MAX_ARRAY_TEXTURE_LAYERS)
        ||Math.max(width,height)>gl.getParameter(gl.MAX_TEXTURE_SIZE))throw new Error("BlendPCR exceeds GPU texture limits");
      this.width=width;this.height=height;
      this.smallWidth=Math.max(1,Math.ceil(width/4));this.smallHeight=Math.max(1,Math.ceil(height/4));
      this.groups=Math.ceil(count/4);
      this.metadata=gpuTexture(gl,gl.TEXTURE_2D_ARRAY,8,gl.R32UI,width,height,count);
      this.owner=gpuTexture(gl,gl.TEXTURE_2D,11,gl.R32UI,this.smallWidth,this.smallHeight);
      this.counts=gpuTexture(gl,gl.TEXTURE_2D_ARRAY,9,gl.RGBA8UI,this.smallWidth,this.smallHeight,this.groups);
      this.weights=gpuTexture(gl,gl.TEXTURE_2D_ARRAY,10,gl.RGBA8,this.smallWidth,this.smallHeight,this.groups,true);
      this.compositeColor=gpuTexture(gl,gl.TEXTURE_2D,12,gl.RGBA8,width,height);
      this.compositeDepth=gpuTexture(gl,gl.TEXTURE_2D,13,gl.DEPTH_COMPONENT24,width,height);
      this.framebuffer=gl.createFramebuffer();
      this.depth=gl.createRenderbuffer();gl.bindRenderbuffer(gl.RENDERBUFFER,this.depth);
      gl.renderbufferStorage(gl.RENDERBUFFER,gl.DEPTH_COMPONENT24,width,height);
      gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
      gl.framebufferRenderbuffer(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,gl.RENDERBUFFER,this.depth);
      gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,this.metadata,0,0);
      gl.drawBuffers([gl.COLOR_ATTACHMENT0]);
      if(gl.checkFramebufferStatus(gl.FRAMEBUFFER)!==gl.FRAMEBUFFER_COMPLETE||gl.getError()!==gl.NO_ERROR)
        throw new Error("BlendPCR integer framebuffer unavailable");
      this.signature=signature;
    } catch(error) {this.clearTargets();this.failedSignature=signature;throw error;}
    finally {gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.bindRenderbuffer(gl.RENDERBUFFER,null);}
  }

  gridBuffer(width,height) {
    const key=`${width}:${height}`;
    if(this.gridBuffers.has(key))return this.gridBuffers.get(key);
    const gl=this.gl,cells=(width-1)*(height-1),grid=width*height;
    const indices=new Uint32Array(cells*6);
    let offset=0;
    // Small strips keep adjacent rows' shared vertices in the transform
    // cache. Primitive IDs remain the original row-major IDs.
    for(let by=0;by<height-1;by+=8)for(let bx=0;bx<width-1;bx+=4)
    for(let y=by;y<Math.min(by+8,height-1);y++)for(let x=bx;x<Math.min(bx+4,width-1);x++) {
      const cell=y*(width-1)+x,a=y*width+x,b=a+1,d=a+width+1;
      indices[offset++]=a;indices[offset++]=b;indices[offset++]=grid+cell*2;
      indices[offset++]=b;indices[offset++]=d;indices[offset++]=grid+cell*2+1;
    }
    const buffer=gl.createBuffer();gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER,buffer);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER,indices,gl.STATIC_DRAW);
    if(gl.getError()!==gl.NO_ERROR){gl.deleteBuffer(buffer);throw new Error("BlendPCR grid allocation failed");}
    const entry={buffer,count:indices.length};this.gridBuffers.set(key,entry);return entry;
  }

  use(name,rig,view) {
    const gl=this.gl,{program,uniforms:u}=this.programs[name];
    gl.useProgram(program);
    if(name==="dominance"||name==="composite") {
      rig.bind(program);
      gl.activeTexture(gl.TEXTURE8);gl.bindTexture(gl.TEXTURE_2D_ARRAY,this.metadata);
      gl.uniform1i(u.u_metadata,8);
      gl.uniform2i(u.u_output_size,this.width,this.height);
      gl.uniformMatrix4fv(u.u_viewer,false,view.viewer);
      gl.uniformMatrix4fv(u.u_view,false,view.inverseViewer);
      gl.uniformMatrix4fv(u.u_projection,false,view.projection);
      gl.uniform1f(u.u_fusion_depth_tolerance,0.035);
      gl.uniform1i(u.u_robot_mask_enabled,this.maskCapsules?1:0);
      if(this.maskCapsules){gl.uniform4fv(u.u_mask_capsule_a,this.maskCapsules.starts);gl.uniform4fv(u.u_mask_capsule_b,this.maskCapsules.ends);}
    }
    return u;
  }

  attach(texture,layer,depth=false) {
    const gl=this.gl;
    gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
    gl.framebufferRenderbuffer(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,gl.RENDERBUFFER,depth?this.depth:null);
    if(layer===null)gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,texture,0);
    else gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,texture,0,layer);
  }

  render(rig,cameras,view,maskCapsules,settings) {
    const gl=this.gl;
    this.prepare(view.width,view.height,cameras.length);
    this.maskCapsules=maskCapsules;
    const previousVao=view.restoreVertexArray===undefined?gl.getParameter(gl.VERTEX_ARRAY_BINDING):view.restoreVertexArray;
    gl.bindVertexArray(this.vao);gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);gl.depthMask(true);
    try {
      const key=`${rig.version}:${view.viewProjection}:${maskCapsules?.starts}:${maskCapsules?.ends}:${settings.white_background_enabled}`;
      if(this.cacheKey!==key) {
        // One dense native mesh pass per camera; no second color rasterization.
        const indexed=this.indexedMetadata&&!maskCapsules;
        const u=this.use(indexed?"metadataIndexed":"metadata",rig,view);
        for(const [name,unit] of [["u_depth",0],["u_color",1],["u_fusion_depth",2],["u_fusion_color",3],["u_rig_quality",6]])gl.uniform1i(u[name],unit);
        gl.uniform1i(u.u_fusion_enabled,0);gl.uniform1i(u.u_quality_enabled,1);
        gl.activeTexture(gl.TEXTURE6);gl.bindTexture(gl.TEXTURE_2D_ARRAY,rig.quality);
        gl.uniform1i(u.u_render_mode,1);
        gl.uniform1f(u.u_mesh_depth_delta,Number(settings.mesh_depth_delta||0.045));
        gl.uniform1f(u.u_mesh_max_edge,Number(settings.mesh_max_edge||0.08));
        gl.uniformMatrix4fv(u.u_view_projection,false,view.viewProjection);
        gl.uniform1i(u.u_robot_mask_enabled,maskCapsules?1:0);
        if(maskCapsules){gl.uniform4fv(u.u_mask_capsule_a,maskCapsules.starts);gl.uniform4fv(u.u_mask_capsule_b,maskCapsules.ends);}
        gl.viewport(0,0,this.width,this.height);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);
        const rasterKey=`${view.viewProjection}:${maskCapsules?.starts}:${maskCapsules?.ends}:${settings.mesh_depth_delta}:${settings.mesh_max_edge}`;
        cameras.forEach((camera,index)=> {
          const fresh=rig_data_fresh(camera,view.now);
          const layerKey=`${rasterKey}:${camera.metadata.id}:${camera.uploadRevision??camera.lastUpdateMs}:${camera.width}:${camera.height}:${camera.modelMatrix}:${camera.intrinsicsArray}:${fresh}`;
          if(this.metadataLayerCameras[index]===camera&&this.metadataLayerKeys[index]===layerKey)return;
          this.attach(this.metadata,index,true);
          gl.clearBufferuiv(gl.COLOR,0,new Uint32Array(4));gl.clearBufferfv(gl.DEPTH,0,new Float32Array([1]));
          if(fresh) {
            gl.uniform1i(u.u_rig_index,index);
            gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,camera.depthTexture);
            gl.activeTexture(gl.TEXTURE1);gl.bindTexture(gl.TEXTURE_2D,camera.colorTexture);
            gl.uniformMatrix4fv(u.u_model,false,camera.modelMatrix);
            gl.uniform4fv(u.u_intrinsics,camera.intrinsicsArray);
            gl.uniform2i(u.u_size,camera.width,camera.height);
            if(indexed) {
              const grid=this.gridBuffer(camera.width,camera.height);
              gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER,grid.buffer);
              gl.drawElements(gl.TRIANGLES,grid.count,gl.UNSIGNED_INT,0);
            } else gl.drawArrays(gl.TRIANGLES,0,(camera.width-1)*(camera.height-1)*6);
          }
          this.metadataLayerKeys[index]=layerKey;
          this.metadataLayerCameras[index]=camera;
          this.metadataPasses++;
        });
        gl.disable(gl.DEPTH_TEST);gl.viewport(0,0,this.smallWidth,this.smallHeight);
        this.attach(this.owner,null);
        const dominant=this.use("dominance",rig,view);
        gl.uniform2i(dominant.u_pass_size,this.smallWidth,this.smallHeight);
        gl.drawArrays(gl.TRIANGLES,0,3);
        const horizontal=this.use("horizontal",rig,view);
        gl.activeTexture(gl.TEXTURE11);gl.bindTexture(gl.TEXTURE_2D,this.owner);
        gl.uniform1i(horizontal.u_owner,11);gl.uniform2i(horizontal.u_pass_size,this.smallWidth,this.smallHeight);
        for(let group=0;group<this.groups;group++) {
          this.attach(this.counts,group);gl.uniform1i(horizontal.u_group,group);gl.drawArrays(gl.TRIANGLES,0,3);
        }
        const vertical=this.use("vertical",rig,view);
        gl.activeTexture(gl.TEXTURE9);gl.bindTexture(gl.TEXTURE_2D_ARRAY,this.counts);
        gl.uniform1i(vertical.u_counts,9);gl.uniform2i(vertical.u_pass_size,this.smallWidth,this.smallHeight);
        for(let group=0;group<this.groups;group++) {
          this.attach(this.weights,group);gl.uniform1i(vertical.u_group,group);gl.drawArrays(gl.TRIANGLES,0,3);
        }
        this.attach(this.compositeColor,null);
        gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,gl.TEXTURE_2D,this.compositeDepth,0);
        gl.viewport(0,0,this.width,this.height);
        gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);
        gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
        const composite=this.use("composite",rig,view);
        gl.activeTexture(gl.TEXTURE10);gl.bindTexture(gl.TEXTURE_2D_ARRAY,this.weights);
        gl.uniform1i(composite.u_weights,10);
        gl.drawArrays(gl.TRIANGLES,0,3);
        this.cacheKey=key;this.compositions++;
      }
      // Identical captures and viewpoints reuse the exact full-resolution
      // color + depth, not an accumulated or reprojected approximation. Camera
      // updates, motion, intrinsics, freshness and mask changes invalidate it.
      gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.viewport(0,0,this.width,this.height);
      gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.ALWAYS);
      const present=this.use("present",rig,view);
      gl.activeTexture(gl.TEXTURE12);gl.bindTexture(gl.TEXTURE_2D,this.compositeColor);
      gl.activeTexture(gl.TEXTURE13);gl.bindTexture(gl.TEXTURE_2D,this.compositeDepth);
      gl.uniform1i(present.u_present_color,12);gl.uniform1i(present.u_present_depth,13);
      gl.drawArrays(gl.TRIANGLES,0,3);
    } finally {
      gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.bindVertexArray(previousVao);
      gl.viewport(0,0,view.width,view.height);gl.enable(gl.DEPTH_TEST);gl.depthMask(true);gl.depthFunc(gl.LEQUAL);
    }
  }
}

function rig_data_fresh(camera,now) {return camera.width>1&&camera.height>1&&now-camera.lastUpdateMs<1000;}
