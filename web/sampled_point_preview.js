import {sampleDepthPoints} from './point_preview_sampling.js?v=20261027_compact_gpu';
import {compileGpuProgram} from './hybrid_camera_quality.js?v=20261027_compact_gpu';
import {COLOR_PROJECTION_SHADER,bindColorProjection} from './hybrid_color_projection.js?v=20261021_original_rgb';
import {MODELED_WORKSPACE_SHADER,bindModeledWorkspace} from './modeled_workspace.js?v=20261021_original_rgb';

const VERTEX = `#version 300 es
precision highp float;precision highp int;precision highp usampler2D;
layout(location=0) in uint a_sample;
layout(location=1) in uint a_depth;
layout(location=2) in vec3 a_color;
uniform int u_compact;
uniform usampler2D u_depth;uniform sampler2D u_color;
uniform ivec2 u_size;uniform vec4 u_intrinsics;
uniform mat4 u_model;uniform mat4 u_view_projection;
uniform float u_point_scale;uniform float u_max_point_size;uniform int u_gray;
uniform int u_mask_enabled;uniform vec4 u_mask_a[6];uniform vec4 u_mask_b[6];
out vec3 v_color;flat out int v_valid;
${COLOR_PROJECTION_SHADER}
${MODELED_WORKSPACE_SHADER}
void main(){
  v_valid=0;v_color=vec3(.55);gl_Position=vec4(2,2,2,1);gl_PointSize=1.;
  ivec2 pixel=ivec2(int(a_sample)%u_size.x,int(a_sample)/u_size.x);
  float z=float(u_compact==1?a_depth:texelFetch(u_depth,pixel,0).r)*.001;if(z<=0.)return;
  vec3 local=vec3((float(pixel.x)-u_intrinsics.z)*z/u_intrinsics.x,
    -(float(pixel.y)-u_intrinsics.w)*z/u_intrinsics.y,-z);
  vec3 world=(u_model*vec4(local,1)).xyz;
  if(modeled_workspace_outside(world))return;
  if(u_mask_enabled==1)for(int i=0;i<6;i++){
    vec3 segment=u_mask_b[i].xyz-u_mask_a[i].xyz;float squared=dot(segment,segment);
    float along=squared>.000001?clamp(dot(world-u_mask_a[i].xyz,segment)/squared,0.,1.):0.;
    vec3 offset=world-u_mask_a[i].xyz-segment*along;
    if(dot(offset,offset)<=u_mask_a[i].w*u_mask_a[i].w)return;
  }
  vec4 clip=u_view_projection*vec4(world,1);if(clip.w<=0.)return;
  vec4 color=u_compact==1?vec4(a_color,1):camera_sample_color(u_color,local,pixel);
  v_color=u_gray==1?vec3(.6):color.a>0.?color.rgb:vec3(.55);
  gl_Position=clip;gl_PointSize=clamp(u_point_scale/clip.w,1.,u_max_point_size);v_valid=1;
}`;
const FRAGMENT = `#version 300 es
precision highp float;precision highp int;
in vec3 v_color;flat in int v_valid;out vec4 out_color;
void main(){if(v_valid==0)discard;out_color=vec4(v_color,1);}`;

function dummyTexture(gl,internal,format,type,data){
  const texture=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,texture);
  for(const parameter of [gl.TEXTURE_MIN_FILTER,gl.TEXTURE_MAG_FILTER])gl.texParameteri(gl.TEXTURE_2D,parameter,gl.NEAREST);
  gl.texImage2D(gl.TEXTURE_2D,0,internal,1,1,0,format,type,data);return texture;
}
function compactPacket(sample,budget){
  const count=budget===0?sample.indices.length:Math.min(sample.indices.length,budget);
  // Reuse the transferred block when no budget change requires a subset.
  if(count===sample.indices.length&&sample.indices.byteOffset===0&&sample.depths.buffer===sample.indices.buffer&&sample.depths.byteOffset===count*4&&sample.colors.buffer===sample.indices.buffer&&sample.colors.byteOffset===count*6)
    return {bytes:new Uint8Array(sample.indices.buffer,0,count*9),count,valid:sample.valid};
  const buffer=new ArrayBuffer(count*9);
  const indices=new Uint32Array(buffer,0,count),depths=new Uint16Array(buffer,count*4,count),colors=new Uint8Array(buffer,count*6,count*3);
  for(let i=0;i<count;i++){
    const source=Math.floor((i+.5)*sample.indices.length/count);
    indices[i]=sample.indices[source];depths[i]=sample.depths[source];
    for(let c=0;c<3;c++)colors[i*3+c]=sample.colors[source*3+c];
  }
  return {bytes:new Uint8Array(buffer),count,valid:sample.valid};
}
// The sparse wire representation is also the GPU representation: 9 bytes/point.
export class SampledPointPreview {
  constructor(gl) {
    this.gl=gl;this.entries=new Map();this.vao=gl.createVertexArray();
    this.program=compileGpuProgram(gl,VERTEX,FRAGMENT);this.u={};
    this.emptyDepth=dummyTexture(gl,gl.R16UI,gl.RED_INTEGER,gl.UNSIGNED_SHORT,new Uint16Array(1));
    this.emptyColor=dummyTexture(gl,gl.RGBA8,gl.RGBA,gl.UNSIGNED_BYTE,new Uint8Array([0,0,0,255]));
    for(const name of ['u_compact','u_depth','u_color','u_size','u_intrinsics','u_model','u_view_projection',
      'u_point_scale','u_max_point_size','u_gray','u_mask_enabled','u_mask_a[0]','u_mask_b[0]'])
      this.u[name]=gl.getUniformLocation(this.program,name);
    this.maxPointSize=Math.min(32,gl.getParameter(gl.ALIASED_POINT_SIZE_RANGE)[1]);
  }
  dropCamera(camera) {const entry=this.entries.get(camera);if(entry){this.gl.deleteBuffer(entry.buffer);this.entries.delete(camera);}}
  render(cameras,view,mask,settings) {
    const cleanup={input:0,removed:0,matches:true};
    const gl=this.gl,u=this.u,counts={},sourceCounts={};let selectionMs=0,gpuUploadBytes=0,compactCameras=0;const budget=settings.point_preview_budget===0?0:settings.point_preview_budget===40000?40000:20000;
    gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.bindVertexArray(this.vao);gl.useProgram(this.program);
    gl.viewport(0,0,view.width,view.height);gl.enable(gl.DEPTH_TEST);gl.depthMask(true);gl.depthFunc(gl.LEQUAL);
    gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);
    gl.uniform1i(u.u_depth,0);gl.uniform1i(u.u_color,1);gl.uniform1i(u.u_gray,settings.depth_only_enabled===true?1:0);
    gl.uniformMatrix4fv(u.u_view_projection,false,view.viewProjection);
    // Match Three.js PointsMaterial's 0.005 size and viewport attenuation.
    gl.uniform1f(u.u_point_scale,.005*view.height*.5);gl.uniform1f(u.u_max_point_size,this.maxPointSize);
    gl.uniform1i(u.u_mask_enabled,mask?1:0);
    if(mask){gl.uniform4fv(u['u_mask_a[0]'],mask.starts);gl.uniform4fv(u['u_mask_b[0]'],mask.ends);}
    bindModeledWorkspace(gl,this.program,settings);
    try {
      for(const camera of cameras) {
        if(view.now-camera.lastUpdateMs>=1000)continue;
        let entry=this.entries.get(camera);
        if(!entry){entry={buffer:gl.createBuffer()};this.entries.set(camera,entry);}
        if(entry.depth!==camera.depth||entry.revision!==camera.uploadRevision||entry.budget!==budget) {
          const started=performance.now();
          entry.compact=Boolean(camera.previewSamples?.depths);
          const sample=entry.compact?compactPacket(camera.previewSamples,budget):(camera.previewSamples?.budget===budget?camera.previewSamples:sampleDepthPoints(camera.depth,budget));
          entry.count=entry.compact?sample.count:sample.indices.length;entry.valid=sample.valid;
          const bytes=entry.compact?sample.bytes:sample.indices;
          gl.bindBuffer(gl.ARRAY_BUFFER,entry.buffer);gl.bufferData(gl.ARRAY_BUFFER,bytes,gl.STREAM_DRAW);gpuUploadBytes+=bytes.byteLength;
          entry.depth=camera.depth;entry.revision=camera.uploadRevision;entry.budget=budget;
          selectionMs+=performance.now()-started;
        }else gl.bindBuffer(gl.ARRAY_BUFFER,entry.buffer);
        const meta=camera.metadata;
        cleanup.input+=Number(meta.preview_cleanup_input_points??entry.count);
        cleanup.removed+=Number(meta.preview_cleanup_removed||0);
        cleanup.matches &&= meta.preview_budget===budget && meta.preview_cleanup_kind==='selected_radius_v1'
          && meta.preview_cleanup_enabled===(settings.point_cleanup_enabled===true)
          && meta.preview_cleanup_support===settings.point_cleanup_support
          && meta.preview_cleanup_radius_mm===settings.point_cleanup_radius_mm;
        const id=String(camera.metadata.id);counts[id]=entry.count;sourceCounts[id]=entry.valid;
        gl.enableVertexAttribArray(0);gl.vertexAttribIPointer(0,1,gl.UNSIGNED_INT,0,0);gl.vertexAttribDivisor(0,0);
        gl.uniform1i(u.u_compact,entry.compact?1:0);
        if(entry.compact){
          compactCameras++;
          gl.enableVertexAttribArray(1);gl.vertexAttribIPointer(1,1,gl.UNSIGNED_SHORT,0,entry.count*4);gl.vertexAttribDivisor(1,0);
          gl.enableVertexAttribArray(2);gl.vertexAttribPointer(2,3,gl.UNSIGNED_BYTE,true,0,entry.count*6);gl.vertexAttribDivisor(2,0);
        }else{gl.disableVertexAttribArray(1);gl.disableVertexAttribArray(2);}

        gl.uniform2i(u.u_size,camera.width,camera.height);gl.uniform4fv(u.u_intrinsics,camera.intrinsicsArray);
        gl.uniformMatrix4fv(u.u_model,false,camera.modelMatrix);bindColorProjection(gl,this.program,camera);
        gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,entry.compact?this.emptyDepth:camera.depthTexture);
        gl.activeTexture(gl.TEXTURE1);gl.bindTexture(gl.TEXTURE_2D,entry.compact?this.emptyColor:camera.colorTexture);
        gl.drawArrays(gl.POINTS,0,entry.count);
      }
      for(const [camera,entry]of this.entries)if(!cameras.includes(camera)){gl.deleteBuffer(entry.buffer);this.entries.delete(camera);}
    }finally{gl.bindVertexArray(null);}
    return {budget,counts,sourceCounts,cleanup,total:Object.values(counts).reduce((a,b)=>a+b,0),selectionMs,gpuUploadBytes,compactCameras};
  }
  dispose(){const gl=this.gl;for(const entry of this.entries.values())gl.deleteBuffer(entry.buffer);
    this.entries.clear();gl.deleteTexture(this.emptyDepth);gl.deleteTexture(this.emptyColor);gl.deleteVertexArray(this.vao);gl.deleteProgram(this.program);}
}
