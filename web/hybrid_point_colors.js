// A point's calibrated RGB is independent of the operator's viewpoint. Evaluate
// the existing rig color rules once per capture, retaining every native point.
import {FULLSCREEN_VERTEX_SHADER,RIG_SAMPLING_SHADER,compileGpuProgram,gpuTexture} from './hybrid_camera_quality.js?v=20261021_original_rgb';

const FRAGMENT=`#version 300 es
#define RIG_CAMERA_CAPACITY 32
#define RIG_UNIFORM_DATA
#define RIG_NATIVE_RGB_ONLY
precision highp float;
precision highp int;
uniform highp usampler2D u_depth;
uniform ivec2 u_size;
uniform vec4 u_intrinsics;
uniform mat4 u_model;
uniform int u_source_index;
out vec4 out_color;
${RIG_SAMPLING_SHADER}
void main(){
  ivec2 pixel=ivec2(gl_FragCoord.xy);
  float z=float(texelFetch(u_depth,pixel,0).r)*0.001;
  if(z<=0.0){out_color=vec4(0);return;}
  vec3 local=vec3((float(pixel.x)-u_intrinsics.z)*z/u_intrinsics.x,
    -(float(pixel.y)-u_intrinsics.w)*z/u_intrinsics.y,-z);
  vec3 world=(u_model*vec4(local,1)).xyz,normal=vec3(0,0,1);
  RigHit source;
  if(rig_surface(u_source_index,world,source))normal=rig_world_normal(u_source_index,source.normal);
  out_color=vec4(rig_projected_color(world,normal,vec3(0.34),u_source_index),1);
}`;

export class NativePointColors {
  constructor(gl){this.gl=gl;this.rebuilds=0;this.key='';}
  dispose(){
    const gl=this.gl;
    if(this.texture)gl.deleteTexture(this.texture);
    if(this.framebuffer)gl.deleteFramebuffer(this.framebuffer);
    if(this.program)gl.deleteProgram(this.program);
    if(this.vao)gl.deleteVertexArray(this.vao);
    this.texture=this.framebuffer=this.program=this.vao=this.rig=null;this.signature=this.key='';
  }
  update(rig,cameras,mask,viewport){
    const gl=this.gl,width=Math.max(...cameras.map(c=>c.width)),height=Math.max(...cameras.map(c=>c.height));
    const signature=`${width}:${height}:${cameras.length}`;
    const key=`${rig.version}:${mask?`${mask.starts}:${mask.ends}`:''}`;
    if(signature===this.signature&&key===this.key&&rig===this.rig)return;
    try{
      if(signature!==this.signature){
        this.dispose();
        this.texture=gpuTexture(gl,gl.TEXTURE_2D_ARRAY,8,gl.RGBA8,width,height,cameras.length);
        this.framebuffer=gl.createFramebuffer();this.vao=gl.createVertexArray();
        let fragment=FRAGMENT.replace('#define RIG_CAMERA_CAPACITY 32',`#define RIG_CAMERA_CAPACITY ${cameras.length}`);
        if(cameras.length*13+64>gl.getParameter(gl.MAX_FRAGMENT_UNIFORM_VECTORS))fragment=fragment.replace('#define RIG_UNIFORM_DATA','');
        this.program=compileGpuProgram(gl,FULLSCREEN_VERTEX_SHADER,fragment);
        this.u={};for(const name of ['u_depth','u_size','u_intrinsics','u_model','u_source_index','u_robot_mask_enabled','u_mask_capsule_a[0]','u_mask_capsule_b[0]'])this.u[name]=gl.getUniformLocation(this.program,name);
        gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,this.texture,0,0);
        if(gl.checkFramebufferStatus(gl.FRAMEBUFFER)!==gl.FRAMEBUFFER_COMPLETE)throw new Error('Point color framebuffer unavailable');
        this.signature=signature;
      }
      const u=this.u;gl.useProgram(this.program);rig.bind(this.program);
      gl.bindVertexArray(this.vao);gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
      gl.disable(gl.DEPTH_TEST);gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);gl.disable(gl.SCISSOR_TEST);
      gl.uniform1i(u.u_depth,0);gl.uniform1i(u.u_robot_mask_enabled,mask?1:0);
      if(mask){gl.uniform4fv(u['u_mask_capsule_a[0]'],mask.starts);gl.uniform4fv(u['u_mask_capsule_b[0]'],mask.ends);}
      cameras.forEach((camera,index)=>{
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,this.texture,0,index);
        gl.viewport(0,0,camera.width,camera.height);
        gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,camera.depthTexture);
        gl.uniform2i(u.u_size,camera.width,camera.height);gl.uniform4fv(u.u_intrinsics,camera.intrinsicsArray);
        gl.uniformMatrix4fv(u.u_model,false,camera.modelMatrix);gl.uniform1i(u.u_source_index,index);
        gl.drawArrays(gl.TRIANGLES,0,3);this.rebuilds++;
      });
      this.key=key;this.rig=rig;
    }finally{
      gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.bindVertexArray(null);gl.viewport(...viewport);gl.enable(gl.DEPTH_TEST);
    }
  }
  bind(program){
    const gl=this.gl;gl.activeTexture(gl.TEXTURE8);gl.bindTexture(gl.TEXTURE_2D_ARRAY,this.texture);
    gl.uniform1i(gl.getUniformLocation(program,'u_point_color_cache'),8);
  }
}
