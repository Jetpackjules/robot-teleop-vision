// Cache the existing capsule test at each measured native depth sample.
// Every triangle still rejects if any of its three exact corners is masked.
import {FULLSCREEN_VERTEX_SHADER,compileGpuProgram,gpuTexture} from './hybrid_camera_quality.js?v=20261021_original_rgb';
const FRAGMENT=`#version 300 es
precision highp float;precision highp int;precision highp usampler2D;
uniform usampler2D u_depth;uniform vec4 u_intrinsics;uniform mat4 u_model;
uniform vec4 u_mask_capsule_a[6];uniform vec4 u_mask_capsule_b[6];
layout(location=0) out highp uint out_mask;
void main(){
  ivec2 p=ivec2(gl_FragCoord.xy);float z=float(texelFetch(u_depth,p,0).r)*0.001;
  out_mask=0u;if(z<=0.0)return;
  vec3 world=(u_model*vec4((float(p.x)-u_intrinsics.z)*z/u_intrinsics.x,
    -(float(p.y)-u_intrinsics.w)*z/u_intrinsics.y,-z,1)).xyz;
  for(int i=0;i<6;i++){
    vec3 segment=u_mask_capsule_b[i].xyz-u_mask_capsule_a[i].xyz;
    float length_sq=dot(segment,segment);
    float along=length_sq>0.000001?clamp(dot(world-u_mask_capsule_a[i].xyz,segment)/length_sq,0.0,1.0):0.0;
    vec3 offset=world-u_mask_capsule_a[i].xyz-segment*along;
    if(dot(offset,offset)<=u_mask_capsule_a[i].w*u_mask_capsule_a[i].w){out_mask=1u;return;}
  }
}`;
export class NativeRobotMask {
  constructor(gl){this.gl=gl;this.rebuilds=0;this.layers=[];}
  dispose(){const gl=this.gl;
    if(this.texture)gl.deleteTexture(this.texture);if(this.framebuffer)gl.deleteFramebuffer(this.framebuffer);
    if(this.program)gl.deleteProgram(this.program);if(this.vao)gl.deleteVertexArray(this.vao);
    this.texture=this.framebuffer=this.program=this.vao=null;this.signature='';this.layers=[];
  }
  update(cameras,mask,viewport){
    const gl=this.gl,w=Math.max(...cameras.map(c=>c.width)),h=Math.max(...cameras.map(c=>c.height));
    const signature=`${w}:${h}:${cameras.length}`,maskKey=`${mask.starts}:${mask.ends}`;
    try{
      if(signature!==this.signature){
        this.dispose();this.texture=gpuTexture(gl,gl.TEXTURE_2D_ARRAY,3,gl.R8UI,w,h,cameras.length);
        this.framebuffer=gl.createFramebuffer();this.vao=gl.createVertexArray();
        this.program=compileGpuProgram(gl,FULLSCREEN_VERTEX_SHADER,FRAGMENT);this.u={};
        for(const n of ['u_depth','u_intrinsics','u_model','u_mask_capsule_a[0]','u_mask_capsule_b[0]'])this.u[n]=gl.getUniformLocation(this.program,n);
        gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,this.texture,0,0);
        if(gl.checkFramebufferStatus(gl.FRAMEBUFFER)!==gl.FRAMEBUFFER_COMPLETE)throw Error('Native robot mask framebuffer unavailable');
        this.signature=signature;
      }
      const u=this.u;gl.useProgram(this.program);gl.bindVertexArray(this.vao);gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
      gl.disable(gl.DEPTH_TEST);gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);gl.disable(gl.SCISSOR_TEST);
      gl.uniform1i(u.u_depth,0);gl.uniform4fv(u['u_mask_capsule_a[0]'],mask.starts);gl.uniform4fv(u['u_mask_capsule_b[0]'],mask.ends);
      cameras.forEach((c,i)=>{
        const key=`${c.uploadRevision??c.lastUpdateMs}:${c.modelMatrix}:${c.intrinsicsArray}:${maskKey}`;
        if(this.layers[i]?.camera===c&&this.layers[i].key===key)return;
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,this.texture,0,i);gl.viewport(0,0,c.width,c.height);
        gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,c.depthTexture);
        gl.uniform4fv(u.u_intrinsics,c.intrinsicsArray);gl.uniformMatrix4fv(u.u_model,false,c.modelMatrix);
        gl.drawArrays(gl.TRIANGLES,0,3);this.layers[i]={camera:c,key};this.rebuilds++;
      });
    }finally{gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.bindVertexArray(null);gl.viewport(...viewport);gl.enable(gl.DEPTH_TEST);}
  }
}
