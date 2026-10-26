// Color visibility from the entire measured rig, independent of depth ownership.
// Full native triangles are cached in each original RGB camera's optical frame.
// SPDX-License-Identifier: EUPL-1.2
const VERTEX=`#version 300 es
precision highp float;precision highp int;precision highp usampler2D;
uniform usampler2D u_depth;
uniform highp sampler2DArray u_quality;
uniform int u_source_index;
uniform ivec2 u_size;
uniform vec4 u_intrinsics;
uniform mat4 u_to_rgb_clip;
flat out float v_valid;
void main(){
  int width=u_size.x,vertex=gl_VertexID;
  ivec2 pixel;
  v_valid=1.0;
  if(vertex<width*2)pixel=ivec2(vertex%width,vertex/width+gl_InstanceID);
  else {
    int primitive=vertex-width*2,x=primitive/2;
    ivec2 top=ivec2(x,gl_InstanceID);
    vec4 q=texelFetch(u_quality,ivec3(top,u_source_index),0);
    bool second=(primitive%2)==1;
    v_valid=(second?q.g:q.r)>0.5?1.0:0.0;
    pixel=v_valid>0.5?top+ivec2(0,1):top+ivec2(second?1:0,0);
  }
  float z=float(texelFetch(u_depth,pixel,0).r)*0.001;
  vec3 p=vec3((float(pixel.x)-u_intrinsics.z)*z/u_intrinsics.x,
    -(float(pixel.y)-u_intrinsics.w)*z/u_intrinsics.y,-z);
  gl_Position=u_to_rgb_clip*vec4(p,1.0);
}`;
const FRAGMENT=`#version 300 es
precision highp float;flat in float v_valid;
void main(){if(v_valid<0.5)discard;}`;
export const RGB_VISIBILITY_NEAR=0.01,RGB_VISIBILITY_FAR=65.535;
function multiply(a,b){
  const m=new Float32Array(16);
  for(let c=0;c<4;c++)for(let r=0;r<4;r++)for(let k=0;k<4;k++)m[c*4+r]+=a[k*4+r]*b[c*4+k];
  return m;
}
function projection(p){
  const e=p.extrinsics,k=p.intrinsics,[w,h]=p.size,n=RGB_VISIBILITY_NEAR,f=RGB_VISIBILITY_FAR;
  // RealSense CV axes are +Y down/+Z forward; Godot depth axes are opposite.
  const cv=new Float32Array([e[0],e[1],e[2],0,-e[3],-e[4],-e[5],0,-e[6],-e[7],-e[8],0,e[9],e[10],e[11],1]);
  const clip=new Float32Array([2*k[0]/w,0,0,0,0,2*k[1]/h,0,0,
    2*(k[2]+.5)/w-1,2*(k[3]+.5)/h-1,(f+n)/(f-n),1,0,0,-2*f*n/(f-n),0]);
  return multiply(clip,cv);
}
export class RgbVisibility {
  constructor(gl,compile,allocate){this.gl=gl;this.compile=compile;this.allocate=allocate;this.buffers=new Map();this.passes=0;}
  clear(){const gl=this.gl;if(this.texture)gl.deleteTexture(this.texture);if(this.framebuffer)gl.deleteFramebuffer(this.framebuffer);
    if(this.program)gl.deleteProgram(this.program);if(this.vao)gl.deleteVertexArray(this.vao);
    for(const b of this.buffers.values())gl.deleteBuffer(b.buffer);this.buffers.clear();
    if(this.copyFramebuffer)gl.deleteFramebuffer(this.copyFramebuffer);
    this.texture=this.framebuffer=this.copyFramebuffer=this.program=this.vao=null;
    this.signature='';this.historyKey='';}
  update(cameras,quality,inverse,enabled,now,active=true){
    const gl=this.gl,width=enabled?Math.max(...cameras.map(c=>c.colorWidth||c.width)):1,
      height=enabled?Math.max(...cameras.map(c=>c.colorHeight||c.height)):1;
    const signature=`${width}:${height}:${cameras.length}`;
    if(signature!==this.signature){this.clear();
      // One sampler contains rig-wide depth, source-camera-only depth and the
      // previous rig-wide observation. No additional fragment sampler is needed.
      if(cameras.length*3>gl.getParameter(gl.MAX_ARRAY_TEXTURE_LAYERS))throw new Error('RGB visibility exceeds GPU layer limits');
      this.texture=this.allocate(gl,gl.TEXTURE_2D_ARRAY,15,gl.DEPTH_COMPONENT24,width,height,cameras.length*3);
      this.framebuffer=gl.createFramebuffer();this.copyFramebuffer=gl.createFramebuffer();this.vao=gl.createVertexArray();
      this.program=this.compile(gl,VERTEX,FRAGMENT);this.u={};
      for(const name of ['u_depth','u_quality','u_source_index','u_size','u_intrinsics','u_to_rgb_clip'])this.u[name]=gl.getUniformLocation(this.program,name);
      gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
      gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,this.texture,0,0);
      gl.drawBuffers([gl.NONE]);gl.readBuffer(gl.NONE);
      if(gl.checkFramebufferStatus(gl.FRAMEBUFFER)!==gl.FRAMEBUFFER_COMPLETE)throw new Error('RGB visibility framebuffer unavailable');
      this.signature=signature;
    }
    if(!enabled||!active)return;
    gl.useProgram(this.program);gl.bindVertexArray(this.vao);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LESS);
    gl.depthMask(true);gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);gl.disable(gl.SCISSOR_TEST);
    gl.uniform1i(this.u.u_depth,0);gl.uniform1i(this.u.u_quality,6);
    gl.activeTexture(gl.TEXTURE6);gl.bindTexture(gl.TEXTURE_2D_ARRAY,quality);
    gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);gl.drawBuffers([gl.NONE]);gl.readBuffer(gl.NONE);
    // A calibration/profile change invalidates temporal evidence immediately.
    const historyKey=JSON.stringify(cameras.map(c=>[c.metadata.id,Array.from(c.modelMatrix),
      c.metadata.color_projection,now-c.lastUpdateMs<1000]));
    const hasHistory=this.historyKey===historyKey;
    const copyLayer=(from,to,w,h)=>{
      gl.bindFramebuffer(gl.READ_FRAMEBUFFER,this.framebuffer);
      gl.framebufferTextureLayer(gl.READ_FRAMEBUFFER,gl.DEPTH_ATTACHMENT,this.texture,0,from);
      gl.bindFramebuffer(gl.DRAW_FRAMEBUFFER,this.copyFramebuffer);
      gl.framebufferTextureLayer(gl.DRAW_FRAMEBUFFER,gl.DEPTH_ATTACHMENT,this.texture,0,to);
      gl.drawBuffers([gl.NONE]);gl.readBuffer(gl.NONE);
      gl.blitFramebuffer(0,0,w,h,0,0,w,h,gl.DEPTH_BUFFER_BIT,gl.NEAREST);
      gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
      gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,this.texture,0,from);
    };
    for(let target=0;target<cameras.length;target++){
      const camera=cameras[target];if(camera.metadata.color_space!=='original_rgb')continue;
      gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,this.texture,0,target);
      if(hasHistory)copyLayer(target,cameras.length*2+target,camera.colorWidth,camera.colorHeight);
      gl.viewport(0,0,camera.colorWidth,camera.colorHeight);gl.clearDepth(1);gl.clear(gl.DEPTH_BUFFER_BIT);
      const targetProjection=multiply(projection(camera.metadata.color_projection),inverse(camera.modelMatrix));
      // Draw the RGB camera's own depth first, retaining its self-occlusion
      // evidence before peers contribute. The total triangle draw count is unchanged.
      for(let step=0;step<cameras.length;step++){
        const source=step===0?target:step<=target?step-1:step;
        const c=cameras[source];
        if(now-c.lastUpdateMs>=1000){
          if(step===0)copyLayer(target,cameras.length+target,camera.colorWidth,camera.colorHeight);
          continue;
        }
        let index=this.buffers.get(c.width);
        if(!index){const values=[];for(let x=0;x<c.width-1;x++){
          const a=x,b=x+1,d=c.width+x+1;values.push(a,b,c.width*2+x*2,b,d,c.width*2+x*2+1);
        }
          const wide=4*c.width>65535,indices=wide?new Uint32Array(values):new Uint16Array(values),buffer=gl.createBuffer();
          gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER,buffer);gl.bufferData(gl.ELEMENT_ARRAY_BUFFER,indices,gl.STATIC_DRAW);
          index={buffer,count:indices.length,type:wide?gl.UNSIGNED_INT:gl.UNSIGNED_SHORT};this.buffers.set(c.width,index);
        }
        gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER,index.buffer);
        gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,c.depthTexture);
        gl.uniform1i(this.u.u_source_index,source);gl.uniform2i(this.u.u_size,c.width,c.height);
        gl.uniform4fv(this.u.u_intrinsics,c.intrinsicsArray);
        gl.uniformMatrix4fv(this.u.u_to_rgb_clip,false,multiply(targetProjection,c.modelMatrix));
        gl.drawElementsInstanced(gl.TRIANGLES,index.count,index.type,0,c.height-1);this.passes++;
        if(step===0)copyLayer(target,cameras.length+target,camera.colorWidth,camera.colorHeight);
      }
      // On first use, previous=current; a new rig must not inherit a clear
      // history layer that could falsely authorize color behind an occluder.
      if(!hasHistory)copyLayer(target,cameras.length*2+target,camera.colorWidth,camera.colorHeight);
    }
    this.historyKey=historyKey;
    gl.depthFunc(gl.LEQUAL);
  }
}
