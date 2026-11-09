// Only completed color/depth crosses APIs, in one lossless packed surface.
// Intermediate visibility, metadata and color selection stay on WebGPU.
// SPDX-License-Identifier: EUPL-1.2
const EXPORT=`@group(0) @binding(0) var color:texture_2d<f32>;
@group(0) @binding(1) var depth:texture_depth_2d;
@vertex fn vertex(@builtin(vertex_index) id:u32)->@builtin(position) vec4f{let p=vec2f(f32((id<<1u)&2u),f32(id&2u));return vec4f(p*2-1,0,1);}
@fragment fn fragment(@builtin(position) p:vec4f)->@location(0) vec4f{let width=i32(textureDimensions(color).x);let pixel=vec2i(i32(p.x)%width,i32(p.y));if(i32(p.x)<width){return textureLoad(color,pixel,0);}let z=textureLoad(depth,pixel,0);let bits=u32(round(z*16777215.0));return vec4f(f32(bits&255u),f32((bits>>8u)&255u),f32((bits>>16u)&255u),255)/255;}`;
const VERTEX=`#version 300 es
precision highp float;void main(){vec2 p=vec2(float((gl_VertexID<<1)&2),float(gl_VertexID&2));gl_Position=vec4(p*2.0-1.0,0,1);}`;
const FRAGMENT=`#version 300 es
precision highp float;precision highp int;
uniform highp sampler2D u_color;
out vec4 color;
void main(){ivec2 p=ivec2(gl_FragCoord.xy);color=texelFetch(u_color,p,0);p.x+=textureSize(u_color,0).x/2;uvec3 rgb=uvec3(round(texelFetch(u_color,p,0).rgb*255.0));uint bits=rgb.r|(rgb.g<<8u)|(rgb.b<<16u);float base=float(bits)*(1.0/16777216.0);gl_FragDepth=uintBitsToFloat(floatBitsToUint(base)+(bits==0u?0u:1u));}`;
export class WebGpuBlendPcrPresenter{
 static async create(native,gl,compile){const result=new WebGpuBlendPcrPresenter(native,gl,compile);try{await result.initialize();return result;}catch(e){result.destroy();throw e;}}
 constructor(native,gl,compile){this.native=native;this.device=native.device;this.gl=gl;this.compile=compile;this.slots=[];this.slot=-1;this.transfers=0;}
 async initialize(){
  const d=this.device,module=d.createShaderModule({code:EXPORT});this.pipeline=await d.createRenderPipelineAsync({layout:'auto',vertex:{module,entryPoint:'vertex'},fragment:{module,entryPoint:'fragment',targets:[{format:'rgba8unorm'}]}});
  this.canvases=[new OffscreenCanvas(1,1)];this.contexts=this.canvases.map(canvas=>{const context=canvas.getContext('webgpu');context.configure({device:d,format:'rgba8unorm',alphaMode:'opaque'});return context;});
  const gl=this.gl;this.program=this.compile(gl,VERTEX,FRAGMENT);this.vao=gl.createVertexArray();this.colorUniform=gl.getUniformLocation(this.program,'u_color');
 }
 prepare(){
  const n=this.native,gl=this.gl;
  if(this.width!==n.width||this.height!==n.height){for(const slot of this.slots)for(const t of slot)gl.deleteTexture(t);this.slots=[];this.slot=-1;this.width=n.width;this.height=n.height;
   if(n.width*2>Math.min(this.device.limits.maxTextureDimension2D,gl.getParameter(gl.MAX_TEXTURE_SIZE)))throw Error('Packed WebGPU presentation exceeds texture limits');
   for(const canvas of this.canvases){canvas.width=n.width*2;canvas.height=n.height;}
   for(let ring=0;ring<3;ring++){const texture=gl.createTexture();gl.bindTexture(gl.TEXTURE_2D,texture);gl.texStorage2D(gl.TEXTURE_2D,1,gl.RGBA8,n.width*2,n.height);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);this.slots.push([texture]);}this.lastComposition=-1;
  }
  if(this.sourceColor!==n.color||this.sourceDepth!==n.compositeDepth){this.sourceColor=n.color;this.sourceDepth=n.compositeDepth;this.group=this.device.createBindGroup({layout:this.pipeline.getBindGroupLayout(0),entries:[{binding:0,resource:n.color.createView()},{binding:1,resource:n.compositeDepth.createView({aspect:'depth-only'})}]});}
 }
 present(restoreVertexArray=null,target=null){
  this.prepare();const d=this.device,gl=this.gl,n=this.native;
  if(this.lastComposition!==n.compositions){
   const e=d.createCommandEncoder(),pass=e.beginRenderPass({colorAttachments:this.contexts.map(context=>({view:context.getCurrentTexture().createView(),loadOp:'clear',clearValue:[0,0,0,1],storeOp:'store'}))});pass.setPipeline(this.pipeline);pass.setBindGroup(0,this.group);pass.draw(3);pass.end();d.queue.submit([e.finish()]);
   this.slot=(this.slot+1)%this.slots.length;gl.pixelStorei(gl.UNPACK_COLORSPACE_CONVERSION_WEBGL,gl.NONE);gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL,false);gl.pixelStorei(gl.UNPACK_PREMULTIPLY_ALPHA_WEBGL,false);
   gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,this.slots[this.slot][0]);gl.texSubImage2D(gl.TEXTURE_2D,0,0,0,gl.RGBA,gl.UNSIGNED_BYTE,this.canvases[0]);
   gl.pixelStorei(gl.UNPACK_COLORSPACE_CONVERSION_WEBGL,gl.BROWSER_DEFAULT_WEBGL);this.lastComposition=n.compositions;this.transfers++;
  }
  gl.bindFramebuffer(gl.FRAMEBUFFER,target);gl.bindVertexArray(this.vao);gl.viewport(0,0,n.width,n.height);gl.useProgram(this.program);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.ALWAYS);gl.depthMask(true);gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);gl.disable(gl.SCISSOR_TEST);
  gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,this.slots[this.slot][0]);gl.uniform1i(this.colorUniform,0);gl.drawArrays(gl.TRIANGLES,0,3);gl.depthFunc(gl.LEQUAL);gl.bindVertexArray(restoreVertexArray);
 }
 destroy(){const gl=this.gl;for(const slot of this.slots)for(const texture of slot)gl.deleteTexture(texture);this.slots=[];if(this.program)gl.deleteProgram(this.program);if(this.vao)gl.deleteVertexArray(this.vao);for(const context of this.contexts||[])context.unconfigure();}
}
