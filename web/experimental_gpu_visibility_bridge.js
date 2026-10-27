// Opt-in, same-frame WebGPU visibility bridge for full-frame BlendPCR composition.
// Never imported by production. Canvas interchange can stall; benchmark the UI.
// Camera bytes, native vertices, current/previous map rotation and rendering
// remain unchanged. Canvas copies carry exact 24-bit depth, with no JS readback.
// SPDX-License-Identifier: EUPL-1.2
import {ExperimentalGpuVisibility} from './experimental_gpu_visibility.js';
const FULLSCREEN=`#version 300 es
precision highp float;void main(){vec2 p=vec2(float((gl_VertexID<<1)&2),float(gl_VertexID&2));gl_Position=vec4(p*2.0-1.0,0,1);}`;
const FLAGS=`#version 300 es
precision highp float;precision highp int;
uniform highp sampler2DArray u_quality;uniform int u_source;uniform int u_height;
out vec4 color;void main(){ivec2 pixel=ivec2(gl_FragCoord.xy);pixel.y=u_height-1-pixel.y;
color=vec4(texelFetch(u_quality,ivec3(pixel,u_source),0).rg,0,1);}`;
const IMPORT=`#version 300 es
precision highp float;precision highp int;uniform highp sampler2D u_depth;
void main(){uvec3 bits=uvec3(round(texelFetch(u_depth,ivec2(gl_FragCoord.xy),0).rgb*255.0));
uint value=bits.r|(bits.g<<8u)|(bits.b<<16u);float base=float(value)*(1.0/16777216.0);
// For every nonzero 24-bit integer, correctly-rounded value/(2^24-1)
// is the float immediately above exact value/2^24. Avoid approximate RCP.
gl_FragDepth=uintBitsToFloat(floatBitsToUint(base)+(value==0u?0u:1u));}`;
const EXPORT=`@group(0) @binding(0) var depth:texture_depth_2d_array;
@group(0) @binding(1) var<uniform> layer:vec4u;
@vertex fn vertex(@builtin(vertex_index) id:u32)->@builtin(position) vec4f{let p=vec2f(f32((id<<1u)&2u),f32(id&2u));return vec4f(p*2-1,0,1);}
@fragment fn fragment(@builtin(position) position:vec4f)->@location(0) vec4f{
let d=textureLoad(depth,vec2i(position.xy),i32(layer.x),0);let value=u32(round(d*16777215.0));
return vec4f(f32(value&255u),f32((value>>8u)&255u),f32((value>>16u)&255u),255)/255;
}`;
export class ExperimentalVisibilityBridge{
  static async create(visibility,cameras,inverse,{projection,multiply}={}){
    if(!projection||!multiply)throw Error('Canonical camera projection functions required');
    const bridge=new ExperimentalVisibilityBridge(visibility,cameras,inverse,projection,multiply);
    try{await bridge.initialize();return bridge;}catch(error){bridge.destroy();throw error;}
  }
  constructor(visibility,cameras,inverse,projection,multiply){
    this.visibility=visibility;this.gl=visibility.gl;this.compile=visibility.compile;this.inverse=inverse;this.projection=projection;this.multiply=multiply;
    this.cameras=cameras;this.signature=this.cameraSignature(cameras);this.originalDraw=visibility.drawSource;this.originalUpdate=visibility.update;
    this.draws=0;this.frames=0;this.copied=new Set();this.revisions=[];
  }
  cameraSignature(cameras){return cameras.map(c=>[c.width,c.height,c.colorWidth,c.colorHeight].join(':')).join('|');}
  matrices(cameras){return cameras.map(target=>{const p=this.multiply(this.projection(target.metadata.color_projection),this.inverse(target.modelMatrix));return cameras.map(source=>this.multiply(p,source.modelMatrix));});}
  async initialize(){
    const gl=this.gl,cameras=this.cameras;
    this.gpu=await ExperimentalGpuVisibility.create(cameras.map(c=>({...c,intrinsics:c.intrinsicsArray,quality:new Uint8Array(c.width*c.height*4)})),this.matrices(cameras),{matrixMath:'webgl_exact'});
    this.gpu.device.addEventListener('uncapturederror',event=>{this.failure=event.error.message;});
    const d=this.gpu.device;this.canvas=new OffscreenCanvas(this.gpu.width,this.gpu.height);this.context=this.canvas.getContext('webgpu');this.context.configure({device:d,format:'rgba8unorm',alphaMode:'opaque'});
    this.flagsProgram=this.compile(gl,FULLSCREEN,FLAGS);this.importProgram=this.compile(gl,FULLSCREEN,IMPORT);this.vao=gl.createVertexArray();
    this.flagsU=Object.fromEntries(['u_quality','u_source','u_height'].map(name=>[name,gl.getUniformLocation(this.flagsProgram,name)]));
    this.importU=gl.getUniformLocation(this.importProgram,'u_depth');this.texture=gl.createTexture();gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,this.texture);gl.texStorage2D(gl.TEXTURE_2D,1,gl.RGBA8,this.gpu.width,this.gpu.height);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MIN_FILTER,gl.NEAREST);gl.texParameteri(gl.TEXTURE_2D,gl.TEXTURE_MAG_FILTER,gl.NEAREST);
    const module=d.createShaderModule({code:EXPORT});this.exportPipeline=await d.createRenderPipelineAsync({layout:'auto',vertex:{module,entryPoint:'vertex'},fragment:{module,entryPoint:'fragment',targets:[{format:'rgba8unorm'}]}});
    this.exportParams=d.createBuffer({size:16,usage:GPUBufferUsage.UNIFORM|GPUBufferUsage.COPY_DST});this.exportGroup=d.createBindGroup({layout:this.exportPipeline.getBindGroupLayout(0),entries:[{binding:0,resource:this.gpu.depth.createView({dimension:'2d-array',aspect:'depth-only'})},{binding:1,resource:{buffer:this.exportParams}}]});
    gl.bindFramebuffer(gl.FRAMEBUFFER,null);
  }
  attach(){
    if(this.attached)return;this.attached=true;const bridge=this;
    this.visibility.drawSource=function(...args){return bridge.draw(...args);};
    this.visibility.update=function(...args){return bridge.update(...args);};
  }
  detach(){if(this.attached){this.visibility.update=this.originalUpdate;this.visibility.drawSource=this.originalDraw;this.attached=false;}}
  update(cameras,quality,inverse,enabled,now,active=true,revisions=null,options={}){
    const gl=this.gl,dimensionsFit=cameras.every(c=>gl.canvas.width>=c.width&&gl.canvas.height>=c.height);
    if(this.failure||!enabled||!active||this.cameraSignature(cameras)!==this.signature||!dimensionsFit){this.rendering=false;return this.originalUpdate.call(this.visibility,cameras,quality,inverse,enabled,now,active,revisions,options);}
    const next=revisions||cameras.map(c=>c.uploadRevision??c.lastUpdateMs),changed=cameras.map((c,i)=>this.cameras[i]!==c||this.revisions[i]!==next[i]);
    const history=JSON.stringify(cameras.map(c=>[c.metadata.id,Array.from(c.modelMatrix),c.metadata.color_projection,now-c.lastUpdateMs<1000]));
    const reset=history!==this.visibility.historyKey,layers=new Set();
    cameras.forEach((c,i)=>{if(reset||changed[i])layers.add(i*2);if(reset||changed.some((v,j)=>v&&j!==i))layers.add(i*2+1);});
    this.cameras=cameras;this.inverse=inverse;this.currentMatrices=this.matrices(cameras);this.copied.clear();this.rendering=true;
    if(layers.size){
      const d=this.gpu.device;
      cameras.forEach((c,i)=>{
        if(reset||changed[i]){
          d.queue.writeTexture({texture:this.gpu.sources[i].depth},c.depth,{bytesPerRow:c.width*2},[c.width,c.height]);
          gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.bindVertexArray(this.vao);gl.viewport(0,0,c.width,c.height);gl.disable(gl.DEPTH_TEST);gl.disable(gl.BLEND);gl.disable(gl.SCISSOR_TEST);gl.disable(gl.CULL_FACE);gl.useProgram(this.flagsProgram);
          gl.activeTexture(gl.TEXTURE6);gl.bindTexture(gl.TEXTURE_2D_ARRAY,quality);gl.uniform1i(this.flagsU.u_quality,6);gl.uniform1i(this.flagsU.u_source,i);gl.uniform1i(this.flagsU.u_height,c.height);gl.drawArrays(gl.TRIANGLES,0,3);
          d.queue.copyExternalImageToTexture({source:gl.canvas,origin:[0,gl.canvas.height-c.height]},{texture:this.gpu.sources[i].quality},[c.width,c.height]);
        }
      });
      for(let target=0;target<cameras.length;target++)for(let source=0;source<cameras.length;source++)for(let half=0;half<2;half++){
        const c=cameras[source],bytes=new ArrayBuffer(96),floats=new Float32Array(bytes),ints=new Uint32Array(bytes);floats.set(this.currentMatrices[target][source]);floats.set(c.intrinsicsArray,16);ints.set([c.width,c.height,half,0],20);d.queue.writeBuffer(this.gpu.groups[target*cameras.length+source].params[half],0,bytes);
      }
      // Once initialization has completed this call submits synchronously.
      this.gpu.render(true,{timing:false,wait:false,layers,activeSources:cameras.map(c=>now-c.lastUpdateMs<1000)}).catch(error=>{this.failure=error.message;});
      this.frames++;
    }
    this.revisions=next.slice();
    try{return this.originalUpdate.call(this.visibility,cameras,quality,inverse,enabled,now,active,next,{...options,regions:false});}
    finally{this.rendering=false;}
  }
  draw(camera,source,matrix,tiled,cullingMatrix){
    if(!this.rendering||camera.depthTexture!==this.cameras[source].depthTexture)return this.originalDraw.call(this.visibility,camera,source,matrix,tiled,cullingMatrix);
    const attached=glLayer(this.gl);let target=-1,kind=-1;
    for(let i=0;i<this.cameras.length;i++)for(let k=0;k<2;k++)if(this.visibility.layers[i*4+k]===attached){target=i;kind=k===0?1:0;}
    if(target<0)return this.originalDraw.call(this.visibility,camera,source,matrix,tiled,cullingMatrix);
    const layer=target*2+kind;if(this.copied.has(layer))return;this.copied.add(layer);
    const gl=this.gl,d=this.gpu.device,framebuffer=gl.getParameter(gl.FRAMEBUFFER_BINDING);
    d.queue.writeBuffer(this.exportParams,0,new Uint32Array([layer,0,0,0]));const encoder=d.createCommandEncoder(),pass=encoder.beginRenderPass({colorAttachments:[{view:this.context.getCurrentTexture().createView(),clearValue:[1,1,1,1],loadOp:'clear',storeOp:'store'}]});pass.setPipeline(this.exportPipeline);pass.setBindGroup(0,this.exportGroup);pass.draw(3);pass.end();d.queue.submit([encoder.finish()]);
    gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,this.texture);gl.pixelStorei(gl.UNPACK_COLORSPACE_CONVERSION_WEBGL,gl.NONE);gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL,false);gl.pixelStorei(gl.UNPACK_PREMULTIPLY_ALPHA_WEBGL,false);gl.texSubImage2D(gl.TEXTURE_2D,0,0,0,gl.RGBA,gl.UNSIGNED_BYTE,this.canvas);gl.pixelStorei(gl.UNPACK_COLORSPACE_CONVERSION_WEBGL,gl.BROWSER_DEFAULT_WEBGL);
    gl.bindFramebuffer(gl.FRAMEBUFFER,framebuffer);gl.bindVertexArray(this.vao);gl.useProgram(this.importProgram);gl.uniform1i(this.importU,0);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LESS);gl.depthMask(true);gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);gl.disable(gl.SCISSOR_TEST);gl.drawArrays(gl.TRIANGLES,0,3);this.draws++;
  }
  destroy(){
    this.detach();
    const gl=this.gl;if(this.flagsProgram)gl.deleteProgram(this.flagsProgram);if(this.importProgram)gl.deleteProgram(this.importProgram);if(this.vao)gl.deleteVertexArray(this.vao);if(this.texture)gl.deleteTexture(this.texture);
    this.exportParams?.destroy();this.context?.unconfigure();this.gpu?.destroy();
  }
}
function glLayer(gl){return gl.getFramebufferAttachmentParameter(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,gl.FRAMEBUFFER_ATTACHMENT_TEXTURE_LAYER);}
