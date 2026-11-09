// Explicit opt-in only. A full-frame parity check precedes native presentation.
// No host setting, camera preset or robot command is modified by this adapter.
// SPDX-License-Identifier: EUPL-1.2
import {ExperimentalWebGpuBlendPCR} from './experimental_webgpu_blendpcr.js';
import {WebGpuBlendPcrPresenter} from './webgpu_blendpcr_present.js';
import {compileGpuProgram,FULLSCREEN_VERTEX_SHADER} from './hybrid_camera_quality.js?v=20261021_original_rgb';
export async function installExperimentalWebGpuBlendPCR(state,inverse){
 if(!state.blendPCR||!state.cameras.length)throw Error('Warm the original-RGB BlendPCR view before testing WebGPU');
 const native=await ExperimentalWebGpuBlendPCR.create(state.cameras,inverse);
 let presenter;
 try{await native.prepareDepthReadback();presenter=await WebGpuBlendPcrPresenter.create(native,state.gl,compileGpuProgram);}catch(e){native.destroy();throw e;}
 const controller=new NativeBlendPcrController(state,native,presenter);controller.install();return controller;
}
class NativeBlendPcrController{
 constructor(state,native,presenter){this.state=state;this.native=native;this.presenter=presenter;this.original=state.blendPCR;this.rig=state.cameraFusion;this.originalUpdate=this.rig.update;this.requested=false;this.verified=false;this.active=false;this.frames=0;this.savedMask=state.useCachedRobotMask;}
 eligible(){const s=this.state;return s.settings.display_mode!=='rgb_camera'&&s.settings.geometry_mode==='mesh'&&s.settings.blendpcr_enabled&&s.cameras.every(c=>c.metadata.color_space==='original_rgb');}
 install(){const controller=this;
  this.rig.update=function(...args){if(controller.requested&&controller.verified&&!controller.native.failed&&controller.eligible())return;return controller.originalUpdate.apply(this,args);};
  const adapter={render(...args){return controller.render(...args);},clearTargets(){controller.invalidate();controller.original.clearTargets();controller.native.clearTargets();},dispose(){controller.dispose();}};
  for(const key of ['signature','cropStats','compositions','metadataPasses'])Object.defineProperty(adapter,key,{get:()=>controller.active?controller.native[key]:controller.original[key]});
  this.adapter=adapter;this.state.blendPCR=adapter;this.state.experimentalWebGpuController=this;
 }
 invalidate(){this.generation=(this.generation||0)+1;this.active=false;this.verified=false;this.verifying=false;this.native.rgbReady.clear();this.native.historyKey='';this.native.visibilityRevisions=[];this.native.observations.fill(0);this.state.useCachedRobotMask=this.savedMask;this.rig.metadataSignature='';this.rig.uploads=[];this.rig.visibility.historyKey='';}
 request(enabled){this.invalidate();this.requested=Boolean(enabled);return this.status();}
 acceptDecoded(decoded){if(!this.requested||!this.eligible())return;try{this.native.acceptDecoded(decoded);}catch(e){this.native.failed=e.message;}}
 status(){return {requested:this.requested,verified:this.verified,active:this.active,frames:this.frames,verification:this.verification||null,failure:this.native.failed,presentations:this.presenter.transfers};}
 render(rig,cameras,view,capsules,settings){
  if(!this.requested||!this.eligible()||this.native.failed){if(this.active||this.verified||this.verifying)this.invalidate();this.active=false;this.state.useCachedRobotMask=this.savedMask;return this.original.render(rig,cameras,view,capsules,settings);}
  if(this.native.rgbReady.size!==cameras.length){this.active=false;return this.original.render(rig,cameras,view,capsules,settings);}
  if(!this.verified){
   this.active=false;this.original.render(rig,cameras,view,capsules,settings);this.native.render(cameras,view,capsules,settings);
   if(!this.verifying&&this.native.observations.every(count=>count>=2)){const reference=this.readReference(view),generation=this.generation;this.verifying=true;
    Promise.all([this.native.readTexture(this.native.color,'rgba'),this.native.readDepth(this.native.compositeDepth)]).then(([color,depth])=>{
     if(this.disposed||this.generation!==generation)return;
     const changed=(a,b)=>{let n=0;for(let i=0;i<a.length;i++)n+=a[i]!==b[i];return n;};this.verification={colorDifferent:changed(reference.color,color),depthDifferent:changed(reference.depth,depth),pixels:view.width*view.height};
     this.verified=this.verification.colorDifferent===0&&this.verification.depthDifferent===0;if(!this.verified){this.native.failed='Live full-frame parity gate rejected WebGPU';this.requested=false;}
    }).catch(e=>{if(!this.disposed&&this.generation===generation){this.native.failed=e.message;this.requested=false;}});
   }return;
  }
  this.state.useCachedRobotMask=false;
  try{this.native.render(cameras,view,capsules,settings);this.presenter.present(view.restoreVertexArray??null);this.active=true;this.frames++;}
  catch(e){this.native.failed=e.message;this.active=false;this.state.useCachedRobotMask=this.savedMask;this.rig.metadataSignature='';this.rig.uploads=[];this.rig.visibility.historyKey='';this.originalUpdate.call(this.rig,cameras,this.native.inverse,view.now,settings);this.original.render(this.rig,cameras,view,capsules,settings);}
 }
 readReference(view){
  const gl=this.state.gl,color=new Uint8Array(view.width*view.height*4),depth=new Uint32Array(view.width*view.height),fb=gl.createFramebuffer(),bits=gl.createTexture(),vao=gl.createVertexArray();
  const program=compileGpuProgram(gl,FULLSCREEN_VERTEX_SHADER,`#version 300 es\nprecision highp float;precision highp int;uniform highp sampler2D u_depth;layout(location=0)out highp uint bits;void main(){bits=floatBitsToUint(texelFetch(u_depth,ivec2(gl_FragCoord.xy),0).r);}`);
  try{gl.bindFramebuffer(gl.FRAMEBUFFER,fb);gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,this.original.compositeColor,0);gl.readPixels(0,0,view.width,view.height,gl.RGBA,gl.UNSIGNED_BYTE,color);
   gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,bits);gl.texStorage2D(gl.TEXTURE_2D,1,gl.R32UI,view.width,view.height);gl.framebufferTexture2D(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,gl.TEXTURE_2D,bits,0);gl.drawBuffers([gl.COLOR_ATTACHMENT0]);gl.readBuffer(gl.COLOR_ATTACHMENT0);gl.bindVertexArray(vao);gl.viewport(0,0,view.width,view.height);gl.disable(gl.DEPTH_TEST);gl.disable(gl.BLEND);gl.useProgram(program);gl.bindTexture(gl.TEXTURE_2D,this.original.compositeDepth);gl.uniform1i(gl.getUniformLocation(program,'u_depth'),0);gl.drawArrays(gl.TRIANGLES,0,3);gl.readPixels(0,0,view.width,view.height,gl.RED_INTEGER,gl.UNSIGNED_INT,depth);return {color,depth};
  }finally{gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.bindVertexArray(view.restoreVertexArray??null);gl.viewport(0,0,view.width,view.height);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);gl.depthMask(true);gl.deleteProgram(program);gl.deleteTexture(bits);gl.deleteFramebuffer(fb);gl.deleteVertexArray(vao);}
 }
 dispose(){if(this.disposed)return;this.disposed=true;this.request(false);if(this.rig.update!==this.originalUpdate)this.rig.update=this.originalUpdate;if(this.state.blendPCR===this.adapter)this.state.blendPCR=this.original;if(this.state.experimentalWebGpuController===this)this.state.experimentalWebGpuController=null;this.presenter.destroy();this.native.destroy();this.original.dispose();}
}
