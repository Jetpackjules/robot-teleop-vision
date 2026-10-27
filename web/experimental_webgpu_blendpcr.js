// Opt-in original-RGB BlendPCR: quality, visibility, metadata and composition
// share one device. Production WebGL remains the oracle and fallback.
// SPDX-License-Identifier: EUPL-1.2
import {ExperimentalGpuVisibility} from './experimental_gpu_visibility.js';
import {blendPcrShaders} from './webgpu_blendpcr_shaders.js';
import {rgbVisibilityProjection,rgbVisibilityMultiply} from './hybrid_rgb_visibility.js?v=20261021_original_rgb';
import {validatedColorProjection} from './hybrid_color_projection.js?v=20261021_original_rgb';
import {workspaceCropBounds,WORKSPACE_CROP_CONTEXT,workspaceVisibleTiles} from './workspace_crop.js?v=20261021_original_rgb';
import {modeledWorkspace,modeledWorkspaceBounds} from './modeled_workspace.js?v=20261021_original_rgb';
import {visibleNativeTiles,nativeSourceTiles} from './native_geometry.js?v=20261026_native_render';
const mul=rgbVisibilityMultiply;
function metadataValues(cameras,inverse,now){
 const result=new Float32Array(cameras.length*52);
 cameras.forEach((c,i)=>{const m=inverse(c.modelMatrix),w=c.modelMatrix;
  result.set([m[0],m[4],m[8],m[12],m[1],m[5],m[9],m[13],m[2],m[6],m[10],m[14],...c.intrinsicsArray,c.width,c.height,now-c.lastUpdateMs<1000?1:0,0,w[0],w[4],w[8],w[12],w[1],w[5],w[9],w[13],w[2],w[6],w[10],w[14]],i*52);
  const p=validatedColorProjection(c.metadata);if(!p)throw Error('Native WebGPU requires original RGB calibration');const e=p.extrinsics;
  result.set([e[0],e[3],e[6],e[9],e[1],e[4],e[7],e[10],e[2],e[5],e[8],e[11],...p.intrinsics,...p.size,1,1],i*52+32);
 });return result;
}
function tileIndices(){const values=[];
 for(let bx=0;bx<32;bx+=4)for(let y=0;y<8;y++)for(let x=bx;x<bx+4;x++){const a=y*33+x,b=a+1,d=a+34,primitive=(y*32+x)*2;values.push(297+primitive,a,b,297+primitive+1,b,d);}
 return new Uint16Array(values);
}
export class ExperimentalWebGpuBlendPCR{
 static async create(cameras,inverse,{exact=true}={}){
  if(cameras.length<1||cameras.length>4)throw Error('Native WebGPU opt-in currently supports one to four cameras');
  const matrices=cameras.map(target=>{const p=mul(rgbVisibilityProjection(target.metadata.color_projection),inverse(target.modelMatrix));return cameras.map(c=>mul(p,c.modelMatrix));});
  if(!cameras.every(c=>c.metadata.color_space==='original_rgb'))throw Error('Native WebGPU currently supports original-RGB BlendPCR only');
  const gpu=await ExperimentalGpuVisibility.create(cameras.map(c=>({...c,intrinsics:c.intrinsicsArray,quality:new Uint8Array(c.width*c.height*4)})),matrices,{history:true,matrixMath:exact?'webgl_exact':'native'});
  const result=new ExperimentalWebGpuBlendPCR(gpu,cameras,inverse,exact);result.observations=new Uint32Array(cameras.length);try{await result.initialize();return result;}catch(e){result.destroy();throw e;}
 }
 constructor(gpu,cameras,inverse,exact){this.gpu=gpu;this.device=gpu.device;this.cameras=cameras;this.inverse=inverse;this.exact=exact;this.count=cameras.length;this.resources=[];this.revisions=[];this.visibilityRevisions=[];this.layerIndices=new Uint32Array(this.count*4);this.rgbReady=new Set();this.version=0;this.compositions=0;this.metadataPasses=0;this.failed=null;this.signature=this.cameraSignature(cameras);this.device.addEventListener('uncapturederror',e=>this.failed=e.error.message);this.device.lost.then(info=>{if(!this.destroyed)this.failed=`WebGPU device lost: ${info.message||info.reason}`;});}
 cameraSignature(cameras){return cameras.map(c=>`${c.width}:${c.height}:${c.colorWidth}:${c.colorHeight}:${c.metadata.id}`).join('|');}
 buffer(size,usage){const b=this.device.createBuffer({size:Math.max(16,size),usage:usage|GPUBufferUsage.COPY_DST});this.resources.push(b);return b;}
 texture(width,height,layers,format,usage){const t=this.device.createTexture({size:[width,height,layers],format,usage});this.resources.push(t);return t;}
 group(pipeline,entries){const layout=pipeline.getBindGroupLayout(0);return this.device.createBindGroup({layout,entries:entries.map(([binding,resource])=>({binding,resource}))});}
 async initialize(){
  const d=this.device,w=Math.max(...this.cameras.map(c=>c.width)),h=Math.max(...this.cameras.map(c=>c.height)),n=this.count;
  this.depthWidth=w;this.depthHeight=h;this.frameBytes=new ArrayBuffer((164+4*n)*4);this.frameFloats=new Float32Array(this.frameBytes);this.frameInts=new Uint32Array(this.frameBytes);
  this.frame=this.buffer(this.frameBytes.byteLength,GPUBufferUsage.UNIFORM);this.cameraData=this.buffer(n*52*4,GPUBufferUsage.STORAGE);this.limits=this.buffer(16,GPUBufferUsage.UNIFORM);
  this.depth=this.texture(w,h,n,'r16uint',GPUTextureUsage.TEXTURE_BINDING|GPUTextureUsage.COPY_DST);
  this.quality=this.texture(w,h,n,'rgba8unorm',GPUTextureUsage.TEXTURE_BINDING|GPUTextureUsage.STORAGE_BINDING|GPUTextureUsage.COPY_SRC);
  this.mask=this.texture(w,h,n,'r32uint',GPUTextureUsage.TEXTURE_BINDING|GPUTextureUsage.STORAGE_BINDING|GPUTextureUsage.COPY_SRC);
  this.rgb=this.texture(this.gpu.width,this.gpu.height,n,'rgba8unorm',GPUTextureUsage.TEXTURE_BINDING|GPUTextureUsage.COPY_DST|GPUTextureUsage.RENDER_ATTACHMENT);
  this.sampler=d.createSampler({magFilter:'linear',minFilter:'linear',addressModeU:'clamp-to-edge',addressModeV:'clamp-to-edge'});
  const shaders=blendPcrShaders(n,{exact:this.exact});
  const shader=async code=>{const m=d.createShaderModule({code});const info=await m.getCompilationInfo();const errors=info.messages.filter(x=>x.type==='error');if(errors.length)throw Error(errors.map(e=>e.message).join('\n'));return m;};
  this.qualityPipeline=await d.createComputePipelineAsync({layout:'auto',compute:{module:await shader(shaders.quality),entryPoint:'main'}});
  this.maskPipeline=await d.createComputePipelineAsync({layout:'auto',compute:{module:await shader(shaders.mask),entryPoint:'main'}});
  this.constants=this.buffer((16+4*n)*4,GPUBufferUsage.STORAGE);
  this.constantsPipeline=await d.createComputePipelineAsync({layout:'auto',compute:{module:await shader(shaders.constants),entryPoint:'main'}});
  this.constantsGroup=this.group(this.constantsPipeline,[[0,{buffer:this.frame}],[1,{buffer:this.cameraData}],[13,{buffer:this.constants}]]);
  const raster=await shader(shaders.metadata);this.metadataPipeline=await d.createRenderPipelineAsync({layout:'auto',vertex:{module:raster,entryPoint:'vertex'},fragment:{module:raster,entryPoint:'fragment',targets:[{format:'r32uint'}]},primitive:{topology:'triangle-list',cullMode:'none'},depthStencil:{format:'depth24plus-stencil8',depthWriteEnabled:true,depthCompare:'less-equal',stencilWriteMask:0}});
  this.compositionLayout=d.createBindGroupLayout({entries:[{binding:0,visibility:GPUShaderStage.FRAGMENT,buffer:{type:'uniform'}},...[1,13].map(binding=>({binding,visibility:GPUShaderStage.FRAGMENT,buffer:{type:'read-only-storage'}})),...[2,3,4,5,7,8].map(binding=>({binding,visibility:GPUShaderStage.FRAGMENT,texture:{sampleType:binding===2||binding===7||binding===8?'uint':binding===5?'depth':'float',viewDimension:'2d-array'}})),{binding:6,visibility:GPUShaderStage.FRAGMENT,sampler:{type:'filtering'}}]});
  const composite=await shader(shaders.composition);this.compositionPipeline=await d.createRenderPipelineAsync({layout:d.createPipelineLayout({bindGroupLayouts:[this.compositionLayout]}),vertex:{module:composite,entryPoint:'vertex'},fragment:{module:composite,entryPoint:'fragment',targets:[{format:'rgba8unorm'}]},depthStencil:{format:'depth24plus-stencil8',depthWriteEnabled:true,depthCompare:'less-equal',stencilWriteMask:0}});
  this.index=this.buffer(32*8*6*2,GPUBufferUsage.INDEX);d.queue.writeBuffer(this.index,0,tileIndices());
  this.source=this.cameras.map((_,i)=>{const b=this.buffer(16,GPUBufferUsage.UNIFORM);d.queue.writeBuffer(b,0,new Uint32Array([i,0,0,0]));return b;});
  const basic=[[0,{buffer:this.frame}],[1,{buffer:this.cameraData}],[2,this.depth.createView({dimension:'2d-array'})],[10,null]];
  this.qualityGroups=this.source.map(source=>this.group(this.qualityPipeline,[...basic.filter(([b])=>b!==10),[10,{buffer:source}],[11,this.quality.createView({dimension:'2d-array'})],[12,{buffer:this.limits}]]));
  this.maskGroups=this.source.map(source=>this.group(this.maskPipeline,[...basic.filter(([b])=>b!==10),[10,{buffer:source}],[11,this.mask.createView({dimension:'2d-array'})]]));
  this.tileBuffers=this.cameras.map(()=>({buffer:this.buffer(Math.ceil(w/32)*Math.ceil(h/8)*8,GPUBufferUsage.STORAGE)}));
  this.metadataGroups=this.source.map((source,i)=>this.group(this.metadataPipeline,[[0,{buffer:this.frame}],[1,{buffer:this.cameraData}],[2,this.depth.createView({dimension:'2d-array'})],[3,this.quality.createView({dimension:'2d-array'})],[8,this.mask.createView({dimension:'2d-array'})],[9,{buffer:this.tileBuffers[i].buffer}],[10,{buffer:source}]]));
 }
 acceptDecoded(decoded){
  const i=this.cameras.findIndex(c=>c.metadata.id===decoded.metadata.id);if(i<0)return;const c=this.cameras[i],d=this.device;let rgba=decoded.colorRgbaPixels;
  if(decoded.width!==c.width||decoded.height!==c.height||Number(decoded.metadata.color_width||decoded.colorWidth)!==c.colorWidth||Number(decoded.metadata.color_height||decoded.colorHeight)!==c.colorHeight)throw Error('Camera layout changed; native backend must be recreated');
  if(decoded.bitmap){d.queue.copyExternalImageToTexture({source:decoded.bitmap},{texture:this.rgb,origin:[0,0,i]},[c.colorWidth,c.colorHeight]);this.rgbReady.add(i);return;}
  if(!rgba&&decoded.colorPixels){const raw=decoded.colorPixels;rgba=new Uint8Array(c.colorWidth*c.colorHeight*4);for(let p=0,j=0;p<raw.length;p+=3,j+=4){rgba[j]=raw[p];rgba[j+1]=raw[p+1];rgba[j+2]=raw[p+2];rgba[j+3]=255;}}
  if(rgba){d.queue.writeTexture({texture:this.rgb,origin:[0,0,i]},rgba,{bytesPerRow:c.colorWidth*4},[c.colorWidth,c.colorHeight]);this.rgbReady.add(i);}
  if(Array.isArray(decoded.colorTileUpdates))for(const tile of decoded.colorTileUpdates){const raw=tile.pixels,out=new Uint8Array(tile.width*tile.height*4);for(let p=0,j=0;p<raw.length;p+=3,j+=4){out[j]=raw[p];out[j+1]=raw[p+1];out[j+2]=raw[p+2];out[j+3]=255;}d.queue.writeTexture({texture:this.rgb,origin:[tile.x,tile.y,i]},out,{bytesPerRow:tile.width*4},[tile.width,tile.height]);}
 }
 frameData(view,settings,capsules){
  const f=this.frameFloats,u=this.frameInts;f.fill(0);f.set(view.viewProjection,0);f.set(view.viewer,16);f.set(view.inverseViewer,32);f.set(view.projection,48);u.set([view.width,view.height,this.count,settings.depth_only_enabled===true?1:0],64);
  const crop=workspaceCropBounds(settings),stage=modeledWorkspace(settings);
  if(crop){const context=workspaceCropBounds(settings,WORKSPACE_CROP_CONTEXT);f.set([...crop.localMin,1],68);f.set(crop.localMax,72);f.set(crop.center,76);for(let c=0;c<3;c++)f.set(crop.inverse.slice(c*3,c*3+3),80+c*4);f.set(context.localMin,156+4*this.count);f.set(context.localMax,160+4*this.count);}
  if(stage){f.set([...stage.center,1],92);f.set([stage.width/2,stage.depth/2,stage.clearance,stage.height],96);f.set([stage.cos,stage.sin,0,0],100);}
  if(capsules){f.set(capsules.starts,104);f.set(capsules.ends,128);}u.set(this.layerIndices,152);u.set([capsules?1:0,0,settings.white_background_enabled?1:0,0],152+this.count*4);
  this.device.queue.writeBuffer(this.frame,0,this.frameBytes);
 }
 update(cameras,view,settings,capsules){
  if(this.cameraSignature(cameras)!==this.signature)throw Error('Native WebGPU camera layout changed; recreate backend');if(this.failed)throw Error(this.failed);
  this.cameras=cameras;const d=this.device,now=view.now,delta=Number(settings.mesh_depth_delta||0.045),edge=Number(settings.mesh_max_edge||0.08);
  const revisions=cameras.map(c=>`${c.uploadRevision??c.lastUpdateMs}:${delta}:${edge}:${c.intrinsicsArray}`),changed=revisions.map((r,i)=>r!==this.revisions[i]);
  const history=JSON.stringify(cameras.map(c=>[c.metadata.id,Array.from(c.modelMatrix),c.metadata.color_projection,now-c.lastUpdateMs<1000]));const reset=this.historyKey!==history;
  const visible=settings.depth_only_enabled!==true,visChanged=revisions.map((r,i)=>r!==this.visibilityRevisions[i]),observation=visible&&(reset||visChanged.some(Boolean));
  const slots=this.count*2,layers=new Set(),layerMap=new Uint32Array(slots);
  cameras.forEach((c,i)=>{const oldPeer=this.layerIndices[i*4],oldSelf=this.layerIndices[i*4+1];const peer=observation&&(reset||visChanged.some((v,j)=>v&&j!==i)),self=observation&&(reset||visChanged[i]);
   const nextPeer=peer?(reset?i:(oldPeer+slots)%(slots*2)):oldPeer,nextSelf=self?(reset?this.count+i:(oldSelf+slots)%(slots*2)):oldSelf;
   if(observation)this.layerIndices.set([nextPeer,nextSelf,reset?nextPeer:oldPeer,reset?nextSelf:oldSelf],i*4);if(observation&&visChanged[i])this.observations[i]++;layerMap.set([nextSelf,nextPeer],i*2);if(self)layers.add(i*2);if(peer)layers.add(i*2+1);
  });
  this.frameData(view,settings,capsules);
  const metadataKey=history+cameras.map(c=>c.intrinsicsArray).join('|');
  if(metadataKey!==this.metadataKey){
   d.queue.writeBuffer(this.cameraData,0,metadataValues(cameras,this.inverse,now));
   const matrices=cameras.map(target=>{const p=mul(rgbVisibilityProjection(target.metadata.color_projection),this.inverse(target.modelMatrix));return cameras.map(c=>mul(p,c.modelMatrix));});
   for(let target=0;target<this.count;target++)for(let source=0;source<this.count;source++)for(let half=0;half<2;half++){const c=cameras[source],bytes=new ArrayBuffer(96),floats=new Float32Array(bytes),ints=new Uint32Array(bytes);floats.set(matrices[target][source]);floats.set(c.intrinsicsArray,16);ints.set([c.width,c.height,half,0],20);d.queue.writeBuffer(this.gpu.groups[target*this.count+source].params[half],0,bytes);}
   this.metadataKey=metadataKey;
  }
  const limitsKey=`${delta}:${edge}`;if(limitsKey!==this.limitsKey){d.queue.writeBuffer(this.limits,0,new Float32Array([delta,edge,0,0]));this.limitsKey=limitsKey;}
  const maskKey=`${capsules?.starts}:${capsules?.ends}:${history}`,maskChanged=maskKey!==this.maskKey;const encoder=d.createCommandEncoder();
  cameras.forEach((c,i)=>{if(changed[i]||reset){d.queue.writeTexture({texture:this.depth,origin:[0,0,i]},c.depth,{bytesPerRow:c.width*2},[c.width,c.height]);d.queue.writeTexture({texture:this.gpu.sources[i].depth},c.depth,{bytesPerRow:c.width*2},[c.width,c.height]);const pass=encoder.beginComputePass();pass.setPipeline(this.qualityPipeline);pass.setBindGroup(0,this.qualityGroups[i]);pass.dispatchWorkgroups(Math.ceil(c.width/8),Math.ceil(c.height/8));pass.end();encoder.copyTextureToTexture({texture:this.quality,origin:[0,0,i]},{texture:this.gpu.sources[i].quality},[c.width,c.height]);}
   if(capsules&&(maskChanged||changed[i])){const pass=encoder.beginComputePass();pass.setPipeline(this.maskPipeline);pass.setBindGroup(0,this.maskGroups[i]);pass.dispatchWorkgroups(Math.ceil(c.width/8),Math.ceil(c.height/8));pass.end();}});
  d.queue.submit([encoder.finish()]);if(layers.size)this.gpu.render(true,{timing:false,wait:false,layers,layerMap,activeSources:cameras.map(c=>now-c.lastUpdateMs<1000)}).catch(e=>this.failed=e.message);
  if(changed.some(Boolean)||reset||maskChanged)this.version++;this.revisions=revisions;if(visible){this.historyKey=history;this.visibilityRevisions=revisions.slice();}this.maskKey=maskKey;
 }
 targets(width,height){
  if(this.width===width&&this.height===height)return;
  this.clearTargets();const usage=GPUTextureUsage.TEXTURE_BINDING|GPUTextureUsage.RENDER_ATTACHMENT|GPUTextureUsage.COPY_SRC;
  this.width=width;this.height=height;this.metadata=this.texture(width,height,this.count,'r32uint',usage);this.rasterDepth=this.texture(width,height,1,'depth24plus-stencil8',usage);this.color=this.texture(width,height,1,'rgba8unorm',usage);this.compositeDepth=this.texture(width,height,1,'depth24plus-stencil8',usage);
  this.compositionGroup=this.group(this.compositionPipeline,[[0,{buffer:this.frame}],[1,{buffer:this.cameraData}],[2,this.depth.createView({dimension:'2d-array'})],[3,this.quality.createView({dimension:'2d-array'})],[4,this.rgb.createView({dimension:'2d-array'})],[5,this.gpu.depth.createView({dimension:'2d-array',aspect:'depth-only'})],[6,this.sampler],[7,this.metadata.createView({dimension:'2d-array'})],[8,this.mask.createView({dimension:'2d-array'})],[13,{buffer:this.constants}]]);this.cacheKey='';this.metadataKeys=[];
 }
 render(cameras,view,capsules,settings){
  this.targets(view.width,view.height);this.update(cameras,view,settings,capsules);
  if(settings.depth_only_enabled!==true&&cameras.some((c,i)=>view.now-c.lastUpdateMs<1000&&!this.rgbReady.has(i)))throw Error('Native RGB upload is not ready');
  const crop=modeledWorkspaceBounds(settings,workspaceCropBounds(settings)),context=modeledWorkspaceBounds(settings,workspaceCropBounds(settings,WORKSPACE_CROP_CONTEXT),WORKSPACE_CROP_CONTEXT),key=`${this.version}:${view.viewProjection}:${capsules?.starts}:${capsules?.ends}:${settings.white_background_enabled}:${settings.depth_only_enabled}:${crop?.key}`;
  if(key===this.cacheKey)return false;
  const d=this.device,encoder=d.createCommandEncoder();this.cropStats={enabled:Boolean(crop),sourceCells:0,submittedCells:0};
  const constants=encoder.beginComputePass();constants.setPipeline(this.constantsPipeline);constants.setBindGroup(0,this.constantsGroup);constants.dispatchWorkgroups(1);constants.end();
  cameras.forEach((c,i)=>{
   const fresh=view.now-c.lastUpdateMs<1000,rasterKey=`${view.viewProjection}:${c.uploadRevision}:${c.modelMatrix}:${c.intrinsicsArray}:${context?.key}:${capsules?.starts}:${capsules?.ends}:${fresh}:${settings.mesh_depth_delta}:${settings.mesh_max_edge}`;
   this.cropStats.sourceCells+=(c.width-1)*(c.height-1);const tiles=visibleNativeTiles(c,view.viewProjection,context)||workspaceVisibleTiles(c,context)||nativeSourceTiles(c);this.cropStats.submittedCells+=tiles.cellCount||tiles.length/2*256;
   if(this.metadataKeys?.[i]===rasterKey)return;
   const pass=encoder.beginRenderPass({colorAttachments:[{view:this.metadata.createView({dimension:'2d',baseArrayLayer:i,arrayLayerCount:1}),loadOp:'clear',clearValue:[0,0,0,0],storeOp:'store'}],depthStencilAttachment:{view:this.rasterDepth.createView(),depthLoadOp:'clear',depthClearValue:1,depthStoreOp:'discard',stencilReadOnly:true}});
   if(fresh&&tiles.length){d.queue.writeBuffer(this.tileBuffers[i].buffer,0,Uint32Array.from(tiles));pass.setPipeline(this.metadataPipeline);pass.setBindGroup(0,this.metadataGroups[i]);pass.setIndexBuffer(this.index,'uint16');pass.drawIndexed(1536,tiles.length/2);}
   pass.end();this.metadataKeys||=[];this.metadataKeys[i]=rasterKey;this.metadataPasses++;
  });
  const pass=encoder.beginRenderPass({colorAttachments:[{view:this.color.createView(),loadOp:'clear',clearValue:settings.white_background_enabled?[1,1,1,1]:[.03,.035,.04,1],storeOp:'store'}],depthStencilAttachment:{view:this.compositeDepth.createView(),depthLoadOp:'clear',depthClearValue:1,depthStoreOp:'store',stencilReadOnly:true}});pass.setPipeline(this.compositionPipeline);pass.setBindGroup(0,this.compositionGroup);pass.draw(3);pass.end();d.queue.submit([encoder.finish()]);this.cacheKey=key;this.compositions++;return true;
 }
 async readTexture(texture,format,layers=1){
  const bytesPerPixel=format==='float4'?16:4,width=texture.width,height=texture.height,row=Math.ceil(width*bytesPerPixel/256)*256,size=row*height*layers;
  if(format==='depth')throw Error('Read depth with the shared depth-bit compute probe');
  const read=this.device.createBuffer({size,usage:GPUBufferUsage.COPY_DST|GPUBufferUsage.MAP_READ});try{const e=this.device.createCommandEncoder();e.copyTextureToBuffer({texture},{buffer:read,bytesPerRow:row,rowsPerImage:height},[width,height,layers]);this.device.queue.submit([e.finish()]);await read.mapAsync(GPUMapMode.READ);const src=new Uint8Array(read.getMappedRange()),out=new Uint8Array(width*height*bytesPerPixel*layers);for(let y=0;y<height*layers;y++)out.set(src.subarray(y*row,y*row+width*bytesPerPixel),y*width*bytesPerPixel);return out;}finally{if(read.mapState==='mapped')read.unmap();read.destroy();}
 }
 async prepareDepthReadback(){
  if(this.depthProbe)return;const d=this.device;
  const code=`@group(0) @binding(0) var depth:texture_depth_2d_array;@group(0) @binding(1) var<storage,read_write> output:array<u32>;@compute @workgroup_size(8,8) fn main(@builtin(global_invocation_id) id:vec3u){let size=textureDimensions(depth);if(id.x>=size.x||id.y>=size.y){return;}output[(id.z*size.y+id.y)*size.x+id.x]=bitcast<u32>(textureLoad(depth,vec2i(id.xy),i32(id.z),0));}`;
  this.depthProbe=await d.createComputePipelineAsync({layout:'auto',compute:{module:d.createShaderModule({code}),entryPoint:'main'}});
 }
 async readDepth(texture,layers=1){
  if(!this.depthProbe)await this.prepareDepthReadback();
  const d=this.device,width=texture.width,height=texture.height,size=width*height*layers*4;
  const out=d.createBuffer({size,usage:GPUBufferUsage.STORAGE|GPUBufferUsage.COPY_SRC}),read=d.createBuffer({size,usage:GPUBufferUsage.COPY_DST|GPUBufferUsage.MAP_READ});
  try{const encoder=d.createCommandEncoder(),pass=encoder.beginComputePass();pass.setPipeline(this.depthProbe);pass.setBindGroup(0,this.group(this.depthProbe,[[0,texture.createView({dimension:'2d-array',aspect:'depth-only'})],[1,{buffer:out}]]));pass.dispatchWorkgroups(Math.ceil(width/8),Math.ceil(height/8),layers);pass.end();encoder.copyBufferToBuffer(out,0,read,0,size);d.queue.submit([encoder.finish()]);await read.mapAsync(GPUMapMode.READ);return new Uint32Array(read.getMappedRange().slice(0));}finally{if(read.mapState==='mapped')read.unmap();read.destroy();out.destroy();}
 }
 clearTargets(){const targets=new Set([this.metadata,this.rasterDepth,this.color,this.compositeDepth]);for(const texture of targets)texture?.destroy();this.resources=this.resources.filter(resource=>!targets.has(resource));this.metadata=this.rasterDepth=this.color=this.compositeDepth=null;this.width=this.height=0;this.cacheKey='';this.metadataKeys=[];}
 destroy(){this.destroyed=true;for(const r of this.resources)r.destroy();this.resources=[];this.gpu.destroy();}
}
