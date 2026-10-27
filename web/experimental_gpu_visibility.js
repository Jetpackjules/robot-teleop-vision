// Isolated WebGPU visibility experiment. Production keeps the WebGL renderer.
// Consume the existing quality flags; retain every accepted native triangle.
// SPDX-License-Identifier: EUPL-1.2
const PARAMS=`struct Params { matrix:mat4x4f, intrinsics:vec4f, size:vec2u, half:u32, padding:u32 };`;
// WGSL fma may expand to separate multiply/add operations. Preserve the
// production compiler's single rounding for finite, normal-range projection
// arithmetic using an error-free product and compensated sums instead.
const GL_MATCHED_MATH=`
fn rounded(value:vec4f)->vec4f{return bitcast<vec4f>(bitcast<vec4u>(value)^vec4u(p.padding));}
fn two_sum(a:vec4f,b:vec4f)->mat2x4f{
  let sum=rounded(a+b);let bb=rounded(sum-a);
  let error=rounded(rounded(a-rounded(sum-bb))+rounded(b-bb));return mat2x4f(sum,error);
}
fn fused(a:vec4f,b:vec4f,c:vec4f)->vec4f{
  let product=rounded(a*b);
  let ah=bitcast<vec4f>(bitcast<vec4u>(a)&vec4u(0xfffff000u));let al=rounded(a-ah);
  let bh=bitcast<vec4f>(bitcast<vec4u>(b)&vec4u(0xfffff000u));let bl=rounded(b-bh);
  let err1=rounded(product-rounded(ah*bh));
  let err2=rounded(err1-rounded(al*bh));let err3=rounded(err2-rounded(ah*bl));
  let product_error=rounded(rounded(al*bl)-err3);
  let sum=two_sum(product,c);let correction=two_sum(sum[1],product_error);
  let result=two_sum(sum[0],correction[0]);let tail=two_sum(result[1],correction[1]);
  let bits=bitcast<vec4u>(result[0]);let positive=tail[0]>vec4f(0);
  let step=select(vec4u(0xffffffffu),vec4u(1u),(result[0]>=vec4f(0))==positive);
  var next=bitcast<vec4f>(bits+step);
  next=select(next,bitcast<vec4f>(select(vec4u(0x80000001u),vec4u(1u),positive)),result[0]==vec4f(0));
  let half=abs(rounded(next-result[0]))*0.5;
  let same_sign=(tail[1]>vec4f(0))==positive;
  let tie=select(same_sign,(bits&vec4u(1u))!=vec4u(0),tail[1]==vec4f(0));
  let away=(abs(tail[0])>half) | ((abs(tail[0])==half) & tie);
  let value=select(result[0],next,away & (tail[0]!=vec4f(0)));
  let signed_zero=bitcast<vec4f>((bitcast<vec4u>(a)^bitcast<vec4u>(b))&bitcast<vec4u>(c)&vec4u(0x80000000u));
  return select(value,signed_zero,((a==vec4f(0)) | (b==vec4f(0))) & (c==vec4f(0)));
}
fn project_gl(point:vec3f)->vec4f{
  let x=rounded(p.matrix[0]*point.x);
  let xy=fused(p.matrix[1],vec4f(point.y),x);
  return rounded(fused(p.matrix[2],vec4f(point.z),xy)+p.matrix[3]);
}`;
const COMPACT=`${PARAMS}
struct Args { count:atomic<u32>, instances:u32, first:u32, base:i32, firstInstance:u32 };
@group(0) @binding(0) var<uniform> p:Params;
@group(0) @binding(1) var quality:texture_2d<f32>;
@group(0) @binding(2) var<storage,read_write> indices:array<u32>;
@group(0) @binding(3) var<storage,read_write> args:Args;
var<workgroup> scan:array<u32,256>;
var<workgroup> start:u32;
@compute @workgroup_size(256)
fn main(@builtin(global_invocation_id) global:vec3u,@builtin(local_invocation_index) lane:u32){
  // Preserve the existing four-column/eight-row vertex-cache locality.
  let columns=(p.size.x-1u+3u)/4u;let block=global.x/32u;let within=global.x%32u;
  let pixel=vec2u((block%columns)*4u+within%4u,(block/columns)*8u+within/4u);
  var first=false;var second=false;
  if(all(pixel<p.size-vec2u(1))){let flags=textureLoad(quality,vec2i(pixel),0);first=flags.r>0.5;second=flags.g>0.5;}
  let count=select(0u,3u,first)+select(0u,3u,second);scan[lane]=count;workgroupBarrier();
  for(var offset=1u;offset<256u;offset*=2u){
    var previous=0u;if(lane>=offset){previous=scan[lane-offset];}workgroupBarrier();
    scan[lane]+=previous;workgroupBarrier();
  }
  if(lane==255u){start=atomicAdd(&args.count,scan[255]);}workgroupBarrier();
  var out=start+scan[lane]-count;let a=pixel.y*p.size.x+pixel.x;let b=a+1u;let c=a+p.size.x;let d=c+1u;
  if(first){indices[out]=a;indices[out+1u]=b;indices[out+2u]=c;out+=3u;}
  if(second){indices[out]=b;indices[out+1u]=d;indices[out+2u]=c;}
}`;
const VERTEX=`${PARAMS}
@group(0) @binding(0) var<uniform> p:Params;
@group(0) @binding(1) var depth:texture_2d<u32>;
@group(0) @binding(2) var quality:texture_2d<f32>;
${GL_MATCHED_MATH}
struct Vertex { @builtin(position) @invariant position:vec4f, @location(0) @interpolate(flat) valid:u32 };
@vertex fn main(@builtin(vertex_index) index:u32)->Vertex{
  var pixel=vec2i(i32(index%p.size.x),i32(index/p.size.x));var valid=1u;
  // BASELINE_ONLY
  var z=f32(textureLoad(depth,pixel,0).r)*0.001;
  // BASELINE_ZERO
  let point=vec3f((f32(pixel.x)-p.intrinsics.z)*z/p.intrinsics.x,-(f32(pixel.y)-p.intrinsics.w)*z/p.intrinsics.y,-z);
  let clip=p.matrix*vec4f(point,1);var result:Vertex;
  // Texture row zero matches WebGL readPixels row zero. WebGPU clips Z to [0,1].
  result.position=vec4f(clip.x,-clip.y,(clip.z+clip.w)*0.5,clip.w);result.valid=valid;return result;
}
@fragment fn fragment(input:Vertex){if(input.valid==0u){discard;}}
`;
const BASELINE_FLAGS=`let top=pixel-vec2i(0,1);valid=0u;
  if(all(top>=vec2i(0))&&all(top<vec2i(p.size)-vec2i(1))){let flags=textureLoad(quality,top,0);valid=select(0u,1u,select(flags.r,flags.g,p.half==1u)>0.5);}`;
const BASELINE_ZERO=`if(z==0.0){
    let offsets=array<vec2i,4>(vec2i(-1,0),vec2i(1,0),vec2i(0,-1),vec2i(0,1));
    for(var i=0u;i<4u;i++){let next=clamp(pixel+offsets[i],vec2i(0),vec2i(p.size)-vec2i(1));let neighbor=f32(textureLoad(depth,next,0).r)*0.001;
      if(neighbor>0.0){pixel=next;z=neighbor;break;}}
  }`;
const READ_DEPTH=`@group(0) @binding(0) var depth:texture_depth_2d_array;
@group(0) @binding(1) var<storage,read_write> output:array<u32>;
@compute @workgroup_size(8,8,1) fn main(@builtin(global_invocation_id) id:vec3u){
  let size=textureDimensions(depth);if(id.x>=size.x||id.y>=size.y){return;}
  output[(id.z*size.y+id.y)*size.x+id.x]=bitcast<u32>(textureLoad(depth,vec2i(id.xy),i32(id.z),0));
}`;
function baselineIndices(width,height){
  const result=new Uint32Array((width-1)*(height-1)*6);let out=0;
  for(let half=0;half<2;half++)for(let by=0;by<height-1;by+=8)for(let bx=0;bx<width-1;bx+=4)
  for(let y=by;y<Math.min(by+8,height-1);y++)for(let x=bx;x<Math.min(bx+4,width-1);x++){
    const a=y*width+x,b=a+1,c=a+width,d=c+1;
    // WebGPU flat interpolation uses the first vertex. Keep bottom-left first.
    result.set(half?[c,b,d]:[c,a,b],out);out+=3;
  }return result;
}
function buffer(device,bytes,usage){const b=device.createBuffer({size:Math.max(4,bytes.byteLength),usage:usage|GPUBufferUsage.COPY_DST});device.queue.writeBuffer(b,0,bytes);return b;}
export class ExperimentalGpuVisibility{
  static async create(cameras,matrices,{matrixMath='webgl_exact',history=false}={}){
    const adapter=await navigator.gpu?.requestAdapter({powerPreference:'high-performance'});if(!adapter)throw Error('Hardware WebGPU unavailable');
    const timestamps=adapter.features.has('timestamp-query');
    const device=await adapter.requestDevice({requiredFeatures:timestamps?['timestamp-query']:[]});
    const result=new ExperimentalGpuVisibility(device,timestamps,cameras,matrices,matrixMath,history);
    result.adapterInfo={vendor:adapter.info.vendor,architecture:adapter.info.architecture,device:adapter.info.device,description:adapter.info.description,isFallbackAdapter:adapter.info.isFallbackAdapter};
    try{await result.ready;result.initialized=true;return result;}catch(error){result.destroy();throw error;}
  }
  constructor(device,timestamps,cameras,matrices,matrixMath,history=false){
    this.device=device;this.timestamps=timestamps;this.cameras=cameras;this.resources=[];this.sources=[];this.groups=[];
    this.width=Math.max(...cameras.map(c=>c.colorWidth));this.height=Math.max(...cameras.map(c=>c.colorHeight));this.layers=cameras.length*(history?4:2);
    this.ready=this.initialize(matrices,matrixMath);
  }
  async initialize(matrices,matrixMath){
    const d=this.device;
    const shader=async code=>{const s=d.createShaderModule({code});const info=await s.getCompilationInfo();const errors=info.messages.filter(m=>m.type==='error');if(errors.length)throw Error(errors.map(m=>m.message).join('\n'));return s;};
    this.compact=await d.createComputePipelineAsync({layout:'auto',compute:{module:await shader(COMPACT),entryPoint:'main'}});
    const expressions={native:'p.matrix*vec4f(point,1)',pairwise:'(p.matrix[0]*point.x+p.matrix[1]*point.y)+(p.matrix[2]*point.z+p.matrix[3])',sequential:'((p.matrix[0]*point.x+p.matrix[1]*point.y)+p.matrix[2]*point.z)+p.matrix[3]',webgl:'fma(p.matrix[2],vec4f(point.z),fma(p.matrix[1],vec4f(point.y),p.matrix[0]*point.x))+p.matrix[3]',webgl_exact:'project_gl(point)'};
    if(!expressions[matrixMath])throw Error('Unknown matrix evaluation mode');
    const vertexSource=VERTEX.replace('p.matrix*vec4f(point,1)',expressions[matrixMath]);
    const compactVertex=await shader(vertexSource),baselineVertex=await shader(vertexSource.replace('// BASELINE_ONLY',BASELINE_FLAGS).replace('// BASELINE_ZERO',BASELINE_ZERO));
    const pipeline=module=>d.createRenderPipelineAsync({layout:'auto',vertex:{module,entryPoint:'main'},fragment:{module,entryPoint:'fragment',targets:[]},primitive:{topology:'triangle-list',cullMode:'none'},depthStencil:{format:'depth24plus-stencil8',depthWriteEnabled:true,depthCompare:'less',stencilWriteMask:0}});
    this.baselinePipeline=await pipeline(baselineVertex);this.compactPipeline=await pipeline(compactVertex);
    this.readPipeline=await d.createComputePipelineAsync({layout:'auto',compute:{module:await shader(READ_DEPTH),entryPoint:'main'}});
    this.depth=d.createTexture({size:[this.width,this.height,this.layers],format:'depth24plus-stencil8',usage:GPUTextureUsage.RENDER_ATTACHMENT|GPUTextureUsage.TEXTURE_BINDING});this.resources.push(this.depth);
    this.views=Array.from({length:this.layers},(_,baseArrayLayer)=>this.depth.createView({dimension:'2d',baseArrayLayer,arrayLayerCount:1}));
    for(let source=0;source<this.cameras.length;source++){
      const c=this.cameras[source],total=(c.width-1)*(c.height-1)*6;
      const depth=d.createTexture({size:[c.width,c.height],format:'r16uint',usage:GPUTextureUsage.TEXTURE_BINDING|GPUTextureUsage.COPY_DST});
      const quality=d.createTexture({size:[c.width,c.height],format:'rgba8unorm',usage:GPUTextureUsage.TEXTURE_BINDING|GPUTextureUsage.COPY_DST|GPUTextureUsage.RENDER_ATTACHMENT});
      d.queue.writeTexture({texture:depth},c.depth,{bytesPerRow:c.width*2},[c.width,c.height]);d.queue.writeTexture({texture:quality},c.quality,{bytesPerRow:c.width*4},[c.width,c.height]);
      const indices=d.createBuffer({size:total*4,usage:GPUBufferUsage.INDEX|GPUBufferUsage.STORAGE|GPUBufferUsage.COPY_SRC});
      const args=buffer(d,new Uint32Array([0,1,0,0,0]),GPUBufferUsage.STORAGE|GPUBufferUsage.INDIRECT|GPUBufferUsage.COPY_SRC);
      const baseline=buffer(d,baselineIndices(c.width,c.height),GPUBufferUsage.INDEX);this.resources.push(depth,quality,indices,args,baseline);
      this.sources.push({depth,quality,indices,args,baseline,total});
      for(let target=0;target<this.cameras.length;target++){
        const groups={baseline:[],params:[]};
        for(let half=0;half<2;half++){
          const bytes=new ArrayBuffer(96),floats=new Float32Array(bytes),ints=new Uint32Array(bytes);floats.set(matrices[target][source]);floats.set(c.intrinsics,16);ints.set([c.width,c.height,half,0],20);
          const params=buffer(d,bytes,GPUBufferUsage.UNIFORM);this.resources.push(params);
          groups.params.push(params);
          const entries=[{binding:0,resource:{buffer:params}},{binding:1,resource:depth.createView()},{binding:2,resource:quality.createView()}];
          groups.baseline.push(d.createBindGroup({layout:this.baselinePipeline.getBindGroupLayout(0),entries}));
          if(!half){
            // The compact vertex shader no longer consumes triangle validity.
            groups.compact=d.createBindGroup({layout:this.compactPipeline.getBindGroupLayout(0),entries:entries.slice(0,2)});
            if(target===0)this.sources[source].compute=d.createBindGroup({layout:this.compact.getBindGroupLayout(0),entries:[{binding:0,resource:{buffer:params}},{binding:1,resource:quality.createView()},{binding:2,resource:{buffer:indices}},{binding:3,resource:{buffer:args}}]});
          }
        }
        this.groups[target*this.cameras.length+source]=groups;
      }
    }
    if(this.timestamps){this.queries=d.createQuerySet({type:'timestamp',count:4});this.queryResolve=d.createBuffer({size:512,usage:GPUBufferUsage.QUERY_RESOLVE|GPUBufferUsage.COPY_SRC});this.queryRead=d.createBuffer({size:512,usage:GPUBufferUsage.COPY_DST|GPUBufferUsage.MAP_READ});this.resources.push(this.queries,this.queryResolve,this.queryRead);}
  }
  async render(compacted,{timing=true,compute=true,wait=true,layers=null,activeSources=null,layerMap=null}={}){
    if(!this.initialized)await this.ready;const d=this.device,encoder=d.createCommandEncoder();
    const selected=Array.from({length:this.cameras.length*2},(_,i)=>i).filter(i=>!layers||layers.has(i));
    if(compacted&&compute){
      for(const source of this.sources)d.queue.writeBuffer(source.args,0,new Uint32Array([0,1,0,0,0]));
      const pass=encoder.beginComputePass(this.timestamps&&timing?{timestampWrites:{querySet:this.queries,beginningOfPassWriteIndex:0,endOfPassWriteIndex:1}}:{});pass.setPipeline(this.compact);
      this.sources.forEach((source,i)=>{const c=this.cameras[i];pass.setBindGroup(0,source.compute);pass.dispatchWorkgroups(Math.ceil(Math.ceil((c.width-1)/4)*Math.ceil((c.height-1)/8)*32/256));});pass.end();
    }
    for(let target=0;target<this.cameras.length;target++)for(let kind=0;kind<2;kind++){
      if(layers&&!layers.has(target*2+kind))continue;
      const layer=layerMap?layerMap[target*2+kind]:target*2+kind,descriptor={colorAttachments:[],depthStencilAttachment:{view:this.views[layer],depthClearValue:1,depthLoadOp:'clear',depthStoreOp:'store',stencilReadOnly:true}};
      const logical=target*2+kind;
      if(this.timestamps&&timing&&(logical===selected[0]||logical===selected.at(-1)))descriptor.timestampWrites={querySet:this.queries,...(logical===selected[0]?{beginningOfPassWriteIndex:2}:{}),...(logical===selected.at(-1)?{endOfPassWriteIndex:3}:{})};
      const pass=encoder.beginRenderPass(descriptor);pass.setViewport(0,0,this.cameras[target].colorWidth,this.cameras[target].colorHeight,0,1);pass.setPipeline(compacted?this.compactPipeline:this.baselinePipeline);
      this.sources.forEach((source,i)=>{
        if(activeSources&&!activeSources[i])return;
        if((i===target)!==(kind===0))return;const groups=this.groups[target*this.cameras.length+i];
        pass.setIndexBuffer(compacted?source.indices:source.baseline,'uint32');
        if(compacted){pass.setBindGroup(0,groups.compact);pass.drawIndexedIndirect(source.args,0);}
        else for(let half=0;half<2;half++){pass.setBindGroup(0,groups.baseline[half]);pass.drawIndexed(source.total/2,1,half*source.total/2);}
      });pass.end();
    }
    if(this.timestamps&&timing){if(compacted&&compute)encoder.resolveQuerySet(this.queries,0,2,this.queryResolve,0);if(selected.length)encoder.resolveQuerySet(this.queries,2,2,this.queryResolve,256);encoder.copyBufferToBuffer(this.queryResolve,0,this.queryRead,0,512);}
    const started=performance.now();d.queue.submit([encoder.finish()]);
    if(!timing&&!wait)return {wallMs:performance.now()-started};
    if(!this.timestamps||!timing){await d.queue.onSubmittedWorkDone();return {wallMs:performance.now()-started};}
    await this.queryRead.mapAsync(GPUMapMode.READ);const values=new BigUint64Array(this.queryRead.getMappedRange());const result={computeMs:compacted&&compute?Number(values[1]-values[0])/1e6:0,renderMs:selected.length?Number(values[33]-values[32])/1e6:0,wallMs:performance.now()-started};this.queryRead.unmap();return result;
  }
  async readDepth(){
    const d=this.device,size=this.width*this.height*this.layers*4,out=d.createBuffer({size,usage:GPUBufferUsage.STORAGE|GPUBufferUsage.COPY_SRC}),read=d.createBuffer({size,usage:GPUBufferUsage.COPY_DST|GPUBufferUsage.MAP_READ});
    try{const group=d.createBindGroup({layout:this.readPipeline.getBindGroupLayout(0),entries:[{binding:0,resource:this.depth.createView({dimension:'2d-array',aspect:'depth-only'})},{binding:1,resource:{buffer:out}}]}),encoder=d.createCommandEncoder(),pass=encoder.beginComputePass();pass.setPipeline(this.readPipeline);pass.setBindGroup(0,group);pass.dispatchWorkgroups(Math.ceil(this.width/8),Math.ceil(this.height/8),this.layers);pass.end();encoder.copyBufferToBuffer(out,0,read,0,size);d.queue.submit([encoder.finish()]);await read.mapAsync(GPUMapMode.READ);return new Uint32Array(read.getMappedRange().slice(0));}
    finally{if(read.mapState==='mapped')read.unmap();out.destroy();read.destroy();}
  }
  async readIndices(source){
    const d=this.device,s=this.sources[source],args=d.createBuffer({size:20,usage:GPUBufferUsage.COPY_DST|GPUBufferUsage.MAP_READ});let read;
    try{let encoder=d.createCommandEncoder();encoder.copyBufferToBuffer(s.args,0,args,0,20);d.queue.submit([encoder.finish()]);await args.mapAsync(GPUMapMode.READ);const count=new Uint32Array(args.getMappedRange())[0];args.unmap();if(!count)return new Uint32Array();read=d.createBuffer({size:count*4,usage:GPUBufferUsage.COPY_DST|GPUBufferUsage.MAP_READ});encoder=d.createCommandEncoder();encoder.copyBufferToBuffer(s.indices,0,read,0,count*4);d.queue.submit([encoder.finish()]);await read.mapAsync(GPUMapMode.READ);return new Uint32Array(read.getMappedRange().slice(0));}
    finally{if(args.mapState==='mapped')args.unmap();args.destroy();if(read){if(read.mapState==='mapped')read.unmap();read.destroy();}}
  }
  destroy(){for(const resource of this.resources)resource.destroy();this.resources=[];this.device.destroy();}
}
export function webglProjectionArithmetic(zeroExpression){
  return GL_MATCHED_MATH.slice(0,GL_MATCHED_MATH.indexOf('fn project_gl')).replaceAll('p.padding',zeroExpression);
}
