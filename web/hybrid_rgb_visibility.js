// Color visibility from the entire measured rig, independent of depth ownership.
// Full native triangles are cached in each original RGB camera's optical frame.
// SPDX-License-Identifier: EUPL-1.2
import {bindWorkspaceTiles,unbindWorkspaceTiles} from './workspace_crop.js?v=20261021_original_rgb';
import {visibleNativeTiles,nativeSourceTiles} from './native_geometry.js?v=20261026_native_render';
import {rgbVisibilityRegion,regionContains,rgbRegionClip} from './rgb_visibility_region.js?v=20261027_visibility_region';
const VERTEX=`#version 300 es
precision highp float;precision highp int;precision highp usampler2D;
uniform usampler2D u_depth;
uniform highp sampler2DArray u_quality;
uniform int u_source_index;
uniform ivec2 u_size;
uniform vec4 u_intrinsics;
uniform mat4 u_to_rgb_clip;
#ifdef SHARED_VISIBILITY
uniform int u_triangle_half;
#endif
#ifdef TILED_VISIBILITY
layout(location=0) in uvec2 a_source_tile;
layout(location=1) in uint a_source_vertex;
#endif
flat out float v_valid;
void main(){
  int width=u_size.x,vertex=gl_VertexID;
  ivec2 origin=ivec2(0,gl_InstanceID);
#ifdef TILED_VISIBILITY
  width=33;vertex=int(a_source_vertex);origin=ivec2(a_source_tile);
#endif
  ivec2 pixel;
  v_valid=1.0;
  int grid=width*2;
#ifdef TILED_VISIBILITY
  grid=33*9;
#endif
  if(vertex<grid)pixel=ivec2(vertex%width,vertex/width)+origin;
  else {
    int primitive=vertex-grid,cell=primitive/2;
    ivec2 top=ivec2(cell%(width-1),cell/(width-1))+origin;
    bool inside=all(lessThan(top,u_size-ivec2(1)));
    vec4 q=inside?texelFetch(u_quality,ivec3(top,u_source_index),0):vec4(0);
    bool second=(primitive%2)==1;
    v_valid=(second?q.g:q.r)>0.5?1.0:0.0;
    pixel=v_valid>0.5?top+ivec2(0,1):top+ivec2(second?1:0,0);
  }
#ifdef SHARED_VISIBILITY
  // Both triangle groups use their native bottom-left corner as the
  // provoking vertex. The shared vertex therefore supplies the exact cell
  // validity while all three positions retain their measured geometry.
  ivec2 top=pixel-ivec2(0,1);
  bool inside=all(greaterThanEqual(top,ivec2(0)))&&all(lessThan(top,u_size-ivec2(1)));
  vec4 q=inside?texelFetch(u_quality,ivec3(top,u_source_index),0):vec4(0);
  v_valid=(u_triangle_half==0?q.r:q.g)>0.5?1.0:0.0;
#endif
  pixel=min(pixel,u_size-ivec2(1));
  float z=float(texelFetch(u_depth,pixel,0).r)*0.001;
#ifdef SHARED_VISIBILITY
  // A triangle touching zero depth has already failed its flat validity
  // test. Place only that discarded vertex near a measured neighbor so it
  // cannot create a huge off-screen triangle before fragment rejection.
  if(z==0.0){
    const ivec2 offsets[4]=ivec2[4](ivec2(-1,0),ivec2(1,0),ivec2(0,-1),ivec2(0,1));
    for(int i=0;i<4;i++){
      ivec2 next=clamp(pixel+offsets[i],ivec2(0),u_size-ivec2(1));
      float neighbor=float(texelFetch(u_depth,next,0).r)*0.001;
      if(neighbor>0.0){pixel=next;z=neighbor;break;}
    }
  }
#endif
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
  constructor(gl,compile,allocate,replayQuality=null){this.gl=gl;this.compile=compile;this.allocate=allocate;this.replayQuality=replayQuality;this.buffers=new Map();this.tileUploads=new Map();this.passes=0;this.tiled=true;this.sharedVertices=true;this.useRegions=true;this.layers=new Int32Array(0);this.sources=[];this.revisions=[];this.layerStates=[];this.snapshotCache=new WeakMap();}
  clear(){const gl=this.gl;if(this.texture)gl.deleteTexture(this.texture);if(this.framebuffer)gl.deleteFramebuffer(this.framebuffer);
    if(this.program)gl.deleteProgram(this.program);if(this.tileProgram)gl.deleteProgram(this.tileProgram);if(this.vao)gl.deleteVertexArray(this.vao);
    for(const b of this.buffers.values())gl.deleteBuffer(b.buffer);this.buffers.clear();
    for(const b of this.tileUploads.values()){gl.deleteBuffer(b.buffer);gl.deleteBuffer(b.vertexBuffer);}this.tileUploads.clear();
    this.texture=this.framebuffer=this.program=this.tileProgram=this.vao=null;
    this.signature='';this.historyKey='';this.layers=new Int32Array(0);this.sources=[];this.revisions=[];this.layerStates=[];this.snapshotCache=new WeakMap();}
  update(cameras,quality,inverse,enabled,now,active=true,revisions=null,options={}){
    const gl=this.gl,width=enabled?Math.max(...cameras.map(c=>c.colorWidth||c.width)):1,
      height=enabled?Math.max(...cameras.map(c=>c.colorHeight||c.height)):1;
    const tiled=this.tiled;
    const signature=`${width}:${height}:${cameras.length}:${tiled}:${this.sharedVertices}`;
    if(signature!==this.signature){this.clear();
      // Ping-pong self and peer minima instead of copying full RGB depth maps.
      // min(self,peers) is exactly the original rig-wide depth observation.
      if(cameras.length*4>gl.getParameter(gl.MAX_ARRAY_TEXTURE_LAYERS))throw new Error('RGB visibility exceeds GPU layer limits');
      this.texture=this.allocate(gl,gl.TEXTURE_2D_ARRAY,15,gl.DEPTH_COMPONENT24,width,height,cameras.length*4);
      this.layers=new Int32Array(cameras.length*4);
      this.framebuffer=gl.createFramebuffer();this.vao=gl.createVertexArray();
      this.program=this.compile(gl,VERTEX,FRAGMENT);this.u={};
      for(const name of ['u_depth','u_quality','u_source_index','u_size','u_intrinsics','u_to_rgb_clip'])this.u[name]=gl.getUniformLocation(this.program,name);
      if(tiled){
        const defines='#version 300 es\n#define TILED_VISIBILITY'+(this.sharedVertices?'\n#define SHARED_VISIBILITY':'');
        this.tileProgram=this.compile(gl,VERTEX.replace('#version 300 es',defines),FRAGMENT.replace('#version 300 es',defines));this.tileU={};
        for(const name of Object.keys(this.u))this.tileU[name]=gl.getUniformLocation(this.tileProgram,name);
        this.tileU.u_triangle_half=gl.getUniformLocation(this.tileProgram,'u_triangle_half');
      }
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
    if(!hasHistory){this.layerStates=[];this.snapshotCache=new WeakMap();}
    const regions=options.regions===true&&Boolean(this.replayQuality);
    const sourceRevisions=revisions||cameras.map(c=>c.uploadRevision??c.lastUpdateMs);
    const changed=cameras.map((camera,index)=>this.sources[index]!==camera||this.revisions[index]!==sourceRevisions[index]);
    const slotSize=cameras.length*2;
    for(let target=0;target<cameras.length;target++){
      const camera=cameras[target];if(camera.metadata.color_space!=='original_rgb')continue;
      const offset=target*4,oldPeer=this.layers[offset],oldSelf=this.layers[offset+1];
      const dirtySelf=!hasHistory||changed[target];
      const dirtyPeers=!hasHistory||changed.some((dirty,index)=>dirty&&index!==target);
      const nextPeer=dirtyPeers?(hasHistory?(oldPeer+slotSize)%(slotSize*2):target):oldPeer;
      const nextSelf=dirtySelf?(hasHistory?(oldSelf+slotSize)%(slotSize*2):cameras.length+target):oldSelf;
      // Each map rotates independently. Unchanged maps are the same exact
      // current and previous observation; no copy or repeated raster is needed.
      if(!options.regionOnly)this.layers.set([nextPeer,nextSelf,hasHistory?oldPeer:nextPeer,hasHistory?oldSelf:nextSelf],offset);
      const targetProjection=multiply(projection(camera.metadata.color_projection),inverse(camera.modelMatrix));
      const region=rgbVisibilityRegion(regions?options.bounds:null,targetProjection,camera.colorWidth,camera.colorHeight);
      const narrow=regions&&(region.x!==0||region.y!==0||region.width!==camera.colorWidth||region.height!==camera.colorHeight);
      // Expanding or disabling a crop restores historical maps from their
      // original raw captures. Never discard temporal evidence on a crop edit.
      for(const layer of new Set(this.layers.slice(offset,offset+4))){
        if(layer===nextPeer&&dirtyPeers||layer===nextSelf&&dirtySelf)continue;
        const previous=this.layerStates[layer];
        if(previous&&!regionContains(previous.region,region))this.replayLayer(layer,previous,region,tiled,cameras.length);
      }
      if(!dirtySelf&&!dirtyPeers)continue;
      gl.viewport(0,0,camera.colorWidth,camera.colorHeight);gl.clearDepth(1);
      for(const [dirty,layer,self]of [[dirtySelf,nextSelf,true],[dirtyPeers,nextPeer,false]]){
        if(!dirty)continue;
        gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);gl.bindVertexArray(this.vao);
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,this.texture,0,layer);gl.clear(gl.DEPTH_BUFFER_BIT);
        gl.activeTexture(gl.TEXTURE6);gl.bindTexture(gl.TEXTURE_2D_ARRAY,quality);
        const records=[];
        for(let source=0;source<cameras.length;source++){
          if((source===target)!==self)continue;
          const c=cameras[source];
          if(now-c.lastUpdateMs>=1000)continue;
          const matrix=multiply(targetProjection,c.modelMatrix);
          if(narrow){
            let snapshot=this.snapshotCache.get(c);
            if(!snapshot||snapshot.revision!==sourceRevisions[source]){
              snapshot={revision:sourceRevisions[source],camera:{...c,modelMatrix:new Float32Array(c.modelMatrix),intrinsicsArray:new Float32Array(c.intrinsicsArray)},delta:options.delta,edge:options.edge};this.snapshotCache.set(c,snapshot);
            }
            records.push({snapshot,source,matrix});
          }
          if(region.width&&region.height)this.drawSource(c,source,matrix,tiled,rgbRegionClip(matrix,region,camera.colorWidth,camera.colorHeight));
        }
        this.layerStates[layer]=narrow?{records,region,width:camera.colorWidth,height:camera.colorHeight}:null;
      }
    }
    this.sources=cameras.slice();this.revisions=sourceRevisions.slice();
    this.historyKey=historyKey;
    const referenced=new Set(this.layers);
    this.layerStates.forEach((state,layer)=>{if(!referenced.has(layer))this.layerStates[layer]=null;});
    if(!regions||!this.layerStates.some(Boolean)){this.layerStates=[];this.snapshotCache=new WeakMap();}
    gl.depthFunc(gl.LEQUAL);
  }
  replayLayer(layer,previous,region,tiled,count){
    const gl=this.gl;gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,this.texture,0,layer);gl.disable(gl.SCISSOR_TEST);gl.depthMask(true);gl.clearDepth(1);gl.clear(gl.DEPTH_BUFFER_BIT);
    if(region.width&&region.height)for(const record of previous.records){
      const replay=this.replayQuality(record.snapshot,record.source,count);
      gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);gl.bindVertexArray(this.vao);gl.viewport(0,0,previous.width,previous.height);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LESS);gl.depthMask(true);gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);gl.disable(gl.SCISSOR_TEST);
      gl.activeTexture(gl.TEXTURE6);gl.bindTexture(gl.TEXTURE_2D_ARRAY,replay.quality);
      this.drawSource({...record.snapshot.camera,depthTexture:replay.depth},record.source,record.matrix,tiled,rgbRegionClip(record.matrix,region,previous.width,previous.height));
    }
    previous.region=region;this.regionReplays=(this.regionReplays||0)+1;
  }
  drawSource(c,source,matrix,tiled,cullingMatrix=matrix){
    let tiles=tiled?visibleNativeTiles(c,cullingMatrix,null,[0,0],true):null;
    if(tiled&&!tiles&&(c.metadata.stream_depth_origin||this.sharedVertices))tiles=nativeSourceTiles(c);
    tiled=Boolean(tiles);
    const shared=tiled&&this.sharedVertices;
    const gl=this.gl,key=shared?'shared':tiled?'tile':c.width,u=tiled?this.tileU:this.u;
    gl.useProgram(tiled?this.tileProgram:this.program);gl.uniform1i(u.u_depth,0);gl.uniform1i(u.u_quality,6);
    let index=this.buffers.get(key);
    if(!index){const values=[],w=tiled?33:c.width,h=tiled?9:2;
      for(let y=0;y<h-1;y++)for(let x=0;x<w-1;x++){
        const a=y*w+x,b=a+1,d=a+w+1,primitive=(y*(w-1)+x)*2;
        if(shared)continue;
        values.push(a,b,w*h+primitive,b,d,w*h+primitive+1);
      }
      if(shared)for(let half=0;half<2;half++)for(let bx=0;bx<32;bx+=4)for(let y=0;y<8;y++)for(let x=bx;x<bx+4;x++){
        const a=y*33+x,b=a+1,c=a+33,d=c+1;
        values.push(...(half===0?[a,b,c]:[b,d,c]));
      }
      const wide=4*w*h>65535,indices=wide?new Uint32Array(values):new Uint16Array(values),buffer=gl.createBuffer();
      gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER,buffer);gl.bufferData(gl.ELEMENT_ARRAY_BUFFER,indices,gl.STATIC_DRAW);
      index={buffer,count:indices.length,type:wide?gl.UNSIGNED_INT:gl.UNSIGNED_SHORT};this.buffers.set(key,index);
    }
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER,index.buffer);
    gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,c.depthTexture);
    gl.uniform1i(u.u_source_index,source);gl.uniform2i(u.u_size,c.width,c.height);gl.uniform4fv(u.u_intrinsics,c.intrinsicsArray);
    if(matrix)gl.uniformMatrix4fv(u.u_to_rgb_clip,false,matrix);
    if(tiled){this.tileUploads.set(c,bindWorkspaceTiles(gl,tiles,this.tileUploads.get(c)||{}));
      if(shared){const count=index.count/2,bytes=index.type===gl.UNSIGNED_SHORT?2:4;
        for(let half=0;half<2;half++){gl.uniform1i(u.u_triangle_half,half);gl.drawElementsInstanced(gl.TRIANGLES,count,index.type,half*count*bytes,tiles.length/2);}
      }else gl.drawElementsInstanced(gl.TRIANGLES,index.count,index.type,0,tiles.length/2);
      unbindWorkspaceTiles(gl);
    }else gl.drawElementsInstanced(gl.TRIANGLES,index.count,index.type,0,c.height-1);
    this.passes++;
  }
}
