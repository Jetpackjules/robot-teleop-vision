// Selected points stay in their original 9-byte GPU layout. Reusable, native
// pixel atlases are rebuilt only for changed captures; no CPU geometry rebuild.
import {compileGpuProgram,gpuTexture} from './hybrid_camera_quality.js?v=20261027_compact_gpu';
import {COLOR_PROJECTION_SHADER,bindColorProjection,validatedColorProjection} from './hybrid_color_projection.js?v=20261021_original_rgb';

const CAPACITY=4;
export function bindSquareSamples(gl,entry,camera){
  gl.bindBuffer(gl.ARRAY_BUFFER,entry.buffer);
  gl.enableVertexAttribArray(0);gl.vertexAttribIPointer(0,1,gl.UNSIGNED_INT,0,0);gl.vertexAttribDivisor(0,0);
  if(entry.compact){
    gl.enableVertexAttribArray(1);gl.vertexAttribIPointer(1,1,gl.UNSIGNED_SHORT,0,entry.count*4);gl.vertexAttribDivisor(1,0);
    gl.enableVertexAttribArray(2);gl.vertexAttribPointer(2,3,gl.UNSIGNED_BYTE,true,0,entry.count*6);gl.vertexAttribDivisor(2,0);
  }else{gl.disableVertexAttribArray(1);gl.disableVertexAttribArray(2);}
}
export function inverseSquareCamera(m){
  const out=new Float32Array([m[0],m[4],m[8],0,m[1],m[5],m[9],0,m[2],m[6],m[10],0,0,0,0,1]);
  for(let row=0;row<3;row++)out[12+row]=-(out[row]*m[12]+out[4+row]*m[13]+out[8+row]*m[14]);
  return out;
}

const ATLAS_VERTEX=`#version 300 es
precision highp float;precision highp int;precision highp usampler2D;
layout(location=0) in uint a_sample;layout(location=1) in uint a_depth;layout(location=2) in vec3 a_color;
uniform int u_compact;uniform usampler2D u_depth;uniform sampler2D u_color;
uniform ivec2 u_size;uniform vec4 u_intrinsics;
uniform int u_rgb_pass;uniform vec2 u_target_size;uniform vec2 u_rgb_vis_size;
flat out uint v_depth;out vec4 v_color;
${COLOR_PROJECTION_SHADER}
void main(){
  gl_Position=vec4(2,2,2,1);gl_PointSize=1.;v_depth=0u;v_color=vec4(0);
  ivec2 pixel=ivec2(int(a_sample)%u_size.x,int(a_sample)/u_size.x);
  uint mm=u_compact==1?a_depth:texelFetch(u_depth,pixel,0).r;if(mm==0u)return;
  float z=float(mm)*.001;vec3 local=vec3((float(pixel.x)-u_intrinsics.z)*z/u_intrinsics.x,-(float(pixel.y)-u_intrinsics.w)*z/u_intrinsics.y,-z);
  vec2 xy=vec2(pixel)+.5;v_depth=mm;
  vec4 color=u_compact==1?vec4(a_color,camera_has_color(local)?1.:0.):camera_sample_color(u_color,local,pixel);
  v_color=color;
  if(u_rgb_pass==1){
    vec2 rgb_pixel;if(!camera_color_pixel(local,rgb_pixel))return;
    vec3 rgb=u_depth_to_color_rotation*(local*vec3(1,-1,-1))+u_depth_to_color_translation;
    xy=(rgb_pixel+.5)/u_raw_color_size*u_rgb_vis_size;
    v_depth=uint(clamp(floor(rgb.z*1000.+.5),1.,65535.));
    gl_PointSize=3.;
    gl_Position=vec4(xy/u_target_size*2.-1.,rgb.z/65.535*2.-1.,1.);
  }else gl_Position=vec4(xy/u_target_size*2.-1.,0,1);
}`;
const ATLAS_FRAGMENT=`#version 300 es
precision highp float;precision highp int;
flat in uint v_depth;in vec4 v_color;
layout(location=0) out highp uint out_depth;layout(location=1) out vec4 out_color;
void main(){if(v_depth==0u)discard;out_depth=v_depth;out_color=v_color;}`;

export const SQUARES_FUSION_SHADER=`
uniform highp usampler2DArray u_square_depth;
uniform highp sampler2DArray u_square_color;
uniform highp usampler2DArray u_square_rgb_visibility;
uniform highp int u_square_count;uniform highp int u_square_source;
uniform highp mat4 u_square_world_to_camera[4];
uniform highp vec4 u_square_intrinsics[4];
uniform highp vec2 u_square_size[4];
uniform highp mat3 u_square_rgb_rotation[4];
uniform highp vec3 u_square_rgb_translation[4];
uniform highp vec4 u_square_rgb_intrinsics[4];
uniform highp vec2 u_square_rgb_size[4];
uniform highp vec2 u_square_visibility_size[4];
uniform highp float u_square_capture_delta[4];
bool square_color_visible(int camera,vec3 local){
  vec3 p=u_square_rgb_rotation[camera]*(local*vec3(1,-1,-1))+u_square_rgb_translation[camera];
  if(p.z<=0.)return false;
  vec2 pixel=p.xy/p.z*u_square_rgb_intrinsics[camera].xy+u_square_rgb_intrinsics[camera].zw;
  if(any(lessThan(pixel,vec2(0)))||any(greaterThanEqual(pixel,u_square_rgb_size[camera]-1.)))return false;
  ivec2 cell=ivec2(floor((pixel+.5)/u_square_rgb_size[camera]*u_square_visibility_size[camera]));
  float nearest=float(texelFetch(u_square_rgb_visibility,ivec3(cell,camera),0).r)*.001;
  // No known front surface is treated as unknown, never as proof of visibility.
  vec2 focal=u_square_rgb_intrinsics[camera].xy*u_square_visibility_size[camera]/u_square_rgb_size[camera];
  float footprint=clamp(2.*p.z/min(focal.x,focal.y),.006,.025);
  return nearest>0. && p.z<=nearest+footprint;
}
// False rejects a measured point; colour is borrowed only on the same surface.
bool fuse_square_surface(vec3 world,vec3 local,inout vec4 color){
  if(u_square_count<2||u_square_source<0)return true;
  int own=u_square_source;
  if(color.a>0. && !square_color_visible(own,local))color.a=0.;
  float best=-1.;vec3 borrowed=vec3(0);int contradictions=0,agreements=0;
  for(int peer=0;peer<4;peer++){
    if(peer>=u_square_count)break;
    if(peer==own||u_square_capture_delta[peer]>.05)continue;
    vec3 p=(u_square_world_to_camera[peer]*vec4(world,1)).xyz;
    float z=-p.z;if(z<=0.)continue;
    vec4 k=u_square_intrinsics[peer];
    vec2 uv=vec2(p.x,-p.y)/z*k.xy+k.zw;
    ivec2 center=ivec2(floor(uv+.5)),size=ivec2(u_square_size[peer]);
    if(any(lessThan(center,ivec2(1)))||any(greaterThanEqual(center,size-1)))continue;
    float tolerance=min(.012,.008+.002*z),nearest=1e5,farthest=0.;int support=0;
    float candidate_error=1e5,surface_error=1e5;vec3 candidate=vec3(0);bool candidate_valid=false;
    for(int y=-1;y<=1;y++)for(int x=-1;x<=1;x++){
      ivec2 pixel=center+ivec2(x,y);float d=float(texelFetch(u_square_depth,ivec3(pixel,peer),0).r)*.001;
      if(d<=0.)continue;
      support++;nearest=min(nearest,d);farthest=max(farthest,d);
      float error=abs(d-z);surface_error=min(surface_error,error);
      if(color.a<=0. && error<=tolerance && error<candidate_error){
        vec4 rgb=texelFetch(u_square_color,ivec3(pixel,peer),0);
        vec3 donor=vec3((float(pixel.x)-k.z)*d/k.x,-(float(pixel.y)-k.w)*d/k.y,-d);
        if(rgb.a>.5 && square_color_visible(peer,donor)){
          candidate_error=error;candidate=rgb.rgb;candidate_valid=true;
        }
      }
    }
    // Reject only a dense, smooth peer patch that positively contradicts the
    // point in free space. A nearer peer surface merely occludes the point.
    bool stable=support==9 && farthest-nearest<max(.006,z*.01);
    if(stable && z<nearest-tolerance)contradictions++;
    if(surface_error<=tolerance)agreements++;
    if(candidate_valid){
      float score=1./(.001+candidate_error);
      if(score>best){best=score;borrowed=candidate;}
    }
  }
  // One misaligned camera must not veto a surface corroborated by another.
  if(contradictions>0 && agreements==0)return false;
  if(color.a<=0. && best>0.)color=vec4(borrowed,1);
  return true;
}
`;

export class SquaresCameraFusion {
  constructor(gl){this.gl=gl;this.bindings=new WeakMap();this.layers=[];this.rebuilds=0;this.cameras=[];}
  dispose(){
    const gl=this.gl;
    for(const key of ['depth','color','visibility'])if(this[key])gl.deleteTexture(this[key]);
    if(this.framebuffer)gl.deleteFramebuffer(this.framebuffer);if(this.depthBuffer)gl.deleteRenderbuffer(this.depthBuffer);
    if(this.program)gl.deleteProgram(this.program);if(this.vao)gl.deleteVertexArray(this.vao);
    this.depth=this.color=this.visibility=this.framebuffer=this.depthBuffer=this.program=this.vao=null;this.signature='';this.layers=[];
  }
  update(cameras,entries,emptyDepth,emptyColor){
    const gl=this.gl,started=performance.now();
    this.cameras=cameras.filter(c=>entries.has(c)&&validatedColorProjection(c.metadata));
    if(this.cameras.length<2||this.cameras.length>CAPACITY)return {active:false,message:'Needs 2–4 cameras with calibrated RGB coverage.',rebuilds:this.rebuilds};
    const projections=this.cameras.map(c=>validatedColorProjection(c.metadata));
    const w=Math.max(...this.cameras.map(c=>c.width)),h=Math.max(...this.cameras.map(c=>c.height));
    this.visibilitySizes=projections.map(p=>{const scale=Math.min(1,1024/p.size[0]);return p.size.map(v=>Math.ceil(v*scale));});
    const vw=Math.max(...this.visibilitySizes.map(s=>s[0])),vh=Math.max(...this.visibilitySizes.map(s=>s[1]));
    const signature=`${w}:${h}:${vw}:${vh}:${this.cameras.length}`;
    if(this.signature!==signature){
      this.dispose();
      this.depth=gpuTexture(gl,gl.TEXTURE_2D_ARRAY,10,gl.R16UI,w,h,this.cameras.length);
      this.color=gpuTexture(gl,gl.TEXTURE_2D_ARRAY,11,gl.RGBA8,w,h,this.cameras.length);
      this.visibility=gpuTexture(gl,gl.TEXTURE_2D_ARRAY,12,gl.R16UI,vw,vh,this.cameras.length);
      this.framebuffer=gl.createFramebuffer();this.depthBuffer=gl.createRenderbuffer();gl.bindRenderbuffer(gl.RENDERBUFFER,this.depthBuffer);
      gl.renderbufferStorage(gl.RENDERBUFFER,gl.DEPTH_COMPONENT24,vw,vh);
      this.vao=gl.createVertexArray();this.program=compileGpuProgram(gl,ATLAS_VERTEX,ATLAS_FRAGMENT);this.u={};
      for(const name of ['u_compact','u_depth','u_color','u_size','u_intrinsics','u_rgb_pass','u_target_size','u_rgb_vis_size'])this.u[name]=gl.getUniformLocation(this.program,name);
      this.signature=signature;
    }
    const u=this.u;gl.useProgram(this.program);gl.bindVertexArray(this.vao);gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
    gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);gl.disable(gl.SCISSOR_TEST);gl.depthMask(true);
    try{
      this.cameras.forEach((camera,index)=>{
        const entry=entries.get(camera),key=`${camera.uploadRevision}:${entry.count}:${projections[index].extrinsics}:${projections[index].intrinsics}:${projections[index].size}`;
        if(this.layers[index]?.camera===camera&&this.layers[index].key===key)return;
        bindSquareSamples(gl,entry,camera);bindColorProjection(gl,this.program,camera);
        gl.uniform1i(u.u_compact,entry.compact?1:0);gl.uniform2i(u.u_size,camera.width,camera.height);gl.uniform4fv(u.u_intrinsics,camera.intrinsicsArray);
        gl.uniform1i(u.u_depth,0);gl.uniform1i(u.u_color,1);
        gl.activeTexture(gl.TEXTURE0);gl.bindTexture(gl.TEXTURE_2D,entry.compact?emptyDepth:camera.depthTexture);
        gl.activeTexture(gl.TEXTURE1);gl.bindTexture(gl.TEXTURE_2D,entry.compact?emptyColor:camera.colorTexture);
        gl.framebufferRenderbuffer(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,gl.RENDERBUFFER,null);
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,this.depth,0,index);
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT1,this.color,0,index);
        gl.drawBuffers([gl.COLOR_ATTACHMENT0,gl.COLOR_ATTACHMENT1]);gl.viewport(0,0,w,h);gl.disable(gl.DEPTH_TEST);
        gl.clearBufferuiv(gl.COLOR,0,new Uint32Array(4));gl.clearBufferfv(gl.COLOR,1,new Float32Array(4));
        if(gl.checkFramebufferStatus(gl.FRAMEBUFFER)!==gl.FRAMEBUFFER_COMPLETE)throw Error('Squares fusion framebuffer unavailable');
        gl.uniform1i(u.u_rgb_pass,0);gl.uniform2f(u.u_target_size,w,h);gl.drawArrays(gl.POINTS,0,entry.count);
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT1,null,0,0);
        gl.framebufferTextureLayer(gl.FRAMEBUFFER,gl.COLOR_ATTACHMENT0,this.visibility,0,index);
        gl.framebufferRenderbuffer(gl.FRAMEBUFFER,gl.DEPTH_ATTACHMENT,gl.RENDERBUFFER,this.depthBuffer);
        gl.drawBuffers([gl.COLOR_ATTACHMENT0]);gl.viewport(0,0,vw,vh);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LESS);
        gl.clearBufferuiv(gl.COLOR,0,new Uint32Array(4));gl.clearDepth(1);gl.clear(gl.DEPTH_BUFFER_BIT);
        gl.uniform1i(u.u_rgb_pass,1);gl.uniform2f(u.u_target_size,vw,vh);gl.uniform2fv(u.u_rgb_vis_size,this.visibilitySizes[index]);
        gl.drawArrays(gl.POINTS,0,entry.count);this.layers[index]={camera,key};this.rebuilds++;
      });
    }finally{gl.bindFramebuffer(gl.FRAMEBUFFER,null);gl.bindVertexArray(null);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);}
    return {active:true,cameras:this.cameras.length,rebuilds:this.rebuilds,updateMs:performance.now()-started};
  }
  bind(program,source,now){
    const gl=this.gl;let u=this.bindings.get(program);
    if(!u){u={};for(const n of ['u_square_depth','u_square_color','u_square_rgb_visibility','u_square_count','u_square_source','u_square_world_to_camera[0]','u_square_intrinsics[0]','u_square_size[0]','u_square_rgb_rotation[0]','u_square_rgb_translation[0]','u_square_rgb_intrinsics[0]','u_square_rgb_size[0]','u_square_visibility_size[0]','u_square_capture_delta[0]'])u[n]=gl.getUniformLocation(program,n);this.bindings.set(program,u);}
    // Bind textures even when disabled: active sampler types must never alias.
    for(const [name,key,unit] of [['u_square_depth','depth',10],['u_square_color','color',11],['u_square_rgb_visibility','visibility',12]]){
      gl.uniform1i(u[name],unit);gl.activeTexture(gl.TEXTURE0+unit);gl.bindTexture(gl.TEXTURE_2D_ARRAY,this[key]||null);
    }
    const own=this.cameras.indexOf(source),active=this.depth&&this.cameras.length>=2&&own>=0;
    gl.uniform1i(u.u_square_count,active?this.cameras.length:0);gl.uniform1i(u.u_square_source,own);
    if(!active)return;
    const arrays={matrices:[],intrinsics:[],sizes:[],rotations:[],translations:[],rgbIntrinsics:[],rgbSizes:[],visSizes:[],deltas:[]};
    const stamp=c=>Number(c.metadata.geometry_capture_unix_ms||c.metadata.capture_unix_ms||0),ownTime=stamp(source);
    for(let i=0;i<CAPACITY;i++){
      const camera=this.cameras[i]||source,p=validatedColorProjection(camera.metadata),time=stamp(camera);
      arrays.matrices.push(...inverseSquareCamera(camera.modelMatrix));arrays.intrinsics.push(...camera.intrinsicsArray);arrays.sizes.push(camera.width,camera.height);
      arrays.rotations.push(...p.extrinsics.slice(0,9));arrays.translations.push(...p.extrinsics.slice(9));arrays.rgbIntrinsics.push(...p.intrinsics);arrays.rgbSizes.push(...p.size);arrays.visSizes.push(...(this.visibilitySizes[i]||this.visibilitySizes[own]));
      arrays.deltas.push(ownTime>0&&time>0&&now-camera.lastUpdateMs<250?Math.abs(time-ownTime)*.001:1000);
    }
    gl.uniformMatrix4fv(u['u_square_world_to_camera[0]'],false,arrays.matrices);gl.uniform4fv(u['u_square_intrinsics[0]'],arrays.intrinsics);gl.uniform2fv(u['u_square_size[0]'],arrays.sizes);
    gl.uniformMatrix3fv(u['u_square_rgb_rotation[0]'],false,arrays.rotations);gl.uniform3fv(u['u_square_rgb_translation[0]'],arrays.translations);gl.uniform4fv(u['u_square_rgb_intrinsics[0]'],arrays.rgbIntrinsics);
    gl.uniform2fv(u['u_square_rgb_size[0]'],arrays.rgbSizes);gl.uniform2fv(u['u_square_visibility_size[0]'],arrays.visSizes);gl.uniform1fv(u['u_square_capture_delta[0]'],arrays.deltas);
  }
}
