// Native-sample Quality BlendPCR shaders. Opt-in until image/depth parity passes.
// SPDX-License-Identifier: EUPL-1.2
import {webglProjectionArithmetic} from './experimental_gpu_visibility.js';
export function blendPcrShaders(count,{exact=true}={}){
 const declarations=`
struct Frame {
 view_projection:mat4x4f,viewer:mat4x4f,view:mat4x4f,projection:mat4x4f,
 size:vec2u,count:u32,gray:u32,
 crop_min:vec4f,crop_max:vec4f,crop_center:vec4f,crop_inverse:mat3x4f,
 stage_center:vec4f,stage_extent:vec4f,stage_rotation:vec4f,
 mask_start:array<vec4f,6>,mask_end:array<vec4f,6>,layers:array<vec4u,${count}>,flags:vec4u,context_min:vec4f,context_max:vec4f,
};
@group(0) @binding(0) var<uniform> frame:Frame;
@group(0) @binding(1) var<storage,read> cameras:array<vec4f>;
@group(0) @binding(2) var raw_depth:texture_2d_array<u32>;
@group(0) @binding(3) var quality:texture_2d_array<f32>;
@group(0) @binding(4) var rgb:texture_2d_array<f32>;
@group(0) @binding(5) var visibility:texture_depth_2d_array;
@group(0) @binding(6) var linear_sampler:sampler;
@group(0) @binding(7) var metadata:texture_2d_array<u32>;
@group(0) @binding(8) var mask:texture_2d_array<u32>;
@group(0) @binding(9) var<storage,read> tiles:array<vec2u>;
@group(0) @binding(10) var<uniform> source:vec4u;
${exact?webglProjectionArithmetic('frame.flags.y'):''}
fn rn(v:vec4f)->vec4f {${exact?'return rounded(v);':'return v;'}}
fn mad3(a:vec3f,b:vec3f,c:vec3f)->vec3f {${exact?'return fused(vec4f(a,0),vec4f(b,0),vec4f(c,0)).xyz;':'return a*b+c;'}}
fn mad(a:f32,b:f32,c:f32)->f32 {${exact?'return fused(vec4f(a),vec4f(b),vec4f(c)).x;':'return a*b+c;'}}
fn dot3(a:vec3f,b:vec3f)->f32 {${exact?'return mad(a.z,b.z,mad(a.y,b.y,rn(vec4f(a.x*b.x)).x));':'return dot(a,b);'}}
fn dot4(a:vec4f,b:vec4f)->f32 {${exact?'return rn(vec4f(mad(a.z,b.z,mad(a.y,b.y,rn(vec4f(a.x*b.x)).x))+a.w*b.w)).x;':'return dot(a,b);'}}
fn mul4(m:mat4x4f,v:vec4f)->vec4f {${exact?'return rn(fused(m[2],vec4f(v.z),fused(m[1],vec4f(v.y),rn(m[0]*v.x)))+m[3]*v.w);':'return m*v;'}}
fn cross3(a:vec3f,b:vec3f)->vec3f {${exact?'return vec3f(mad(a.y,b.z,-rn(vec4f(a.z*b.y)).x),mad(a.z,b.x,-rn(vec4f(a.x*b.z)).x),mad(a.x,b.y,-rn(vec4f(a.y*b.x)).x));':'return cross(a,b);'}}
fn normal3(v:vec3f)->vec3f{return v*inverseSqrt(dot3(v,v));}
fn data(camera:u32,column:u32)->vec4f{return cameras[camera*13u+column];}
fn local(camera:u32,p:vec3f)->vec3f{let h=vec4f(p,1);return vec3f(dot4(data(camera,0),h),dot4(data(camera,1),h),dot4(data(camera,2),h));}
fn world(camera:u32,p:vec3f)->vec3f{let h=vec4f(p,1);return vec3f(dot4(data(camera,5),h),dot4(data(camera,6),h),dot4(data(camera,7),h));}
// ANGLE cancels the equal translations in world(n)-world(0) before
// rounding. Transform the vector directly to retain that exact result.
fn world_vector(camera:u32,n:vec3f)->vec3f{return vec3f(dot3(data(camera,5).xyz,n),dot3(data(camera,6).xyz,n),dot3(data(camera,7).xyz,n));}
fn camera_origin(camera:u32)->vec3f{return vec3f(data(camera,5).w,data(camera,6).w,data(camera,7).w);}
fn direction(camera:u32,p:vec3f)->vec3f{return vec3f(dot3(data(camera,0).xyz,p),dot3(data(camera,1).xyz,p),dot3(data(camera,2).xyz,p));}
fn mm(camera:u32,p:vec2i)->u32 {if(any(p<vec2i(0))||any(p>=vec2i(data(camera,4).xy))){return 0u;}return textureLoad(raw_depth,p,i32(camera),0).r;}
fn point(p:vec2i,z:f32,k:vec4f)->vec3f{return vec3f((f32(p.x)-k.z)*z/k.x,-(f32(p.y)-k.w)*z/k.y,-z);}
fn triangle_edge(pixel:vec2i,z:f32,k:vec4f,origin:vec3f)->vec3f{
 // ANGLE contracts the final unprojection multiply into the edge/origin
 // subtraction. Rounding a complete second point first changes the plane.
 let dx=rn(vec4f((f32(pixel.x)-k.z)*z)).x;let dy=rn(vec4f(-(f32(pixel.y)-k.w)*z)).x;
 return vec3f(mad(dx,1.0/k.x,-origin.x),mad(dy,1.0/k.y,-origin.y),-z-origin.z);
}
fn outside_crop(p:vec3f,margin:f32)->bool{
 if(frame.crop_min.w==0){return false;}let v=p-frame.crop_center.xyz;
 let rotated=mad3(frame.crop_inverse[2].xyz,vec3f(v.z),mad3(frame.crop_inverse[1].xyz,vec3f(v.y),rn(vec4f(frame.crop_inverse[0].xyz*v.x,0)).xyz));let q=frame.crop_center.xyz+rotated;
 let lo=select(frame.crop_min.xyz,frame.context_min.xyz,margin>0);let hi=select(frame.crop_max.xyz,frame.context_max.xyz,margin>0);
 return any(q<lo)||any(q>hi);
}
fn outside_stage(p:vec3f)->bool{
 if(frame.stage_center.w==0){return false;}let v=p-frame.stage_center.xyz;let r=frame.stage_rotation;
 let q=vec2f(mad(r.x,v.x,-rn(vec4f(r.y*v.z)).x),mad(r.x,v.z,rn(vec4f(r.y*v.x)).x));
 return any(abs(q)>frame.stage_extent.xy)||v.y<frame.stage_extent.z||v.y>frame.stage_extent.w;
}
fn masked(p:vec3f)->bool{
 if(frame.flags.x==0u){return false;}
 for(var i=0u;i<6u;i++){let segment=frame.mask_end[i].xyz-frame.mask_start[i].xyz;let length_sq=dot3(segment,segment);var along=0.0;if(length_sq>0.000001){along=clamp(dot3(p-frame.mask_start[i].xyz,segment)/length_sq,0.0,1.0);}let offset=mad3(-segment,vec3f(along),p-frame.mask_start[i].xyz);if(dot3(offset,offset)<=frame.mask_start[i].w*frame.mask_start[i].w){return true;}}
 return false;
}
`;
 const qualityShader=`${declarations}
@group(0) @binding(11) var quality_out:texture_storage_2d_array<rgba8unorm,write>;
@group(0) @binding(12) var<uniform> limits:vec4f;
fn support(camera:u32,p:vec2i,z:u32,noise_a:f32,noise_b:f32)->vec3f{
 let offsets=array<vec2i,4>(vec2i(-1,0),vec2i(1,0),vec2i(0,-1),vec2i(0,1));var n=vec3f(0);let range=f32(z)*0.001;
 let footprint=range/max(min(data(camera,3).x,data(camera,3).y),1.0);
 let delta=min(limits.x*f32(z),max(4.0,6.0*range*range+2000.0*footprint));
 for(var i=0u;i<4u;i++){let neighbor=mm(camera,p+offsets[i]);if(neighbor==0u){continue;}let difference=abs(f32(neighbor)-f32(z));if(difference<=delta){n.x+=1;}if(difference<=noise_a){n.y+=1;}if(difference<=noise_b){n.z+=1;}}
 return n;
}
fn geometry(camera:u32,za:u32,zb:u32,zc:u32,a:vec3f,b:vec3f,c:vec3f)->bool{
 let lo=min(za,min(zb,zc));let hi=max(za,max(zb,zc));let range=f32(hi)*0.001;let footprint=range/max(min(data(camera,3).x,data(camera,3).y),1.0);
 let delta=min(limits.x*f32(hi),max(4.0,6.0*range*range+2000.0*footprint));
 return lo>0u&&f32(hi-lo)<=delta&&max(dot3(a-b,a-b),max(dot3(b-c,b-c),dot3(c-a,c-a)))<=limits.y*limits.y;
}
@compute @workgroup_size(8,8) fn main(@builtin(global_invocation_id) id:vec3u){
 let camera=source.x+frame.flags.y;let p=vec2i(id.xy);let size=vec2i(data(camera,4).xy);if(any(p>=size)){return;}var result=vec4f(0);
 if(all(p<size-vec2i(1))){let b=p+vec2i(1,0);let c=p+vec2i(0,1);let d=p+vec2i(1,1);let za=mm(camera,p);let zb=mm(camera,b);let zc=mm(camera,c);let zd=mm(camera,d);let k=data(camera,3);
 let pa=point(p,f32(za)*0.001,k);let pb=point(b,f32(zb)*0.001,k);let pc=point(c,f32(zc)*0.001,k);let pd=point(d,f32(zd)*0.001,k);
 var first=geometry(camera,za,zb,zc,pa,pb,pc);var second=geometry(camera,zb,zd,zc,pb,pd,pc);
 if(first||second){let noise_a=max(3.0,0.006*pow(f32(max(za,max(zb,zc)))*0.001,2.0)*1000.0);let noise_b=max(3.0,0.006*pow(f32(max(zb,max(zd,zc)))*0.001,2.0)*1000.0);
 var sa=vec3f(0);var sd=vec3f(0);if(first){sa=support(camera,p,za,noise_a,noise_b);}if(second){sd=support(camera,d,zd,noise_a,noise_b);}let sb=support(camera,b,zb,noise_a,noise_b);let sc=support(camera,c,zc,noise_a,noise_b);
 first=first&&min(sa.x,min(sb.x,sc.x))>=2.0;second=second&&min(sb.x,min(sd.x,sc.x))>=2.0;
 let a=min(sa.y,min(sb.y,sc.y))*0.25;let bn=min(sb.z,min(sd.z,sc.z))*0.25;
 result=vec4f(select(0.0,1.0,first),select(0.0,1.0,second),select(0.0,0.25+0.75*a*a,first),select(0.0,0.25+0.75*bn*bn,second));}}
 textureStore(quality_out,p,i32(camera),result);
}`;
 const metadataShader=`${declarations}
struct Vertex { @builtin(position) @invariant position:vec4f,@location(0) world:vec3f,@location(1) @interpolate(flat) id:u32,@location(2) @interpolate(flat) valid:u32 };
@vertex fn vertex(@builtin(vertex_index) index:u32,@builtin(instance_index) instance:u32)->Vertex{
 let camera=source.x;let size=vec2u(data(camera,4).xy);let origin=vec2i(tiles[instance]);var pixel:vec2i;var valid=1u;var triangle=0u;
 if(index<297u){pixel=vec2i(i32(index%33u),i32(index/33u))+origin;}
 else{let primitive=index-297u;let cell=primitive/2u;let top=vec2i(i32(cell%32u),i32(cell/32u))+origin;let second=(primitive%2u)==1u;var q=vec4f(0);if(all(top<vec2i(size)-vec2i(1))){q=textureLoad(quality,top,i32(camera),0);}valid=select(0u,1u,select(q.r,q.g,second)>0.5);triangle=(u32(top.y)*(size.x-1u)+u32(top.x))*2u+primitive%2u+1u;
 if(valid==1u&&frame.flags.x==1u){var a=top;var b=top+vec2i(1,0);let c=top+vec2i(0,1);if(second){a=b;b=top+vec2i(1,1);}if(textureLoad(mask,a,i32(camera),0).r!=0u||textureLoad(mask,b,i32(camera),0).r!=0u||textureLoad(mask,c,i32(camera),0).r!=0u){valid=0u;triangle=0u;}}
 pixel=top+select(vec2i(select(0,1,second),0),vec2i(0,1),valid==1u);}
 pixel=min(pixel,vec2i(size)-vec2i(1));let depth=mm(camera,pixel);var result:Vertex;result.valid=valid;result.id=triangle;
 if(depth==0u){result.position=vec4f(2,-2,1.5,1);return result;}
 let local_point=point(pixel,f32(depth)*0.001,data(camera,3));let p=world(camera,local_point);result.world=p;let clip=mul4(frame.view_projection,vec4f(p,1));result.position=vec4f(clip.x,-clip.y,(clip.z+clip.w)*0.5,clip.w);return result;
}
@fragment fn fragment(input:Vertex)->@location(0) u32{if(input.valid==0u||input.id==0u||outside_crop(input.world,0.05)){discard;}return input.id;}
`;
 const constantDeclaration=`struct Constants { clip:mat4x4f,origins:array<vec4f,${count}> };`;
 const constants=`${declarations}${constantDeclaration}
 @group(0) @binding(13) var<storage,read_write> constants:Constants;
 @compute @workgroup_size(1) fn main(){
  constants.clip=mat4x4f(mul4(frame.projection,frame.view[0]),mul4(frame.projection,frame.view[1]),mul4(frame.projection,frame.view[2]),mul4(frame.projection,frame.view[3]));
  for(var camera=0u;camera<${count}u;camera++){constants.origins[camera]=vec4f(local(camera,frame.viewer[3].xyz),0);}
 }`;
 const sampling=`${constantDeclaration}
 @group(0) @binding(13) var<storage,read> constants:Constants;
struct Hit { local:vec3f,world:vec3f,normal:vec3f,confidence:f32,score:f32,valid:u32 };
fn triangle(camera:u32,id:u32,origin:vec3f,ray:vec3f,check_mask:bool)->Hit{
 var hit:Hit;if(id==0u||data(camera,4).z<0.5){return hit;}let primitive=id-1u;let width=u32(data(camera,4).x)-1u;let cell=primitive/2u;let p=vec2i(i32(cell%width),i32(cell/width));if(any(p>=vec2i(data(camera,4).xy)-vec2i(1))){return hit;}
 let second=(primitive%2u)==1u;let cache=textureLoad(quality,p,i32(camera),0);if(select(cache.r,cache.g,second)<0.5){return hit;}
 var a=p;var b=p+vec2i(1,0);let c=p+vec2i(0,1);if(second){a=b;b=p+vec2i(1,1);}let k=data(camera,3);
 let pa=point(a,f32(mm(camera,a))*0.001,k);let pb=point(b,f32(mm(camera,b))*0.001,k);let pc=point(c,f32(mm(camera,c))*0.001,k);
 if(check_mask&&frame.flags.x==1u&&(textureLoad(mask,a,i32(camera),0).r!=0u||textureLoad(mask,b,i32(camera),0).r!=0u||textureLoad(mask,c,i32(camera),0).r!=0u)){return hit;}
 let v0=triangle_edge(b,f32(mm(camera,b))*0.001,k,pa);let v1=triangle_edge(c,f32(mm(camera,c))*0.001,k,pa);let n=cross3(v0,v1);let denominator=dot3(n,ray);if(abs(denominator)<1e-15){return hit;}let plane_origin=triangle_edge(a,f32(mm(camera,a))*0.001,k,origin);let t=dot3(n,plane_origin)/denominator;if(t<=0){return hit;}let p3=mad3(ray,vec3f(t),origin);
 let d00=dot3(v0,v0);let d01=dot3(v0,v1);let d11=dot3(v1,v1);let det=d00*d11-d01*d01;if(det<=1e-25){return hit;}
 hit.local=p3;hit.world=world(camera,p3);hit.normal=normal3(n);hit.confidence=select(cache.b,cache.a,second);let range_sq=max(dot3(p3,p3),0.0001);
 hit.score=k.x*k.y*1e-6*hit.confidence*abs(dot3(hit.normal,p3/sqrt(range_sq)))/range_sq;hit.valid=1u;return hit;
}
fn surface(camera:u32,p_world:vec3f)->Hit{
 let p=local(camera,p_world);let z=-p.z;var empty:Hit;if(z<=0.001||data(camera,4).z<0.5){return empty;}let k=data(camera,3);let info=data(camera,4);
 let pixel=vec2f(mad(rn(vec4f(p.x*k.x)).x,1.0/z,k.z),mad(rn(vec4f(-p.y*k.y)).x,1.0/z,k.w));if(any(pixel<vec2f(0))||any(pixel>=info.xy-vec2f(1))){return empty;}
 let cell=vec2u(floor(pixel));let id=(cell.y*(u32(info.x)-1u)+cell.x)*2u+select(0u,1u,fract(pixel.x)+fract(pixel.y)>1.0)+1u;return triangle(camera,id,vec3f(0),p,true);
}
fn tolerance(z:f32)->f32{return min(0.035,max(0.004,0.005*z*z));}
fn rank(q:f32)->f32{return floor(log(max(q,1e-20))*50.4983498);}
fn better(q:f32,index:u32,other:f32,other_index:u32)->bool{let a=rank(q);let b=rank(other);return a>b||(a==b&&index<other_index);}
fn world_normal(camera:u32,n:vec3f)->vec3f{return normal3(world_vector(camera,n));}
fn free_margin(confidence:f32,expected:f32)->f32{return max(select(0.008,0.012,confidence>=0.95),tolerance(expected)*select(2.0,3.0,confidence>=0.95));}
fn may_free(camera:u32,p:vec3f,margin:f32)->bool{
 let info=data(camera,4);let k=data(camera,3);if(info.z<0.5||p.z>=-0.001){return false;}let pixel=vec2f(p.x*k.x/-p.z+k.z,-p.y*k.y/-p.z+k.w);if(any(pixel<vec2f(0))||any(pixel>=info.xy-vec2f(1))){return false;}
 let cell=vec2i(floor(pixel));var a=cell;let b=cell+vec2i(1,0);let c=cell+vec2i(0,1);if(fract(pixel.x)+fract(pixel.y)>1.0){a+=vec2i(1);}
 return f32(max(mm(camera,a),max(mm(camera,b),mm(camera,c))))*0.001>-p.z+margin;
}
fn veto(own_index:u32,p_world:vec3f,own:Hit)->bool{
 let required=own.score*select(1.15,1.5,own.confidence>=0.95);
 for(var peer=0u;peer<${count}u;peer++){if(peer==own_index){continue;}let p=local(peer,p_world);let k=data(peer,3);if(p.z>=-0.001||k.x*k.y*1e-6/max(dot3(p,p),0.0001)<=required){continue;}
 if(!may_free(peer,p,free_margin(own.confidence,-p.z))){continue;}let other=surface(peer,p_world);if(other.valid==1u&&other.confidence>0.95&&other.score>required&&-other.local.z>-p.z+free_margin(own.confidence,-p.z)){return true;}}
 return false;
}
fn sample_camera(camera:u32,pixel:vec2u)->Hit{
 var empty:Hit;let id=textureLoad(metadata,vec2i(pixel),i32(camera),0).r;if(id==0u){return empty;}let screen=(vec2f(pixel)+vec2f(0.5))/vec2f(frame.size)*2-1;
 let ray=mul4(frame.viewer,vec4f(screen.x/frame.projection[0][0],screen.y/frame.projection[1][1],-1,0)).xyz;
 let hit=triangle(camera,id,constants.origins[camera].xyz,direction(camera,ray),false);
 if(hit.valid==0u||-mul4(frame.view,vec4f(hit.world,1)).z<=0.0||veto(camera,hit.world,hit)){return empty;}return hit;
}
fn visible_depth(q:vec2i,end:vec2i,layer:u32)->f32{
 let a=textureLoad(visibility,q,i32(layer),0);let b=textureLoad(visibility,min(q+vec2i(1,0),end),i32(layer),0);let c=textureLoad(visibility,min(q+vec2i(0,1),end),i32(layer),0);let d=textureLoad(visibility,min(q+vec2i(1),end),i32(layer),0);
 let observed=min(min(a,b),min(c,d));return (0.01*65.535)/mad(-observed,65.535-0.01,65.535);
}
fn same_surface(camera:u32,p_world:vec3f,n:vec3f)->bool{let measured=surface(camera,p_world);if(measured.valid==0u||measured.confidence<0.95){return false;}let mn=world_normal(camera,measured.normal);return abs(dot3(n,mn))>=0.985&&abs(dot3(p_world-measured.world,mn))<=0.012&&abs(local(camera,p_world).z-measured.local.z)<=0.012;}
struct Rgb {color:vec3f,score:f32,valid:u32};
fn rgb_sample(camera:u32,p_world:vec3f,n:vec3f)->Rgb{
 var result:Rgb;if(data(camera,4).z<0.5||data(camera,12).z<0.5){return result;}let lp=local(camera,p_world)*vec3f(1,-1,-1);let h=vec4f(lp,1);let p=vec3f(dot4(data(camera,8),h),dot4(data(camera,9),h),dot4(data(camera,10),h));let k=data(camera,11);let size=data(camera,12).xy;
 if(p.z<=0.01){return result;}let projected=p.xy/p.z;let pixel=vec2f(mad(projected.x,k.x,k.z),mad(projected.y,k.y,k.w));if(any(pixel<vec2f(0))||any(pixel>size-vec2f(1))){return result;}let q=vec2i(floor(pixel));let end=vec2i(size)-vec2i(1);let margin=max(0.004,0.004*p.z*p.z);let layers=frame.layers[camera];
 let own_z=visible_depth(q,end,layers.y);if(own_z<p.z-margin){return result;}let visible_z=min(own_z,visible_depth(q,end,layers.x));let gap=p.z-visible_z;
 if(gap>margin){if(!same_surface(camera,p_world,n)){return result;}if(gap>0.012){if(gap>0.016){return result;}let previous=min(visible_depth(q,end,layers.z),visible_depth(q,end,layers.w));if(p.z-previous>0.012){return result;}}}
 let ray=p_world-camera_origin(camera);let range_sq=max(dot3(ray,ray),0.0001);let incidence=abs(dot3(n,ray*inverseSqrt(range_sq)));let edge=min(min(pixel.x,pixel.y),min(size.x-1-pixel.x,size.y-1-pixel.y));
 // Match the fused polynomial used by ANGLE's smoothstep. Native WGSL's
 // separately rounded 3-2*t changes edge weights and occasional RGB bytes.
 let t=clamp(edge*0.125,0.0,1.0);let edge_weight=t*t*mad(-2.0,t,3.0);
 result.score=k.x*k.y*1e-6*max(incidence,0.02)/range_sq*edge_weight;if(result.score<=0){return result;}
 result.color=textureSampleLevel(rgb,linear_sampler,(pixel+vec2f(0.5))/vec2f(textureDimensions(rgb)),i32(camera),0.0).rgb;result.valid=1u;return result;
}
fn gl_mix(a:vec3f,b:vec3f,mask:vec3<bool>)->vec3f{return mad3(b-a,select(vec3f(0),vec3f(1),mask),a);}
// ANGLE's lerp contracts the low branch into the high-minus-low difference.
// A branch select or a separately rounded subtraction changes a few RGB bytes.
fn to_linear(c:vec3f)->vec3f{let lo=c/12.92;let hi=pow(max((c+0.055)/1.055,vec3f(0)),vec3f(2.4));let difference=mad3(-c,vec3f(1.0/12.92),hi);return mad3(difference,select(vec3f(0),vec3f(1),c>=vec3f(0.04045)),lo);}
fn to_srgb(c:vec3f)->vec3f{let lo=c*12.92;let hi=mad3(vec3f(1.055),pow(max(c,vec3f(0)),vec3f(1.0/2.4)),vec3f(-0.055));let difference=mad3(-c,vec3f(12.92),hi);return mad3(difference,select(vec3f(0),vec3f(1),c>=vec3f(0.0031308)),lo);}
fn projected_color(p:vec3f,n:vec3f,owner:u32)->vec3f{var color=vec3f(0);var source_color=vec3f(0.34);var total=0.0;var verified=false;
 for(var camera=0u;camera<${count}u;camera++){let sample=rgb_sample(camera,p,n);if(sample.valid==0u){continue;}if(camera==owner){source_color=sample.color;verified=true;}let weight=sample.score*sample.score;color=mad3(to_linear(sample.color),vec3f(weight),color);total=mad(sample.score,sample.score,total);}
 if(total>1e-12){return to_srgb(color/total);}if(verified){return source_color;}return vec3f(0.34);
}
`;
 const composition=`${declarations}${sampling}
@vertex fn vertex(@builtin(vertex_index) id:u32)->@builtin(position) vec4f{let p=vec2f(f32((id<<1u)&2u),f32(id&2u));return vec4f(p*2-1,0,1);}
struct Output {@location(0) color:vec4f,@builtin(frag_depth) depth:f32};
@fragment fn fragment(@builtin(position) position:vec4f)->Output{
 let pixel=vec2u(position.xy);var hits:array<Hit,${count}>;var depths:array<f32,${count}>;var nearest=1e30;
 for(var camera=0u;camera<${count}u;camera++){var hit=sample_camera(camera,pixel);if(hit.valid==1u&&(outside_crop(hit.world,0)||outside_stage(hit.world))){hit.valid=0u;}hits[camera]=hit;let z=-mul4(frame.view,vec4f(hit.world,1)).z;depths[camera]=z;if(hit.valid==1u){nearest=min(nearest,z);}}
 var owner=0xffffffffu;var score=-1.0;var best_world=vec3f(0);var best_normal=vec3f(0);
 for(var camera=0u;camera<${count}u;camera++){let hit=hits[camera];if(hit.valid==0u||abs(depths[camera]-nearest)>tolerance(nearest)){continue;}if(owner==0xffffffffu||better(hit.score,camera,score,owner)){owner=camera;score=hit.score;best_world=hit.world;best_normal=world_vector(camera,hit.normal);}}
 if(owner==0xffffffffu){discard;}var color:vec3f;
 if(frame.gray==1u){let length_sq=dot3(best_normal,best_normal);var diffuse=0.6;if(length_sq>1e-20){diffuse=abs(dot3(best_normal*inverseSqrt(length_sq),normal3(vec3f(0.35,0.8,0.45))));}color=vec3f(mad(0.52,diffuse,0.28));}
 else{color=projected_color(best_world,normal3(best_normal),owner);}
 let clip=mul4(constants.clip,vec4f(best_world,1));var result:Output;result.color=vec4f(color,1);result.depth=clip.z/clip.w*0.5+0.5;return result;
}`;
 const maskShader=`${declarations}
 @group(0) @binding(11) var mask_out:texture_storage_2d_array<r32uint,write>;
 @compute @workgroup_size(8,8) fn main(@builtin(global_invocation_id) id:vec3u){let camera=source.x;let p=vec2i(id.xy);if(any(p>=vec2i(data(camera,4).xy))){return;}let z=mm(camera,p);if(z==0u){textureStore(mask_out,p,i32(camera),vec4u(0));return;}let position=world(camera,point(p,f32(z)*0.001,data(camera,3)));textureStore(mask_out,p,i32(camera),vec4u(select(0u,1u,masked(position))));}`;
 return {quality:qualityShader,mask:maskShader,metadata:metadataShader,composition,constants};
}
