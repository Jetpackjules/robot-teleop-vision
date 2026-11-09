// Selected points stay in their original 9-byte GPU layout. Reusable, native
// pixel atlases are rebuilt only for changed captures; no CPU geometry rebuild.
import {prepareGpuProgram,gpuTexture} from './hybrid_camera_quality.js?v=20261109_background_startup';
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
uniform int u_rgb_pass;uniform int u_center_rgb_cells;uniform vec2 u_target_size;uniform vec2 u_rgb_vis_size;
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
    // The reader uses floor(xy). Write at that texel center so point
    // rasterization cannot choose its neighbor at an RGB depth edge.
    if(u_center_rgb_cells==1)xy=floor(xy)+.5;
    v_depth=uint(clamp(floor(rgb.z*1000.+.5),1.,65535.));
    // Test visibility at the projected RGB cell. Expanding a foreground
    // sample into neighboring cells hides valid color across depth edges.
    gl_PointSize=1.;
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
uniform highp ivec3 u_square_priority; // A supplement, fixed B/C references; -1 when absent.
uniform highp mat4 u_square_world_to_camera[4];
uniform highp vec4 u_square_intrinsics[4];
uniform highp vec2 u_square_size[4];
uniform highp mat3 u_square_rgb_rotation[4];
uniform highp vec3 u_square_rgb_translation[4];
uniform highp vec4 u_square_rgb_intrinsics[4];
uniform highp vec2 u_square_rgb_size[4];
uniform highp vec2 u_square_visibility_size[4];
uniform highp float u_square_capture_delta[4];
int square_rgb_visibility(int camera,vec3 local){
  vec3 p=u_square_rgb_rotation[camera]*(local*vec3(1,-1,-1))+u_square_rgb_translation[camera];
  if(p.z<=0.)return 0;
  vec2 pixel=p.xy/p.z*u_square_rgb_intrinsics[camera].xy+u_square_rgb_intrinsics[camera].zw;
  if(any(lessThan(pixel,vec2(0)))||any(greaterThanEqual(pixel,u_square_rgb_size[camera]-1.)))return 0;
  ivec2 cell=ivec2(floor((pixel+.5)/u_square_rgb_size[camera]*u_square_visibility_size[camera]));
  float nearest=float(texelFetch(u_square_rgb_visibility,ivec3(cell,camera),0).r)*.001;
  // No known front surface is treated as unknown, never as proof of visibility.
  vec2 focal=u_square_rgb_intrinsics[camera].xy*u_square_visibility_size[camera]/u_square_rgb_size[camera];
  float footprint=clamp(2.*p.z/min(focal.x,focal.y),.006,.025);
  if(nearest<=0.)return 0;
  return p.z<=nearest+footprint?1:-1;
}
bool square_color_visible(int camera,vec3 local){
  return square_rgb_visibility(camera,local)==1;
}
// Native depth remains untouched. Only A's preview geometry is supplementary.
vec3 square_local_point(int camera,ivec2 pixel,float depth){
  vec4 k=u_square_intrinsics[camera];
  return vec3((float(pixel.x)-k.z)*depth/k.x,-(float(pixel.y)-k.w)*depth/k.y,-depth);
}
float square_depth_at(int camera,ivec2 pixel){
  if(any(lessThan(pixel,ivec2(0)))||any(greaterThanEqual(pixel,ivec2(u_square_size[camera]))))return 0.;
  return float(texelFetch(u_square_depth,ivec3(pixel,camera),0).r)*.001;
}
bool square_patch_normal(int camera,ivec2 pixel,vec3 center,bool reference,out vec3 normal){
  int step=int(clamp(floor(.003*u_square_intrinsics[camera].x/(-center.z)+.5),1.,4.));
  ivec2 dx=ivec2(step,0),dy=ivec2(0,step);
  float l=square_depth_at(camera,pixel-dx),r=square_depth_at(camera,pixel+dx);
  float t=square_depth_at(camera,pixel-dy),b=square_depth_at(camera,pixel+dy);
  float spread=reference?max(.008,-center.z*.012):(-center.z*.5);
  if(min(min(l,r),min(t,b))<=0. || max(max(abs(l+center.z),abs(r+center.z)),max(abs(t+center.z),abs(b+center.z)))>spread)return false;
  vec3 x=square_local_point(camera,pixel+dx,r)-square_local_point(camera,pixel-dx,l);
  vec3 y=square_local_point(camera,pixel+dy,b)-square_local_point(camera,pixel-dy,t);
  vec3 n=cross(x,y);if(length(n)<1e-9)return false;
  n=normalize(n);if(abs(dot(center-square_local_point(camera,pixel-dx,l),n))>.003)return false;
  normal=normalize(transpose(mat3(u_square_world_to_camera[camera]))*n);return true;
}
bool square_reference_patch(int camera,vec3 world,out vec3 anchor,out vec3 normal){
  vec3 local=(u_square_world_to_camera[camera]*vec4(world,1)).xyz;
  float z=-local.z;if(z<=0.)return false;
  vec4 k=u_square_intrinsics[camera];
  ivec2 center=ivec2(floor(vec2(local.x,-local.y)/z*k.xy+k.zw+.5));
  // A small physical search catches noise on either side of a surface, including
  // edge pixels. Bounded 25 samples per reference, only for A's points.
  int step=int(clamp(floor(.008*k.x/z+.5),1.,12.));
  float best=.018*.018;vec3 selected=vec3(0);ivec2 selected_pixel=center;bool found=false;
  for(int y=-2;y<=2;y++)for(int x=-2;x<=2;x++){
    ivec2 pixel=center+ivec2(x,y)*step;float d=square_depth_at(camera,pixel);if(d<=0.)continue;
    vec3 p=square_local_point(camera,pixel,d);float error=dot(p-local,p-local);
    if(error<best){best=error;selected=p;selected_pixel=pixel;found=true;}
  }
  if(!found)return false;
  // Edges need not have a complete central-difference stencil. Require local
  // measured support without joining opposite sides of a depth discontinuity.
  int support=0;
  for(int y=-1;y<=1;y++)for(int x=-1;x<=1;x++){
    ivec2 pixel=selected_pixel+ivec2(x,y);float d=square_depth_at(camera,pixel);
    if(d<=0.)continue;
    vec3 delta=square_local_point(camera,pixel,d)-selected;
    if(dot(delta,delta)<=.006*.006)support++;
  }
  if(support<4)return false;
  normal=vec3(0);square_patch_normal(camera,selected_pixel,selected,true,normal);
  mat4 inverse=u_square_world_to_camera[camera];
  anchor=transpose(mat3(inverse))*(selected-inverse[3].xyz);return true;
}
bool square_keep_supplement(vec3 world,vec3 local){
  if(u_square_source!=u_square_priority.x || any(lessThan(u_square_priority,ivec3(0))))return true;
  int b=u_square_priority.y,c=u_square_priority.z;
  if(u_square_capture_delta[b]>.05 || u_square_capture_delta[c]>.05)return true;
  vec3 bp,bn,cp,cn;
  if(!square_reference_patch(b,world,bp,bn) || !square_reference_patch(c,world,cp,cn))return true;
  // Parallel but separated reference planes are genuine disagreement, not
  // coverage of one edge. Do not let that ambiguity erase A's measurement.
  if(abs(dot(bn,cn))>.88 && max(abs(dot(bp-cp,bn)),abs(dot(bp-cp,cn)))>.006)return true;
  // B/C own their covered edges even when they see different cube faces.
  // A remains a colour donor, but does not draw a second nearby surface layer.
  // Keep distinct new faces when all three local normals establish one.
  vec4 k=u_square_intrinsics[u_square_source];
  ivec2 pixel=ivec2(floor(vec2(local.x,-local.y)/(-local.z)*k.xy+k.zw+.5));vec3 own_normal;
  if(length(bn)>.5 && length(cn)>.5 && square_patch_normal(u_square_source,pixel,local,false,own_normal) &&
      max(abs(dot(own_normal,bn)),abs(dot(own_normal,cn)))<.65)return true;
  return false;
}
float square_chroma(vec3 rgb){
  return max(rgb.r,max(rgb.g,rgb.b))-min(rgb.r,min(rgb.g,rgb.b));
}
vec3 square_hue(vec3 rgb){
  return (rgb-min(rgb.r,min(rgb.g,rgb.b)))/max(.001,square_chroma(rgb));
}
bool square_has_pigment(vec3 rgb){
  return square_chroma(rgb)>=max(.085,.36*max(rgb.r,max(rgb.g,rgb.b)));
}
bool square_matching_pigment(vec3 a,vec3 b){
  return square_has_pigment(a) && square_has_pigment(b) && distance(square_hue(a),square_hue(b))<=.28;
}
// Find consistent RGB just inside a depth silhouette. A second camera
// must corroborate the pigment at the measured 3D location.
bool square_edge_reference(int camera,vec3 local,vec3 rgb,out vec3 reference){
  reference=vec3(0);
  float brightness=max(rgb.r,max(rgb.g,rgb.b)),chroma=square_chroma(rgb);
  if(brightness<.24 || brightness>.86 || chroma>.30)return false;
  float z=-local.z;
  vec4 k=u_square_intrinsics[camera];
  ivec2 center=ivec2(floor(vec2(local.x,-local.y)/z*k.xy+k.zw+.5));
  int step=int(clamp(floor(.0025*k.x/z+.5),1.,4.));
  float tolerance=max(.007,z*.012);
  const ivec2 directions[4]=ivec2[4](ivec2(-1,0),ivec2(1,0),ivec2(0,-1),ivec2(0,1));
  for(int i=0;i<4;i++){
    ivec2 offset=directions[i]*step;
    float outer=square_depth_at(camera,center+offset);
    float farther=square_depth_at(camera,center+2*offset);
    bool at_edge=outer<=0. || outer>z+.012 || farther<=0. || farther>z+.012;
    for(int j=1;j<=2;j++){
      ivec2 near_pixel=center-j*offset,far_pixel=near_pixel-offset;
      float near_depth=square_depth_at(camera,near_pixel),far_depth=square_depth_at(camera,far_pixel);
      if(near_depth<=0. || far_depth<=0. || max(abs(near_depth-z),abs(far_depth-z))>tolerance)continue;
      vec4 near_rgb=texelFetch(u_square_color,ivec3(near_pixel,camera),0);
      vec4 far_rgb=texelFetch(u_square_color,ivec3(far_pixel,camera),0);
      if(min(near_rgb.a,far_rgb.a)<=.5 || !square_matching_pigment(near_rgb.rgb,far_rgb.rgb))continue;
      vec3 inner_rgb=(near_rgb.rgb+far_rgb.rgb)*.5;
      if(square_chroma(inner_rgb)<chroma+.045 || (chroma>=.075 && distance(square_hue(rgb),square_hue(inner_rgb))>.46))continue;
      if(!at_edge && (chroma<.075 || !square_matching_pigment(rgb,inner_rgb)))continue;
      if(!square_color_visible(camera,square_local_point(camera,near_pixel,near_depth)) ||
         !square_color_visible(camera,square_local_point(camera,far_pixel,far_depth)))continue;
      reference=inner_rgb;
      return true;
    }
  }
  return false;
}
// Preserve points on an object wall. Suppression needs nearby pigment
// above the plane as well as measured tabletop below it.
bool square_has_colored_contact(int camera,ivec2 center,vec3 local,vec3 rgb){
  mat4 inverse=u_square_world_to_camera[camera];
  vec3 pigment=vec3(0);
  int support=0;
  for(int y=-2;y<=2;y++)for(int x=-2;x<=2;x++){
    if(x==0 && y==0)continue;
    ivec2 pixel=center+ivec2(x,y)*4;
    float d=square_depth_at(camera,pixel);
    if(d<=0.)continue;
    vec3 other=square_local_point(camera,pixel,d);
    vec3 delta=transpose(mat3(inverse))*(other-local);
    float lateral=length(delta.xz);
    if(delta.y<.003 || delta.y>.042 || lateral>.019)continue;
    vec4 sample_rgb=texelFetch(u_square_color,ivec3(pixel,camera),0);
    if(sample_rgb.a<=.5)continue;
    if(lateral<=.011 && distance(sample_rgb.rgb,rgb)<.16)return false;
    if(!square_has_pigment(sample_rgb.rgb))continue;
    if(support==0){pigment=sample_rgb.rgb;support=1;}
    else if(square_matching_pigment(sample_rgb.rgb,pigment))support++;
  }
  return support>=2;
}
// Cull only neutral points backed by actual samples of the tabletop.
bool square_table_fringe(vec3 world,vec3 local,vec4 color,float plane_y){
  float height=world.y-plane_y;
  float brightness=max(color.r,max(color.g,color.b));
  if(u_square_source<0 || color.a<=0. || height<0. || height>.013 ||
     brightness<.28 || square_chroma(color.rgb)>.065)return false;
  int camera=u_square_source;
  float z=-local.z;
  vec4 k=u_square_intrinsics[camera];
  ivec2 center=ivec2(floor(vec2(local.x,-local.y)/z*k.xy+k.zw+.5));
  const ivec2 directions[4]=ivec2[4](ivec2(1,0),ivec2(-1,0),ivec2(0,1),ivec2(0,-1));
  mat4 inverse=u_square_world_to_camera[camera];
  int support=0;
  for(int i=0;i<4;i++){
    for(int step=1;step<=3;step++){
      ivec2 pixel=center+directions[i]*step*2;
      float d=square_depth_at(camera,pixel);
      if(d<=0.)continue;
      vec3 neighbor=square_local_point(camera,pixel,d);
      if(distance(neighbor,local)>.021 ||
         abs(dot(inverse[1].xyz,neighbor-inverse[3].xyz)-plane_y)>.003)continue;
      vec4 sample_rgb=texelFetch(u_square_color,ivec3(pixel,camera),0);
      if(sample_rgb.a<=.5 || distance(sample_rgb.rgb,color.rgb)>.13 ||
         square_chroma(sample_rgb.rgb)>.07)continue;
      support++;
      break;
    }
    if(support>=2)return square_has_colored_contact(camera,center,local,color.rgb);
  }
  return false;
}
// Color may read the background at a measured depth discontinuity.
bool square_native_edge(vec3 local){
  int camera=u_square_source;
  if(camera<0)return false;
  float z=-local.z;
  vec4 k=u_square_intrinsics[camera];
  ivec2 center=ivec2(floor(vec2(local.x,-local.y)/z*k.xy+k.zw+.5));
  const ivec2 sides[4]=ivec2[4](ivec2(-2,0),ivec2(2,0),ivec2(0,-2),ivec2(0,2));
  for(int i=0;i<4;i++){
    float nearby=square_depth_at(camera,center+sides[i]);
    if(nearby<=0. || abs(nearby-z)>max(.007,z*.013))return true;
  }
  return false;
}
// Recover a partially washed color from the visible interior of its own depth
// silhouette when another camera cannot corroborate a replacement.
bool square_recover_silhouette_color(int camera, vec3 local, vec3 original, out vec3 recovered) {
  float light = max(original.r, max(original.g, original.b));
  float chroma = square_chroma(original);
  if (light < .125 || light > .725 || chroma < max(.046, .31 * light) || chroma > .75 * light ||
      !square_color_visible(camera, local)) return false;

  float z = -local.z;
  vec4 k = u_square_intrinsics[camera];
  ivec2 center = ivec2(floor(vec2(local.x, -local.y) / z * k.xy + k.zw + .5));
  int edge_step = int(clamp(round(.003 * k.x / z), 1., 4.));
  float outside_depth = z + max(.008, .013 * z);
  float minimum_saturation = max(.65, chroma / light + .14);
  vec3 original_hue = square_hue(original);
  float best_cost = .012;
  const ivec2 directions[8] = ivec2[8](ivec2(1, 0), ivec2(-1, 0), ivec2(0, 1), ivec2(0, -1),
    ivec2(1, 1), ivec2(-1, 1), ivec2(1, -1), ivec2(-1, -1));
  for (int i = 0; i < 8; i++) {
    ivec2 axis = directions[i];
    float outside_near = square_depth_at(camera, center - edge_step * axis);
    float outside_mid = square_depth_at(camera, center - (edge_step + 1) * axis);
    float outside_far = square_depth_at(camera, center - (edge_step + 2) * axis);
    if ((outside_near > 0. && outside_near <= outside_depth) ||
        (outside_mid > 0. && outside_mid <= outside_depth) ||
        (outside_far > 0. && outside_far <= outside_depth)) continue;

    float spacing = z * length(vec2(axis) / k.xy);
    int previous_radius = 0;
    float previous_depth = 0.;
    vec3 previous_color = vec3(0.);
    for (int radius = 2; radius <= 7; radius++) {
      float travel = float(radius) * spacing;
      if (travel > .009 || travel >= best_cost) break;
      ivec2 pixel = center + radius * axis;
      float depth = square_depth_at(camera, pixel);
      float dz = depth - z;
      if (depth <= 0. || travel * travel + dz * dz > .012 * .012) continue;
      vec4 donor = texelFetch(u_square_color, ivec3(pixel, camera), 0);
      float donor_light = max(donor.r, max(donor.g, donor.b));
      float donor_chroma = square_chroma(donor.rgb);
      float neutral_drop = (light - chroma) - (donor_light - donor_chroma);
      if (donor.a <= .5 || donor_chroma < minimum_saturation * donor_light ||
          donor_light < .56 * light || donor_light > 1.50 * light ||
          (donor_chroma < chroma + .012 && (donor_chroma < chroma - .010 || neutral_drop < .018)) ||
          distance(square_hue(donor.rgb), original_hue) > .35) continue;

      // The two interior samples must extrapolate back to the measured depth.
      if (previous_radius > 0) {
        float error = abs(dz * float(previous_radius) - (previous_depth - z) * float(radius)) /
          float(radius - previous_radius);
        float cost = travel + .60 * error;
        vec3 difference = abs(previous_color - donor.rgb);
        if (error <= .004 && cost < best_cost && max(difference.r, max(difference.g, difference.b)) < .07 &&
            square_color_visible(camera, square_local_point(camera, center + previous_radius * axis, previous_depth)) &&
            square_color_visible(camera, square_local_point(camera, pixel, depth))) {
          best_cost = cost;
          recovered = (previous_color + donor.rgb) * .5;
        }
      }
      previous_radius = radius;
      previous_depth = depth;
      previous_color = donor.rgb;
    }
  }
  return best_cost < .012;
}
// False rejects a measured point; colour is borrowed only on the same surface.
bool fuse_square_surface(vec3 world,vec3 local,inout vec4 color,bool repair_edges){
  if(u_square_count<2||u_square_source<0)return true;
  int own=u_square_source;
  if(!square_keep_supplement(world,local))return false;
  if(color.a>0. && !square_color_visible(own,local))color.a=0.;
  vec3 edge_reference;
  bool repair_edge=repair_edges && color.a>0. && square_edge_reference(own,local,color.rgb,edge_reference);
  float best=-1.,repair_error=1e5;vec3 borrowed=vec3(0),repaired=vec3(0);int contradictions=0,agreements=0;
  for(int peer=0;peer<4;peer++){
    if(peer>=u_square_count)break;
    if(peer==own||u_square_capture_delta[peer]>.05)continue;
    vec3 p=(u_square_world_to_camera[peer]*vec4(world,1)).xyz;
    float z=-p.z;if(z<=0.)continue;
    vec4 k=u_square_intrinsics[peer];
    vec2 uv=vec2(p.x,-p.y)/z*k.xy+k.zw;
    ivec2 center=ivec2(floor(uv+.5)),size=ivec2(u_square_size[peer]);
    if(any(lessThan(center,ivec2(1)))||any(greaterThanEqual(center,size-1)))continue;
    // Verify the receiver's projected pixel as well as the donor. A visible
    // neighboring donor cannot supply color through an occluder at the target.
    bool canBorrow=color.a<=0. && square_rgb_visibility(peer,p)>=0;
    bool canRepair=repair_edge && square_rgb_visibility(peer,p)>=0;
    int repair_support=0;
    float repair_distance=1e5;vec3 repair_sum=vec3(0);
    float tolerance=min(.012,.008+.002*z),nearest=1e5,farthest=0.;int support=0;
    float candidate_error=1e5,surface_error=1e5;vec3 candidate=vec3(0);bool candidate_valid=false;
    for(int y=-1;y<=1;y++)for(int x=-1;x<=1;x++){
      ivec2 pixel=center+ivec2(x,y);float d=float(texelFetch(u_square_depth,ivec3(pixel,peer),0).r)*.001;
      if(d<=0.)continue;
      support++;nearest=min(nearest,d);farthest=max(farthest,d);
      float error=abs(d-z);surface_error=min(surface_error,error);
      if(canRepair && error<=.007){
        vec4 rgb=texelFetch(u_square_color,ivec3(pixel,peer),0);
        vec3 donor=square_local_point(peer,pixel,d);
        float distance_to_edge=distance(donor,p);
        if(rgb.a>.5 && distance_to_edge<=.010 && square_color_visible(peer,donor)){
          if(square_matching_pigment(rgb.rgb,edge_reference)){
            repair_support++;repair_sum+=rgb.rgb;
            repair_distance=min(repair_distance,distance_to_edge);
          }
        }
      }
      if(canBorrow && error<=tolerance && error<candidate_error){
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
    // A smooth matching patch can bridge a tiny colour hole. Keep this
    // fallback local and require each donor to pass the normal RGB visibility test.
    if(canBorrow && !candidate_valid && stable && surface_error<=tolerance){
      const ivec2 offsets[4]=ivec2[4](ivec2(-2,0),ivec2(2,0),ivec2(0,-2),ivec2(0,2));
      for(int direction=0;direction<4;direction++){
        ivec2 pixel=center+offsets[direction];
        float d=square_depth_at(peer,pixel);
        if(d<=0. || abs(d-z)>tolerance || d<nearest-.002 || d>farthest+.002)continue;
        vec3 donor=square_local_point(peer,pixel,d);
        float distance=length(donor-p);
        if(distance>.012 || distance+.012>=candidate_error)continue;
        vec4 rgb=texelFetch(u_square_color,ivec3(pixel,peer),0);
        if(rgb.a>.5 && square_color_visible(peer,donor)){
          candidate_error=distance+.012;candidate=rgb.rgb;candidate_valid=true;
        }
      }
    }
    // The nearest supporting sample must be on the same measured surface.
    // A slightly wider search covers a donor pixel displaced by registration noise.
    if(canRepair && repair_support<2 && surface_error<=tolerance){
      const ivec2 extra[8]=ivec2[8](ivec2(-2,-1),ivec2(-2,1),ivec2(2,-1),ivec2(2,1),ivec2(-1,-2),ivec2(1,-2),ivec2(-1,2),ivec2(1,2));
      for(int i=0;i<8;i++){
        ivec2 pixel=center+extra[i];
        float d=square_depth_at(peer,pixel);
        if(d<=0. || abs(d-z)>.007)continue;
        vec3 donor=square_local_point(peer,pixel,d);
        float distance_to_edge=distance(donor,p);
        vec4 rgb=texelFetch(u_square_color,ivec3(pixel,peer),0);
        if(rgb.a<=.5 || distance_to_edge>.007 || !square_color_visible(peer,donor) || !square_matching_pigment(rgb.rgb,edge_reference))continue;
        repair_support++;repair_sum+=rgb.rgb;
        repair_distance=min(repair_distance,distance_to_edge);
      }
    }
    if(repair_support>=2 && repair_distance<=.005 && repair_distance<repair_error){
      repaired=repair_sum/float(repair_support);
      repair_error=repair_distance;
    }
    if(stable && z<nearest-tolerance)contradictions++;
    if(surface_error<=tolerance)agreements++;
    if(candidate_valid){
      float score=1./(.001+candidate_error);
      if(score>best){best=score;borrowed=candidate;}
    }
  }
  // One misaligned camera must not veto a surface corroborated by another.
  if(contradictions>0 && agreements==0)return false;
  if (repair_error < 1e5) color.rgb = repaired;
  else if (repair_edges && color.a > 0. && square_recover_silhouette_color(own, local, color.rgb, repaired)) {
    color.rgb = repaired;
  }
  if (color.a <= 0. && best > 0.) color = vec4(borrowed, 1);
  return true;
}
`;

export class SquaresCameraFusion {
  constructor(gl){this.gl=gl;this.bindings=new WeakMap();this.layers=[];this.rebuilds=0;this.cameras=[];}
  releaseTargets() {
    const gl = this.gl;
    for (const key of ['depth', 'color', 'visibility']) if (this[key]) gl.deleteTexture(this[key]);
    if (this.framebuffer) gl.deleteFramebuffer(this.framebuffer);
    if (this.depthBuffer) gl.deleteRenderbuffer(this.depthBuffer);
    this.depth = this.color = this.visibility = this.framebuffer = this.depthBuffer = null;
    this.signature = '';
    this.layers = [];
  }
  dispose() {
    this.releaseTargets();
    this.pendingProgram?.dispose();
    if (this.program) this.gl.deleteProgram(this.program);
    if (this.vao) this.gl.deleteVertexArray(this.vao);
    this.program = this.vao = this.pendingProgram = null;
  }
  update(cameras,entries,emptyDepth,emptyColor,centerRgbCells=false){
    const gl=this.gl,started=performance.now();
    this.cameras=cameras.filter(c=>entries.has(c)&&validatedColorProjection(c.metadata));
    if(this.cameras.length<2||this.cameras.length>CAPACITY)return {active:false,message:'Needs 2â€“4 cameras with calibrated RGB coverage.',rebuilds:this.rebuilds};
    if (!this.program) {
      const pending = {active: false, message: 'Preparing camera fusion…', rebuilds: this.rebuilds};
      if (!this.pendingProgram) {
        this.pendingProgram = prepareGpuProgram(gl, ATLAS_VERTEX, ATLAS_FRAGMENT);
        return pending;
      }
      this.program = this.pendingProgram.take();
      if (!this.program) return pending;
      this.pendingProgram = null;
      this.vao = gl.createVertexArray();
      this.u = Object.fromEntries(['u_compact', 'u_depth', 'u_color', 'u_size', 'u_intrinsics',
        'u_rgb_pass', 'u_center_rgb_cells', 'u_target_size', 'u_rgb_vis_size']
        .map(name => [name, gl.getUniformLocation(this.program, name)]));
    }
    const projections=this.cameras.map(c=>validatedColorProjection(c.metadata));
    const w=Math.max(...this.cameras.map(c=>c.width)),h=Math.max(...this.cameras.map(c=>c.height));
    this.visibilitySizes=projections.map(p=>{const scale=Math.min(1,1024/p.size[0]);return p.size.map(v=>Math.ceil(v*scale));});
    const vw=Math.max(...this.visibilitySizes.map(s=>s[0])),vh=Math.max(...this.visibilitySizes.map(s=>s[1]));
    const signature=`${w}:${h}:${vw}:${vh}:${this.cameras.length}`;
    if(this.signature!==signature){
      this.releaseTargets();
      this.depth=gpuTexture(gl,gl.TEXTURE_2D_ARRAY,10,gl.R16UI,w,h,this.cameras.length);
      this.color=gpuTexture(gl,gl.TEXTURE_2D_ARRAY,11,gl.RGBA8,w,h,this.cameras.length);
      this.visibility=gpuTexture(gl,gl.TEXTURE_2D_ARRAY,12,gl.R16UI,vw,vh,this.cameras.length);
      this.framebuffer=gl.createFramebuffer();this.depthBuffer=gl.createRenderbuffer();gl.bindRenderbuffer(gl.RENDERBUFFER,this.depthBuffer);
      gl.renderbufferStorage(gl.RENDERBUFFER,gl.DEPTH_COMPONENT24,vw,vh);
      this.signature=signature;
    }
    const u=this.u;gl.useProgram(this.program);gl.bindVertexArray(this.vao);gl.bindFramebuffer(gl.FRAMEBUFFER,this.framebuffer);
    gl.uniform1i(u.u_center_rgb_cells,centerRgbCells?1:0);
    gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);gl.disable(gl.SCISSOR_TEST);gl.depthMask(true);
    try{
      this.cameras.forEach((camera,index)=>{
        const entry=entries.get(camera),key=`${camera.uploadRevision}:${entry.count}:${centerRgbCells}:${projections[index].extrinsics}:${projections[index].intrinsics}:${projections[index].size}`;
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
    if(!u){u={};for(const n of ['u_square_depth','u_square_color','u_square_rgb_visibility','u_square_count','u_square_source','u_square_priority','u_square_world_to_camera[0]','u_square_intrinsics[0]','u_square_size[0]','u_square_rgb_rotation[0]','u_square_rgb_translation[0]','u_square_rgb_intrinsics[0]','u_square_rgb_size[0]','u_square_visibility_size[0]','u_square_capture_delta[0]'])u[n]=gl.getUniformLocation(program,n);this.bindings.set(program,u);}
    // Bind textures even when disabled: active sampler types must never alias.
    for(const [name,key,unit] of [['u_square_depth','depth',10],['u_square_color','color',11],['u_square_rgb_visibility','visibility',12]]){
      gl.uniform1i(u[name],unit);gl.activeTexture(gl.TEXTURE0+unit);gl.bindTexture(gl.TEXTURE_2D_ARRAY,this[key]||null);
    }
    const own=this.cameras.indexOf(source),active=this.depth&&this.cameras.length>=2&&own>=0;
    gl.uniform1i(u.u_square_count,active?this.cameras.length:0);gl.uniform1i(u.u_square_source,own);
    const roles=['262522074294','262522075787','262522076048'].map(id=>this.cameras.findIndex(c=>String(c.metadata.id).replace(/^realsense:/,'')===id));
    gl.uniform3iv(u.u_square_priority,roles);
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
