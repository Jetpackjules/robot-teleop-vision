// Native RGB reconstruction from the winning depth triangle. The dense mesh
// is rasterized once for metadata; RGB comes from its original texture.
// SPDX-License-Identifier: EUPL-1.2
uniform sampler2D source_depth_0 : filter_nearest,repeat_disable;
uniform sampler2D source_depth_1 : filter_nearest,repeat_disable;
uniform sampler2D source_depth_2 : filter_nearest,repeat_disable;
uniform sampler2D source_depth_3 : filter_nearest,repeat_disable;
uniform sampler2D source_quality : filter_nearest,repeat_disable;
uniform mat4 source_projection_inverse;
uniform bool source_orthographic=false;
uniform vec2 source_output_size=vec2(1.0);
uniform mat4 source_view_to_local_0;
uniform mat4 source_view_to_local_1;
uniform mat4 source_view_to_local_2;
uniform mat4 source_view_to_local_3;
uniform mat4 source_local_to_view_0;
uniform mat4 source_local_to_view_1;
uniform mat4 source_local_to_view_2;
uniform mat4 source_local_to_view_3;
uniform vec4 source_k[4];
uniform vec4 source_grid[4];
uniform vec4 source_limits[4];
uniform vec4 source_gain[4];
uniform vec4 source_bias[4];
uniform vec4 source_flags[4]; // color, correction, edge feather, reserved
uniform vec4 source_edges[4]; // feather width, min shade, reserved, reserved
uniform vec4 source_quality_ids=vec4(0.0);
uniform vec4 source_quality_grid=vec4(1.0);
uniform int source_quality_mask=0;

mat4 view_to_local(int i) {
    if(i==0)return source_view_to_local_0;
    if(i==1)return source_view_to_local_1;
    if(i==2)return source_view_to_local_2;
    return source_view_to_local_3;
}
mat4 local_to_view(int i) {
    if(i==0)return source_local_to_view_0;
    if(i==1)return source_local_to_view_1;
    if(i==2)return source_local_to_view_2;
    return source_local_to_view_3;
}
float source_z(int i,vec2 pixel) {
    vec2 uv=(pixel+vec2(0.5))/source_grid[i].xy;
    if(i==0)return texture(source_depth_0,uv).r;
    if(i==1)return texture(source_depth_1,uv).r;
    if(i==2)return texture(source_depth_2,uv).r;
    return texture(source_depth_3,uv).r;
}
vec3 source_point(int i,vec2 pixel,float z) {
    vec4 k=source_k[i];
    pixel*=source_grid[i].z;
    return vec3((pixel.x-k.z)*z/k.x,-(pixel.y-k.w)*z/k.y,-z);
}
float source_support(int i,vec2 pixel,float z,float delta) {
    vec4 neighbors=vec4(source_z(i,pixel+vec2(-1,0)),source_z(i,pixel+vec2(1,0)),
        source_z(i,pixel+vec2(0,-1)),source_z(i,pixel+vec2(0,1)));
    float count=0.0;
    for(int n=0;n<4;n++) if(neighbors[n]>=source_limits[i].x && neighbors[n]<=source_limits[i].y && abs(neighbors[n]-z)<=delta)count+=1.0;
    return count;
}
bool source_triangle(int i,vec2 cell,bool second,vec3 origin,vec3 ray,
        out vec2 rgb_uv,out float score,out float view_z,out float edge) {
    vec2 a=cell,b=cell+vec2(1,0),c=cell+vec2(0,1);
    if(second)a=cell+vec2(1);
    float za=source_z(i,a),zb=source_z(i,b),zc=source_z(i,c);
    if(min(za,min(zb,zc))<source_limits[i].x || max(za,max(zb,zc))>source_limits[i].y)return false;
    vec4 cached=vec4(0.0);
    if((source_quality_mask&(1<<i))!=0) {
        int camera=int(source_quality_ids[i]),columns=int(source_quality_grid.x);
        ivec2 offset=ivec2((camera%columns)*int(source_quality_grid.z),(camera/columns)*int(source_quality_grid.w));
        cached=texelFetch(source_quality,offset+ivec2(cell),0);
    }
    vec3 pa=source_point(i,a,za),pb=source_point(i,b,zb),pc=source_point(i,c,zc);
    // This exact primitive passed the native vertex/fragment validity checks
    // when its ID was rasterized. The confidence map never controls geometry.
    vec3 v0=pb-pa,v1=pc-pa,n=cross(v0,v1);
    float denominator=dot(n,ray);
    if(abs(denominator)<0.000000000001)return false;
    vec3 p=origin+ray*(dot(n,pa-origin)/denominator);
    vec3 v2=p-pa;
    float d00=dot(v0,v0),d01=dot(v0,v1),d11=dot(v1,v1);
    float det=d00*d11-d01*d01;
    if(det<0.00000000000000000001)return false;
    float v=(d11*dot(v2,v0)-d01*dot(v2,v1))/det;
    float w=(d00*dot(v2,v1)-d01*dot(v2,v0))/det;
    // Rasterization already proved coverage. Repeating a barycentric edge
    // comparison can lose boundary pixels to floating-point rounding.
    rgb_uv=((1.0-v-w)*a+v*b+w*c+vec2(0.5))/source_grid[i].xy;
    view_z=-(local_to_view(i)*vec4(p,1.0)).z;
    float confidence=1.0;
    if((source_quality_mask&(1<<i))!=0) {
        confidence=second?cached.a:cached.b;
    } else if(camera_count>1) {
        float noise=max(0.003,0.006*max(za,max(zb,zc))*max(za,max(zb,zc)));
        float support=min(source_support(i,a,za,noise),min(source_support(i,b,zb,noise),source_support(i,c,zc,noise)))/4.0;
        confidence=0.25+0.75*support*support;
    }
    float range_sq=max(dot(p,p),0.0001);
    float facing=abs(dot(normalize(n),p/sqrt(range_sq)));
    score=source_k[i].x*source_k[i].y/(source_grid[i].z*source_grid[i].z*1000000.0)*confidence*facing/range_sq;
    score=floor(clamp(score/(1.0+score),1.0/255.0,254.0/255.0)*255.0+0.5)/255.0;
    edge=1.0;
    if(source_flags[i].z>0.5) {
        float jump=max(abs(za-zb),max(abs(zb-zc),abs(zc-za)));
        float limit=source_limits[i].z*max(za,max(zb,zc));
        float feather=1.0-smoothstep(max(limit-source_edges[i].x,0.0),limit,jump);
        edge=mix(source_edges[i].y,1.0,feather);
    }
    return view_z>0.0;
}
bool native_sample(int i,vec2 uv,float tag,out vec2 rgb_uv,out float quality,out float refined_z,out float edge) {
    // Dominance is lower resolution than metadata. Its nearest metadata fetch
    // identifies the ray through that texel's center, not the unsnapped UV.
    uv=(floor(uv*source_output_size)+vec2(0.5))/source_output_size;
    vec4 h=source_projection_inverse*vec4(uv.x*2.0-1.0,1.0-uv.y*2.0,1.0,1.0);
    vec3 near_point=h.xyz/h.w;
    mat4 to_local=view_to_local(i);
    vec3 origin=(to_local*vec4(source_orthographic?vec3(near_point.xy,0.0):vec3(0.0),1.0)).xyz;
    vec3 ray=mat3(to_local)*(source_orthographic?vec3(0,0,-1):near_point);
    int primitive=int(round(tag))-1;
    int cell_index=primitive/2;
    int width=int(source_grid[i].x);
    vec2 cell=vec2(float(cell_index%width),float(cell_index/width));
    if(primitive<0 || any(greaterThanEqual(cell,source_grid[i].xy-vec2(1))))return false;
    return source_triangle(i,cell,(primitive%2)==1,origin,ray,rgb_uv,quality,refined_z,edge);
}
vec3 native_linear_to_srgb(vec3 c) {
    return mix(c*12.92,1.055*pow(max(c,vec3(0)),vec3(1.0/2.4))-0.055,step(vec3(0.0031308),c));
}
vec3 native_srgb_to_linear(vec3 c) {
    return mix(c/12.92,pow(max((c+0.055)/1.055,vec3(0)),vec3(2.4)),step(vec3(0.04045),c));
}
vec4 native_color(int i,vec2 uv,float edge) {
    vec4 c=sample_color(i,uv);
    if(source_flags[i].y>0.5)c.rgb=native_srgb_to_linear(clamp(native_linear_to_srgb(c.rgb)*source_gain[i].xyz+source_bias[i].xyz,vec3(0),vec3(1)));
    if(source_flags[i].x<0.5)c=vec4(vec3(dot(c.rgb,vec3(0.299,0.587,0.114))),1.0);
    c.rgb*=edge;
    return c;
}
