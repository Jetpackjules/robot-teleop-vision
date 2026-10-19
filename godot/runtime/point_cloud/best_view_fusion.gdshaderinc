// Per-surface camera confidence. Shared by unified mesh and BlendPCR.
// Camera count comes from the metadata texture; no pair-only special case.
uniform bool rig_best_view = false;
uniform bool rig_select_owner = true;
uniform bool rig_quality_ready = false;
uniform sampler2D rig_quality : filter_nearest,repeat_disable;
uniform vec4 rig_quality_grid=vec4(1.0);

vec4 rig_triangle_quality(int camera,vec2 cell) {
    int columns=int(rig_quality_grid.x);
    ivec2 offset=ivec2((camera%columns)*int(rig_quality_grid.z),(camera/columns)*int(rig_quality_grid.w));
    return texelFetch(rig_quality,offset+ivec2(cell),0);
}

bool rig_observe(int camera, vec3 point, out float error, out float quality, out float confidence) {
    quality = 0.0;
    confidence = 0.0;
    error = 0.0;
    vec4 info = rig_data(camera,6);
    if (info.w < 0.5 || point.z >= -0.001) return false;
    vec4 k = rig_data(camera,3);
    vec4 grid = rig_data(camera,4);
    vec4 limits = rig_data(camera,5);
    float z = -point.z;
    vec2 pixel = vec2(point.x*k.x/z+k.z, -point.y*k.y/z+k.w)/grid.z;
    if (any(lessThan(pixel,vec2(0.0))) || any(greaterThanEqual(pixel,grid.xy-vec2(1.0)))) return false;
    vec2 cell = floor(pixel);
    vec2 a = cell;
    vec2 b = cell+vec2(1.0,0.0);
    vec2 c = cell+vec2(0.0,1.0);
    if (fract(pixel.x)+fract(pixel.y)>1.0) a=cell+vec2(1.0);
    float za=rig_z(camera,a), zb=rig_z(camera,b), zc=rig_z(camera,c);
    if (!rig_in_range(za,limits) || !rig_in_range(zb,limits) || !rig_in_range(zc,limits)) return false;
    if (abs(za-zb)>limits.z*max(za,zb) || abs(zb-zc)>limits.z*max(zb,zc) || abs(zc-za)>limits.z*max(zc,za)) return false;
    vec4 cached=vec4(0.0);
    bool second=fract(pixel.x)+fract(pixel.y)>1.0;
    if (rig_quality_ready) {
        cached=rig_triangle_quality(camera,cell);
        if ((second?cached.g:cached.r)<0.5) return false;
    } else if (rig_neighbors(camera,a,za,max(0.002,limits.z*za*0.5),limits)<2.0 ||
        rig_neighbors(camera,b,zb,max(0.002,limits.z*zb*0.5),limits)<2.0 ||
        rig_neighbors(camera,c,zc,max(0.002,limits.z*zc*0.5),limits)<2.0) return false;
    vec3 pa=rig_project(a,za,k,grid.z), pb=rig_project(b,zb,k,grid.z), pc=rig_project(c,zc,k,grid.z);
    if (!rig_quality_ready && (distance(pa,pb)>limits.w || distance(pb,pc)>limits.w || distance(pc,pa)>limits.w)) return false;
    vec3 n=cross(pb-pa,pc-pa);
    float length_n=length(n);
    vec3 ray=point/z;
    float divisor=dot(n,ray);
    if (length_n<0.0000000001 || abs(divisor)<0.0000000001) return false;
    float observed_z=dot(n,pa)/divisor;
    if (!rig_in_range(observed_z,limits)) return false;
    // Intersect the actual triangle, instead of treating nearest-texel depth
    // as a flat surface. This retains accurate sloped surfaces.
    error=observed_z-z;
    float noise=max(0.003,0.006*observed_z*observed_z);
    if (rig_quality_ready) {
        confidence=second?cached.a:cached.b;
    } else {
        float support=min(rig_neighbors(camera,a,za,noise,limits),
            min(rig_neighbors(camera,b,zb,noise,limits),rig_neighbors(camera,c,zc,noise,limits)))/4.0;
        confidence=0.25+0.75*support*support;
    }
    vec4 cleanup=rig_data(camera,7);
    vec2 center=floor(pixel+vec2(0.5));
    float center_z=rig_z(camera,center);
    if (cleanup.x>0.5 && rig_neighbors(camera,center,center_z,cleanup.y,limits)<cleanup.z) return false;
    if (info.z>0.5) {
        vec2 uv=((pixel+vec2(0.5))/grid.xy)*info.xy/vec2(textureSize(rig_color,0).xy);
        if (texture(rig_color,vec3(uv,float(camera))).a<0.5) return false;
    }
    float facing=abs(dot(n/length_n,normalize(ray)));
    float range_sq=max(dot(ray,ray)*observed_z*observed_z,0.0001);
    quality=(k.x*k.y/(grid.z*grid.z*1000000.0))*facing*confidence/range_sq;
    return quality>0.0;
}

vec3 rig_local_point(int camera, vec4 world) {
    return vec3(dot(rig_data(camera,0),world),dot(rig_data(camera,1),world),dot(rig_data(camera,2),world));
}

bool rig_best_overlap_known(float own_quality,float own_confidence) {
    if (!rig_fusion_enabled) return false;
    if (rig_data(rig_camera_index,6).w<0.5) return true;
    vec4 world=vec4(rig_world_position,1.0);
    if (!rig_select_owner && own_confidence>=0.95) return false;
    // Quantized scores form a transitive ordering. Pairwise "within 2%" ties
    // can form A->B->C->A cycles and make every camera discard the surface.
    int own_rank=int(floor(log(max(own_quality,0.000000000001))*50.4983498));
    int count=textureSize(rig_cameras,0).y;
    for (int camera=0; camera<count; camera++) {
        if (camera==rig_camera_index) continue;
        vec3 point=rig_local_point(camera,world);
        float error, quality, confidence;
        if (!rig_observe(camera,point,error,quality,confidence)) continue;
        float noise=max(0.003,0.004*point.z*point.z);
        float agreement=min(fusion_depth_tolerance,noise);
        // Closer peer depth means an occlusion, not evidence against this point.
        // Only a coherent, better-resolved peer can contradict empty space,
        // and only for an edge-uncertain source triangle. Isolated good data
        // and thin parts are never eroded just for lacking a matching peer.
        if (own_confidence<0.95 && confidence>0.99 && quality>own_quality*1.15
                && error>max(0.008,noise*2.0)) return true;
        if (!rig_select_owner || abs(error)>agreement) continue;
        int rank=int(floor(log(max(quality,0.000000000001))*50.4983498));
        if (rank>own_rank || (rank==own_rank && camera<rig_camera_index)) return true;
    }
    return false;
}

bool rig_best_overlap() {
    if (!rig_fusion_enabled) return false;
    if (rig_data(rig_camera_index,6).w<0.5) return true;
    float error,quality,confidence;
    if (!rig_observe(rig_camera_index,rig_local_point(rig_camera_index,vec4(rig_world_position,1.0)),error,quality,confidence)) return false;
    return rig_best_overlap_known(quality,confidence);
}
