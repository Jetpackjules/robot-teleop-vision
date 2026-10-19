@tool
extends RefCounted
## Evaluate each depth-grid triangle once per capture, rather than once per
## vertex and again per fragment. Works on Compatibility; no GPU readback.

const DECLARATIONS := """
uniform sampler2D mesh_triangle_map : filter_nearest, repeat_disable;
varying float mesh_edge_confidence;
"""
const VERTEX := """
void vertex() {
    point_uv = UV;
    tri_uv_a = UV2;
    tri_uv_b = CUSTOM0.xy;
    tri_uv_c = CUSTOM0.zw;
    vec2 cell = min(tri_uv_a, min(tri_uv_b, tri_uv_c));
    vec2 centroid = ((tri_uv_a + tri_uv_b + tri_uv_c) / 3.0 - cell) / texel_size;
    bool second = centroid.x + centroid.y > 1.0;
    vec4 cached = texture(mesh_triangle_map, cell);
    point_valid = second ? cached.g : cached.r;
    mesh_edge_confidence = second ? cached.a : cached.b;
    if (point_valid < 0.5) {
        // Collapse rejected triangles before rasterization, as the original
        // native vertex shader does. Discard alone can cause huge overdraw.
        VERTEX = project_uv((tri_uv_a+tri_uv_b+tri_uv_c)/3.0,depth_range.y);
    } else {
        VERTEX = project_uv(UV,texture(depth_tex,UV).r);
    }
    point_position_local = VERTEX;
}
"""
const FAST_OVERLAP := """
bool mesh_fast_rig_overlap() {
    if (!rig_fusion_enabled) return false;
    if (!rig_best_view) return rig_overlap();
    vec3 normal=normalize(cross(dFdx(point_position_local),dFdy(point_position_local)));
    float range_sq=max(dot(point_position_local,point_position_local),0.0001);
    float facing=abs(dot(normal,point_position_local/sqrt(range_sq)));
    float score=(intrinsics.x*intrinsics.y/(grid_stride*grid_stride*1000000.0))*facing*mesh_edge_confidence/range_sq;
    return rig_best_overlap_known(score,mesh_edge_confidence);
}
"""
const BAKE := """
float confidence(vec2 a, float za, vec2 b, float zb, vec2 c, float zc) {
    // Confidence affects ownership only. A discontinuity is never filled or
    // removed merely because a surface is thin or has no second camera.
    float delta = max(0.003, 0.006 * max(za, max(zb,zc)) * max(za, max(zb,zc)));
    float support = min(close_neighbor_count_delta(a,za,delta),
        min(close_neighbor_count_delta(b,zb,delta),close_neighbor_count_delta(c,zc,delta))) / 4.0;
    return 0.25 + 0.75 * support * support;
}
bool triangle_ok(vec2 a, float za, vec2 b, float zb, vec2 c, float zc) {
    return triangle_depth_ok(za,zb,zc)
        && triangle_edge_ok(project_uv(a,za),project_uv(b,zb),project_uv(c,zc))
        && triangle_support_ok(a,za,b,zb,c,zc);
}
void fragment() {
    vec2 size = 1.0 / texel_size;
    vec2 pixel = floor(UV * size);
    if (any(greaterThanEqual(pixel, size - vec2(1.0)))) {
        COLOR=vec4(0.0);
    } else {
        vec2 a = (pixel + vec2(0.5)) * texel_size;
        vec2 b = a + vec2(texel_size.x,0.0);
        vec2 c = a + vec2(0.0,texel_size.y);
        vec2 d = a + texel_size;
        float za = texture(depth_tex,a).r;
        float zb = texture(depth_tex,b).r;
        float zc = texture(depth_tex,c).r;
        float zd = texture(depth_tex,d).r;
        bool first = triangle_ok(a,za,c,zc,b,zb);
        bool second = triangle_ok(b,zb,c,zc,d,zd);
        COLOR=vec4(first?1.0:0.0, second?1.0:0.0,
            first?confidence(a,za,c,zc,b,zb):0.0, second?confidence(b,zb,c,zc,d,zd):0.0);
    }
}
"""

var viewport: SubViewport
var material: ShaderMaterial
var _signature: Array = []

static func function_span(code: String, name: String) -> Vector2i:
    var start := code.find("void " + name + "()")
    if start < 0:
        return Vector2i(-1,-1)
    var brace := code.find("{", start)
    var depth := 1
    var cursor := brace + 1
    while cursor < code.length() and depth > 0:
        if code[cursor] == "{": depth += 1
        if code[cursor] == "}": depth -= 1
        cursor += 1
    return Vector2i(start,cursor) if depth == 0 else Vector2i(-1,-1)

static func supports(shader: Shader) -> bool:
    return shader != null and shader.code.contains("bool triangle_support_ok(") and shader.code.contains("tri_uv_a = UV2;")

static func adapt(code: String) -> String:
    if code.contains("uniform sampler2D mesh_triangle_map"):
        return code
    var span := function_span(code,"vertex")
    if span.x < 0:
        return code
    # Retain native rig-world coordinates when this adapter is used by unified
    # mesh too. The original material remains available for other mesh modes.
    var vertex := VERTEX
    if code.contains("varying vec3 rig_world_position;"):
        vertex = vertex.replace("point_position_local = VERTEX;", "point_position_local = VERTEX;\n    rig_world_position = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;")
    var helpers:=DECLARATIONS
    if code.contains("best_view_fusion.gdshaderinc"):
        helpers+=FAST_OVERLAP
        code=code.replace("if (rig_best_view ? rig_best_overlap() : rig_overlap()) discard;","if (mesh_fast_rig_overlap()) discard;")
        span=function_span(code,"vertex")
    code = code.substr(0,span.x) + helpers + vertex + code.substr(span.y)
    # Those checks are invariant across a triangle and already encoded in the
    # map. Per-fragment cleanup and robot masking deliberately remain intact.
    var start := code.find("    float da = texture(depth_tex, tri_uv_a).r;",code.find("void fragment()"))
    var cleanup := code.find("    if (point_cleanup_enabled)",start)
    if start >= 0 and cleanup > start:
        code = code.substr(0,start) + code.substr(cleanup)
    start = code.find("    vec3 pa = project_uv(tri_uv_a, da);",code.find("void fragment()"))
    var shade := code.find("    float edge_shade = 1.0;",start)
    if start >= 0 and shade > start:
        code = code.substr(0,start) + code.substr(shade)
    code = code.replace("    if (edge_feather_enabled) {", "    if (edge_feather_enabled) {\n        float da = texture(depth_tex, tri_uv_a).r;\n        float db = texture(depth_tex, tri_uv_b).r;\n        float dc = texture(depth_tex, tri_uv_c).r;")
    return code

static func adapt_quality(code: String) -> String:
    # Private BlendPCR worlds retain native triangle validation. Read shared
    # confidence without making their geometry depend on a viewport mask.
    if code.contains("uniform sampler2D mesh_triangle_map"):
        return code
    var span:=function_span(code,"vertex")
    if span.x<0: return code
    var vertex:=code.substr(span.x,span.y-span.x)
    var tail:=vertex.rfind("}")
    var confidence_code:="""
    vec2 cell=min(tri_uv_a,min(tri_uv_b,tri_uv_c));
    vec2 centroid=((tri_uv_a+tri_uv_b+tri_uv_c)/3.0-cell)/texel_size;
    vec4 cached=texture(mesh_triangle_map,cell);
    mesh_edge_confidence=centroid.x+centroid.y>1.0?cached.a:cached.b;
"""
    vertex=vertex.substr(0,tail)+confidence_code+"}\n"
    return code.substr(0,span.x)+DECLARATIONS+vertex+code.substr(span.y)

func create(parent: Node, shader: Shader) -> void:
    var code := shader.code
    var uniforms := code.substr(code.find("uniform sampler2D depth_tex"),code.find("varying vec2 point_uv;")-code.find("uniform sampler2D depth_tex"))
    var functions := code.substr(code.find("vec3 project_uv("),code.find("bool inside_robot_capsule(")-code.find("vec3 project_uv("))
    var bake := Shader.new()
    bake.code = "shader_type canvas_item;\nrender_mode blend_disabled;\n" + uniforms + functions + BAKE
    material = ShaderMaterial.new()
    material.shader = bake
    viewport = SubViewport.new()
    viewport.disable_3d = true
    viewport.transparent_bg = true
    viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
    var rect := ColorRect.new()
    rect.material = material
    viewport.add_child(rect)
    parent.add_child(viewport)

func update(source: MeshInstance3D, original: ShaderMaterial) -> void:
    var depth := original.get_shader_parameter("depth_tex") as Texture2D
    if depth == null:
        return
    var signature := [depth.get_instance_id(),depth.get_size()]
    # Native textures are updated in place: object identity alone is unsafe.
    var sequence := int(source.call("get_frame_sequence")) if source.has_method("get_frame_sequence") else Engine.get_process_frames()
    signature.append(sequence)
    for name in ["depth_tex","intrinsics","texel_size","grid_stride","depth_range","max_depth_delta","max_mesh_edge"]:
        var value = original.get_shader_parameter(name)
        signature.append(value)
        if value != null:
            material.set_shader_parameter(name,value)
    if signature == _signature:
        return
    _signature = signature
    viewport.size = Vector2i(depth.get_size())
    (viewport.get_child(0) as ColorRect).size = Vector2(viewport.size)
    viewport.render_target_update_mode = SubViewport.UPDATE_ONCE

func clear() -> void:
    if is_instance_valid(viewport): viewport.queue_free()
    viewport = null
    material = null
    _signature.clear()
