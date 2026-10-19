@tool
extends RefCounted
## Add camera-aware ownership to existing native shaders without requiring a
## new native ABI. This also lets installed older extensions use the fix.
const INCLUDE := "#include \"res://godot/runtime/point_cloud/best_view_fusion.gdshaderinc\"\n"

static func adapt(material: ShaderMaterial) -> void:
    if material == null or material.shader == null:
        return
    var code := material.shader.code
    if code.contains("best_view_fusion.gdshaderinc") or not code.contains("if (rig_overlap()) discard;"):
        return
    code = code.replace("void vertex()",INCLUDE + "\nvoid vertex()")
    code = code.replace("if (rig_overlap()) discard;", "if (rig_best_view ? rig_best_overlap() : rig_overlap()) discard;")
    var shader := Shader.new()
    shader.code = code
    material.shader = shader
