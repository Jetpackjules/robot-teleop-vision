@tool
extends RefCounted
## Presentation-only world box. Never changes capture, calibration or robot limits.
const DECLARATIONS := """
uniform bool workspace_crop_enabled = false;
uniform vec3 workspace_crop_min = vec3(-0.6, -0.45, -0.6);
uniform vec3 workspace_crop_max = vec3(0.6, 0.45, 0.6);
uniform mat3 workspace_crop_inverse = mat3(1.0);
varying vec3 workspace_crop_world;
"""
const REJECT := """
    vec3 center = (workspace_crop_min + workspace_crop_max) * 0.5;
    vec3 local = center + workspace_crop_inverse * (workspace_crop_world - center);
    if (workspace_crop_enabled && (any(lessThan(local, workspace_crop_min))
        || any(greaterThan(local, workspace_crop_max)))) discard;
"""

static func apply_to_material(material: ShaderMaterial, enabled: bool, minimum: Vector3, maximum: Vector3, rotation_degrees: Vector3 = Vector3.ZERO) -> void:
    if material == null or material.shader == null:
        return
    var code := material.shader.code
    if not code.contains("varying vec3 workspace_crop_world;"):
        if not enabled or not code.contains("point_position_local = VERTEX;"):
            return
        code = code.replace("void vertex()", DECLARATIONS + "\nvoid vertex()")
        code = code.replace("point_position_local = VERTEX;", "point_position_local = VERTEX;\n    workspace_crop_world = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;")
        code = code.replace("void fragment() {", "void fragment() {\n" + REJECT)
        var shader := Shader.new()
        shader.code = code
        material.shader = shader
    var signature := [material.shader.get_instance_id(), enabled, minimum, maximum, rotation_degrees]
    if material.get_meta("workspace_crop_signature", []) == signature:
        return
    material.set_shader_parameter("workspace_crop_enabled", enabled)
    material.set_shader_parameter("workspace_crop_min", minimum)
    material.set_shader_parameter("workspace_crop_max", maximum)
    material.set_shader_parameter("workspace_crop_inverse", rotation_basis(rotation_degrees).transposed())
    material.set_meta("workspace_crop_signature", signature)

static func rotation_basis(degrees: Vector3) -> Basis:
    return Basis(Vector3.BACK, deg_to_rad(degrees.z)) * Basis(Vector3.UP, deg_to_rad(degrees.y)) * Basis(Vector3.RIGHT, deg_to_rad(degrees.x))

static func box_mesh(minimum: Vector3, maximum: Vector3, rotation_degrees: Vector3 = Vector3.ZERO) -> ImmediateMesh:
    var mesh := ImmediateMesh.new()
    mesh.surface_begin(Mesh.PRIMITIVE_LINES)
    for corner in range(8):
        var point := Vector3(maximum.x if corner & 1 else minimum.x,
            maximum.y if corner & 2 else minimum.y, maximum.z if corner & 4 else minimum.z)
        for axis in range(3):
            var other: int = corner ^ (1 << axis)
            if other < corner:
                continue
            var end := point
            end[axis] = maximum[axis] if other & (1 << axis) else minimum[axis]
            var center := (minimum + maximum) * 0.5
            var basis := rotation_basis(rotation_degrees)
            mesh.surface_add_vertex(center + basis * (point - center))
            mesh.surface_add_vertex(center + basis * (end - center))
    mesh.surface_end()
    return mesh
