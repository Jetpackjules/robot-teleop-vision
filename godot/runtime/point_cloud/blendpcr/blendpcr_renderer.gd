@tool
extends Node3D
## Godot/RealSense adaptation of BlendPCR's separate-view, dominance, spatial
## weight smoothing and depth-compatible composition passes. No GPU readback.
## SPDX-License-Identifier: EUPL-1.2 (see LICENSE and README.md in this directory).

const MAX_CAMERAS := 4
const DominanceShader = preload("res://godot/runtime/point_cloud/blendpcr/dominance.gdshader")
const WeightsShader = preload("res://godot/runtime/point_cloud/blendpcr/weights.gdshader")
const CompositeShader = preload("res://godot/runtime/point_cloud/blendpcr/composite.gdshader")
const METADATA_DECLARATIONS := """
uniform float blendpcr_depth_span = 100.0;
uniform float blendpcr_sample_density = 1.0;
uniform sampler2D blendpcr_compat_lut : filter_nearest, repeat_disable;
vec3 blendpcr_compat_data(vec3 c) {
    // Packed metadata contains only byte values. Precompute the inverse
    // conversion once instead of solving it 14 times per rendered fragment.
    vec3 u = (floor(c * 255.0 + 0.5) + vec3(0.5)) / 256.0;
    return vec3(texture(blendpcr_compat_lut, vec2(u.r, 0.5)).r,
                texture(blendpcr_compat_lut, vec2(u.g, 0.5)).r,
                texture(blendpcr_compat_lut, vec2(u.b, 0.5)).r);
}
vec3 blendpcr_to_linear(vec3 c) {
    return mix(c / 12.92, pow((c + vec3(0.055)) / 1.055, vec3(2.4)), step(vec3(0.04045), c));
}
"""
const METADATA_OUTPUT := """
    float eye_z = -VERTEX.z;
    if (eye_z <= 0.0 || eye_z >= blendpcr_depth_span) discard;
    float encoded = floor(clamp(eye_z / blendpcr_depth_span, 0.0, 1.0) * 65535.0 + 0.5);
    vec3 normal = normalize(cross(dFdx(point_position_local), dFdy(point_position_local)));
    float range_m = max(length(point_position_local), 0.01);
    float facing = abs(dot(normal, point_position_local / range_m));
    float quality = clamp(blendpcr_sample_density * facing / (range_m * range_m), 1.0 / 255.0, 1.0);
    vec3 packed = vec3(floor(encoded / 256.0), mod(encoded, 256.0), floor(quality * 255.0 + 0.5)) / 255.0;
    ALBEDO = vec3(0.0);
    EMISSION = OUTPUT_IS_SRGB ? blendpcr_compat_data(packed) : blendpcr_to_linear(packed);
"""

var status := ""
var _entries: Dictionary = {}
var _dominance: SubViewport
var _weights: Array[SubViewport] = []
var _output: MeshInstance3D
var _composite: ShaderMaterial
var _dominant_material: ShaderMaterial
var _source_size := Vector2i.ZERO
var _dummy: Texture2D
var _last_viewer: Camera3D
var _compat_lut: Texture2D

static func compatibility_lut_image() -> Image:
    var image := Image.create(256, 1, false, Image.FORMAT_RF)
    for byte in 256:
        var wanted := pow((float(byte) / 255.0 + 0.055) / 1.055, 2.4)
        var low := 0.0
        var high := 1.0
        for iteration in 14:
            var mid := (low + high) * 0.5
            var converted := mid * (mid * (mid * 0.305306011 + 0.682171111) + 0.012522878)
            if converted <= wanted:
                low = mid
            else:
                high = mid
        image.set_pixel(byte, 0, Color((low + high) * 0.5, 0, 0, 1))
    return image

func clear() -> void:
    for entry in _entries.values():
        _restore(entry)
        entry.color_view.queue_free()
        entry.meta_view.queue_free()
    _entries.clear()
    if is_instance_valid(_dominance):
        _dominance.queue_free()
    _dominance = null
    for viewport in _weights:
        viewport.queue_free()
    _weights.clear()
    if is_instance_valid(_output):
        _output.queue_free()
    _output = null
    _composite = null
    _dominant_material = null
    _last_viewer = null
    _compat_lut = null
    _source_size = Vector2i.ZERO

func _exit_tree() -> void:
    clear()

static func _restore(entry: Dictionary) -> void:
    var source = entry.source
    if is_instance_valid(source):
        source.layers = entry.layers

static func metadata_shader(source: Shader) -> Shader:
    var code := source.code
    if not code.contains("point_position_local") or not code.contains("void fragment()"):
        return null
    code = code.replace("unshaded,", "")
    code = code.replace("void fragment()", METADATA_DECLARATIONS + "\nvoid fragment()")
    var end := code.rfind("}")
    if end < 0:
        return null
    var shader := Shader.new()
    shader.code = code.substr(0, end) + METADATA_OUTPUT + "\n}\n"
    return shader

func _viewport(size: Vector2i, three_d: bool) -> SubViewport:
    var viewport := SubViewport.new()
    viewport.size = size
    viewport.disable_3d = not three_d
    viewport.transparent_bg = true
    viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
    if three_d:
        viewport.own_world_3d = true
        viewport.msaa_3d = Viewport.MSAA_DISABLED
        viewport.screen_space_aa = Viewport.SCREEN_SPACE_AA_DISABLED
        var environment := WorldEnvironment.new()
        environment.environment = Environment.new()
        environment.environment.background_mode = Environment.BG_COLOR
        environment.environment.background_color = Color.BLACK
        environment.environment.tonemap_mode = Environment.TONE_MAPPER_LINEAR
        viewport.add_child(environment)
    add_child(viewport)
    return viewport

func _screen_pass(shader: Shader, size: Vector2i) -> SubViewport:
    var viewport := _viewport(size, false)
    var rect := ColorRect.new()
    rect.size = Vector2(size)
    rect.material = ShaderMaterial.new()
    (rect.material as ShaderMaterial).shader = shader
    viewport.add_child(rect)
    return viewport

static func _pass_material(viewport: SubViewport) -> ShaderMaterial:
    return viewport.get_child(0).material as ShaderMaterial

func _ensure_passes(viewer: Camera3D, size: Vector2i) -> void:
    var small := Vector2i(maxi(1, size.x / 4), maxi(1, size.y / 4))
    if _compat_lut == null:
        _compat_lut = ImageTexture.create_from_image(compatibility_lut_image())
    if _output == null:
        _dominance = _screen_pass(DominanceShader, small)
        _dominant_material = _pass_material(_dominance)
        for first_camera in [0]:
            var viewport := _screen_pass(WeightsShader, small)
            var material := _pass_material(viewport)
            material.set_shader_parameter("first_camera", first_camera)
            material.set_shader_parameter("dominance", _dominance.get_texture())
            _weights.append(viewport)
        _composite = ShaderMaterial.new()
        _composite.shader = CompositeShader
        _composite.set_shader_parameter("weights_a", _weights[0].get_texture())
        _output = MeshInstance3D.new()
        _output.name = "BlendPCRComposite"
        var quad := QuadMesh.new()
        quad.size = Vector2(2, 2)
        _output.mesh = quad
        _output.material_override = _composite
        _output.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
        _output.extra_cull_margin = 16384
        add_child(_output)
        var black := Image.create(1, 1, false, Image.FORMAT_RGBA8)
        black.fill(Color(0, 0, 0, 0))
        _dummy = ImageTexture.create_from_image(black)
    if _source_size != size:
        _dominance.size = small
        (_dominance.get_child(0) as ColorRect).size = Vector2(small)
        for viewport in _weights:
            viewport.size = small
            (viewport.get_child(0) as ColorRect).size = Vector2(small)
        _source_size = size
    _output.layers = viewer.cull_mask
    _output.global_position = viewer.global_position
    _last_viewer = viewer

func _entry(source: MeshInstance3D, size: Vector2i) -> Dictionary:
    var id := source.get_instance_id()
    if _entries.has(id):
        return _entries[id]
    var color_view := _viewport(size, true)
    var meta_view := _viewport(size, true)
    var entry := {"source": source, "layers": source.layers, "color_view": color_view, "meta_view": meta_view, "shader": null, "material": null, "uniform_names": [], "uniform_values": {}, "camera_settings": []}
    for kind in ["color", "meta"]:
        var viewport: SubViewport = entry[kind + "_view"]
        var camera := Camera3D.new()
        viewport.add_child(camera)
        camera.current = true
        var surface := MeshInstance3D.new()
        surface.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
        viewport.add_child(surface)
        entry[kind + "_camera"] = camera
        entry[kind + "_surface"] = surface
    _entries[id] = entry
    return entry

static func _sync_camera(camera: Camera3D, viewer: Camera3D) -> void:
    camera.projection = viewer.projection
    camera.fov = viewer.fov
    camera.size = viewer.size
    camera.keep_aspect = viewer.keep_aspect
    camera.near = viewer.near
    camera.far = viewer.far
    camera.frustum_offset = viewer.frustum_offset
    camera.h_offset = viewer.h_offset
    camera.v_offset = viewer.v_offset
    camera.global_transform = viewer.global_transform

func update_surfaces(sources: Array[MeshInstance3D], viewer: Camera3D, scale: float = 0.75, tolerance: float = 0.035, radius: int = 3) -> bool:
    status = ""
    if sources.size() > MAX_CAMERAS:
        status = "BlendPCR supports up to %d cameras; showing independent meshes." % MAX_CAMERAS
        clear()
        return false
    if sources.is_empty():
        clear()
        return true
    if viewer == null:
        status = "BlendPCR needs an active 3D camera; showing independent meshes."
        clear()
        return false
    var rect := viewer.get_viewport().get_visible_rect().size
    var size := Vector2i(maxi(16, int(rect.x * scale)), maxi(16, int(rect.y * scale)))
    # Remove disabled/disconnected sources before constructing this frame's rig.
    var ids: Array[int] = []
    for source in sources:
        ids.append(source.get_instance_id())
    for id in _entries.keys():
        if id not in ids:
            var entry: Dictionary = _entries[id]
            _restore(entry)
            entry.color_view.queue_free()
            entry.meta_view.queue_free()
            _entries.erase(id)
    _ensure_passes(viewer, size)
    var camera_settings := [viewer.projection, viewer.fov, viewer.size, viewer.keep_aspect,
        viewer.near, viewer.far, viewer.frustum_offset, viewer.h_offset, viewer.v_offset, viewer.global_transform]
    var active := 0
    for i in sources.size():
        var source := sources[i]
        var entry := _entry(source, size)
        var material := source.get_active_material(0) as ShaderMaterial if source.mesh != null else null
        if material == null:
            _restore(entry)
            continue
        if entry.shader != material.shader or entry.get("material") != material:
            var shader := metadata_shader(material.shader)
            if shader == null:
                status = "BlendPCR needs a depth-mesh shader; showing independent meshes."
                clear()
                return false
            entry.shader = material.shader
            entry.material = material
            entry.uniform_names = []
            entry.uniform_values = {}
            for uniform in material.shader.get_shader_uniform_list():
                if str(uniform.name) not in ["fusion_enabled", "rig_fusion_enabled"]:
                    entry.uniform_names.append(str(uniform.name))
            entry.color_surface.material_override = material.duplicate()
            var meta := ShaderMaterial.new()
            meta.shader = shader
            meta.set_shader_parameter("blendpcr_compat_lut", _compat_lut)
            entry.meta_surface.material_override = meta
            for target in [entry.color_surface.material_override, meta]:
                target.set_shader_parameter("fusion_enabled", false)
                target.set_shader_parameter("rig_fusion_enabled", false)
        var changed := {}
        for name in entry.uniform_names:
            var value = material.get_shader_parameter(name)
            # Native mat4 uniforms can initially report Projection defaults and
            # later receive Transform3D values. Comparing those directly throws.
            if value != null and (not entry.uniform_values.has(name)
                    or typeof(entry.uniform_values[name]) != typeof(value)
                    or entry.uniform_values[name] != value):
                changed[name] = value
                entry.uniform_values[name] = value
        for kind in ["color", "meta"]:
            var viewport: SubViewport = entry[kind + "_view"]
            if viewport.size != size:
                viewport.size = size
            if entry.get("camera_settings", []) != camera_settings:
                _sync_camera(entry[kind + "_camera"], viewer)
            var surface: MeshInstance3D = entry[kind + "_surface"]
            if surface.mesh != source.mesh:
                surface.mesh = source.mesh
            if surface.global_transform != source.global_transform:
                surface.global_transform = source.global_transform
            if surface.custom_aabb != source.custom_aabb:
                surface.custom_aabb = source.custom_aabb
            var target := surface.material_override as ShaderMaterial
            for name in changed:
                target.set_shader_parameter(name, changed[name])
            if kind == "meta":
                target.set_shader_parameter("blendpcr_depth_span", viewer.far)
                var k: Vector4 = source.call("get_current_intrinsics")
                var stride := maxf(1, float(source.call("get_current_grid_stride")))
                target.set_shader_parameter("blendpcr_sample_density", k.x * k.y / (stride * stride * 1000000.0))
        entry.camera_settings = camera_settings
        var fresh := not source.has_method("get_display_frame_age_ms") or float(source.call("get_display_frame_age_ms")) < 1000.0
        if source.is_visible_in_tree() and fresh and (int(entry.layers) & viewer.cull_mask) != 0:
            active |= 1 << i
        # Capture keeps running; only drawing in the original world is disabled.
        source.layers = 0
        for target in [_dominant_material, _composite]:
            target.set_shader_parameter("camera_color_%d" % i, entry.color_view.get_texture())
            target.set_shader_parameter("camera_meta_%d" % i, entry.meta_view.get_texture())
    for i in range(sources.size(), MAX_CAMERAS):
        for target in [_dominant_material, _composite]:
            target.set_shader_parameter("camera_color_%d" % i, _dummy)
            target.set_shader_parameter("camera_meta_%d" % i, _dummy)
    for target in [_dominant_material, _composite]:
        target.set_shader_parameter("camera_count", sources.size())
        target.set_shader_parameter("active_mask", active)
        target.set_shader_parameter("depth_span", viewer.far)
        target.set_shader_parameter("surface_tolerance", tolerance)
    for viewport in _weights:
        _pass_material(viewport).set_shader_parameter("seam_radius", radius)
    return true
