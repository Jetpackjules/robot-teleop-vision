@tool
extends Node3D
## Godot/RealSense adaptation of BlendPCR's separate-view, dominance, spatial
## weight smoothing and depth-compatible composition passes. No GPU readback.
## SPDX-License-Identifier: EUPL-1.2 (see LICENSE and README.md in this directory).

const MAX_CAMERAS := 4
const MeshCache = preload("res://godot/runtime/point_cloud/depth_mesh_cache.gd")
const DominanceShader = preload("res://godot/runtime/point_cloud/blendpcr/dominance.gdshader")
const WeightsShader = preload("res://godot/runtime/point_cloud/blendpcr/weights.gdshader")
const VerticalWeightsShader = preload("res://godot/runtime/point_cloud/blendpcr/weights_vertical.gdshader")
const CompositeShader = preload("res://godot/runtime/point_cloud/blendpcr/composite.gdshader")
const METADATA_DECLARATIONS := """
uniform float blendpcr_depth_span = 100.0;
uniform float blendpcr_sample_density = 1.0;
uniform bool blendpcr_direct_color = false;
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
    float score = blendpcr_sample_density * facing / (range_m * range_m);
    // Monotonic encoding retains differences between close/high-density views
    // instead of saturating every score above one to the same camera quality.
    float quality = clamp(score / (1.0 + score), 1.0 / 255.0, 254.0 / 255.0);
    vec3 packed = vec3(floor(encoded / 256.0), mod(encoded, 256.0), floor(quality * 255.0 + 0.5)) / 255.0;
    __NATIVE_ID_OUTPUT__
    ALBEDO = vec3(0.0);
    EMISSION = OUTPUT_IS_SRGB ? blendpcr_compat_data(packed) : blendpcr_to_linear(packed);
"""
const NATIVE_ID_OUTPUT:="""
    if(blendpcr_direct_color) {
        // A triangle ID is exact. Reprojecting quantized depth back to a sensor
        // pixel required a costly, ambiguous neighborhood search at edges.
        vec2 grid=round(1.0/texel_size);
        vec2 cell=floor(min(tri_uv_a,min(tri_uv_b,tri_uv_c))*grid);
        vec2 centroid=((tri_uv_a+tri_uv_b+tri_uv_c)/3.0-(cell+vec2(0.5))/grid)*grid;
        encoded=(cell.y*grid.x+cell.x)*2.0+(centroid.x+centroid.y>1.0?1.0:0.0)+1.0;
        packed=vec3(floor(encoded/65536.0),floor(mod(encoded,65536.0)/256.0),mod(encoded,256.0))/255.0;
    }
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
var _depth_span:=100.0

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
        if entry.get("cache") != null and not entry.get("cache_external",false): entry.cache.clear()
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
    var cached := MeshCache.supports(source)
    # The metadata pass owns free-space validation. The color pass can retain
    # the whole valid source mesh because rejected metadata is never composed.
    code=code.replace("if (rig_best_view ? rig_best_overlap() : rig_overlap()) discard;","")
    code=code.replace("if (mesh_fast_rig_overlap()) discard;","")
    if cached: code = MeshCache.adapt_quality(code)
    code = code.replace("unshaded,", "")
    code = code.replace("void fragment()", METADATA_DECLARATIONS + "\nvoid fragment()")
    var end := code.rfind("}")
    if end < 0:
        return null
    var shader := Shader.new()
    var output := METADATA_OUTPUT.replace("__NATIVE_ID_OUTPUT__",NATIVE_ID_OUTPUT if cached else "")
    if cached:
        output = output.replace("blendpcr_sample_density * facing", "blendpcr_sample_density * mesh_edge_confidence * facing")
        if source.code.contains("best_view_fusion.gdshaderinc"):
            output=output.replace("    // Monotonic encoding", "    if (rig_best_view && rig_best_overlap_known(score,mesh_edge_confidence)) discard;\n    // Monotonic encoding")
    shader.code = code.substr(0, end) + output + "\n}\n"
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
        var horizontal := _screen_pass(WeightsShader, small)
        _pass_material(horizontal).set_shader_parameter("dominance", _dominance.get_texture())
        _weights.append(horizontal)
        var vertical := _screen_pass(VerticalWeightsShader, small)
        _pass_material(vertical).set_shader_parameter("horizontal_counts", horizontal.get_texture())
        _weights.append(vertical)
        _composite = ShaderMaterial.new()
        _composite.shader = CompositeShader
        _composite.set_shader_parameter("weights_a", _weights[1].get_texture())
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

func update_surfaces(sources: Array[MeshInstance3D], viewer: Camera3D, scale: float = 0.75, tolerance: float = 0.035, radius: int = 3, quality_rig: Node = null) -> bool:
    status = ""
    radius = clampi(radius,0,10)
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
    _depth_span=_effective_depth_span(sources,viewer)
    var size := Vector2i(maxi(16, int(rect.x * scale)), maxi(16, int(rect.y * scale)))
    # Remove disabled/disconnected sources before constructing this frame's rig.
    var ids: Array[int] = []
    for source in sources:
        ids.append(source.get_instance_id())
    for id in _entries.keys():
        if id not in ids:
            var entry: Dictionary = _entries[id]
            _restore(entry)
            if entry.get("cache") != null and not entry.get("cache_external",false): entry.cache.clear()
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
            if entry.get("cache") != null and not entry.get("cache_external",false): entry.cache.clear()
            entry.cache = null
            entry.cache_external=false
            if MeshCache.supports(material.shader):
                entry.cache = quality_rig.get_cache(source) if quality_rig!=null else null
                entry.cache_external=entry.cache!=null
                if entry.cache==null:
                    entry.cache=MeshCache.new()
                    entry.cache.create(self,material.shader)
            var meta := ShaderMaterial.new()
            meta.shader = shader
            meta.set_shader_parameter("blendpcr_compat_lut", _compat_lut)
            entry.meta_surface.material_override = meta
            for target in [entry.color_surface.material_override, meta]:
                target.set_shader_parameter("fusion_enabled", false)
                target.set_shader_parameter("rig_fusion_enabled", false)
        if entry.get("cache") != null:
            if not entry.get("cache_external",false): entry.cache.update(source,material)
            for target in [entry.color_surface.material_override,entry.meta_surface.material_override]:
                target.set_shader_parameter("mesh_triangle_map",entry.cache.viewport.get_texture())
        var direct: bool = entry.get("cache")!=null and material.get_shader_parameter("color_source_is_linear")==true and material.get_shader_parameter("depth_tex") is Texture2D and material.get_shader_parameter("color_tex") is Texture2D
        if direct:
            var native_grid: Texture2D=material.get_shader_parameter("depth_tex")
            direct=native_grid.get_width()*native_grid.get_height()*2<=16777214
        var direct_changed: bool = entry.get("direct",false)!=direct
        entry.direct=direct
        entry.color_view.render_target_update_mode=SubViewport.UPDATE_DISABLED if direct else SubViewport.UPDATE_ALWAYS
        if direct_changed: entry.uniform_values.clear()
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
            if kind=="color" and direct: continue
            var viewport: SubViewport = entry[kind + "_view"]
            if viewport.size != size:
                viewport.size = size
            if direct_changed or entry.get("camera_settings", []) != camera_settings:
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
            # BlendPCR smooths ownership itself. The shared camera predicate
            # only rejects independently contradicted edge artifacts here.
            if kind=="meta" and material.get_shader_parameter("rig_best_view") == true:
                target.set_shader_parameter("rig_fusion_enabled",true)
                target.set_shader_parameter("rig_select_owner",false)
            elif kind=="color":
                target.set_shader_parameter("rig_fusion_enabled",false)
            if kind == "meta":
                target.set_shader_parameter("blendpcr_direct_color",direct)
                target.set_shader_parameter("blendpcr_depth_span", _depth_span)
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
            target.set_shader_parameter("camera_color_%d" % i, material.get_shader_parameter("color_tex") if direct else entry.color_view.get_texture())
            target.set_shader_parameter("camera_meta_%d" % i, entry.meta_view.get_texture())
    for i in range(sources.size(), MAX_CAMERAS):
        for target in [_dominant_material, _composite]:
            target.set_shader_parameter("camera_color_%d" % i, _dummy)
            target.set_shader_parameter("camera_meta_%d" % i, _dummy)
    _configure_native_inputs(sources,viewer)
    for target in [_dominant_material, _composite]:
        target.set_shader_parameter("camera_count", sources.size())
        target.set_shader_parameter("active_mask", active)
        target.set_shader_parameter("depth_span", _depth_span)
        target.set_shader_parameter("surface_tolerance", tolerance)
    for viewport in _weights:
        _pass_material(viewport).set_shader_parameter("seam_radius", radius)
    return true

static func _effective_depth_span(sources: Array[MeshInstance3D],viewer: Camera3D) -> float:
    var bound:=0.0
    var eye:=viewer.get_camera_transform().origin
    for source in sources:
        var mat:=source.get_active_material(0) as ShaderMaterial if source.mesh!=null else null
        if mat==null or not MeshCache.supports(mat.shader): return viewer.far
        var depth:=mat.get_shader_parameter("depth_tex") as Texture2D
        var k: Vector4=mat.get_shader_parameter("intrinsics")
        var limits: Vector2=mat.get_shader_parameter("depth_range")
        var stride:=maxf(1,float(mat.get_shader_parameter("grid_stride")))
        if depth==null or k.x<=0 or k.y<=0 or limits.y<=0: return viewer.far
        var x:=maxf(absf(k.z/k.x),absf(((depth.get_width()-1)*stride-k.z)/k.x))
        var y:=maxf(absf(k.w/k.y),absf(((depth.get_height()-1)*stride-k.w)/k.y))
        var basis:=source.global_basis
        # Frobenius norm safely bounds rotation, nonuniform scale and shear.
        var scale_bound:=sqrt(basis.x.length_squared()+basis.y.length_squared()+basis.z.length_squared())
        var range_bound:=limits.y*sqrt(1+x*x+y*y)*scale_bound
        bound=maxf(bound,eye.distance_to(source.global_position)+range_bound)
    return minf(viewer.far,maxf(viewer.near*2.0,bound*1.02+0.1))

func _configure_native_inputs(sources: Array[MeshInstance3D],viewer: Camera3D) -> void:
    var parameters: Dictionary={}
    for name in ["source_k","source_grid","source_limits","source_gain","source_bias","source_flags","source_edges"]:
        parameters[name]=PackedVector4Array([Vector4.ZERO,Vector4.ZERO,Vector4.ZERO,Vector4.ZERO])
    var native_mask:=0
    var quality_mask:=0
    var quality_ids:=Vector4.ZERO
    var quality_grid:=Vector4.ONE
    var quality_texture: Texture2D=_dummy
    var projection:=Projection.IDENTITY
    for i in MAX_CAMERAS:
        var depth: Texture2D=_dummy
        var to_local:=Transform3D.IDENTITY
        var to_view:=Transform3D.IDENTITY
        if i<sources.size():
            var source:=sources[i]
            var entry: Dictionary=_entries[source.get_instance_id()]
            if entry.get("direct",false):
                native_mask|=1<<i
                var mat: ShaderMaterial=entry.material
                depth=mat.get_shader_parameter("depth_tex")
                var k: Vector4=mat.get_shader_parameter("intrinsics")
                var stride:=maxf(1,float(mat.get_shader_parameter("grid_stride")))
                parameters.source_k[i]=k
                parameters.source_grid[i]=Vector4(depth.get_width(),depth.get_height(),stride,0)
                var limits: Vector2=mat.get_shader_parameter("depth_range")
                parameters.source_limits[i]=Vector4(limits.x,limits.y,float(mat.get_shader_parameter("max_depth_delta")),float(mat.get_shader_parameter("max_mesh_edge")))
                var gain: Vector3=mat.get_shader_parameter("color_gain")
                var bias: Vector3=mat.get_shader_parameter("color_bias")
                parameters.source_gain[i]=Vector4(gain.x,gain.y,gain.z,0)
                parameters.source_bias[i]=Vector4(bias.x,bias.y,bias.z,0)
                parameters.source_flags[i]=Vector4(float(bool(mat.get_shader_parameter("color_enabled"))),float(bool(mat.get_shader_parameter("color_match_enabled"))),float(bool(mat.get_shader_parameter("edge_feather_enabled"))),0)
                parameters.source_edges[i]=Vector4(float(mat.get_shader_parameter("edge_feather_width")),float(mat.get_shader_parameter("edge_feather_min_alpha")),0,0)
                to_local=source.global_transform.affine_inverse()*viewer.get_camera_transform()
                to_view=viewer.get_camera_transform().affine_inverse()*source.global_transform
                projection=(entry.meta_camera as Camera3D).get_camera_projection().inverse()
                if mat.get_shader_parameter("rig_quality_ready")==true:
                    quality_texture=mat.get_shader_parameter("rig_quality")
                    quality_grid=mat.get_shader_parameter("rig_quality_grid")
                    quality_ids[i]=float(mat.get_shader_parameter("rig_camera_index"))
                    quality_mask|=1<<i
                elif sources.size()==1:
                    quality_texture=entry.cache.viewport.get_texture()
                    var size: Vector2i=entry.cache.viewport.size
                    quality_grid=Vector4(1,1,size.x,size.y)
                    quality_mask|=1<<i
        for target in [_dominant_material,_composite]:
            target.set_shader_parameter("source_depth_%d"%i,depth)
            target.set_shader_parameter("source_view_to_local_%d"%i,to_local)
            target.set_shader_parameter("source_local_to_view_%d"%i,to_view)
    for target in [_dominant_material,_composite]:
        for name in parameters: target.set_shader_parameter(name,parameters[name])
        target.set_shader_parameter("native_mask",native_mask)
        target.set_shader_parameter("source_quality",quality_texture)
        target.set_shader_parameter("source_quality_grid",quality_grid)
        target.set_shader_parameter("source_quality_ids",quality_ids)
        target.set_shader_parameter("source_quality_mask",quality_mask)
        target.set_shader_parameter("source_projection_inverse",projection)
        target.set_shader_parameter("source_orthographic",viewer.projection==Camera3D.PROJECTION_ORTHOGONAL)
        target.set_shader_parameter("source_output_size",Vector2(_source_size))
