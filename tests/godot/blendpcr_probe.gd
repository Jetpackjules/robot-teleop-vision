extends SceneTree
const Renderer = preload("res://godot/runtime/point_cloud/blendpcr/blendpcr_renderer.gd")
var checks := 0
var failures := 0
var renderer: Node3D
var viewport: SubViewport
var viewer: Camera3D
var surfaces: Array[MeshInstance3D] = []

class Surface extends MeshInstance3D:
    var age := 0.0
    var focal := 600.0
    var image_size := 16
    var native_grid := false
    func get_display_frame_age_ms() -> float: return age
    func get_current_intrinsics() -> Vector4:
        return Vector4(image_size,image_size,(image_size-1)*0.5,(image_size-1)*0.5) if native_grid else Vector4(focal,focal,32,32)
    func get_current_grid_stride() -> int: return 1
    func fill(z: float, rgb: Color) -> void:
        var mat := material_override as ShaderMaterial
        if native_grid:
            var depth := Image.create(image_size,image_size,false,Image.FORMAT_RF)
            depth.fill(Color(z,0,0,1))
            var color := Image.create(image_size,image_size,false,Image.FORMAT_RGBA8)
            color.fill(rgb)
            mat.set_shader_parameter("depth_tex", ImageTexture.create_from_image(depth))
            mat.set_shader_parameter("color_tex", ImageTexture.create_from_image(color))
            mat.set_shader_parameter("intrinsics", get_current_intrinsics())
            mat.set_shader_parameter("texel_size", Vector2.ONE / float(image_size))
            mat.set_shader_parameter("depth_range", Vector2(0.1,4))
            mat.set_shader_parameter("max_mesh_edge", 1.0)
            mat.set_shader_parameter("grid_stride", 1.0)
            return
        mat.set_shader_parameter("depth_m", z)
        mat.set_shader_parameter("rgb", Vector3(rgb.r,rgb.g,rgb.b))

func check(ok: bool, label: String) -> void:
    checks += 1
    if not ok:
        failures += 1
        push_error(label)

func _initialize() -> void:
    call_deferred("run")

func settle() -> void:
    for i in 8:
        renderer.update_surfaces(surfaces, viewer, 1.0, 0.035, 2)
        await process_frame
        await RenderingServer.frame_post_draw

func sample(label: String, expected: Color) -> void:
    await settle()
    var actual := viewport.get_texture().get_image().get_pixel(viewport.size.x / 2, viewport.size.y / 2)
    print(label, ": ", actual)
    check(Vector3(actual.r,actual.g,actual.b).distance_to(Vector3(expected.r,expected.g,expected.b)) < 0.10, "%s expected %s got %s" % [label,expected,actual])

func grid_mesh(size: int = 16) -> ArrayMesh:
    var vertices := PackedVector3Array()
    var uvs := PackedVector2Array()
    var triangle_a := PackedVector2Array()
    var other_corners := PackedFloat32Array()
    for y in size - 1:
        for x in size - 1:
            var a := (Vector2(x, y) + Vector2(0.5, 0.5)) / float(size)
            var b := a + Vector2(1.0/float(size), 0)
            var c := a + Vector2(0, 1.0/float(size))
            var d := a + Vector2.ONE / float(size)
            for tri in [[a, c, b], [b, c, d]]:
                for uv in tri:
                    vertices.append(Vector3.ZERO)
                    uvs.append(uv)
                    triangle_a.append(tri[0])
                    other_corners.append_array(PackedFloat32Array([tri[1].x, tri[1].y, tri[2].x, tri[2].y]))
    var arrays := []
    arrays.resize(Mesh.ARRAY_MAX)
    arrays[Mesh.ARRAY_VERTEX] = vertices
    arrays[Mesh.ARRAY_TEX_UV] = uvs
    arrays[Mesh.ARRAY_TEX_UV2] = triangle_a
    arrays[Mesh.ARRAY_CUSTOM0] = other_corners
    var mesh := ArrayMesh.new()
    mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays, [], {}, Mesh.ARRAY_CUSTOM_RGBA_FLOAT << Mesh.ARRAY_FORMAT_CUSTOM0_SHIFT)
    mesh.custom_aabb = AABB(Vector3(-2,-2,-3), Vector3(4,4,4))
    return mesh


func run() -> void:
    print("BLENDPCR_RENDERER=", RenderingServer.get_current_rendering_method())
    viewport = SubViewport.new()
    viewport.size = Vector2i(128,128)
    viewport.own_world_3d = true
    viewport.transparent_bg = true
    viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
    root.add_child(viewport)
    var world := Node3D.new()
    viewport.add_child(world)
    viewer = Camera3D.new()
    viewer.projection = Camera3D.PROJECTION_ORTHOGONAL
    viewer.size = 1.5
    viewer.far = 100
    viewer.current = true
    world.add_child(viewer)
    var shader := Shader.new()
    shader.code = """
shader_type spatial;
render_mode unshaded, cull_disabled, fog_disabled;
uniform float depth_m = 1.0;
uniform vec3 rgb = vec3(1.0,0.0,0.0);
uniform mat4 calibration_transform = mat4(1.0);
varying vec3 point_position_local;
void vertex() { VERTEX.z = -depth_m; point_position_local = VERTEX; }
void fragment() { ALBEDO = rgb; }
"""
    for i in 3:
        var surface := Surface.new()
        var mesh := QuadMesh.new()
        mesh.size = Vector2(1.2,1.2)
        mesh.custom_aabb = AABB(Vector3(-1,-1,-3),Vector3(2,2,4))
        surface.mesh = mesh
        surface.material_override = ShaderMaterial.new()
        (surface.material_override as ShaderMaterial).shader = shader
        world.add_child(surface)
        surfaces.append(surface)
    renderer = Renderer.new()
    world.add_child(renderer)
    surfaces[0].focal = 900
    surfaces[0].fill(1.008,Color.RED)
    surfaces[1].fill(1.006,Color.GREEN)
    surfaces[2].fill(1.004,Color.BLUE)
    await sample("higher sampling density wins overlap", Color.RED)
    # Native camera uniforms replace their initial mat4 Projection defaults
    # with Transform3D values after frames start arriving.
    for surface in surfaces:
        (surface.material_override as ShaderMaterial).set_shader_parameter("calibration_transform", Transform3D.IDENTITY)
    await sample("native matrix uniform type changes keep rendering", Color.RED)
    # An already-running @tool renderer retains its entry dictionaries when a
    # new cache field is introduced by an editor script reload.
    renderer._compat_lut = null
    for entry in renderer._entries.values():
        entry.erase("material")
        entry.erase("camera_settings")
    await sample("editor reload rebuilds metadata lookup and material caches", Color.RED)
    var metadata: Image = renderer._entries[surfaces[0].get_instance_id()].meta_view.get_texture().get_image()
    var encoded := metadata.get_pixel(viewport.size.x / 2, viewport.size.y / 2)
    var decoded_depth := (encoded.r * 255.0 * 256.0 + encoded.g * 255.0) / 65535.0 * viewer.far
    print("metadata=", encoded, " depth=", decoded_depth)
    check(abs(decoded_depth - 1.008) < 0.002, "metadata preserves metric depth across color spaces")
    check(surfaces.all(func(s): return s.layers == 0), "original meshes hidden only from rendering")
    surfaces[0].age = 1500
    surfaces[1].fill(0.7,Color.GREEN)
    surfaces[1].age = 1500
    await sample("stale foreground B cannot cover live C", Color.BLUE)
    surfaces[1].age = 0
    await sample("fresh foreground B returns", Color.GREEN)
    surfaces[0].age = 0
    surfaces[1].fill(1.006,Color.GREEN)
    surfaces[0].position.x = 3
    surfaces[1].age = 1500
    await sample("transformed camera does not cover peers", Color.BLUE)
    surfaces[0].position.x = 0
    surfaces[1].age = 0
    # Spatial camera weights blend a seam instead of just switching colors.
    surfaces[2].age = 1500
    surfaces[0].position.x = -0.4
    surfaces[1].position.x = 0.4
    await settle()
    var image := viewport.get_texture().get_image()
    var mixed := false
    for x in range(65, 95):
        var pixel := image.get_pixel(x, 64)
        mixed = mixed or (pixel.r > 0.08 and pixel.g > 0.08 and pixel.b < 0.05)
    check(mixed, "overlap seam contains contributions from both cameras")
    image.save_png("user://blendpcr-seam.png")
    surfaces[0].position.x = 0
    surfaces[1].position.x = 0
    surfaces[2].age = 0
    # A non-cloud overlay must occlude the composite using its reconstructed depth.
    var overlay := MeshInstance3D.new()
    overlay.mesh = QuadMesh.new()
    overlay.position.z = -0.5
    var material := StandardMaterial3D.new()
    material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
    material.albedo_color = Color.YELLOW
    overlay.material_override = material
    world.add_child(overlay)
    await sample("robot overlay in front stays visible", Color.YELLOW)
    overlay.position.z = -2
    await sample("robot overlay behind stays hidden", Color.RED)
    overlay.queue_free()
    for surface in surfaces: surface.age = 1500
    await sample("all stale clears output", Color.BLACK)
    for surface in surfaces: surface.age = 0
    viewport.size = Vector2i(160,128)
    await settle()
    check(renderer._source_size == Vector2i(160,128), "viewport resize updates all passes")
    renderer.clear()
    await process_frame
    check(surfaces.all(func(s): return s.layers == 1), "mode switch restores original drawing layers")
    check(renderer._entries.is_empty() and renderer._weights.is_empty(), "mode switch releases per-camera and screen passes")
    check(renderer.update_surfaces(surfaces, viewer), "re-enter mode after teardown")
    renderer.clear()
    # Layer masks survive switches; disabled cameras are restored too.
    surfaces[0].layers = 4
    renderer.update_surfaces(surfaces, viewer)
    var removed: MeshInstance3D = surfaces.pop_front()
    renderer.update_surfaces(surfaces, viewer)
    check(removed.layers == 4, "camera removal restores custom layer mask")
    removed.visible = false
    renderer.clear()
    # Production native shader with different sensor grids. This exercises
    # RGB/depth textures, actual triangle rejection, and the material adapter.
    var native_shader := load("res://native_template.gdshader") as Shader
    for i in surfaces.size():
        var surface := surfaces[i] as Surface
        surface.native_grid = true
        surface.image_size = 32 if i == 0 else 16
        surface.mesh = grid_mesh(surface.image_size)
        surface.material_override = ShaderMaterial.new()
        (surface.material_override as ShaderMaterial).shader = native_shader
        surface.age = 0
        surface.fill(1.0, Color.GREEN if i == 0 else Color.BLUE)
    await sample("production native shader accepts mixed RGBD resolutions", Color.GREEN)
    surfaces[0].age = 1500
    await sample("native RGBD stale B falls through to C", Color.BLUE)
    surfaces[0].age = 0
    surfaces[0].fill(0.0,Color.GREEN)
    await sample("native missing depth falls through to C", Color.BLUE)
    renderer.clear()
    surfaces[0].fill(1.0, Color(0.25,0.5,0.75))
    surfaces[1].visible = false
    for i in 4:
        await process_frame
        await RenderingServer.frame_post_draw
    var baseline := viewport.get_texture().get_image().get_pixel(viewport.size.x / 2, viewport.size.y / 2)
    await sample("native RGB color space preserved", baseline)
    # A thin native depth strip survives; the port adds no hole filling/erosion.
    var thin := Image.create(32,32,false,Image.FORMAT_RF)
    thin.fill(Color(0,0,0,1))
    for y in range(8,24):
        for x in range(15,18):
            thin.set_pixel(x,y,Color(1,0,0,1))
    (surfaces[0].material_override as ShaderMaterial).set_shader_parameter("depth_tex", ImageTexture.create_from_image(thin))
    await sample("thin native surface survives spatial weighting", baseline)
    var none: Array[MeshInstance3D] = []
    check(renderer.update_surfaces(none, viewer) and renderer._entries.is_empty(), "empty rig restores and releases renderer")
    # Explicit capacity failure restores every original mesh.
    for i in 3:
        var extra := Surface.new()
        extra.mesh = surfaces[0].mesh
        extra.material_override = surfaces[0].material_override
        world.add_child(extra)
        surfaces.append(extra)
    check(not renderer.update_surfaces(surfaces, viewer) and renderer.status.contains("up to 4"), "unsupported camera count is reported")
    check(surfaces.all(func(s): return s.layers != 0), "capacity failure leaves originals visible")
    print("BLENDPCR_PROBE checks=%d failures=%d" % [checks,failures])
    viewport.queue_free()
    await process_frame
    quit(0 if failures == 0 else 1)
