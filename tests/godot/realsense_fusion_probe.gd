extends SceneTree
const Fusion = preload("res://godot/runtime/point_cloud/realsense_mesh_fusion.gd")
var checks := 0
var failures := 0

class CameraSurface extends MeshInstance3D:
	var depth: Image
	var color: Image
	var sequence := 0
	var age := 0.0
	var image_size := 16
	var color_enabled := true
	var point_cleanup_enabled := false
	var point_cleanup_depth_delta := 0.03
	var point_cleanup_min_neighbors := 2
	func get_depth_image() -> Image: return depth
	func get_color_image() -> Image: return color
	func get_frame_sequence() -> int: return sequence
	func get_current_grid_stride() -> int: return 1
	func get_current_intrinsics() -> Vector4: return Vector4(image_size, image_size, (image_size-1)*0.5, (image_size-1)*0.5)
	func get_display_frame_age_ms() -> float: return age
	func fill(z: float, rgb: Color) -> void:
		depth = Image.create(image_size, image_size, false, Image.FORMAT_RF)
		depth.fill(Color(z, 0, 0, 1))
		color = Image.create(image_size, image_size, false, Image.FORMAT_RGBA8)
		color.fill(rgb)
		sequence += 1
		var mat := material_override as ShaderMaterial
		mat.set_shader_parameter("intrinsics", get_current_intrinsics())
		mat.set_shader_parameter("texel_size", Vector2.ONE / float(image_size))
		mat.set_shader_parameter("depth_tex", ImageTexture.create_from_image(depth))
		mat.set_shader_parameter("color_tex", ImageTexture.create_from_image(color))

func check(condition: bool, label: String) -> void:
	checks += 1
	if not condition:
		failures += 1
		push_error(label)

func _initialize() -> void: call_deferred("run")

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

func sample(viewport: SubViewport, surfaces: Array[MeshInstance3D], fusion: RefCounted, enabled: bool, label: String, expected: Color) -> void:
	var limits: Array[Vector2] = []
	for surface in surfaces: limits.append(Vector2(0.1, 4))
	check(fusion.update(surfaces, limits, 0.05, 1.0), label + " layers upload")
	check(fusion.camera_data.get_width() == 8 and fusion.camera_data.get_height() == surfaces.size(), label + " metadata shape")
	for index in surfaces.size():
		var mat := surfaces[index].material_override as ShaderMaterial
		mat.set_shader_parameter("rig_fusion_enabled", enabled)
		mat.set_shader_parameter("rig_camera_index", index)
		mat.set_shader_parameter("rig_depth", fusion.depth_layers)
		mat.set_shader_parameter("rig_color", fusion.color_layers)
		mat.set_shader_parameter("rig_cameras", fusion.camera_data)
	await process_frame
	await RenderingServer.frame_post_draw
	await process_frame
	await RenderingServer.frame_post_draw
	var color := viewport.get_texture().get_image().get_pixel(65, 65)
	check(Vector3(color.r, color.g, color.b).distance_to(Vector3(expected.r, expected.g, expected.b)) < 0.15, "%s expected %s got %s" % [label, expected, color])
	viewport.get_texture().get_image().save_png("user://" + label.replace(" ", "_") + ".png")

func run() -> void:
	check(ClassDB.class_exists("RealSenseSharedMemoryPointCloud"), "rebuilt native extension loaded")
	var native = ClassDB.instantiate("RealSenseSharedMemoryPointCloud")
	native.set("shared_memory_name", "__codex_fusion_test_no_source__")
	native.set("direct_realsense_enabled", false)
	native.set("render_connected_mesh", true)
	native.set("gpu_connected_mesh", true)
	native.set("gpu_mesh_static_shader", true)
	root.add_child(native)
	native.set_process(false)
	var template: ShaderMaterial = native.get_material_override()
	check(template != null and template.shader.code.contains("rig_overlap"), "production native shader contains multi-camera fusion")
	var viewport := SubViewport.new()
	viewport.size = Vector2i(128,128)
	viewport.own_world_3d = true
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	root.add_child(viewport)
	var world := Node3D.new()
	viewport.add_child(world)
	var camera := Camera3D.new()
	camera.projection = Camera3D.PROJECTION_ORTHOGONAL
	camera.size = 1.2
	camera.current = true
	world.add_child(camera)
	var surfaces: Array[MeshInstance3D] = []
	var palette: Array[Color] = [Color.RED, Color.GREEN, Color.BLUE, Color.YELLOW]
	var mesh := grid_mesh()
	for i in 4:
		var surface := CameraSurface.new()
		surface.mesh = mesh
		surface.material_override = template.duplicate()
		var mat := surface.material_override as ShaderMaterial
		for pair in [["intrinsics", Vector4(16,16,7.5,7.5)], ["depth_range", Vector2(0.1,4)], ["texel_size", Vector2(1.0/16.0,1.0/16.0)], ["grid_stride",1.0], ["max_depth_delta",0.05], ["max_mesh_edge",1.0], ["color_enabled",true], ["fusion_depth_tolerance",0.035]]:
			mat.set_shader_parameter(pair[0], pair[1])
		surface.fill(1.008 - i*0.002, palette[i])
		world.add_child(surface)
		surfaces.append(surface)
	var fusion := Fusion.new()
	await sample(viewport, surfaces, fusion, false, "independent closest fourth", Color.YELLOW)
	await sample(viewport, surfaces, fusion, true, "unified first owns all four", Color.RED)
	native.set_fusion_camera_layers(fusion.depth_layers, fusion.color_layers, fusion.camera_data, 3)
	check(bool(template.get_shader_parameter("rig_fusion_enabled")) and int(template.get_shader_parameter("rig_camera_index")) == 3, "native API applies four-camera material parameters")
	var first_texture: Texture2DArray = fusion.depth_layers
	await sample(viewport, surfaces, fusion, true, "unchanged frames reuse layers", Color.RED)
	check(first_texture == fusion.depth_layers, "no texture allocation for unchanged frames")
	surfaces[0].depth.set_pixel(8, 7, Color(0,0,0,1))
	surfaces[0].sequence += 1
	(surfaces[0].material_override as ShaderMaterial).set_shader_parameter("depth_tex", ImageTexture.create_from_image(surfaces[0].depth))
	await sample(viewport, surfaces, fusion, true, "invalid triangle does not hide peers", Color.GREEN)
	surfaces[0].fill(1.008, Color.RED)
	surfaces[2].image_size = 8
	surfaces[2].mesh = grid_mesh(8)
	surfaces[2].fill(1.004, Color.BLUE)
	await sample(viewport, surfaces, fusion, true, "mixed resolutions retain priority", Color.RED)
	surfaces[0].fill(0, Color.RED)
	await sample(viewport, surfaces, fusion, true, "second fills missing first", Color.GREEN)
	surfaces[1].fill(0, Color.GREEN)
	await sample(viewport, surfaces, fusion, true, "third fills missing first pair", Color.BLUE)
	surfaces[2].fill(0, Color.BLUE)
	await sample(viewport, surfaces, fusion, true, "fourth fills all missing", Color.YELLOW)
	for i in 4: surfaces[i].fill(1.008 - i*0.002, palette[i])
	surfaces[0].fill(1.008, Color(1,0,0,0))
	await sample(viewport, surfaces, fusion, true, "invalid color does not hide peers", Color.GREEN)
	surfaces[0].fill(1.008, Color.RED)
	surfaces[0].age = 2000
	await sample(viewport, surfaces, fusion, true, "stale first does not hide peers", Color.GREEN)
	surfaces[0].age = 0
	surfaces[2].fill(0.7, Color.BLUE)
	await sample(viewport, surfaces, fusion, true, "separate foreground stays visible", Color.BLUE)
	surfaces[2].fill(1.004, Color.BLUE)
	surfaces[0].position.x = 2.0
	await sample(viewport, surfaces, fusion, true, "translated camera has no false overlap", Color.GREEN)
	surfaces[0].position.x = 0
	world.transform = Transform3D(Basis(Vector3.UP, 0.3), Vector3(1.2,0.3,-0.1))
	await sample(viewport, surfaces, fusion, true, "world transform preserves ownership", Color.RED)
	var hidden: MeshInstance3D = surfaces.pop_front()
	hidden.visible = false
	await sample(viewport, surfaces, fusion, true, "hidden camera removed from layers", Color.GREEN)
	check(fusion.depth_layers.get_layers() == 3, "camera removal rebuilds layer count")
	print("REALSENSE_FUSION_PROBE checks=%d failures=%d" % [checks, failures])
	native.queue_free()
	viewport.queue_free()
	await process_frame
	quit(0 if failures == 0 else 1)
