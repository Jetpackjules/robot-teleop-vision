extends SceneTree
const Guides = preload("res://godot/runtime/point_cloud/camera_rig_guides.gd")
var checks := 0
var failures := 0

func check(ok: bool, label: String) -> void:
	checks += 1
	if not ok:
		failures += 1
		push_error(label)

func _initialize() -> void:
	call_deferred("run")

func run() -> void:
	var guides := Guides.new()
	root.add_child(guides)
	var specs: Array[Dictionary] = []
	for i in 5:
		var camera := MeshInstance3D.new()
		camera.layers = 0 # BlendPCR hides the original mesh this way.
		root.add_child(camera)
		camera.position = Vector3(i, i * 0.1, -i)
		specs.append({"source": camera, "label": "Camera %d" % i,
			"intrinsics": Vector4(600, 610, 423.5, 239.5), "size": Vector2i(424, 240),
			"stride": 2, "near": 0.2, "far": 1.5})
	guides.update_cameras(specs, 0)
	check(guides._entries.is_empty(), "hidden guides allocate no geometry")
	guides.update_cameras(specs, 1)
	check(guides._entries.size() == 5, "camera count is not capped at two or three")
	for spec in specs:
		var entry: Dictionary = guides._entries[spec.source.get_instance_id()]
		check(entry.root.get_parent() == spec.source, "guides are camera children")
		check(entry.root.global_position == spec.source.global_position, "dot follows calibrated camera position")
		check(entry.sphere.is_visible_in_tree() and spec.source.layers == 0, "dot stays visible when BlendPCR hides source mesh")
		check(not entry.cone.visible, "positions mode hides view cones")
		check(entry.root.owner == null, "generated guides are not serialized into the scene")
	var first: Dictionary = guides._entries[specs[0].source.get_instance_id()]
	var fourth: Dictionary = guides._entries[specs[3].source.get_instance_id()]
	check(first.sphere.material_override.albedo_color == fourth.sphere.material_override.albedo_color, "colors repeat red green blue")
	guides.update_cameras(specs, 2)
	check(first.cone.visible, "view cone mode enables the cone")
	var corners := Guides.frustum_corners(Vector4(600, 610, 423.5, 239.5), Vector2i(424, 240), 2, 1.5)
	check(corners.size() == 4, "cone uses four optical corners")
	check(is_equal_approx(corners[2].x / -corners[2].z * 600 + 423.5, 846.0), "horizontal field follows actual intrinsics and stride")
	check(is_equal_approx(-corners[2].y / -corners[2].z * 610 + 239.5, 478.0), "vertical field follows actual intrinsics and stride")
	check(corners[0].z == -1.5 and corners[0].y > 0, "cone uses Godot optical forward and up")
	guides.update_cameras([specs[0]], 2)
	check(guides._entries.size() == 1, "removed cameras release their guides")
	guides.update_cameras(specs, 0)
	await process_frame
	check(guides._entries.is_empty(), "hidden mode releases all guides")
	for spec in specs:
		spec.source.queue_free()
	guides.queue_free()
	print("CAMERA_GUIDES checks=%d failures=%d" % [checks, failures])
	quit(0 if failures == 0 else 1)
