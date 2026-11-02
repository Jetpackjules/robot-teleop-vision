extends SceneTree

const Overlay := preload("res://robot_modules/so101/godot/so101_robot_overlay.gd")
const Calibrator := preload("res://robot_modules/so101/godot/so101_motion_calibrator.gd")

func _initialize() -> void:
	call_deferred("run")

func run() -> void:
	# Detached nodes never execute _ready: no cameras, servo sockets, or original
	# user-data paths are opened. Only this isolated project's file is writable.
	var overlay := Overlay.new()
	overlay._registration_path = "res://registration.json"
	overlay._registration_status = {"registered": true, "saved_unix_ms": 1.0,
		"joint_refinement": {"kept": true}, "calibrated_through_joint": 4}
	overlay.joint_angle_offsets_degrees = PackedFloat32Array([40.4296875, 91.5, -24, -70, 12, 0])
	overlay.gripper_visual_correction_translation_local = Vector3(0.002, -0.001, 0.003)
	overlay.moving_jaw_calibration_enabled = true
	overlay.moving_jaw_visual_radial_scale = 1.08
	assert(overlay._save_registration())
	var before := overlay._read_registration_payload()
	var target := Transform3D(Basis(Vector3.UP, deg_to_rad(43)), Vector3(0.21, 0.04, -0.45))
	var token := overlay.get_alignment_token()
	assert(token == overlay.get_alignment_token())
	assert(overlay.apply_rigid_registration(target, 0.90, 10.0, {"joint_mapping_changed": false}, token))
	assert(overlay.transform.is_equal_approx(target))
	var after := overlay._read_registration_payload()
	var placement_keys := ["basis_x", "basis_y", "basis_z", "origin", "confidence", "registration_source",
		"base_registration_source", "saved_unix_ms", "calibration_started_unix_ms", "registration_fit"]
	for key in before:
		if key not in placement_keys:
			assert(before[key] == after[key], "Placement changed model field: " + str(key))
	assert(after.registration_fit.joint_mapping_changed == false)
	# An in-flight result cannot overwrite a newer joint edit or external file.
	token = overlay.get_alignment_token()
	overlay.joint_angle_offsets_degrees[1] += 1.0
	assert(not overlay.apply_rigid_registration(Transform3D.IDENTITY, 1.0, 11.0, {}, token))
	assert(overlay.transform.is_equal_approx(target))
	token = overlay.get_alignment_token()
	var file := FileAccess.open("res://registration.json", FileAccess.WRITE)
	file.store_string(JSON.stringify({"changed_by_another_session": true}))
	file.close()
	assert(not overlay.apply_rigid_registration(Transform3D.IDENTITY, 1.0, 11.0, {}, token))
	# Failed atomic save restores the in-memory pose and status as well.
	overlay._registration_path = "res://missing-directory/registration.json"
	var status_before := overlay._registration_status.duplicate(true)
	token = overlay.get_alignment_token()
	assert(not overlay.apply_rigid_registration(Transform3D.IDENTITY, 1.0, 12.0, {}, token))
	assert(overlay.transform.is_equal_approx(target))
	assert(overlay._registration_status == status_before)
	# A prior high-resolution claw query must not enlarge later mesh exports.
	var dense := PackedVector3Array()
	for i in range(1000):
		dense.append(Vector3(float(i) * 0.00001, 0.01, 0.02))
	overlay._link_surface_points = {"base_link": dense, "wrist_link": dense}
	overlay._link_surface_points_maximum_per_link = 20000
	var meshes := overlay.get_pose_mesh_points_local([0, 110, 60, 75, 0, 40], 25)
	assert(meshes.base_link.size() == 25 and meshes.wrist_link.size() == 25)
	assert(overlay._link_surface_points.base_link.size() == 1000)
	# A link owns only Visual; the child joint's mesh must never enter its cache.
	var base := Node3D.new()
	var own_visual := MeshInstance3D.new()
	own_visual.name = "Visual"
	own_visual.mesh = BoxMesh.new()
	own_visual.mesh.size = Vector3(0.03, 0.04, 0.05)
	base.add_child(own_visual)
	var child_joint := Node3D.new()
	child_joint.position = Vector3(1, 2, 3)
	base.add_child(child_joint)
	var child_visual := MeshInstance3D.new()
	child_visual.mesh = BoxMesh.new()
	child_joint.add_child(child_visual)
	overlay._links = {"base_link": base}
	overlay._link_surface_points.clear()
	overlay._ensure_link_surface_points(1000)
	var base_samples: PackedVector3Array = overlay._link_surface_points.base_link.duplicate()
	assert(not base_samples.is_empty())
	for point in base_samples:
		assert(point.length() < 0.05, "Child joint contaminated base mesh")
	child_joint.position = Vector3(4, 5, 6)
	overlay._link_surface_points.clear()
	overlay._ensure_link_surface_points(1000)
	assert(base_samples == overlay._link_surface_points.base_link)
	var mapping_before := overlay.joint_angle_offsets_degrees.duplicate()
	var visual_frames := overlay.get_pose_link_visual_transforms_local([0, 110, 60, 75, 0, 40])
	assert(visual_frames.has("base_link"))
	assert((visual_frames.base_link as Transform3D).origin.is_zero_approx())
	assert(overlay.joint_angle_offsets_degrees == mapping_before)
	# Appearance must not alter geometry, placement or the registration token.
	overlay._model_root = base
	var appearance_token := overlay.get_alignment_token()
	overlay.set_overlay_style("realistic")
	overlay.set_overlay_transparency(0.7)
	assert(own_visual.transparency == 0.0)
	assert(own_visual.get_surface_override_material(0) is StandardMaterial3D)
	assert(overlay.get_alignment_token() == appearance_token)
	overlay.set_overlay_style("alignment")
	assert(own_visual.get_surface_override_material(0) == null)
	assert(is_equal_approx(own_visual.transparency, 0.7))
	overlay._model_root = null
	base.free()
	var calibrator := Calibrator.new()
	calibrator._capture_mode = "registration"
	calibrator._registration_token = "capture-token"
	var frame := {"pose": [0, 110, 60, 75, 0, 40], "points": PackedVector3Array(),
		"capsules": PackedVector4Array(), "camera_points": [], "full_camera_points": [],
		"observed_centroid": Vector3.ZERO, "model_centroid": Vector3.ZERO,
		"mesh_points": {"base_link": base_samples}, "link_visual_transforms": visual_frames}
	assert(calibrator._save_debug_capture([frame], "res://capture.json"))
	var exported: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("res://capture.json"))
	assert(exported.kinematics_token == "capture-token")
	assert(exported.frames[0].mesh_sampling == "link_visual_only_v1")
	assert(exported.frames[0].link_visual_transforms.base_link.has("basis_x"))
	assert(calibrator._sweep_command_type() == "arm_base_axis_sweep")
	calibrator.free()
	overlay.free()
	print("SO101_RIGID_REGISTRATION_PASS")
	quit()
