extends Node3D
## Socket-free SO-101 geometry/FK. Constants follow the bundled URDF/model assets.
## This model intentionally does not load saved hardware registration or calibration.

const ROS_TO_GODOT := Basis(Vector3(0, 0, -1), Vector3(-1, 0, 0), Vector3(0, 1, 0))
const LINK_NAMES := ["shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link", "moving_jaw_so101_v1_link"]
const ORIGINS := [Vector3(0.0388353, 0, 0.0624), Vector3(-0.0303992, -0.0182778, -0.0542), Vector3(-0.11257, -0.028, 0), Vector3(-0.1349, 0.0052, 0), Vector3(0, -0.0611, 0.0181), Vector3(0.0202, 0.0188, -0.0234)]
const RPY := [Vector3(PI, 0, -PI), Vector3(-PI/2, -PI/2, 0), Vector3(0, 0, PI/2), Vector3(0, 0, -PI/2), Vector3(PI/2, 0.0486795, PI), Vector3(PI/2, -0.000000052, 0)]
const LIMITS := [Vector2(-1.91986, 1.91986), Vector2(-1.74533, 1.74533), Vector2(-1.69, 1.69), Vector2(-PI, PI), Vector2(-2.74385, 2.84121)]
const OFFSETS := [40.4296875, 80.0, 0.0, -70.0, 0.0]
const DIRECTIONS := [1.0, -1.0, 1.0, 1.0, 1.0]
const BOUNDS := [AABB(Vector3(-.0506,-.0264,-.0644),Vector3(.0628,.0551,.1106)), AABB(Vector3(-.1302,-.0382,-.0135),Vector3(.1422,.0502,.0673)), AABB(Vector3(-.1451,-.015,-.012),Vector3(.1571,.0369,.0644)), AABB(Vector3(-.0200,-.0658,-.009),Vector3(.0357,.0778,.0623)), AABB(Vector3(-.0352,-.028,-.1044),Vector3(.0656,.052,.1054)), AABB(Vector3(-.0123,-.082,-.0051),Vector3(.0223,.092,.048))]

var normalized: Array = [-40.4296875, 130.0, 38.0, 78.0, 0.0, 100.0]
var _angles: Array[float] = []
var _joints: Array[Node3D] = []
var _origins: Array[Transform3D] = []
var _model: Node3D


func _ready() -> void:
	_model = Node3D.new()
	_model.basis = ROS_TO_GODOT
	add_child(_model)
	_add_visual(_model, "base_link")
	var parent_link: Node3D = _model
	for i in range(6):
		var link := Node3D.new()
		link.name = LINK_NAMES[i]
		parent_link.add_child(link)
		_joints.append(link)
		_origins.append(Transform3D(_rpy(RPY[i]), ORIGINS[i]))
		_angles.append(0.0)
		_add_visual(link, LINK_NAMES[i])
		parent_link = link
	set_normalized(normalized)


static func _rpy(value: Vector3) -> Basis:
	return Basis(Vector3.BACK, value.z) * Basis(Vector3.UP, value.y) * Basis(Vector3.RIGHT, value.x)


func _add_visual(parent: Node3D, link_name: String) -> void:
	# Runtime GLTF loading keeps this scene independent of editor import caches
	# and avoids starting production editor plugins just to prepare the assets.
	var document := GLTFDocument.new()
	var state := GLTFState.new()
	var bytes := FileAccess.get_file_as_bytes("res://robot_modules/so101/assets/%s.glb" % link_name)
	# The bundled trimesh GLBs retain unused STL extras.header strings containing
	# escaped NULs. Godot logs a Unicode warning for every one during JSON parsing.
	# Replace these escapes with equal-length spaces in the in-memory JSON chunk;
	# mesh binary data, offsets and original licensed source files stay untouched.
	if bytes.size() >= 20 and bytes.decode_u32(0) == 0x46546c67:
		var json_size := bytes.decode_u32(12)
		if 20 + json_size <= bytes.size():
			var json_bytes := bytes.slice(20,20+json_size)
			var text := json_bytes.get_string_from_utf8()
			if text.contains("\\u0000"):
				var header_pattern := RegEx.new()
				header_pattern.compile("\"extras\"\\s*:\\s*\\{\\s*\"header\"\\s*:\\s*\"(?:\\\\u0000)+\"")
				var normalized_text := text
				for matched in header_pattern.search_all(text):
					var header: String = matched.get_string()
					normalized_text = normalized_text.replace(header,header.replace("\\u0000","      "))
				var normalized_json := normalized_text.to_utf8_buffer()
				if normalized_json.size() == json_size:
					for i in range(json_size):
						bytes[20+i] = normalized_json[i]
	var error := document.append_from_buffer(bytes,"",state)
	if error == OK:
		var instance := document.generate_scene(state)
		parent.add_child(instance)
		_set_mesh_layer(instance)
	else:
		push_error("Cannot load bundled virtual arm mesh: " + link_name)


func _set_mesh_layer(node: Node) -> void:
	if node is MeshInstance3D:
		node.layers = 1
		var material := ShaderMaterial.new()
		material.shader = preload("res://examples/alignment_demo/godot/simulation_surface.gdshader")
		material.set_shader_parameter("base_color",Color(.20,.30,.40))
		node.material_override = material
		# The depth sensor raycasts the actual rendered mesh, including the open
		# space between gripper fingers. These shapes never push the task block.
		var proxy := StaticBody3D.new()
		proxy.collision_layer = 2
		proxy.collision_mask = 0
		proxy.set_meta("cloud_color", Color(.20,.30,.40))
		var collision := CollisionShape3D.new()
		collision.shape = node.mesh.create_trimesh_shape()
		proxy.add_child(collision)
		node.add_child(proxy)
	for child in node.get_children():
		_set_mesh_layer(child)


func set_normalized(values: Array) -> void:
	if values.size() != 6:
		return
	normalized = values.duplicate()
	if _joints.size() != 6:
		return
	for i in range(5):
		var angle := wrapf(deg_to_rad(float(values[i]) * DIRECTIONS[i] + OFFSETS[i]), -PI, PI)
		_angles[i] = clampf(angle, LIMITS[i].x, LIMITS[i].y)
	_angles[5] = deg_to_rad(lerpf(-20.0, 100.0, clampf((float(values[5])-2.5) / 97.5, 0, 1)))
	_apply_angles()


func _apply_angles() -> void:
	for i in range(_joints.size()):
		_joints[i].transform = _origins[i] * Transform3D(Basis(Vector3.BACK, _angles[i]), Vector3.ZERO)


func tool_transform() -> Transform3D:
	if _joints.size() < 5:
		return global_transform
	return _joints[4].global_transform * Transform3D(_rpy(Vector3(0, PI, 0)), Vector3(-.0079, -.000218121, -.0981274))


func move_tool_to(target: Vector3, iterations: int = 12) -> float:
	# Damped least-squares position IK is for keyboard/replay only. Leader samples
	# always drive their supplied normalized joints directly without this solver.
	for _step in range(iterations):
		var current := tool_transform().origin
		var error := target - current
		if error.length() < .0008:
			break
		var jacobian: Array[Vector3] = []
		var matrix := Basis(Vector3(.002,0,0), Vector3(0,.002,0), Vector3(0,0,.002))
		for i in range(5):
			var original := _angles[i]
			_angles[i] += .01
			_apply_angles()
			var column := (tool_transform().origin - current) / .01
			jacobian.append(column)
			matrix.x += column * column.x
			matrix.y += column * column.y
			matrix.z += column * column.z
			_angles[i] = original
		_apply_angles()
		var gradient := matrix.inverse() * error.limit_length(.04)
		for i in range(5):
			_angles[i] = clampf(_angles[i] + clampf(jacobian[i].dot(gradient), -.22, .22), LIMITS[i].x, LIMITS[i].y)
		_apply_angles()
	for i in range(5):
		normalized[i] = (rad_to_deg(_angles[i]) - OFFSETS[i]) / DIRECTIONS[i]
	return tool_transform().origin.distance_to(target)


func set_gripper(opening: float) -> void:
	normalized[5] = clampf(opening, 0, 100)
	_angles[5] = deg_to_rad(lerpf(-20.0, 100.0, clampf((normalized[5]-2.5) / 97.5, 0, 1)))
	_apply_angles()
