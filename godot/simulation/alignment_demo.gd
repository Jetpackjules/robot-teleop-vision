extends Node3D
## Standalone, hardware-free alignment task. Never loads the production Main scene.

const VirtualArm := preload("res://godot/simulation/virtual_so101.gd")
const Cloud := preload("res://godot/simulation/synthetic_point_cloud.gd")
const SimulationInput := preload("res://godot/simulation/simulation_input.gd")
const BLOCK_START := Vector3(.005, .023, -.075)
const BOWL_CENTER := Vector3(-.075, 0, -.165)
const BLOCK_SIZE := .042

var arm: Node3D
var block: RigidBody3D
var camera: Camera3D
var cloud: MeshInstance3D
var input_bridge: Node
var held := false
var completed := false
var replay_active := false
var replay_time := 0.0
var input_mode := "Keyboard"
var point_cloud_mode := true
var _head_active := false
var _head_neutral_set := false
var _head_neutral := Vector3.ZERO
var _head_displacement := Vector3.ZERO
var _external_arm_active := false
var _target := Vector3.ZERO
var _gripper := 100.0
var _held_local := Transform3D.IDENTITY
var _last_held_position := Vector3.ZERO
var _held_velocity := Vector3.ZERO
var _orbit_yaw := .0
var _orbit_pitch := .27
var _orbit_distance := .55
var _view_target := Vector3(-.045,.105,-.13)
var _status: Label
var _phase: Label
var _capture_path := ""
var _capture_time := 8.0
var _elapsed := 0.0
var _capture_requested := false
var _replay_start_tool := Vector3.ZERO
var _replay_block := BLOCK_START
var _fixed_view := false
var _clean_view := false
var _controls_panel: Control
var _canvas: CanvasLayer


func _ready() -> void:
	Engine.physics_ticks_per_second = 120
	# Centimetre-scale props need a smaller allowed penetration than the engine's
	# default. This changes only this standalone world's physics space.
	PhysicsServer3D.space_set_param(get_world_3d().space, PhysicsServer3D.SPACE_PARAM_CONTACT_MAX_ALLOWED_PENETRATION, .001)
	_build_environment()
	_build_world()
	arm = VirtualArm.new()
	arm.name = "VirtualSO101"
	arm.position = Vector3(.19, 0, .13)
	add_child(arm)
	_target = BLOCK_START + Vector3(0,.11,0)
	arm.move_tool_to(_target, 160)
	cloud = Cloud.new()
	cloud.name = "SyntheticDepthCloud"
	add_child(cloud)
	input_bridge = SimulationInput.new()
	input_bridge.name = "SimulationInputOnly"
	input_bridge.sample_received.connect(_apply_accepted_packet)
	add_child(input_bridge)
	_build_ui()
	for argument in OS.get_cmdline_user_args():
		if argument == "--simulation-mesh":
			point_cloud_mode = false
		if argument == "--simulation-fixed-view":
			_fixed_view = true
		if argument == "--simulation-clean-view":
			_clean_view = true
		if argument == "--simulation-replay":
			call_deferred("start_replay")
		if argument.begins_with("--simulation-capture="):
			_capture_path = argument.trim_prefix("--simulation-capture=")
		if argument.begins_with("--simulation-capture-time="):
			_capture_time = maxf(0.5, argument.get_slice("=",1).to_float())
	_set_render_mode()
	if _clean_view:
		_controls_panel.hide()
		var badge := Label.new()
		badge.position = Vector2(28,22)
		badge.text = "SIMULATION  ·  " + ("Fixed front view" if _fixed_view else "Moving viewpoint") + "  ·  " + ("Synthetic depth" if point_cloud_mode else "Geometry reference")
		badge.add_theme_font_size_override("font_size",21)
		badge.add_theme_color_override("font_color",Color(.08,.14,.20))
		_canvas.add_child(badge)
	_update_camera()


func _build_environment() -> void:
	var environment := Environment.new()
	environment.background_mode = Environment.BG_COLOR
	environment.background_color = Color(.86,.89,.91)
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	environment.ambient_light_color = Color.WHITE
	environment.ambient_light_energy = .35
	environment.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	var world := WorldEnvironment.new()
	world.environment = environment
	add_child(world)
	var light := DirectionalLight3D.new()
	light.rotation_degrees = Vector3(-42,-32,0)
	light.light_energy = .85
	light.shadow_enabled = true
	add_child(light)
	camera = Camera3D.new()
	camera.name = "OperatorView"
	camera.fov = 49
	camera.near = .015
	camera.far = 6
	add_child(camera)
	camera.current = true


func _material(color: Color) -> ShaderMaterial:
	var material := ShaderMaterial.new()
	material.shader = preload("res://godot/simulation/simulation_surface.gdshader")
	material.set_shader_parameter("base_color",color)
	return material


func _box(parent: Node3D, at: Vector3, size: Vector3, color: Color, collision: bool = true) -> Node3D:
	var body: Node3D = StaticBody3D.new() if collision else Node3D.new()
	body.position = at
	body.set_meta("cloud_color", color)
	parent.add_child(body)
	var mesh := MeshInstance3D.new()
	var geometry := BoxMesh.new()
	geometry.size = size
	mesh.mesh = geometry
	mesh.material_override = _material(color)
	mesh.layers = 1
	body.add_child(mesh)
	if collision:
		var shape := BoxShape3D.new()
		shape.size = size
		shape.margin = .001
		var collider := CollisionShape3D.new()
		collider.shape = shape
		body.add_child(collider)
	return body


func _build_world() -> void:
	_box(self, Vector3(0,-.035,-.1), Vector3(.86,.07,.72), Color(.69,.72,.73))
	_box(self, Vector3(0,.055,-.47), Vector3(.86,.11,.025), Color(.66,.72,.75))
	_box(self, Vector3(-.205,.085,-.235), Vector3(.036,.17,.09), Color(.35,.48,.53))
	_box(self, Vector3(0,.001,-.13), Vector3(.38,.002,.28), Color(.25,.31,.34))
	# An open collision bowl: separate bottom and wall segments, never a solid cylinder.
	var bowl := StaticBody3D.new()
	bowl.name = "WhiteBowl"
	bowl.position = BOWL_CENTER
	bowl.set_meta("cloud_color", Color(.97,.97,.94))
	add_child(bowl)
	var bottom := CylinderShape3D.new()
	bottom.radius = .076
	bottom.height = .008
	bottom.margin = .001
	var bottom_shape := CollisionShape3D.new()
	bottom_shape.shape = bottom
	bottom_shape.position.y = .004
	bowl.add_child(bottom_shape)
	var bottom_visual := MeshInstance3D.new()
	var disk := CylinderMesh.new()
	disk.top_radius = .076
	disk.bottom_radius = .076
	disk.height = .008
	bottom_visual.mesh = disk
	bottom_visual.position.y = .004
	bottom_visual.material_override = _material(Color(.97,.97,.94))
	bowl.add_child(bottom_visual)
	for i in range(24):
		var angle := TAU * float(i) / 24.0
		var segment := _box(self, BOWL_CENTER + Vector3(sin(angle)*.070, .038, cos(angle)*.070), Vector3(.019,.06,.012), Color(.97,.97,.94))
		segment.rotation.y = angle
	block = RigidBody3D.new()
	block.name = "PinkBlock"
	block.position = BLOCK_START
	block.mass = .045
	# A held body can retain a sleep timer; after unfreezing at 120 Hz its first
	# gravity step is below the default sleep threshold. Keep this single prop awake.
	block.can_sleep = false
	block.collision_layer = 1
	block.collision_mask = 1
	block.continuous_cd = true
	block.linear_damp = .15
	block.angular_damp = .5
	block.physics_material_override = PhysicsMaterial.new()
	block.physics_material_override.friction = .7
	block.physics_material_override.bounce = .02
	block.set_meta("cloud_color", Color(.94,.08,.52))
	add_child(block)
	var shape := BoxShape3D.new()
	shape.size = Vector3.ONE * BLOCK_SIZE
	shape.margin = .001
	var collider := CollisionShape3D.new()
	collider.shape = shape
	block.add_child(collider)
	var visual := MeshInstance3D.new()
	var cube := BoxMesh.new()
	cube.size = Vector3.ONE * BLOCK_SIZE
	visual.mesh = cube
	visual.material_override = _material(Color(.94,.08,.52))
	block.add_child(visual)


func _build_ui() -> void:
	var canvas := CanvasLayer.new()
	_canvas = canvas
	add_child(canvas)
	var panel := PanelContainer.new()
	_controls_panel = panel
	panel.position = Vector2(16,16)
	var style := StyleBoxFlat.new()
	style.bg_color = Color(.06,.10,.14,.90)
	style.content_margin_left = 14
	style.content_margin_right = 14
	style.content_margin_top = 10
	style.content_margin_bottom = 10
	style.set_corner_radius_all(7)
	panel.add_theme_stylebox_override("panel",style)
	canvas.add_child(panel)
	var column := VBoxContainer.new()
	panel.add_child(column)
	var title := Label.new()
	title.text = "SIMULATION · alignment lab"
	title.add_theme_font_size_override("font_size",22)
	title.add_theme_color_override("font_color",Color(.5,1,.84))
	column.add_child(title)
	_phase = Label.new()
	_phase.text = "Grasp the pink block. Check depth from the side. Drop into the bowl."
	column.add_child(_phase)
	_status = Label.new()
	_status.add_theme_font_size_override("font_size",13)
	column.add_child(_status)
	var buttons := HBoxContainer.new()
	column.add_child(buttons)
	_add_button(buttons, "Replay [P]", start_replay)
	_add_button(buttons, "Reset [R]", reset_demo)
	_add_button(buttons, "Front [1]", func(): set_view_angle(0.0))
	_add_button(buttons, "Side [2]", func(): set_view_angle(-.75))
	_add_button(buttons, "Cloud / mesh [C]", toggle_render_mode)
	var help := Label.new()
	help.text = "WASD: move tool   Q/E: lower/raise   Space: grip/release\nRight drag: view   Wheel: zoom   H: recenter head\nSynthetic depth from one fixed virtual sensor. No follower commands."
	help.add_theme_font_size_override("font_size",13)
	column.add_child(help)


func _add_button(parent: Node, label: String, action: Callable) -> void:
	var button := Button.new()
	button.text = label
	button.pressed.connect(action)
	parent.add_child(button)


func _physics_process(delta: float) -> void:
	_elapsed += delta
	if replay_active:
		_advance_replay(delta)
	elif not _external_arm_active:
		_keyboard_move(delta)
	if _external_arm_active and not input_bridge.is_fresh():
		_external_arm_active = false
		input_mode = "Input stale — virtual arm held"
		_target = arm.tool_transform().origin
		# Retain pose/grasp. Never extrapolate a missing leader sample.
	if _head_active and not input_bridge.is_fresh():
		_head_active = false
	if held:
		var next: Transform3D = arm.tool_transform() * _held_local
		_held_velocity = (next.origin - _last_held_position) / maxf(delta,.0001)
		block.global_transform = next
		_last_held_position = next.origin
	completed = not held and Vector2(block.position.x-BOWL_CENTER.x, block.position.z-BOWL_CENTER.z).length() < .037 and block.position.y > .014 and block.position.y < .065 and block.linear_velocity.length() < .10
	_update_camera()
	_update_ui()
	if not _capture_path.is_empty() and _elapsed >= _capture_time and not _capture_requested:
		_capture_requested = true
		call_deferred("_capture_view")


func _keyboard_move(delta: float) -> void:
	var move := Vector3(float(Input.is_physical_key_pressed(KEY_D))-float(Input.is_physical_key_pressed(KEY_A)), float(Input.is_physical_key_pressed(KEY_E))-float(Input.is_physical_key_pressed(KEY_Q)), float(Input.is_physical_key_pressed(KEY_S))-float(Input.is_physical_key_pressed(KEY_W)))
	if move.length_squared() == 0:
		return
	input_mode = "Keyboard"
	_target += move.normalized() * .12 * delta
	_target.x = clampf(_target.x,-.23,.33)
	_target.y = clampf(_target.y,.024,.38)
	_target.z = clampf(_target.z,-.31,.20)
	arm.move_tool_to(_target)


func set_gripper(opening: float) -> void:
	var was_open := _gripper > 28.0
	_gripper = clampf(opening,0,100)
	arm.set_gripper(_gripper)
	if held and _gripper > 45.0:
		held = false
		block.freeze = false
		block.sleeping = false
		block.linear_velocity = _held_velocity.limit_length(.3)
		block.angular_velocity = Vector3.ZERO
	elif not held and was_open and _gripper <= 28.0:
		try_grasp()


func try_grasp() -> bool:
	if held or _gripper > 28.0 or arm.tool_transform().origin.distance_to(block.global_position) > .039:
		return false
	held = true
	completed = false
	block.freeze = true
	block.linear_velocity = Vector3.ZERO
	block.angular_velocity = Vector3.ZERO
	_held_local = arm.tool_transform().affine_inverse() * block.global_transform
	_last_held_position = block.global_position
	_held_velocity = Vector3.ZERO
	return true


func reset_demo() -> void:
	replay_active = false
	_external_arm_active = false
	held = false
	completed = false
	block.freeze = true
	block.global_transform = Transform3D(Basis.IDENTITY, BLOCK_START)
	block.linear_velocity = Vector3.ZERO
	block.angular_velocity = Vector3.ZERO
	block.freeze = false
	block.sleeping = false
	_gripper = 100
	arm.set_normalized([-40.4296875,130.0,38.0,78.0,0.0,100.0])
	_target = BLOCK_START + Vector3(0,.11,0)
	arm.move_tool_to(_target,160)
	input_mode = "Keyboard"


func start_replay() -> void:
	reset_demo()
	replay_active = true
	replay_time = 0.0
	_replay_start_tool = arm.tool_transform().origin
	_replay_block = BLOCK_START
	input_mode = "Deterministic replay (virtual only)"
	_head_active = false
	_head_displacement = Vector3.ZERO
	set_view_angle(0)


func _advance_replay(delta: float) -> void:
	replay_time += delta
	var t := replay_time
	var above_start := BLOCK_START + Vector3(0,.15,0)
	var ambiguous := BOWL_CENTER + Vector3(0,.17,.09)
	var aligned := BOWL_CENTER + Vector3(0,.17,0)
	if t < 2.0:
		_target = _replay_start_tool.lerp(_replay_block, smoothstep(0,2,t))
		arm.move_tool_to(_target,18)
	elif t < 2.7:
		arm.move_tool_to(_replay_block,18)
		set_gripper(0)
	elif t < 4.7:
		arm.move_tool_to(_replay_block.lerp(above_start,smoothstep(2.7,4.7,t)),18)
	elif t < 6.7:
		arm.move_tool_to(above_start.lerp(ambiguous,smoothstep(4.7,6.7,t)),18)
	elif t < 9.7:
		arm.move_tool_to(ambiguous,18)
		if not _fixed_view:
			_orbit_yaw = lerpf(0,-.75,smoothstep(7.2,8.7,t))
	elif t < 11.7:
		arm.move_tool_to(ambiguous.lerp(aligned,smoothstep(9.7,11.7,t)),18)
	elif t < 13.2:
		arm.move_tool_to(aligned,18)
	elif t < 14.0:
		set_gripper(100)
	elif t < 16.0:
		arm.move_tool_to(aligned + Vector3(.10,.06,.07)*smoothstep(14,16,t),18)
	else:
		replay_active = false
		_target = arm.tool_transform().origin
		input_mode = "Replay finished — keyboard available"


func apply_simulation_packet(payload: Dictionary) -> bool:
	# Every public caller passes the same validation as the UDP receive path.
	return input_bridge.accept_packet(payload)


func _apply_accepted_packet(payload: Dictionary) -> void:
	var had_external_arm := _external_arm_active
	var action: String = str(payload.get("action",""))
	if action == "reset": reset_demo()
	if action == "replay": start_replay()
	if action == "recenter": recenter_head()
	var head: Dictionary = payload.get("head",{})
	_head_active = bool(head.get("active",false))
	if _head_active:
		var point := Vector3(float(head.x),float(head.y),float(head.z)) * .01
		if not _head_neutral_set:
			_head_neutral = point
			_head_neutral_set = true
		_head_displacement = (point-_head_neutral).clamp(Vector3(-.35,-.25,-.35),Vector3(.35,.25,.35))
	var arm_sample: Dictionary = payload.get("arm",{})
	_external_arm_active = bool(arm_sample.get("active",false))
	if _external_arm_active:
		replay_active = false
		arm.set_normalized(arm_sample.normalized)
		set_gripper(float(arm_sample.normalized[5]))
		_target = arm.tool_transform().origin
		input_mode = "Local leader / manual joint input (virtual arm only)"
	elif had_external_arm and action not in ["reset","replay"]:
		_target = arm.tool_transform().origin
		input_mode = "Keyboard · external arm input inactive"


func recenter_head() -> void:
	_head_neutral_set = false
	_head_displacement = Vector3.ZERO


func set_view_angle(yaw: float) -> void:
	_orbit_yaw = yaw
	_orbit_pitch = .27
	_update_camera()


func _update_camera() -> void:
	var offset := Vector3(sin(_orbit_yaw)*cos(_orbit_pitch),sin(_orbit_pitch),cos(_orbit_yaw)*cos(_orbit_pitch)) * _orbit_distance
	offset += Vector3(_head_displacement.x*1.6,_head_displacement.y*1.4,_head_displacement.z)
	camera.position = _view_target + offset
	camera.look_at(_view_target,Vector3.UP)


func toggle_render_mode() -> void:
	point_cloud_mode = not point_cloud_mode
	_set_render_mode()


func _set_render_mode() -> void:
	camera.cull_mask = 2 if point_cloud_mode else 1
	cloud.enabled = point_cloud_mode
	cloud.visible = point_cloud_mode


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion and event.button_mask & MOUSE_BUTTON_MASK_RIGHT:
		_orbit_yaw -= event.relative.x*.005
		_orbit_pitch = clampf(_orbit_pitch+event.relative.y*.004,-.05,1.0)
	if event is InputEventMouseButton and event.pressed:
		if event.button_index == MOUSE_BUTTON_WHEEL_UP: _orbit_distance = maxf(.35,_orbit_distance-.04)
		if event.button_index == MOUSE_BUTTON_WHEEL_DOWN: _orbit_distance = minf(1.5,_orbit_distance+.04)
	if not event is InputEventKey or not event.pressed or event.echo:
		return
	match event.physical_keycode:
		KEY_R: reset_demo()
		KEY_P: start_replay()
		KEY_C: toggle_render_mode()
		KEY_H: recenter_head()
		KEY_1: set_view_angle(0)
		KEY_2: set_view_angle(-.75)
		KEY_SPACE:
			replay_active = false
			_external_arm_active = false
			set_gripper(100 if _gripper < 45 else 0)


func _update_ui() -> void:
	var result := "PLACED" if completed else ("Block held" if held else "Block free")
	_phase.text = "Check alignment from the front and side · %s" % result
	var mode := "Synthetic depth cloud" if point_cloud_mode else "Solid geometry (reference)"
	_status.text = "%s · %s\n%s · %s" % [mode,input_mode,input_bridge.diagnostic,"Head input active" if _head_active else "Mouse / preset view"]


func get_demo_state() -> Dictionary:
	return {"simulation":true,"hardware_commands":0,"held":held,"completed":completed,
		"block_position":[block.position.x,block.position.y,block.position.z],
		"block_velocity":[block.linear_velocity.x,block.linear_velocity.y,block.linear_velocity.z],
		"block_frozen":block.freeze,"block_sleeping":block.sleeping,
		"tool_position":[arm.tool_transform().origin.x,arm.tool_transform().origin.y,arm.tool_transform().origin.z],
		"bowl_center":[BOWL_CENTER.x,BOWL_CENTER.y,BOWL_CENTER.z],"gripper":_gripper,
		"replay":replay_active,"replay_time":replay_time,"input_mode":input_mode,
		"synthetic_points":cloud.sample_count,"point_cloud_mode":point_cloud_mode,
		"input_bound":input_bridge.bound,"accepted_packets":input_bridge.accepted_packets}


func _capture_view() -> void:
	if DisplayServer.get_name() == "headless":
		printerr("Simulation capture requires a rendered display; headless logic is not visual proof.")
		get_tree().quit(2)
		return
	await RenderingServer.frame_post_draw
	var image := get_viewport().get_texture().get_image()
	var error := image.save_png(_capture_path)
	print("SIMULATION_CAPTURE ",error," ",JSON.stringify(get_demo_state()))
	var file := FileAccess.open(_capture_path+".json",FileAccess.WRITE)
	if file != null:
		file.store_string(JSON.stringify(get_demo_state(),"  "))
	get_tree().quit(0 if error == OK else 2)
