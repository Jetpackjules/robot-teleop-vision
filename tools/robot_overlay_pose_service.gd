extends SceneTree
## Publish the existing calibrated overlay from measured telemetry only.
## No cameras, control sockets, calibration actions or servo commands are opened.

const MIRROR := "user://robot_overlay_render_state.json"
var overlay: Node3D
var anchor: Node3D
var next_publish := 0
var publisher_id := ""
var reference_base: Dictionary = {}
var checked := false

func _initialize() -> void:
	if DisplayServer.get_name() != "headless":
		quit(1)
		return
	publisher_id = "measured-pose-%d" % OS.get_process_id()
	call_deferred("start")

func start() -> void:
	var previous = JSON.parse_string(FileAccess.get_file_as_string(MIRROR))
	if previous is Dictionary and float(previous.get("updated", 0)) > Time.get_unix_time_from_system() - 2.0:
		push_error("An overlay publisher is already active; left it untouched.")
		quit(1)
		return
	if previous is Dictionary:
		reference_base = previous.get("robot", {}).get("kinematics", {}).get("base_transform", {})
	var level = JSON.parse_string(FileAccess.get_file_as_string("user://unified_world_level.json"))
	if not level is Dictionary or reference_base.is_empty():
		push_error("Saved world level and calibrated overlay reference are required.")
		quit(1)
		return
	anchor = Node3D.new()
	anchor.transform = Transform3D(Basis(vector(level.basis_x), vector(level.basis_y), vector(level.basis_z)).orthonormalized(), vector(level.origin))
	root.add_child(anchor)
	overlay = Node3D.new()
	overlay.set_script(load("res://robot_modules/so101/godot/so101_robot_overlay.gd"))
	overlay.set("telemetry_port", 14250)
	anchor.add_child(overlay)
	if not bool(overlay.call("get_registration_status").get("registered", false)):
		push_error("Saved registration did not load; no pose was published.")
		quit(1)

func vector(values: Array) -> Vector3:
	return Vector3(float(values[0]), float(values[1]), float(values[2]))

func _process(_delta: float) -> bool:
	if overlay == null or Time.get_ticks_msec() < next_publish:
		return false
	next_publish = Time.get_ticks_msec() + 50
	var current = JSON.parse_string(FileAccess.get_file_as_string(MIRROR))
	if checked and current is Dictionary and current.get("publisher", "") != publisher_id:
		print("Another pose publisher took over; stopping the read-only service.")
		return true
	var robot: Dictionary = overlay.call("get_hybrid_render_state")
	if not checked:
		var base: Dictionary = robot.get("kinematics", {}).get("base_transform", {})
		for key in ["basis_x", "basis_y", "basis_z", "origin"]:
			if not base.has(key) or vector(base[key]).distance_to(vector(reference_base[key])) > 0.00001:
				push_error("Calibrated base differs from the previous owner; no pose was published.")
				return true
		print("Calibrated overlay base matches the previous owner; publishing measured poses.")
		checked = true
	var status: Dictionary = overlay.call("get_latest_status")
	var age := Time.get_unix_time_from_system() * 1000.0 - float(status.get("sent_unix_ms", 0)) + float(status.get("last_read_age_ms", 1000))
	if not bool(status.get("follower_connected", false)) or not bool(status.get("measured_feedback_fresh", false)) or age < -100 or age > 350:
		robot = {"available": false}
	var payload := {"project_root": ProjectSettings.globalize_path("res://"), "updated": Time.get_unix_time_from_system(), "publisher": publisher_id, "robot": robot}
	var path := ProjectSettings.globalize_path(MIRROR)
	var file := FileAccess.open(path + ".pose-new", FileAccess.WRITE)
	if file != null:
		file.store_string(JSON.stringify(payload))
		file.close()
		DirAccess.rename_absolute(path + ".pose-new", path)
	return false
