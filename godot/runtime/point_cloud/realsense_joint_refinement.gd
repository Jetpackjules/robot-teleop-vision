extends RefCounted
## Read-only capture/optimization worker. The owning rig transaction saves poses.

const WORKER := "res://tools/realsense_joint_refine.py"
var status := "Joint refinement: waiting for live capture to resume. Keep the scene still."
var folder := ""
var request_id := ""
var previous_registry := PackedByteArray()
var pid := -1
var started_msec := 0
var last_progress_msec := 0
var launched := false
var python := ""

func begin(view, rig, registry: PackedByteArray) -> bool:
	previous_registry = registry
	request_id = "%d_%d" % [OS.get_process_id(), Time.get_ticks_usec()]
	folder = ProjectSettings.globalize_path("user://camera_joint_refinement/" + request_id)
	if not FileAccess.file_exists(WORKER) or DirAccess.make_dir_recursive_absolute(folder) != OK:
		status = "Joint refinement worker unavailable; previous alignment preserved."
		return false
	var ids: Array[String] = [rig.reference_id]
	for camera in rig.cameras:
		if camera != rig.reference_id: ids.append(camera)
	var serials: Array[String] = []
	var initial: Dictionary = {}
	var previous: Dictionary = {}
	for camera in ids:
		var serial: String = view._realsense_serial(camera)
		serials.append(serial)
		initial[serial] = view._transform_to_payload(rig.transforms[camera])
		if view._alignment_transforms.has(camera):
			previous[serial] = view._transform_to_payload(view._alignment_transforms[camera])
	var source_port := 8780
	if view._shared_capture_preview():
		var configured := OS.get_environment("ROBOT_TELEOP_CAPTURE_PORT")
		source_port = 8781 if configured.is_empty() else int(configured)
	else:
		for node in view.get_tree().root.find_children("*", "", true, false):
			var script = node.get_script()
			if script is Script and script.resource_path == "res://godot/runtime/streaming/viewport_stream_server.gd":
				source_port = int(node.port)
				break
	var request := {
		"request_id": request_id, "cameras": serials, "reference_camera": serials[0],
		"initial_transforms": initial, "previous_transforms": previous,
		# No query parameters: use the live native profile without changing stream settings.
		"source_url": "http://127.0.0.1:%d/rgbd" % source_port,
		"model_path": ProjectSettings.globalize_path(view.REALSENSE_NATIVE_CALIBRATION_MODEL_PATH),
		"min_depth_m": view.min_depth_m, "max_depth_m": view.realsense_calibration_max_depth_m,
	}
	var file := FileAccess.open(folder + "/request.json", FileAccess.WRITE)
	if file == null:
		status = "Could not stage joint refinement; previous alignment preserved."
		return false
	file.store_string(JSON.stringify(request, "\t"))
	file.close()
	python = ProjectSettings.globalize_path("res://.venv/Scripts/python.exe" if OS.has_feature("windows") else "res://.venv/bin/python")
	if not FileAccess.file_exists(python): python = view._python_executable()
	started_msec = Time.get_ticks_msec()
	return true

func poll() -> Variant:
	var elapsed := Time.get_ticks_msec() - started_msec
	if elapsed > 240000:
		cancel()
		return {"ok": false, "status": "Joint refinement timed out; previous alignment preserved."}
	if not launched:
		if elapsed < 2000: return null
		launched = true
		pid = OS.create_process(python, PackedStringArray([ProjectSettings.globalize_path(WORKER), "--request", folder + "/request.json"]), false)
		if pid <= 0:
			return {"ok": false, "status": "Could not launch joint refinement Python worker; previous alignment preserved."}
	if OS.is_process_running(pid):
		if Time.get_ticks_msec() - last_progress_msec > 500:
			last_progress_msec = Time.get_ticks_msec()
			var progress = JSON.parse_string(FileAccess.get_file_as_string(folder + "/progress.json")) if FileAccess.file_exists(folder + "/progress.json") else {}
			if progress is Dictionary: status = str(progress.get("status", status))
		return null
	pid = -1
	var result = JSON.parse_string(FileAccess.get_file_as_string(folder + "/result.json")) if FileAccess.file_exists(folder + "/result.json") else null
	if not result is Dictionary or str(result.get("request_id", "")) != request_id:
		return {"ok": false, "status": "Joint refinement returned no valid result. Install Python calibration dependencies; previous alignment preserved."}
	return result

func backup_registry() -> bool:
	var file := FileAccess.open(folder + "/registry-before.json", FileAccess.WRITE)
	if file == null: return false
	file.store_buffer(previous_registry)
	file.flush()
	var error := file.get_error()
	file.close()
	return error == OK

func cancel() -> void:
	if pid > 0 and OS.is_process_running(pid): OS.kill(pid)
	pid = -1
