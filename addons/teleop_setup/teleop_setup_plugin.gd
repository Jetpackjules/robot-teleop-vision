@tool
extends EditorPlugin

var _dock: VBoxContainer
var _status_label: Label
var _output: RichTextLabel
var _timer: Timer
var _python := ""


func _enter_tree() -> void:
	_python = _find_python()
	_dock = VBoxContainer.new()
	_dock.name = "Teleop Setup"

	var title := Label.new()
	title.text = "Robot Teleop Vision"
	title.add_theme_font_size_override("font_size", 18)
	_dock.add_child(title)

	var description := Label.new()
	description.text = "Setup and diagnostics live here.\nCalibration, motion, and view controls live in the operator website."
	description.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_dock.add_child(description)

	_status_label = Label.new()
	_status_label.text = "Checking runtime…"
	_dock.add_child(_status_label)

	_add_button("1. Create Local Config", _initialize)
	_add_button("2. Run Doctor + Discover Devices", _doctor)
	_add_button("3. Start Local Runtime", _start_runtime)
	_add_button("Open Operator Website", _open_operator)
	_add_button("Safely Stop Runtime", _stop_runtime)

	_output = RichTextLabel.new()
	_output.fit_content = false
	_output.custom_minimum_size = Vector2(280, 220)
	_output.scroll_active = true
	_output.bbcode_enabled = false
	_output.text = "Open README.md for first-run setup."
	_dock.add_child(_output)

	add_control_to_dock(DOCK_SLOT_LEFT_BR, _dock)
	_timer = Timer.new()
	_timer.wait_time = 1.0
	_timer.autostart = true
	_timer.timeout.connect(_refresh_status)
	_dock.add_child(_timer)
	_refresh_status()


func _exit_tree() -> void:
	if _dock != null:
		remove_control_from_docks(_dock)
		_dock.queue_free()


func _add_button(label: String, callback: Callable) -> void:
	var button := Button.new()
	button.text = label
	button.pressed.connect(callback)
	_dock.add_child(button)


func _find_python() -> String:
	for candidate in ["python3", "python"]:
		var output: Array = []
		if OS.execute(candidate, PackedStringArray(["--version"]), output, true) == 0:
			return candidate
	return ""


func _wrapper_path() -> String:
	return ProjectSettings.globalize_path("res://scripts/robot_teleop.py")


func _run_sync(arguments: PackedStringArray) -> int:
	if _python.is_empty():
		_output.text = "Python 3 was not found. Install Python 3.11 or newer and reopen Godot."
		return ERR_CANT_FORK
	var output: Array = []
	var command := PackedStringArray([_wrapper_path()])
	command.append_array(arguments)
	var code := OS.execute(_python, command, output, true)
	_output.text = "\n".join(output) + "\nExit status: %d" % code
	_refresh_status()
	return code


func _run_async(arguments: PackedStringArray) -> int:
	if _python.is_empty():
		_output.text = "Python 3 was not found. Install Python 3.11 or newer and reopen Godot."
		return -1
	var command := PackedStringArray([_wrapper_path()])
	command.append_array(arguments)
	var pid := OS.create_process(_python, command)
	_output.text = "Launcher started (PID %d). Runtime status will update here." % pid
	return pid


func _initialize() -> void:
	_run_sync(PackedStringArray(["init", "--example", "so101_realsense"]))


func _doctor() -> void:
	_run_sync(PackedStringArray(["doctor"]))


func _start_runtime() -> void:
	_run_async(PackedStringArray(["start"]))


func _open_operator() -> void:
	_run_sync(PackedStringArray(["open"]))


func _stop_runtime() -> void:
	_run_sync(PackedStringArray(["stop"]))


func _refresh_status() -> void:
	var path := ProjectSettings.globalize_path("res://.teleop/run.json")
	if not FileAccess.file_exists(path):
		_status_label.text = "Runtime: stopped"
		return
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		_status_label.text = "Runtime: unknown"
		return
	var value = JSON.parse_string(file.get_as_text())
	if value is Dictionary:
		var status := str((value as Dictionary).get("status", "unknown"))
		var url := str((value as Dictionary).get("public_url", ""))
		if url.is_empty():
			url = str((value as Dictionary).get("local_url", ""))
		_status_label.text = "Runtime: %s%s" % [status, "\n" + url if not url.is_empty() else ""]
	else:
		_status_label.text = "Runtime: unreadable state"
