extends Node
## Actual rendered Godot viewport -> fixed local JPEG and state files.
## No renderer substitute, device input, HTTP listener or outgoing transport.

var output_directory := ""
var source: Node
var frame_seq := 0
var _last_capture_msec := -1000
var _last_state: Dictionary = {}
var _available := false


func _ready() -> void:
	if output_directory.is_empty():
		return
	if not output_directory.is_absolute_path():
		output_directory = ProjectSettings.globalize_path("res://" + output_directory)
	output_directory = output_directory.simplify_path()
	if DirAccess.make_dir_recursive_absolute(output_directory) != OK:
		push_error("Cannot create local simulation preview directory")
		return
	if DisplayServer.get_name() == "headless":
		_write_state(false,"Headless mode has no rendered preview")
		return
	_available = true
	RenderingServer.frame_post_draw.connect(_capture_rendered_frame)


func _capture_rendered_frame() -> void:
	var now := Time.get_ticks_msec()
	if not _available or now - _last_capture_msec < 100:
		return
	_last_capture_msec = now
	var rendered := get_viewport().get_texture().get_image()
	if rendered == null or rendered.is_empty():
		return
	# A live, modest-size browser preview; the native viewport stays full size.
	if rendered.get_width() > 960:
		var height := maxi(1,roundi(rendered.get_height()*960.0/rendered.get_width()))
		rendered.resize(960,height,Image.INTERPOLATE_BILINEAR)
	if not _atomic_write("preview.jpg",rendered.save_jpg_to_buffer(.85)):
		return
	frame_seq += 1
	_last_state = source.get_demo_state()
	_write_state(true,"")


func _atomic_write(name: String, bytes: PackedByteArray) -> bool:
	var destination := output_directory.path_join(name)
	var temporary := destination + ".tmp"
	var file := FileAccess.open(temporary,FileAccess.WRITE)
	if file == null:
		return false
	file.store_buffer(bytes)
	file.flush()
	file.close()
	return DirAccess.rename_absolute(temporary,destination) == OK


func _write_state(running: bool, reason: String) -> void:
	var state := _last_state.duplicate(true)
	state["running"] = running
	state["timestamp_ms"] = Time.get_unix_time_from_system()*1000.0
	state["frame_seq"] = frame_seq
	state["reason"] = reason
	_atomic_write("preview-state.json",JSON.stringify(state).to_utf8_buffer())


func _exit_tree() -> void:
	if _available:
		RenderingServer.frame_post_draw.disconnect(_capture_rendered_frame)
		_write_state(false,"Godot simulation closed")
