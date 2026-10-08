@tool
extends Window

const FILE_FORMAT := preload("res://robot_modules/so101/godot/so101_calibration_file.gd")
const CONTROLS := {
	"manual_shoulder_lift_trim_degrees": ["Shoulder lift (J2)", -180.0, 180.0, 0.0],
	"manual_elbow_flex_trim_degrees": ["Elbow bend (J3)", -180.0, 180.0, 0.0],
	"manual_wrist_flex_trim_degrees": ["Wrist bend (J4)", -90.0, 90.0, 0.0],
	"manual_wrist_roll_trim_degrees": ["Wrist rotation (J5)", -180.0, 180.0, 0.0],
	"manual_opening_offset_degrees": ["Claw opening offset", -35.0, 35.0, 0.0],
	"manual_opening_scale": ["Claw opening scale", 0.5, 1.5, 1.0],
}
var robot_module: Node
var _fields: Dictionary = {}
var _message: Label
var _restore_base: CheckBox
var _file_dialog: FileDialog
var _export_file: Dictionary = {}
var _preview_started := false


func _ready() -> void:
	title = "Manual joint tuning / Calibration files"
	size = Vector2i(600, 660)
	min_size = Vector2i(480, 480)
	visible = false
	close_requested.connect(_close)
	var scroll := ScrollContainer.new()
	scroll.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	add_child(scroll)
	var margin := MarginContainer.new()
	margin.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	for side in ["left", "top", "right", "bottom"]:
		margin.add_theme_constant_override("margin_" + side, 16)
	scroll.add_child(margin)
	var box := VBoxContainer.new()
	box.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	box.add_theme_constant_override("separation", 12)
	margin.add_child(box)
	var help := Label.new()
	help.text = "Adjust the overlay to match the real arm. Base placement stays fixed.\nOffsets are relative to the saved calibration; the arm is not commanded."
	help.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	box.add_child(help)
	for key in CONTROLS:
		var row := HBoxContainer.new()
		box.add_child(row)
		var label := Label.new()
		label.text = CONTROLS[key][0]
		label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
		row.add_child(label)
		var input := SpinBox.new()
		input.min_value = CONTROLS[key][1]
		input.max_value = CONTROLS[key][2]
		input.value = CONTROLS[key][3]
		input.step = 0.005 if key == "manual_opening_scale" else 0.1
		input.suffix = "x" if key == "manual_opening_scale" else "deg"
		input.custom_minimum_size.x = 180
		input.value_changed.connect(_preview.bind(key))
		row.add_child(input)
		_fields[key] = input
	var actions := HBoxContainer.new()
	box.add_child(actions)
	_button(actions, "Reset preview", _reset)
	_button(actions, "Cancel preview", _cancel)
	_button(actions, "Save calibration", _save)
	_message = Label.new()
	_message.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	box.add_child(_message)
	var file_help := Label.new()
	file_help.text = "Export JSON makes a portable copy for this arm and motor profile.\nOn another device, keep its current base unless the physical placement and camera alignment match the saved setup."
	file_help.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	box.add_child(file_help)
	_restore_base = CheckBox.new()
	_restore_base.text = "Restore saved base placement when importing"
	box.add_child(_restore_base)
	var files := HBoxContainer.new()
	box.add_child(files)
	_button(files, "Export JSON", _export)
	_button(files, "Import JSON", _import)
	_file_dialog = FileDialog.new()
	_file_dialog.access = FileDialog.ACCESS_FILESYSTEM
	_file_dialog.filters = PackedStringArray(["*.json ; Visual calibration JSON"])
	_file_dialog.file_selected.connect(_file_selected)
	add_child(_file_dialog)


func _button(parent: Node, label: String, callback: Callable) -> void:
	var button := Button.new()
	button.text = label
	button.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	button.pressed.connect(callback)
	parent.add_child(button)


func open_panel() -> void:
	popup_centered()
	_message.text = "Adjust a joint to start a preview, or import an existing JSON file."


func _action(operation: String, extra: Dictionary = {}) -> Dictionary:
	var action := {"operation": operation, "request_id": "godot-%d-%d" % [get_instance_id(), Time.get_ticks_usec()]}
	action.merge(extra)
	var settings := {}
	for key in _fields:
		settings[key] = _fields[key].value
	var result: Dictionary = robot_module.call("manual_calibration_action", action, settings)
	_message.text = str(result.get("message", "No response from the calibration module."))
	return result


func _preview(_value: float, _key: String) -> void:
	if not _preview_started:
		_preview_started = bool(_action("begin").get("ok", false))
	if _preview_started:
		_action("preview")


func _reset_fields() -> void:
	for key in _fields:
		_fields[key].set_value_no_signal(CONTROLS[key][3])


func _reset() -> void:
	if bool(_action("reset").get("ok", false)):
		_reset_fields()
		_preview_started = true


func _cancel() -> bool:
	if _preview_started:
		if not bool(_action("cancel").get("ok", false)):
			return false
	_reset_fields()
	_preview_started = false
	return true


func _close() -> void:
	if _cancel():
		hide()


func _save() -> void:
	if bool(_action("save").get("ok", false)):
		_reset_fields()
		_preview_started = false


func _files_ready() -> bool:
	if _preview_started:
		_message.text = "Save or cancel your preview before importing or exporting."
		return false
	return true


func _export() -> void:
	if not _files_ready():
		return
	var result := _action("export")
	if not bool(result.get("ok", false)):
		return
	_export_file = result.file
	_file_dialog.file_mode = FileDialog.FILE_MODE_SAVE_FILE
	_file_dialog.current_file = "so101-visual-calibration.json"
	_file_dialog.popup_centered_ratio(0.7)


func _import() -> void:
	if not _files_ready():
		return
	_file_dialog.file_mode = FileDialog.FILE_MODE_OPEN_FILE
	_file_dialog.popup_centered_ratio(0.7)


func _file_selected(path: String) -> void:
	if _file_dialog.file_mode == FileDialog.FILE_MODE_SAVE_FILE:
		var file := FileAccess.open(path, FileAccess.WRITE)
		if file == null:
			_message.text = "Could not write the export file."
			return
		file.store_string(JSON.stringify(_export_file, "\t"))
		file.flush()
		_message.text = "Exported " + path if file.get_error() == OK else "Could not finish writing the export file."
		file.close()
	else:
		var file := FileAccess.open(path, FileAccess.READ)
		if file == null or file.get_length() > FILE_FORMAT.MAX_BYTES:
			_message.text = "Choose a readable visual calibration JSON file smaller than 16 KB."
			return
		var data = JSON.parse_string(file.get_as_text())
		file.close()
		_action("import", {"file": data, "restore_base": _restore_base.button_pressed})
