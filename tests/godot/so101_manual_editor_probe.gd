@tool
extends EditorPlugin

const RobotModule := preload("res://robot_modules/so101/godot/so101_module.gd")
var failures: Array[String] = []
var checks := 0


func _enter_tree() -> void:
	call_deferred("_run")


func check(condition: bool, label: String) -> void:
	checks += 1
	if not condition:
		failures.append(label)
		print("FAIL: " + label)


func _find_button(node: Node) -> Button:
	if node is Button and node.text == "Manual Joint Tuning / Calibration Files":
		return node
	for child in node.get_children():
		var button := _find_button(child)
		if button != null:
			return button
	return null


func _run() -> void:
	# A real editor scene and Inspector, but no camera, arm, or operator data.
	await get_tree().process_frame
	EditorInterface.open_scene_from_path("res://test_view.tscn")
	var view: Node = null
	for attempt in range(120):
		await get_tree().process_frame
		view = EditorInterface.get_edited_scene_root()
		if view != null and view.has_method("_open_manual_robot_calibration"):
			break
	check(view != null, "test scene opens in the editor")
	if view == null:
		_finish()
		return
	var module := RobotModule.new()
	view.add_child(module)
	await get_tree().process_frame
	check(module._overlay == null and module._calibrator == null, "hardware stays disabled")
	EditorInterface.edit_node(view)
	var button: Button = null
	for attempt in range(120):
		await get_tree().process_frame
		button = _find_button(EditorInterface.get_inspector())
		if button != null:
			break
	check(button != null, "real Inspector exposes the tuning button")
	if button == null:
		module.free()
		_finish()
		return
	button.pressed.emit()
	await get_tree().process_frame
	var panel: Window = module._manual_panel
	check(is_instance_valid(panel), "Inspector button creates the tuning panel")
	if not is_instance_valid(panel):
		module.free()
		_finish()
		return
	check(panel.visible, "tuning panel is shown")
	check(not view.is_ancestor_of(panel), "panel is outside the edited scene")
	check(panel.get_parent() == EditorInterface.get_base_control(), "panel belongs to the editor UI")
	check(panel.get_parent().get_window() == EditorInterface.get_base_control().get_window(), "panel belongs to the editor window")
	check(panel.transient and not panel.exclusive, "tuning panel stays with editor without blocking scene interaction")
	check(panel._fields.size() == 6, "joint controls are initialized")
	check(is_equal_approx(panel._fields.manual_opening_scale.value, 1.0), "opening scale starts without changing calibration")
	if DisplayServer.get_name() != "headless":
		await get_tree().create_timer(0.5).timeout
		var screenshot: Image = panel.get_texture().get_image()
		check(screenshot != null and not screenshot.is_empty(), "tuning window renders pixels")
		if screenshot != null and not screenshot.is_empty():
			screenshot.save_png("res://manual-tuning-editor.png")
	panel._import()
	check(panel._file_dialog.visible, "Import opens its file chooser")
	check(panel._file_dialog.get_parent().get_window() == panel, "file chooser stays attached to the tuning window")
	panel._file_dialog.hide()
	panel.close_requested.emit()
	check(not panel.visible, "close hides the panel")
	button.pressed.emit()
	await get_tree().process_frame
	check(module._manual_panel == panel and panel.visible, "Inspector button reopens the same panel")
	module.free()
	await get_tree().process_frame
	await get_tree().process_frame
	check(not is_instance_valid(panel), "removing the robot module also removes its editor panel")
	view.call("_open_manual_robot_calibration")
	check("unavailable" in str(view.get("robot_position_calibration_status")), "missing module reports why tuning cannot open")
	_finish()


func _finish() -> void:
	print("SO101_MANUAL_EDITOR_PROBE checks=%d failures=%d" % [checks, failures.size()])
	get_tree().quit(0 if failures.is_empty() else 1)
