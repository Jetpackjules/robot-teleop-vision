extends SceneTree

const Overlay := preload("res://robot_modules/so101/godot/so101_robot_overlay.gd")
const CalibrationPanel := preload("res://robot_modules/so101/godot/so101_calibration_panel.gd")

class PanelModule:
	extends Node
	var overlay: Node3D
	func manual_calibration_action(action: Dictionary, settings: Dictionary) -> Dictionary:
		return overlay.handle_manual_calibration_action(action, settings)

var failures: Array[String] = []
var checks := 0
var sequence := 0

func check(condition: bool, label: String) -> void:
	checks += 1
	if not condition:
		failures.append(label)
		print("FAIL: " + label)

func action(node: Node3D, operation: String, settings: Dictionary = {}, extra: Dictionary = {}) -> Dictionary:
	sequence += 1
	var request := {"operation": operation, "request_id": str(sequence)}
	request.merge(extra)
	return node.handle_manual_calibration_action(request, settings)

func fresh(path: String) -> Node3D:
	# Detached: no telemetry socket, meshes, camera, or motor connection.
	var node := Overlay.new()
	node._registration_path = path
	node._registration_status = {"registered": true, "registration_source": "settled_shoulder_pan_revolute_axis", "calibrated_through_joint": 0}
	node.transform = Transform3D(Basis(Vector3.UP, 0.3), Vector3(0.2, 0.04, -0.4))
	return node

func _initialize() -> void:
	call_deferred("_run")

func _run() -> void:
	var arm := fresh("res://saved.json")
	var base: Transform3D = arm.transform
	check(arm._save_registration(), "initial partial base registration saved")
	var initial: String = FileAccess.get_file_as_string(arm._registration_path)
	var angles := []
	for index in range(6):
		angles.append(arm._normalized_joint_angle(index, 10.0))
	var corrections := {"manual_shoulder_lift_trim_degrees": 12.5, "manual_elbow_flex_trim_degrees": -90.0, "manual_wrist_flex_trim_degrees": 45.0, "manual_wrist_roll_trim_degrees": 90.0, "manual_opening_offset_degrees": 2.0}
	check(action(arm, "begin").ok, "partial automatic base permits manual tuning")
	check(action(arm, "preview", corrections).ok, "preview accepted")
	for index in range(1, 5):
		check(not is_equal_approx(arm._normalized_joint_angle(index, 10.0), angles[index]), "joint %d preview changes angle" % index)
	check(arm.transform == base and is_equal_approx(arm._normalized_joint_angle(0, 10.0), angles[0]), "preview preserves base and pan")
	check(FileAccess.get_file_as_string(arm._registration_path) == initial, "preview does not write saved file")
	action(arm, "cancel")
	for index in range(6):
		check(is_equal_approx(arm._normalized_joint_angle(index, 10.0), angles[index]), "cancel restores joint %d" % index)
	action(arm, "begin")
	action(arm, "preview", corrections)
	var preview := []
	for index in range(6):
		preview.append(arm._normalized_joint_angle(index, 10.0))
	var save_request := {"operation": "save", "request_id": "save-once"}
	var saved: Dictionary = arm.handle_manual_calibration_action(save_request, corrections)
	check(saved.ok, "save succeeds: " + str(saved.message))
	check(not arm.manual_claw_calibration_enabled, "save closes preview")
	check(arm._registration_status.registration_source == "manual_visual_joint_tuning", "manual tuning is not automatic evidence")
	check(arm.handle_manual_calibration_action(save_request, corrections).ok, "duplicate save returns previous result")
	for index in range(6):
		check(is_equal_approx(arm._normalized_joint_angle(index, 10.0), preview[index]), "save preserves preview joint %d" % index)
	var reload := fresh(arm._registration_path)
	reload._load_registration()
	for index in range(6):
		check(is_equal_approx(reload._normalized_joint_angle(index, 10.0), preview[index]), "restart preserves joint %d" % index)
	check(reload.transform.is_equal_approx(base), "restart preserves aligned base")
	var exported := action(arm, "export")
	check(exported.ok and exported.has("file"), "export works: " + str(exported.message))
	if not exported.has("file"):
		arm.free()
		reload.free()
		finish()
		return
	# JSON serialization exercises real file types and float precision.
	var portable: Dictionary = JSON.parse_string(JSON.stringify(exported.file, "\t"))
	check(JSON.stringify(portable, "\t").to_utf8_buffer().size() < 16384, "export fits upload limit")
	check(not portable.registration.has("port") and not portable.registration.has("profile"), "portable file contains visual values only")
	var other := fresh("res://other.json")
	other.transform = Transform3D(Basis(Vector3.UP, -0.6), Vector3(-0.3, 0.1, 0.2))
	var other_base: Transform3D = other.transform
	check(action(other, "import", {}, {"file": portable, "restore_base": false}).ok, "import joint calibration on another device")
	check(other.transform.is_equal_approx(other_base), "default import keeps receiving base")
	check(other.joint_angle_offsets_degrees == arm.joint_angle_offsets_degrees, "import transfers all joint offsets")
	check(action(other, "import", {}, {"file": portable, "restore_base": true}).ok, "explicit full placement import")
	check(other.transform.is_equal_approx(base), "full import restores exported base")
	var committed: String = FileAccess.get_file_as_string(other._registration_path)
	for bad in [null, {}, {"type": "so101_robot_registration"}]:
		check(not action(other, "import", {}, {"file": bad}).ok, "invalid format rejected")
	var bad := portable.duplicate(true)
	bad.kinematics_version = -1
	check(not action(other, "import", {}, {"file": bad}).ok, "incompatible model rejected")
	bad = portable.duplicate(true)
	bad.registration.joint_angle_offsets_degrees[2] = NAN
	check(not action(other, "import", {}, {"file": bad}).ok, "nonfinite offset rejected")
	bad = portable.duplicate(true)
	bad.registration.basis_y = [0, -1, 0]
	check(not action(other, "import", {}, {"file": bad}).ok, "reflected base rejected")
	bad = portable.duplicate(true)
	bad.registration["script"] = "never-execute"
	check(not action(other, "import", {}, {"file": bad}).ok, "unknown registration fields rejected")
	check(FileAccess.get_file_as_string(other._registration_path) == committed, "invalid imports leave saved calibration intact")
	var busy: Dictionary = arm.handle_manual_calibration_action({"operation": "begin", "request_id": "busy"}, {}, true)
	check(not busy.ok and not arm.manual_claw_calibration_enabled, "automatic solve excludes manual preview")
	action(arm, "begin")
	var previous: PackedFloat32Array = arm.joint_angle_offsets_degrees.duplicate()
	arm._registration_path = "res://missing-directory/saved.json"
	var failed := action(arm, "save", corrections)
	check(not failed.ok and arm.manual_claw_calibration_enabled, "disk failure leaves preview open")
	check(arm.joint_angle_offsets_degrees == previous, "disk failure rolls back committed mapping")
	arm._registration_path = "res://saved.json"
	action(arm, "cancel")
	arm._registration_status = {"registered": false}
	check(not action(arm, "import", {}, {"file": portable}).ok, "joint-only import needs current base")
	check(action(arm, "import", {}, {"file": portable, "restore_base": true}).ok, "full import bootstraps clean installation")
	var jaw := fresh("res://moving-jaw.json")
	jaw.moving_jaw_calibration_enabled = true
	action(jaw, "begin")
	action(jaw, "preview", {"manual_wrist_roll_direction": -1.0})
	check(jaw.manual_wrist_roll_direction == -1.0, "wrist direction can be previewed")
	action(jaw, "preview", {"manual_wrist_roll_direction": 0.0})
	check(jaw.manual_wrist_roll_direction == jaw._joint_direction(4), "Keep saved direction restores prior direction")
	action(jaw, "preview", corrections)
	var jaw_preview := []
	for position in [0.0, 20.0, 50.0, 80.0, 100.0]:
		jaw_preview.append(jaw._normalized_joint_angle(5, position))
	var jaw_saved := action(jaw, "save", corrections)
	check(jaw_saved.ok, "calibrated moving jaw saves: " + str(jaw_saved.message))
	jaw._load_registration()
	for index in range(jaw_preview.size()):
		check(is_equal_approx(jaw._normalized_joint_angle(5, [0.0, 20.0, 50.0, 80.0, 100.0][index]), jaw_preview[index]), "moving-jaw pose %d survives restart" % index)
	check(action(jaw, "export").ok, "calibrated moving jaw exports")
	jaw.free()
	# Exercise the real Godot controls and signal wiring without starting a robot module.
	var module := PanelModule.new()
	module.overlay = arm
	root.add_child(module)
	var panel := CalibrationPanel.new()
	panel.robot_module = module
	module.add_child(panel)
	panel.open_panel()
	panel._fields.manual_shoulder_lift_trim_degrees.value = 3.5
	check(arm.manual_claw_calibration_enabled and arm.manual_shoulder_lift_trim_degrees == 3.5, "native numeric field drives preview")
	panel._save()
	check(not arm.manual_claw_calibration_enabled, "native Save persists and closes preview")
	panel._export()
	check(not panel._export_file.is_empty(), "native Export obtains portable data")
	panel._file_dialog.hide()
	panel.hide()
	module.free()
	arm.free()
	reload.free()
	other.free()
	finish()

func finish() -> void:
	print("SO101_MANUAL_PROBE checks=%d failures=%d" % [checks, failures.size()])
	quit(0 if failures.is_empty() else 1)
