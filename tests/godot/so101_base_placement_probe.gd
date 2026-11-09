extends SceneTree

const Overlay := preload("res://robot_modules/so101/godot/so101_robot_overlay.gd")
var checks := 0
var failures: Array[String] = []
var sequence := 0

func check(value: bool, label: String) -> void:
	checks += 1
	if not value:
		failures.append(label)
		print("FAIL " + label)

func action(arm: Node3D, operation: String, values: Dictionary = {}, identifier: String = "") -> Dictionary:
	sequence += 1
	var request := {"operation": operation, "request_id": identifier if not identifier.is_empty() else str(sequence), "session_id": arm._manual_base_session_id}
	request.merge(values, true)
	return arm.handle_manual_calibration_action(request)

func _initialize() -> void:
	call_deferred("run")

func run() -> void:
	# Detached nodes never start cameras, bind telemetry, or connect to motors.
	var arm := Overlay.new()
	arm._registration_path = "res://base.json"
	arm._registration_status = {"registered": true, "confidence": 0.72, "registration_source": "previous_fit", "calibrated_through_joint": 4}
	arm.transform = Transform3D(Basis(Vector3.UP, 0.3) * Basis(Vector3.RIGHT, 0.02), Vector3(0.2, 0.04, -0.4))
	var original: Transform3D = arm.transform
	var joints: PackedFloat32Array = arm.joint_angle_offsets_degrees.duplicate()
	check(arm._save_registration(), "initial registration saved")
	var original_file := FileAccess.get_file_as_string(arm._registration_path)
	var change := {"offset_x": 0.04, "offset_y": 0.03, "offset_z": -0.025, "yaw_degrees": 25.0}
	check(action(arm, "base_begin").ok, "begin after automatic calibration")
	var old_session: String = arm._manual_base_session_id
	check(action(arm, "base_preview", change).ok, "preview accepted")
	check(arm.position.is_equal_approx(original.origin + Vector3(0.04, 0.03, -0.025)), "translation includes manually requested height")
	check(arm.get_manual_calibration_status().render_state.has("pose_unix_ms"), "pose channel has a source timestamp")
	check(arm.get_manual_calibration_status().render_state.parents.get("upper_arm_link") == "shoulder_link", "pose channel supplies the actual joint hierarchy")
	check(arm.basis.is_equal_approx(Basis(Vector3.UP, deg_to_rad(25.0)) * original.basis), "yaw preserves original tilt and scale")
	check(arm.joint_angle_offsets_degrees == joints, "joint offsets unchanged")
	check(FileAccess.get_file_as_string(arm._registration_path) == original_file, "preview never writes calibration")
	check(not action(arm, "begin").ok, "joint tuning blocked during base preview")
	check(not action(arm, "base_preview", {"offset_x": NAN, "offset_z": 0, "yaw_degrees": 0}).ok, "non-finite input rejected")
	check(not action(arm, "base_preview", {"offset_x": 0, "offset_y": NAN, "offset_z": 0, "yaw_degrees": 0}).ok, "non-finite height rejected")
	check(action(arm, "base_reset").ok and arm.transform.is_equal_approx(original), "reset restores starting pose")
	action(arm, "base_preview", change)
	check(action(arm, "base_cancel").ok and arm.transform.is_equal_approx(original), "cancel restores full pose")
	check(FileAccess.get_file_as_string(arm._registration_path) == original_file, "cancel leaves saved bytes unchanged")
	action(arm, "base_begin")
	check(not action(arm, "base_cancel", {"session_id": old_session}).ok, "stale session cannot cancel new edit")
	check(action(arm, "base_save", change, "save-once").ok, "save includes final pose without waiting for preview")
	var saved_pose: Transform3D = arm.transform
	check(not arm.manual_base_calibration_enabled, "save ends preview")
	check(arm._registration_status.confidence == 0.0 and arm._registration_status.registration_source == "manual_base_placement", "manual placement is not automatic validation")
	check(action(arm, "base_save", change, "save-once").ok and arm.transform.is_equal_approx(saved_pose), "duplicate save is idempotent")
	var reload := Overlay.new()
	reload._registration_path = arm._registration_path
	reload._load_registration()
	check(reload.transform.is_equal_approx(saved_pose), "saved placement survives reload")
	check(reload.joint_angle_offsets_degrees == joints, "saved joint calibration preserved")
	var fresh := Overlay.new()
	fresh._registration_path = "res://missing-directory/base.json"
	check(action(fresh, "base_begin").ok, "manual placement also works before automatic registration")
	check(not action(fresh, "base_save", change).ok and fresh.manual_base_calibration_enabled, "failed save leaves preview open")
	check(action(fresh, "base_cancel").ok and fresh.transform.is_equal_approx(Transform3D.IDENTITY), "failed save can be cancelled")
	arm.free()
	reload.free()
	fresh.free()
	print("SO101_BASE_PROBE checks=%d failures=%d" % [checks, failures.size()])
	quit(0 if failures.is_empty() else 1)
