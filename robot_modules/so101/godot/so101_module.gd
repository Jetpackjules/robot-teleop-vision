@tool
extends Node

const OVERLAY_SCRIPT := "res://robot_modules/so101/godot/so101_robot_overlay.gd"
const CALIBRATOR_SCRIPT := "res://robot_modules/so101/godot/so101_motion_calibrator.gd"
const TRANSPORT_CONFIG := preload("res://robot_modules/so101/godot/so101_transport_config.gd")

var _view: Node = null
var _overlay: Node3D = null
var _calibrator: Node = null
var _configuration_error := ""
var _hardware_enabled := true


func _ready() -> void:
	add_to_group("robot_module")
	call_deferred("_install")


func _exit_tree() -> void:
	if is_instance_valid(_calibrator):
		_calibrator.queue_free()
	if is_instance_valid(_overlay):
		_overlay.queue_free()


func _install() -> void:
	var configuration := TRANSPORT_CONFIG.resolve(OS.get_environment(TRANSPORT_CONFIG.ENVIRONMENT_NAME))
	_configuration_error = str(configuration.get("error", ""))
	if not _configuration_error.is_empty():
		push_error(_configuration_error)
		return
	_hardware_enabled = bool(configuration.enabled)
	if not _hardware_enabled:
		return
	var ports: Dictionary = configuration.ports
	_view = get_parent().get_parent()
	if _view == null:
		push_error("SO-101 module requires the shared point-cloud view as its host")
		return
	var world_anchor := _view.get_node_or_null("WorldLevelAnchor") as Node3D
	if world_anchor == null:
		push_error("SO-101 module could not find WorldLevelAnchor")
		return
	_overlay = Node3D.new()
	_overlay.name = "RobotOverlay"
	_overlay.set_script(load(OVERLAY_SCRIPT))
	# Set wiring before add_child: _ready binds the selected telemetry socket.
	_overlay.set("telemetry_port", ports.telemetry_port)
	_overlay.set("editor_telemetry_port", ports.editor_telemetry_port)
	world_anchor.add_child(_overlay)
	_overlay.set("overlay_enabled", true)
	_overlay.set("overlay_transparency", 0.4)
	_overlay.set("mask_scanned_robot", false)

	_calibrator = Node.new()
	_calibrator.name = "RobotCalibrator"
	_calibrator.set_script(load(CALIBRATOR_SCRIPT))
	_calibrator.set("follower_command_port", ports.command_port)
	_calibrator.set("calibration_status_port", configuration.calibration_status_port)
	_view.add_child(_calibrator)
	if _view.has_method("_update_robot_overlay_settings"):
		_view.call("_update_robot_overlay_settings")


func apply_remote_settings(payload: Dictionary) -> void:
	if not is_instance_valid(_overlay) or not is_instance_valid(_calibrator):
		return
	if payload.has("robot_overlay_enabled"):
		_overlay.call("set_overlay_enabled", bool(payload.robot_overlay_enabled))
	if payload.has("robot_overlay_mask_scanned_robot"):
		_overlay.call(
			"set_mask_scanned_robot",
			bool(payload.robot_overlay_mask_scanned_robot),
		)
	elif payload.has("robot_overlay_mask_scanned_arm"):
		# Read the previous shared setting once so existing saved browser state
		# migrates without keeping the legacy name in the core runtime.
		_overlay.call("set_mask_scanned_robot", bool(payload.robot_overlay_mask_scanned_arm))
	var feedback := {}
	var feedback_names := {
		"arm_measured_feedback_enabled": "measured_feedback_enabled",
		"arm_target_ghost_enabled": "target_ghost_enabled",
		"arm_following_error_safety_enabled": "following_error_safety_enabled",
		"arm_freeze_overlay_on_stale_enabled": "freeze_overlay_on_stale_enabled",
		"arm_d455_visual_correction_enabled": "d455_visual_correction_enabled",
	}
	for remote_name in feedback_names:
		if payload.has(remote_name):
			feedback[feedback_names[remote_name]] = bool(payload[remote_name])
	if not feedback.is_empty():
		_overlay.call("configure_feedback_features", feedback)
		_calibrator.call("apply_remote_feedback_settings", feedback)
	if bool(payload.get("calibrate_robot_position", false)):
		start_full_calibration(true)
	if bool(payload.get("refine_robot_joint_alignment", false)):
		refine_calibration(true)
	if bool(payload.get("cancel_robot_position_calibration", false)):
		_calibrator.call("cancel_arm_position_calibration")
	_apply_manual_claw_settings(payload)


func _apply_manual_claw_settings(payload: Dictionary) -> void:
	if bool(payload.get("manual_claw_calibration_begin", false)):
		_overlay.call("begin_manual_claw_calibration")
	if bool(payload.get("manual_claw_calibration_reset", false)):
		_overlay.call("reset_manual_claw_calibration")
	if bool(payload.get("manual_claw_calibration_cancel", false)):
		_overlay.call("cancel_manual_claw_calibration")
	var names := [
		"manual_wrist_flex_trim_degrees", "manual_wrist_roll_trim_degrees",
		"manual_wrist_roll_direction", "manual_tool_x", "manual_tool_y",
		"manual_tool_z", "manual_tool_roll", "manual_tool_pitch",
		"manual_tool_yaw", "manual_opening_offset_degrees", "manual_opening_scale",
	]
	if names.any(func(name: String) -> bool: return payload.has(name)):
		_overlay.call("update_manual_claw_calibration", payload)
	if bool(payload.get("manual_claw_calibration_save", false)):
		_overlay.call("save_manual_claw_calibration")


func start_full_calibration(from_editor: bool = false) -> void:
	if _hardware_enabled and is_instance_valid(_calibrator):
		_calibrator.call("start_automated_arm_calibration", from_editor)


func refine_calibration(from_editor: bool = false) -> void:
	if _hardware_enabled and is_instance_valid(_calibrator):
		_calibrator.call("start_joint_alignment_calibration", from_editor)


func return_to_rest() -> bool:
	return bool(
		_hardware_enabled and is_instance_valid(_calibrator)
		and _calibrator.call("return_arm_to_rest_pose")
	)


func clear_calibration() -> void:
	if is_instance_valid(_calibrator):
		_calibrator.call("clear_arm_position_calibration")


func save_calibration_checkpoint() -> bool:
	return bool(
		is_instance_valid(_overlay)
		and _overlay.call("save_robot_position_checkpoint")
	)


func restore_calibration_checkpoint() -> bool:
	if not is_instance_valid(_overlay):
		return false
	var restored := bool(_overlay.call("restore_saved_robot_position"))
	if restored and is_instance_valid(_calibrator):
		_calibrator.call(
			"report_external_registration",
			"Saved robot position checkpoint restored.",
			1.0,
		)
	return restored


func get_calibration_status() -> Dictionary:
	if not _configuration_error.is_empty() or not _hardware_enabled:
		return {
			"state": "failed",
			"message": _configuration_error if not _configuration_error.is_empty() else "Follower hardware is disabled in the launcher configuration. Enable the intended robot profile and restart before calibration.",
			"frames": 0,
			"confidence": 0.0,
		}
	if not is_instance_valid(_calibrator):
		return {
			"state": "loading",
			"message": "SO-101 calibration module is loading.",
			"frames": 0,
			"confidence": 0.0,
		}
	return _calibrator.call("get_calibration_status")
