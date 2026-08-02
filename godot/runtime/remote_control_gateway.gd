extends Node

## Narrow bridge between the authenticated local web server and Godot. Camera
## navigation and rendering stay client-side; this node only applies settings
## that affect native capture, the robot overlay, or calibration.
@export_range(1024, 65535, 1) var listen_port: int = 4247

var _udp := PacketPeerUDP.new()
var _last_tracking_sent_unix_ms := 0.0
var _last_tracking_remote_sent_unix_ms := 0.0
var _last_tracking_bridge_recv_unix_ms := 0.0
var _last_tracking_transport_age_ms := -1.0
var _tracking_packets := 0
var _tracking_window_started_msec := 0
var _tracking_hz := 0.0


func _ready() -> void:
	var error := _udp.bind(listen_port, "127.0.0.1")
	if error != OK:
		push_error("Remote control gateway could not bind 127.0.0.1:%d (error %d)" % [listen_port, error])
		return
	_tracking_window_started_msec = Time.get_ticks_msec()
	set_process(true)


func _exit_tree() -> void:
	_udp.close()


func _process(_delta: float) -> void:
	while _udp.get_available_packet_count() > 0:
		var packet := _udp.get_packet()
		var value = JSON.parse_string(packet.get_string_from_utf8())
		if value is not Dictionary:
			continue
		var payload := value as Dictionary
		match str(payload.get("type", "tracking")):
			"view_settings":
				_apply_view_settings(payload)
			"tracking":
				_record_tracking(payload)
	_update_tracking_rate()


func _apply_view_settings(payload: Dictionary) -> void:
	if payload.has("white_background_enabled"):
		RenderingServer.set_default_clear_color(
			Color.WHITE if bool(payload.white_background_enabled)
			else Color(0.03, 0.035, 0.04, 1.0)
		)
	if payload.has("robot_overlay_enabled"):
		get_tree().call_group(
			"so101_robot_overlay", "set_overlay_enabled", bool(payload.robot_overlay_enabled)
		)
	if payload.has("robot_overlay_mask_scanned_arm"):
		get_tree().call_group(
			"so101_robot_overlay",
			"set_mask_scanned_robot",
			bool(payload.robot_overlay_mask_scanned_arm),
		)

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
		get_tree().call_group("so101_robot_overlay", "configure_feedback_features", feedback)
		get_tree().call_group("so101_arm_calibrator", "apply_remote_feedback_settings", feedback)

	if bool(payload.get("calibrate_robot_position", false)):
		get_tree().call_group("so101_arm_calibrator", "start_automated_arm_calibration", true)
	if bool(payload.get("refine_robot_joint_alignment", false)):
		get_tree().call_group("so101_arm_calibrator", "start_joint_alignment_calibration", true)
	if bool(payload.get("cancel_robot_position_calibration", false)):
		get_tree().call_group("so101_arm_calibrator", "cancel_arm_position_calibration")

	if bool(payload.get("manual_claw_calibration_begin", false)):
		get_tree().call_group("so101_robot_overlay", "begin_manual_claw_calibration")
	if bool(payload.get("manual_claw_calibration_reset", false)):
		get_tree().call_group("so101_robot_overlay", "reset_manual_claw_calibration")
	if bool(payload.get("manual_claw_calibration_cancel", false)):
		get_tree().call_group("so101_robot_overlay", "cancel_manual_claw_calibration")
	var manual_names := [
		"manual_wrist_flex_trim_degrees", "manual_wrist_roll_trim_degrees",
		"manual_wrist_roll_direction", "manual_tool_x", "manual_tool_y",
		"manual_tool_z", "manual_tool_roll", "manual_tool_pitch",
		"manual_tool_yaw", "manual_opening_offset_degrees", "manual_opening_scale",
	]
	if manual_names.any(func(name: String) -> bool: return payload.has(name)):
		get_tree().call_group("so101_robot_overlay", "update_manual_claw_calibration", payload)
	if bool(payload.get("manual_claw_calibration_save", false)):
		get_tree().call_group("so101_robot_overlay", "save_manual_claw_calibration")


func _record_tracking(payload: Dictionary) -> void:
	_last_tracking_sent_unix_ms = float(payload.get("sent_unix_ms", 0.0))
	_last_tracking_remote_sent_unix_ms = float(payload.get("remote_sent_unix_ms", 0.0))
	_last_tracking_bridge_recv_unix_ms = float(payload.get("bridge_recv_unix_ms", 0.0))
	if _last_tracking_sent_unix_ms > 0.0:
		_last_tracking_transport_age_ms = maxf(
			0.0,
			Time.get_unix_time_from_system() * 1000.0 - _last_tracking_sent_unix_ms,
		)
	_tracking_packets += 1


func _update_tracking_rate() -> void:
	var now := Time.get_ticks_msec()
	var elapsed := now - _tracking_window_started_msec
	if elapsed < 1000:
		return
	_tracking_hz = float(_tracking_packets) * 1000.0 / float(maxi(elapsed, 1))
	_tracking_packets = 0
	_tracking_window_started_msec = now


func get_tracking_latency_metadata() -> Dictionary:
	return {
		"sent_unix_ms": _last_tracking_sent_unix_ms,
		"remote_sent_unix_ms": _last_tracking_remote_sent_unix_ms,
		"bridge_recv_unix_ms": _last_tracking_bridge_recv_unix_ms,
		"transport_age_ms": _last_tracking_transport_age_ms,
		"rx_hz": _tracking_hz,
	}
