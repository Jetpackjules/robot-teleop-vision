extends SceneTree

const Transport := preload("res://robot_modules/so101/godot/so101_transport_config.gd")
const Module := preload("res://robot_modules/so101/godot/so101_module.gd")
const Overlay := preload("res://robot_modules/so101/godot/so101_robot_overlay.gd")

class AckProbe:
	extends "res://robot_modules/so101/godot/so101_motion_calibrator.gd"
	func _emit_status(_force: bool) -> void:
		pass
	func _stop_editor_sweep(_reason: String) -> void:
		_editor_sweep_requested = false
		_editor_sweep_acknowledged = false

class WireProbe:
	extends "res://robot_modules/so101/godot/so101_motion_calibrator.gd"
	func _emit_status(_force: bool) -> void:
		pass

class ClearProbe:
	extends AckProbe
	func _ready() -> void:
		pass
	func _process(_delta: float) -> void:
		pass

class RegistrationProbe:
	extends Node3D
	var clear_count := 0
	var saved_status := {"registration_source": "full_automated_motion_axis_calibration", "calibrated_through_joint": 4, "base_axis_fit": {"reference_camera": "RealSense D435 B"}}
	func get_registration_status() -> Dictionary:
		return saved_status.duplicate(true)
	func clear_saved_registration() -> void:
		clear_count += 1

class ClawStartProbe:
	extends ClearProbe
	func start_claw_visual_calibration(_force_editor_auto_move: bool = false) -> void:
		_state = "capturing"

class SaveProbeOverlay:
	extends "res://robot_modules/so101/godot/so101_robot_overlay.gd"
	func _save_registration() -> bool:
		return true

class RgbProbe:
	extends Node3D
	func get_color_image() -> Image:
		return Image.create(4, 4, false, Image.FORMAT_RGB8)
	func get_current_intrinsics() -> Vector4:
		return Vector4(4, 4, 2, 2)

class IncompleteRawRgbProbe:
	extends RgbProbe
	var valid_raw := false
	func get_raw_color_image() -> Image:
		return Image.create(8, 8, false, Image.FORMAT_RGB8)
	func get_raw_color_intrinsics() -> Vector4:
		return Vector4(8, 8, 4, 4) if valid_raw else Vector4.ZERO
	func get_depth_to_raw_color_extrinsics() -> PackedFloat32Array:
		return PackedFloat32Array([1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0])

var checks := 0
var failures: Array[String] = []


func check(condition: bool, label: String) -> void:
	checks += 1
	if not condition:
		failures.append(label)


func _initialize() -> void:
	call_deferred("_run")


func config() -> Dictionary:
	return {"schema_version": 1, "module": "so101", "enabled": true,
		"calibration_status_port": 14251, "module_settings": {
			"command_port": 14248, "status_port": 14249,
			"telemetry_port": 14250, "editor_telemetry_port": 14252}}


func probe() -> AckProbe:
	var node := AckProbe.new()
	node._state = "capturing"
	node._calibration_request_id = "current-request"
	node._editor_sweep_requested = true
	node._editor_sweep_requested_msec = 1000
	return node


func status(id: String = "current-request") -> Dictionary:
	return {"calibration_request_id": id, "calibration_request_state": "running",
		"calibration_sweep_active": true, "follower_connected": true,
		"state": "calibrating", "sent_unix_ms": Time.get_unix_time_from_system() * 1000.0}


func claw_frames(snapshot: Dictionary) -> Array:
	var frames: Array = []
	for roll in [-20.0, 20.0]:
		for opening in [5.0, 25.0, 50.0, 75.0, 90.0]:
			frames.append({
				"calibration_joint_index": 5,
				"pose": [0, 0, 0, 0, roll, opening],
				"full_camera_points": [{"name": snapshot.name, "points": [Vector3.ZERO]}],
				"rgb_snapshots": [snapshot.duplicate(true)],
			})
	return frames


func check_claw_rgb_views(calibrator: AckProbe, snapshot: Dictionary) -> void:
	for camera_name in ["RealSense D435 A", "RealSense D435 B", "RealSense D455"]:
		var reference_snapshot := snapshot.duplicate(true)
		reference_snapshot.name = camera_name
		calibrator._automation_base_result = {"reference_camera": camera_name}
		var frames := claw_frames(reference_snapshot)
		check(calibrator._claw_capture_coverage(frames).ready, camera_name + " claw motion complete")
		check(calibrator._complete_automated_claw_view_count(frames) == 2, camera_name + " counts both saved RGB views")
		check(camera_name in calibrator._claw_capture_coverage(frames).summary, "coverage reports actual reference camera")
		var missing := frames.duplicate(true)
		missing[0].rgb_snapshots[0].path = "user://missing-claw.png"
		check(calibrator._complete_automated_claw_view_count(missing) == 1, "missing image prevents a complete view")
		var wrong_camera := frames.duplicate(true)
		for frame in wrong_camera:
			frame.rgb_snapshots[0].name = "RealSense other-camera"
		check(calibrator._complete_automated_claw_view_count(wrong_camera) == 0, "RGB from another camera cannot substitute")
		var duplicates := frames.duplicate(true)
		duplicates[0].rgb_snapshots = []
		duplicates.append(frames[1].duplicate(true))
		check(calibrator._complete_automated_claw_view_count(duplicates) == 1, "duplicate opening cannot replace missing RGB state")
		var with_rgb: Dictionary = frames[0].duplicate(true)
		with_rgb.camera_points = [{"points": [Vector3.ZERO]}]
		var without_rgb := with_rgb.duplicate(true)
		without_rgb.rgb_snapshots = []
		without_rgb.camera_points[0].points.append(Vector3.ONE)
		calibrator._automation_claw_accumulated_frames = [with_rgb.duplicate(true)]
		calibrator._merge_automated_claw_capture_frames([without_rgb])
		check(not calibrator._automation_claw_accumulated_frames[0].rgb_snapshots.is_empty(), "better depth cannot erase usable RGB")
		calibrator._automation_claw_accumulated_frames = [without_rgb.duplicate(true)]
		calibrator._merge_automated_claw_capture_frames([with_rgb])
		check(not calibrator._automation_claw_accumulated_frames[0].rgb_snapshots.is_empty(), "RGB retry repairs a depth-only state")


func _run() -> void:
	var solver := AckProbe.new()
	var configured_python := OS.get_environment("ROBOT_TELEOP_SOLVER_PYTHON")
	check(solver._solver_python_executable() == configured_python, "explicit solver Python")
	var solver_result := solver._execute_external_json_solver(
		ProjectSettings.globalize_path("res://solver fixture.py"),
		PackedStringArray([ProjectSettings.globalize_path("user://solver result.json")]),
		ProjectSettings.globalize_path("user://solver result.json"), "base_solve",
	)
	check(solver_result.get("ok", false) and solver_result.get("value") == 42, "real Python solver process and spaced paths")
	OS.set_environment("ROBOT_TELEOP_SOLVER_PYTHON", ProjectSettings.globalize_path("res://missing-python.exe"))
	solver_result = solver._execute_external_json_solver(
		ProjectSettings.globalize_path("res://solver fixture.py"), PackedStringArray(),
		ProjectSettings.globalize_path("user://solver result.json"), "base_solve",
	)
	check(not solver_result.get("ok", true) and "could not start" in solver_result.status, "launch error is not evidence rejection")
	OS.set_environment("ROBOT_TELEOP_SOLVER_PYTHON", configured_python)
	# Exercise replacement of an existing candidate on Windows, where an open
	# read handle prevents the atomic rename used between outward joint solves.
	var candidate_path := solver.AUTOMATED_CANDIDATE_REGISTRATION_PATH
	var candidate := FileAccess.open(candidate_path, FileAccess.WRITE)
	candidate.store_string(JSON.stringify({"origin": [1, 2, 3], "saved": false}))
	candidate.close()
	for through_joint in [1, 2]:
		solver._automation_solver_through_joint = through_joint
		var mapping := {
			"fitted_directions": [1, -1, 1, 1, 1, 1],
			"fitted_offsets_degrees": [40, 80, through_joint, -70, 0, 0],
		}
		check(solver._update_automated_candidate_mapping(mapping), "joint-%d candidate replacement" % through_joint)
		candidate = FileAccess.open(candidate_path, FileAccess.READ)
		var staged: Dictionary = JSON.parse_string(candidate.get_as_text())
		candidate.close()
		check(staged.get("calibrated_through_joint", 0) == through_joint, "candidate advances to joint %d" % through_joint)
		check(staged.origin == [1.0, 2.0, 3.0], "candidate preserves validated base")
		check(not FileAccess.file_exists(candidate_path + ".pending"), "no pending candidate remains")
	check(not solver._update_automated_candidate_mapping({}), "invalid mapping cannot overwrite candidate")
	DirAccess.remove_absolute(ProjectSettings.globalize_path(candidate_path))
	check(solver.has_method("clear_arm_position_calibration"), "module clear action has a calibrator implementation")
	solver.free()
	var clear_host := Node3D.new()
	root.add_child(clear_host)
	var anchor := Node3D.new()
	anchor.name = "WorldLevelAnchor"
	clear_host.add_child(anchor)
	var registration := RegistrationProbe.new()
	registration.name = "RobotOverlay"
	anchor.add_child(registration)
	var clear_probe := ClearProbe.new()
	clear_host.add_child(clear_probe)
	var robot_module := Module.new()
	robot_module._calibrator = clear_probe
	clear_probe._state = "solving"
	robot_module.clear_calibration()
	check(registration.clear_count == 0 and clear_probe._state == "solving", "clear does not race a solver")
	clear_probe._state = "capturing"
	clear_probe._editor_sweep_requested = true
	clear_probe._frames = [{"test_frame": true}]
	robot_module.clear_calibration()
	check(registration.clear_count == 1, "module clear reaches overlay")
	check(clear_probe._state == "idle" and clear_probe._frames.is_empty() and not clear_probe._editor_sweep_requested, "clear stops capture and resets state")
	check("Camera alignment was preserved" in clear_probe._status.message, "clear reports robot-only scope")
	clear_probe._automation_active = true
	clear_probe._automation_base_result = {"reference_camera": "RealSense D435 B"}
	var camera_frames: Array = [{"full_camera_points": [
		{"name": "RealSense D435 A", "points": [1, 2, 3]},
		{"name": "RealSense D435 B", "points": [1]},
	]}]
	check(clear_probe._reference_camera_for_frames(camera_frames) == "RealSense D435 B", "validated camera survives higher background coverage")
	check(not clear_probe._automated_has_required_d455([{"name": "RealSense D435 A"}]), "coverage requires selected camera")
	check(clear_probe._automated_has_required_d455([{"name": "RealSense D435 B"}]), "D435 reference coverage accepted")
	var snapshots: Array = []
	for camera_name in ["RealSense D435 A", "RealSense D435 B"]:
		var rgb := RgbProbe.new()
		rgb.name = camera_name
		clear_host.add_child(rgb)
		snapshots.append(clear_probe._rgb_snapshot_for_renderer(rgb, [0, 0, 0, 0, 0, 5]))
	check(not snapshots[0].is_empty() and not snapshots[1].is_empty(), "D435 RGB images are captured")
	if not snapshots[0].is_empty() and not snapshots[1].is_empty():
		check(snapshots[0].path != snapshots[1].path, "camera RGB images do not overwrite each other")
		check_claw_rgb_views(clear_probe, snapshots[1])
	var claw_start := ClawStartProbe.new()
	clear_host.add_child(claw_start)
	claw_start.start_distal_claw_tip_calibration(true)
	check(claw_start._reference_camera_for_frames(camera_frames) == "RealSense D435 B", "claw-only restart retains saved base camera")
	claw_start.free()
	var raw_rgb := IncompleteRawRgbProbe.new()
	clear_host.add_child(raw_rgb)
	var fallback_rgb := clear_probe._rgb_snapshot_for_renderer(raw_rgb, [0, 0, 0, 0, 0, 5])
	check(not fallback_rgb.is_empty() and not fallback_rgb.get("uses_raw_color", true), "incomplete raw metadata uses calibrated aligned RGB")
	raw_rgb.valid_raw = true
	var native_rgb := clear_probe._rgb_snapshot_for_renderer(raw_rgb, [0, 0, 0, 0, 0, 5])
	check(native_rgb.get("uses_raw_color", false) and native_rgb.get("width", 0) == 8, "complete raw RGB remains preferred")
	clear_probe._automation_base_result = {"reference_camera": "RealSense D435 B"}
	var claw_fit := {"type": "so101_claw_visual_fit", "method": "d455_native_rgb_multiview_tip_fit", "reference_camera": "RealSense D435 B", "gripper_angle_samples_normalized": [5, 25, 50, 75, 90], "gripper_angle_samples_degrees": [-10, 15, 40, 65, 80], "validation_view_count": 2, "all_views_improved": true, "median_tip_residual_px": 1.0, "confidence": 0.95}
	check(clear_probe._validate_automated_claw_result(claw_fit).ok, "renderable RGB result accepted")
	claw_fit.gripper_hinge_direction = -1
	check(not clear_probe._validate_automated_claw_result(claw_fit).ok, "unsupported reversed hinge cannot be saved as a valid stock overlay")
	claw_fit.gripper_mount_correction_local = [0, 0, 0]
	claw_fit.gripper_visual_correction_rpy_degrees = [0, 0, 0]
	claw_fit.gripper_visual_correction_translation_local = [0, 0, 0]
	var saved_overlay := SaveProbeOverlay.new()
	check(not saved_overlay.apply_automated_claw_calibration(claw_fit, 0.0), "overlay itself rejects unsupported hinge transaction")
	claw_fit.gripper_hinge_direction = 1
	check(saved_overlay.apply_automated_claw_calibration(claw_fit, 0.0), "supported stock-jaw transaction still applies")
	saved_overlay.free()
	clear_probe._automation_active = false
	robot_module._calibrator = null
	robot_module.free()
	clear_host.free()
	var resolved := Transport.resolve(JSON.stringify(config()), "res://missing.json")
	check(resolved.ports.command_port == 14248, "custom command port")
	check(resolved.ports.telemetry_port == 14250, "custom runtime telemetry")
	check(resolved.ports.editor_telemetry_port == 14252, "custom editor telemetry")
	check(resolved.calibration_status_port == 14251, "custom calibration reporting")
	check(Transport.resolve("", "res://missing.json").has("error"), "missing settings never guess a follower")
	check(Transport.resolve("{broken", "res://missing.json").has("error"), "invalid explicit settings fail closed")
	check(Transport.resolve("", "res://.teleop/runtime_config.json").ports.command_port == 14248, "editor sidecar")
	var disabled := config()
	disabled.enabled = false
	disabled.module_settings = {}
	check(not Transport.validate(disabled).enabled, "disabled hardware requires no profile")
	var invalid := config()
	invalid.module = "other"
	check(Transport.validate(invalid).has("error"), "wrong module")
	for invalid_port in [0, -1, 65536, 14248.5, "14248", true]:
		invalid = config()
		invalid.module_settings.command_port = invalid_port
		check(Transport.validate(invalid).has("error"), "invalid port %s" % str(invalid_port))
	invalid = config()
	invalid.module_settings.telemetry_port = 14248
	check(Transport.validate(invalid).has("error"), "duplicate consumer ports")
	invalid = config()
	invalid.calibration_status_port = 14250
	check(Transport.validate(invalid).has("error"), "calibration port collision")

	var node := probe()
	var packet := status("old-request")
	packet.calibration_rejection = "old failure"
	check(not node._check_sweep_acknowledgement(packet, 1100), "old rejection ignored")
	check(node._state == "capturing" and not node._editor_sweep_acknowledged, "old ack ignored")
	check(node._check_sweep_acknowledgement(status(), 1200), "current ack accepted")
	check(node._editor_sweep_acknowledged, "ack state")
	packet = status()
	packet.calibration_sweep_active = false
	packet.calibration_request_state = "completed"
	check(node._check_sweep_acknowledgement(packet, 1300), "natural completion accepted")
	node.free()

	var commands := PacketPeerUDP.new()
	check(commands.bind(0, "127.0.0.1") == OK, "fake command port")
	var wire := WireProbe.new()
	wire.follower_command_port = commands.get_local_port()
	wire._state = "capturing"
	wire._auto_move_this_capture = true
	wire._capture_mode = "axis"
	wire._start_editor_sweep()
	await create_timer(0.1).timeout
	check(commands.get_available_packet_count() == 1, "one start reaches configured command port")
	var start_packet = JSON.parse_string(commands.get_packet().get_string_from_utf8())
	check(start_packet.type == "arm_base_axis_sweep" and start_packet.action == "start", "correct sweep command")
	check(start_packet.calibration_request_id == wire._calibration_request_id and not wire._calibration_request_id.is_empty(), "wire request ID")
	check(start_packet.calibration_request_expires_unix_ms > Time.get_unix_time_from_system() * 1000.0, "start has a bounded expiry")
	wire._stop_editor_sweep("test cancellation")
	await create_timer(0.1).timeout
	check(commands.get_available_packet_count() == 1, "one stop reaches configured command port")
	var stop_packet = JSON.parse_string(commands.get_packet().get_string_from_utf8())
	check(stop_packet.action == "stop" and stop_packet.calibration_request_id == start_packet.calibration_request_id, "stop matches start transaction")
	wire._arm_command_udp.close()
	wire.free()
	commands.close()

	node = probe()
	packet = status()
	packet.calibration_rejection = "clearance denied"
	check(not node._check_sweep_acknowledgement(packet, 1100), "current rejection fails")
	check("clearance denied" in node._status.message, "specific rejection exposed")
	node.free()

	node = probe()
	packet = status()
	packet.state = "fault"
	packet.fault = "motor read failed"
	check(not node._check_sweep_acknowledgement(packet, 1100), "fault fails")
	check("motor read failed" in node._status.message, "specific fault exposed")
	node.free()

	for after_ack in [false, true]:
		node = probe()
		node._editor_sweep_acknowledged = after_ack
		packet = status()
		packet.calibration_sweep_active = false
		packet.calibration_request_state = "cancelled"
		packet.message = "operator hold during automatic calibration"
		check(not node._check_sweep_acknowledgement(packet, 1100), "Hold always fails capture")
		check("operator hold" in node._status.message, "Hold reason exposed")
		node.free()

	node = probe()
	packet = status("old-request")
	check(not node._check_sweep_acknowledgement(packet, 9000), "bounded wait allows slow first ack")
	check(node._state == "capturing", "no premature timeout")
	check(not node._check_sweep_acknowledgement(packet, 12000), "timeout eventually fails")
	check(node._state == "failed", "timeout status")
	node.free()

	node = probe()
	packet = status()
	packet.erase("calibration_request_id")
	node._check_sweep_acknowledgement(packet, 12000)
	check("same updated repo" in node._status.message, "mixed versions actionable")
	node.free()

	node = probe()
	node._editor_sweep_acknowledged = true
	packet = status()
	packet.sent_unix_ms -= 2000
	node._check_sweep_acknowledgement(packet, 1400)
	check(node._state == "failed", "stale in-progress telemetry stops capture")
	node.free()

	node = probe()
	node._automation_active = true
	node._fail("transport failed")
	check(not node._automation_active and node._state == "failed", "failure unlocks next user retry")
	node.free()

	# Bind only ephemeral loopback test sockets. No Main scene, camera or
	# follower process is started, and no motor command is sent.
	var occupied := PacketPeerUDP.new()
	check(occupied.bind(0, "127.0.0.1") == OK, "ephemeral port reservation")
	var port := occupied.get_local_port()
	var overlay := Overlay.new()
	overlay.telemetry_port = port
	overlay.editor_telemetry_port = port
	overlay._bind_telemetry()
	check(not overlay.get_telemetry_diagnostic().bound, "busy listener diagnosed")
	node = probe()
	check(str(port) in node._follower_telemetry_error(overlay, {}), "error reports actual port")
	occupied.close()
	overlay._bind_telemetry()
	check(overlay.get_telemetry_diagnostic().bound, "listener can recover after port released")
	var sender := PacketPeerUDP.new()
	check(sender.set_dest_address("127.0.0.1", port) == OK, "fake telemetry endpoint")
	sender.put_packet(JSON.stringify({"type": "arm_status", "follower_connected": true}).to_utf8_buffer())
	await create_timer(0.1).timeout
	overlay._poll_telemetry()
	check(overlay.get_latest_status().get("follower_connected", false), "custom-port packet consumed")
	overlay._udp.close()
	sender.close()
	overlay.free()
	node.free()
	for failure in failures:
		printerr("FAIL: ", failure)
	print("SO101_TRANSPORT_PROBE checks=%d failures=%d" % [checks, failures.size()])
	quit(0 if failures.is_empty() else 1)
