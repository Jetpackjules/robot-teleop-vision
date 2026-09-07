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


func _run() -> void:
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
