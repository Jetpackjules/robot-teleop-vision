extends Node

## Narrow bridge between the authenticated local web server and Godot. Camera
## navigation and rendering stay client-side; robot-owned settings are passed
## opaquely to the selected module.
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
	get_tree().call_group("robot_module", "apply_remote_settings", payload)


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
