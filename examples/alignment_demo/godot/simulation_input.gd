extends Node
## Dedicated receive-only simulation protocol. No production transport is loaded.

signal sample_received(payload: Dictionary)

var port := 0
var bound := false
var diagnostic := "Local keyboard / replay"
var accepted_packets := 0
var rejected_packets := 0
var last_sample_msec := -1
var _udp := PacketPeerUDP.new()
var _sequences: Dictionary = {}


func _ready() -> void:
	for argument in OS.get_cmdline_user_args():
		if argument.begins_with("--simulation-input-port="):
			port = argument.get_slice("=", 1).to_int()
	if port == 0:
		return
	if port < 1024 or port > 65535:
		diagnostic = "Invalid simulation input port"
		return
	var error := _udp.bind(port, "127.0.0.1")
	bound = error == OK
	diagnostic = "Loopback input :%d (receive only)" % port if bound else "Simulation input port is busy"


func _exit_tree() -> void:
	_udp.close()


func _process(_delta: float) -> void:
	if not bound:
		return
	for _index in range(mini(_udp.get_available_packet_count(), 32)):
		var packet := _udp.get_packet()
		if packet.size() > 8192:
			rejected_packets += 1
			continue
		var parsed: Variant = JSON.parse_string(packet.get_string_from_utf8())
		if parsed is Dictionary:
			accept_packet(parsed)
		else:
			rejected_packets += 1


static func finite_number(value: Variant) -> bool:
	return (value is float or value is int) and is_finite(float(value))


func accept_packet(payload: Dictionary, now_ms: float = -1.0) -> bool:
	if now_ms < 0.0:
		now_ms = Time.get_unix_time_from_system() * 1000.0
	if not _valid(payload, now_ms):
		rejected_packets += 1
		return false
	_sequences[str(payload.source)] = int(payload.seq)
	last_sample_msec = Time.get_ticks_msec()
	accepted_packets += 1
	sample_received.emit(payload)
	return true


func _valid(p: Dictionary, now_ms: float) -> bool:
	if p.get("type") != "alignment_demo" or p.get("version") != 1:
		return false
	if not finite_number(p.get("timestamp_ms")) or absf(now_ms - float(p.timestamp_ms)) > 1000.0:
		return false
	if not finite_number(p.get("seq")) or float(p.seq) != floorf(float(p.seq)) or float(p.seq) < 0.0:
		return false
	if not p.get("source") is String or str(p.source).is_empty() or str(p.source).length() > 96:
		return false
	if int(p.seq) <= int(_sequences.get(str(p.source), -1)):
		return false
	if not _sequences.has(str(p.source)) and _sequences.size() >= 128:
		return false
	if p.has("action") and p.action not in ["reset", "recenter", "replay"]:
		return false
	for key in ["arm", "head"]:
		if p.has(key) and not p[key] is Dictionary:
			return false
	var arm: Dictionary = p.get("arm", {})
	var head: Dictionary = p.get("head", {})
	if arm.has("active") and not arm.active is bool:
		return false
	if head.has("active") and not head.active is bool:
		return false
	if bool(arm.get("active", false)):
		var values: Variant = arm.get("normalized")
		if not values is Array or values.size() != 6:
			return false
		for i in range(6):
			if not finite_number(values[i]):
				return false
			if absf(float(values[i])) > 360.0 or (i == 5 and (values[i] < 0 or values[i] > 100)):
				return false
	if bool(head.get("active", false)):
		if head.get("units") != "cm":
			return false
		for axis in ["x", "y", "z"]:
			if not finite_number(head.get(axis)) or absf(float(head[axis])) > 500.0:
				return false
	return true


func is_fresh() -> bool:
	return last_sample_msec >= 0 and Time.get_ticks_msec() - last_sample_msec <= 300
