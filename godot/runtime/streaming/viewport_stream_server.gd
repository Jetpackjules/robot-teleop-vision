@tool
extends Node

@export_storage var stream_enabled: bool = true:
	set(value):
		stream_enabled = value
		if is_inside_tree() and _runs_in_current_context():
			if stream_enabled:
				_start_server()
			else:
				_stop_server()
@export_storage var bind_host: String = "0.0.0.0"
@export_storage var port: int = 8780
@export_storage var target_fps: float = 30.0
@export_storage var max_width: int = 720
@export_storage var jpeg_quality: int = 42
@export_storage var max_clients: int = 4
@export_storage var hybrid_target_fps: float = 60.0
@export_storage var hybrid_context_fps: float = 30.0
@export_storage var hybrid_detail_fps: float = 60.0
@export_storage var hybrid_max_width: int = 384
@export_storage var hybrid_jpeg_quality: int = 78
@export_storage var async_high_resolution_readback: bool = true
@export_storage var async_readback_min_width: int = 960
@export_storage var async_readback_max_inflight: int = 2
@export_storage var rgbd_root_path: NodePath = NodePath("..")
@export_storage var tracking_source_path: NodePath = NodePath("../RemoteControlGateway")
@export_storage var stream_status: String = "stopped"

const BOUNDARY := "godotframe"
const STREAM_SEND_CHUNK_BYTES := 524288
const STREAM_SEND_BUDGET_BYTES := 4194304
const STREAM_SEND_BUDGET_USEC := 5000
const STREAM_STALL_TIMEOUT_MSEC := 2000

var _server := TCPServer.new()
var _clients: Array[Dictionary] = []
var _frame_accum := 0.0
var _last_frame_bytes := PackedByteArray()
var _last_frame_content_type := "image/jpeg"
var _last_raw_frame_bytes := PackedByteArray()
var _last_raw_frame_width := 0
var _last_raw_frame_height := 0
var _last_raw_frame_format := "rgb24"
var _last_rgbd_frame_bytes := PackedByteArray()
var _rgbd_frame_counter := 0
var _rgbd_frame_accum := 0.0
var _rgbd_encoder_accum: Dictionary = {}
var _rgbd_last_request_sequence: Dictionary = {}
var _rgbd_pending_completed: Dictionary = {}
var _last_frame_msec := 0
var _last_frame_unix_ms := 0.0
var _next_capture_due_msec := 0
var _last_tracking_sent_unix_ms := 0.0
var _last_tracking_remote_sent_unix_ms := 0.0
var _last_tracking_bridge_recv_unix_ms := 0.0
var _last_tracking_transport_age_ms := -1.0
var _frame_counter := 0
var _last_status_msec := 0
var _next_listen_retry_msec := 0
var _tracking_source: Node
var _async_capture_inflight := 0
var _async_readback_available := true
var _camera_rgb_cache: Dictionary = {}
var _camera_rgb_stream_frames: Dictionary = {}
var _camera_rgb_frame_counter := 0
var _cpu_profile: Dictionary = {}
var _renderers_cache: Array[Node] = []
var _renderers_cache_frame := -1
var _renderers_cache_root := 0
var _model_property_cache: Dictionary = {}

func _runs_in_current_context() -> bool:
	return not Engine.is_editor_hint() or (is_inside_tree() and get_tree().edited_scene_root == get_parent())

func _runtime_details() -> Dictionary:
	var configuration = JSON.parse_string(OS.get_environment("ROBOT_TELEOP_RUNTIME_CONFIG"))
	if Engine.is_editor_hint() or not configuration is Dictionary:
		if FileAccess.file_exists("res://.teleop/runtime_config.json"):
			configuration = JSON.parse_string(FileAccess.get_file_as_string("res://.teleop/runtime_config.json"))
	if not configuration is Dictionary:
		configuration = {}
	var robot := {}
	for key in ["schema_version", "module", "enabled", "module_settings", "calibration_status_port"]:
		if configuration.has(key): robot[key] = configuration[key]
	var capture: Dictionary = configuration.get("capture", {}).duplicate()
	if OS.get_environment("ROBOT_TELEOP_CAPTURE_PUBLISH") == "1" and not capture.is_empty():
		capture["pid"] = OS.get_process_id()
	return {"pid": OS.get_process_id(), "project_root": ProjectSettings.globalize_path("res://").trim_suffix("/"),
		"editor": Engine.is_editor_hint(), "robot_configuration": robot, "capture": capture}

func _ready() -> void:
	if OS.get_environment("ROBOT_TELEOP_CAPTURE_ONLY") == "1":
		# The capture service stays private and does not share the editor's port.
		bind_host = "127.0.0.1"
		var capture_port := OS.get_environment("ROBOT_TELEOP_CAPTURE_PORT")
		port = 8781 if capture_port.is_empty() else clampi(int(capture_port), 1024, 65535)
		process_priority = 10
	if stream_enabled and _runs_in_current_context():
		_start_server()
	set_process(true)
	if RenderingServer.has_signal("frame_post_draw") and not RenderingServer.frame_post_draw.is_connected(_on_frame_post_draw):
		RenderingServer.frame_post_draw.connect(_on_frame_post_draw)

func _exit_tree() -> void:
	if RenderingServer.has_signal("frame_post_draw") and RenderingServer.frame_post_draw.is_connected(_on_frame_post_draw):
		RenderingServer.frame_post_draw.disconnect(_on_frame_post_draw)
	_stop_server()

func _start_server() -> void:
	_stop_server()
	var listen_host := "127.0.0.1" if Engine.is_editor_hint() else bind_host
	var err := _server.listen(port, listen_host)
	if err != OK:
		stream_status = "Viewport stream failed on %s:%d error=%d" % [listen_host, port, err]
		_next_listen_retry_msec = Time.get_ticks_msec() + 1000
		if not Engine.is_editor_hint(): push_warning(stream_status)
		return
	stream_status = "Viewport stream listening at http://%s:%d/stream" % [listen_host, port]
	print(stream_status)

func _stop_server() -> void:
	for client in _clients:
		var peer := client.get("peer") as StreamPeerTCP
		if peer != null:
			peer.disconnect_from_host()
	_clients.clear()
	if _server.is_listening():
		_server.stop()
	stream_status = "stopped"

func _process(delta: float) -> void:
	if not _runs_in_current_context() or not stream_enabled:
		if _server.is_listening(): _stop_server()
		return
	if not _server.is_listening():
		if Time.get_ticks_msec() < _next_listen_retry_msec:
			return
		_start_server()
		return
	var started := Time.get_ticks_usec()
	_accept_clients()
	_poll_clients()
	_cpu_profile["socket_poll_ms"]=(Time.get_ticks_usec()-started)*0.001
	if _clients.is_empty():
		_update_status()
		return
	if _has_stream_clients("rgbd"):
		started=Time.get_ticks_usec()
		if _pump_rgbd_encoders(delta):
			_write_stream_frame_to_clients()
		_cpu_profile["rgbd_pump_and_send_ms"]=(Time.get_ticks_usec()-started)*0.001
	if _has_stream_clients("camera_rgb"):
		if _pump_camera_rgb_encoder():
			_write_stream_frame_to_clients()
	_frame_accum += delta
	var interval := 1.0 / maxf(1.0, target_fps)
	if _frame_accum >= interval:
		_frame_accum = fmod(_frame_accum, interval)
		_next_capture_due_msec = Time.get_ticks_msec()
	_update_status()

func _on_frame_post_draw() -> void:
	if not stream_enabled or _clients.is_empty():
		return
	if _next_capture_due_msec <= 0:
		return
	if Time.get_ticks_msec() < _next_capture_due_msec:
		return
	_next_capture_due_msec = 0
	var needs_encoded := _has_ready_stream_clients("mjpeg")
	var needs_raw := _has_ready_stream_clients("raw")
	if not needs_encoded and not needs_raw:
		return
	if _begin_async_frame_capture(needs_encoded, needs_raw):
		return
	_capture_frame(needs_encoded, needs_raw)
	_write_stream_frame_to_clients()

func _accept_clients() -> void:
	while _server.is_connection_available():
		var peer := _server.take_connection()
		if peer == null:
			continue
		if peer.has_method("set_no_delay"):
			peer.call("set_no_delay", true)
		if _clients.size() >= max_clients:
			_send_http_response(peer, "503 Service Unavailable", "text/plain; charset=utf-8", "too many viewport stream clients\n".to_utf8_buffer())
			peer.disconnect_from_host()
			continue
		_clients.append({
			"peer": peer,
			"request": "",
				"path": "",
				"streaming": false,
				"stream_type": "",
			"headers_sent": false,
			"snapshot_sent": false,
			"pending_header": PackedByteArray(),
			"pending_frame": PackedByteArray(),
			"pending_footer": PackedByteArray(),
			"pending_segment": -1,
			"pending_offset": 0,
			"pending_started_msec": 0,
			"last_rgbd_frame": -1,
		})

func _poll_clients() -> void:
	var remove_indices: Array[int] = []
	for i in _clients.size():
		var client := _clients[i]
		var peer := client.get("peer") as StreamPeerTCP
		if peer == null or peer.get_status() == StreamPeerTCP.STATUS_NONE or peer.get_status() == StreamPeerTCP.STATUS_ERROR:
			remove_indices.append(i)
			continue
		if not bool(client.get("headers_sent", false)):
			var available := peer.get_available_bytes()
			if available > 0:
				var chunk := peer.get_utf8_string(available)
				client["request"] = str(client.get("request", "")) + chunk
				if str(client["request"]).find("\r\n\r\n") >= 0:
					_handle_http_request(client)
			_clients[i] = client
		elif bool(client.get("streaming", false)):
			if not _flush_client_output(client):
				remove_indices.append(i)
			_clients[i] = client
		elif not bool(client.get("streaming", false)):
			remove_indices.append(i)
	for index in range(remove_indices.size() - 1, -1, -1):
		_disconnect_client(remove_indices[index])

func _handle_http_request(client: Dictionary) -> void:
	var peer := client.get("peer") as StreamPeerTCP
	if peer == null:
		return
	var request := str(client.get("request", ""))
	var first_line := request.get_slice("\r\n", 0)
	var parts := first_line.split(" ")
	var path := "/" if parts.size() < 2 else str(parts[1])
	client["path"] = path
	if path == "/" or path == "/index.html":
		var body := (
			"<html><body style=\"margin:0;background:#111;color:white;font:16px system-ui\">" +
			"<img src=\"/stream\" style=\"width:100vw;height:100vh;object-fit:contain\">" +
			"</body></html>"
		).to_utf8_buffer()
		_send_http_response(peer, "200 OK", "text/html; charset=utf-8", body)
		client["headers_sent"] = true
		client["snapshot_sent"] = true
	elif path.begins_with("/snapshot"):
		if _last_frame_bytes.is_empty():
			_capture_frame()
		if _last_frame_bytes.is_empty():
			_send_http_response(peer, "503 Service Unavailable", "text/plain; charset=utf-8", "viewport frame not ready\n".to_utf8_buffer())
		else:
			_send_http_response(peer, "200 OK", _last_frame_content_type, _last_frame_bytes)
		client["headers_sent"] = true
		client["snapshot_sent"] = true
	elif path == "/diagnostics":
		# Read-only counters for the private desktop viewer and bounded profiling.
		# Do not trigger image encoding, GPU readback, or change capture settings.
		var cameras: Array[Dictionary] = []
		for renderer in _find_rgbd_renderers():
			var stages: Vector4 = renderer.call("get_direct_realsense_capture_stage_ms") if renderer.has_method("get_direct_realsense_capture_stage_ms") else Vector4.ZERO
			var sensor: Vector4 = renderer.call("get_direct_realsense_sensor_rates") if renderer.has_method("get_direct_realsense_sensor_rates") else Vector4.ZERO
			var depth: Image = renderer.call("get_depth_image")
			var color: Image = renderer.call("get_color_image")
			var raw_size: Vector2i = renderer.call("get_raw_color_size") if renderer.has_method("get_raw_color_size") else Vector2i.ZERO
			cameras.append({"id":str(renderer.call("get_direct_realsense_serial")),
				"depth_size":[depth.get_width(),depth.get_height()] if depth != null else [],
				"color_size":[color.get_width(),color.get_height()] if color != null else [],
				"raw_color_size":[raw_size.x,raw_size.y] if raw_size.x>0 else [],
				"stream_profile":str(renderer.call("get_direct_realsense_stream_profile")) if renderer.has_method("get_direct_realsense_stream_profile") else "custom",
				"capture_stage_ms":{"align":stages.x,"filters":stages.y,"pack":stages.z,"total":stages.w},
				"sensor_fps":[sensor.x if sensor.x>0 else null,sensor.y if sensor.y>0 else null],
				"sensor_exposure_us":[sensor.z if sensor.z>0 else null,sensor.w if sensor.w>0 else null],
				"encoder":renderer.call("get_rgbd_encoder_status") if renderer.has_method("get_rgbd_encoder_status") else {},
				"sequence":int(renderer.call("get_frame_sequence")) if renderer.has_method("get_frame_sequence") else 0,
				"stride":int(renderer.call("get_current_grid_stride")),
				"capture_fps":float(renderer.call("get_direct_realsense_capture_fps")),
				"render_fps":float(renderer.call("get_render_fps")),
				"age_ms":float(renderer.call("get_display_frame_age_ms"))})
		var root_node := get_node_or_null(rgbd_root_path)
		var body := JSON.stringify({"runtime":_runtime_details(),"scene_fps":Engine.get_frames_per_second(),"cameras":cameras,
			"stream_preset":root_node.call("get_remote_stream_preset") if root_node!=null and root_node.has_method("get_remote_stream_preset") else "custom",
			"stream_cpu":_cpu_profile,"frame_cpu_ms":Performance.get_monitor(Performance.TIME_PROCESS)*1000.0,
			"view_cpu_ms":root_node.get("_capture_view_cpu_ms") if root_node!=null else 0}).to_utf8_buffer()
		_send_http_response(peer,"200 OK","application/json",body)
		client["headers_sent"] = true
		client["snapshot_sent"] = true
	elif path == "/cameras":
		var cameras: Array[Dictionary] = []
		for renderer in _find_rgbd_renderers():
			var serial := str(renderer.call("get_direct_realsense_serial"))
			var camera_name := str(renderer.get_parent().name) if renderer.get_parent() != null else str(renderer.name)
			var camera := {"id":serial,"name":camera_name.replace(serial, "").strip_edges(),"model":_renderer_model(renderer)}
			if renderer.has_method("get_color_diagnostics"):
				camera["image_levels"] = renderer.call("get_color_diagnostics")
			cameras.append(camera)
		_send_http_response(peer,"200 OK","application/json",JSON.stringify({"cameras":cameras}).to_utf8_buffer())
		client["headers_sent"] = true
		client["snapshot_sent"] = true
	elif path.get_slice("?", 0) == "/camera-rgb-stream":
		var options := {"camera":"", "w":3840, "q":95}
		for pair in path.get_slice("?", 1).split("&", false):
			var key := pair.get_slice("=", 0)
			var value := pair.get_slice("=", 1).uri_decode()
			if key == "camera": options[key] = value
			if key == "w": options[key] = clampi(int(value),160,3840)
			if key == "q": options[key] = clampi(int(value),1,100)
		if str(options.camera).is_empty():
			var renderers := _find_rgbd_renderers()
			if not renderers.is_empty(): options.camera = str(renderers[0].call("get_direct_realsense_serial"))
		client["camera_rgb_options"] = options
		peer.put_data(("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=%s\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n" % BOUNDARY).to_utf8_buffer())
		client["headers_sent"] = true
		client["streaming"] = true
		client["stream_type"] = "camera_rgb"
		client["last_camera_rgb_frame"] = -1
	elif path.get_slice("?", 0) == "/camera-rgb":
		var options: Dictionary = {}
		for pair in path.get_slice("?", 1).split("&", false):
			options[pair.get_slice("=", 0)] = pair.get_slice("=", 1).uri_decode()
		var frame := _get_camera_rgb_frame(str(options.get("camera", "")), clampi(int(options.get("w", 3840)), 160, 3840), clampi(int(options.get("q", 95)), 1, 100))
		if frame.is_empty():
			_send_http_response(peer, "503 Service Unavailable", "text/plain", "original camera RGB not ready\n".to_utf8_buffer())
		else:
			_send_http_response(peer, "200 OK", "image/jpeg", frame["body"], frame["headers"])
		client["headers_sent"] = true
		client["snapshot_sent"] = true
	elif path.begins_with("/stream") or path.begins_with("/raw") or path.begins_with("/rgbd"):
		if path.begins_with("/rgbd"):
			_apply_rgbd_query(path)
		else:
			_apply_stream_query(path)
		var is_raw := path.begins_with("/raw")
		var is_rgbd := path.begins_with("/rgbd")
		var headers := (
			"HTTP/1.1 200 OK\r\n" +
			"Content-Type: multipart/x-mixed-replace; boundary=%s\r\n" % BOUNDARY +
			"Cache-Control: no-cache, no-store, must-revalidate\r\n" +
			"Pragma: no-cache\r\n" +
			"Connection: close\r\n" +
			"Access-Control-Allow-Origin: *\r\n" +
			"\r\n"
		)
		peer.put_data(headers.to_utf8_buffer())
		client["headers_sent"] = true
		client["streaming"] = true
		client["stream_type"] = "rgbd" if is_rgbd else ("raw" if is_raw else "mjpeg")
	else:
		_send_http_response(peer, "404 Not Found", "text/plain; charset=utf-8", "not found\n".to_utf8_buffer())
		client["headers_sent"] = true
		client["snapshot_sent"] = true

func _apply_stream_query(path: String) -> void:
	var query_start := path.find("?")
	if query_start < 0:
		return
	var query := path.substr(query_start + 1)
	for pair in query.split("&", false):
		var equals := pair.find("=")
		if equals <= 0:
			continue
		var key := pair.substr(0, equals)
		var value := pair.substr(equals + 1)
		if value.is_empty():
			continue
		match key:
			"fps":
				target_fps = clampf(float(value), 1.0, 60.0)
			"w", "width", "max_width":
				max_width = clampi(int(value), 160, 3840)
			"q", "quality", "jpeg_quality":
				jpeg_quality = clampi(int(value), 1, 100)

func _apply_rgbd_query(path: String) -> void:
	var query_start := path.find("?")
	if query_start < 0:
		return
	var query := path.substr(query_start + 1)
	for pair in query.split("&", false):
		var equals := pair.find("=")
		if equals <= 0:
			continue
		var key := pair.substr(0, equals)
		var value := pair.substr(equals + 1)
		if value.is_empty():
			continue
		match key:
			"fps":
				hybrid_target_fps = clampf(float(value), 1.0, 60.0)
				hybrid_context_fps = hybrid_target_fps
				hybrid_detail_fps = hybrid_target_fps
			"context_fps":
				hybrid_context_fps = clampf(float(value), 1.0, 60.0)
			"detail_fps":
				hybrid_detail_fps = clampf(float(value), 1.0, 60.0)
			"w", "width", "max_width":
				hybrid_max_width = clampi(int(value), 160, 3840)
			"q", "quality", "jpeg_quality":
				hybrid_jpeg_quality = clampi(int(value), 1, 100)

static func merge_original_rgb_request(requests: Dictionary, options: Dictionary) -> void:
	if options.is_empty():
		return
	# A host recorder can require native RGB while another viewer asks for a
	# smaller preview. Serve the strongest request without mutating client data.
	if requests.has(options.camera):
		requests[options.camera].w = maxi(int(requests[options.camera].w), int(options.w))
		requests[options.camera].q = maxi(int(requests[options.camera].q), int(options.q))
	else:
		requests[options.camera] = options.duplicate()


func _pump_camera_rgb_encoder() -> bool:
	var requests: Dictionary = {}
	for client in _clients:
		if client.get("stream_type", "") == "camera_rgb":
			var options: Dictionary = client.get("camera_rgb_options", {})
			merge_original_rgb_request(requests, options)
	var updated := false
	for renderer in _find_rgbd_renderers():
		var serial := str(renderer.call("get_direct_realsense_serial"))
		if not requests.has(serial):
			continue
		var options: Dictionary = requests[serial]
		var profile := str(renderer.call("get_direct_realsense_stream_profile"))
		var previous: Dictionary = _camera_rgb_stream_frames.get(serial, {})
		if previous.get("profile", "") != profile:
			_camera_rgb_stream_frames.erase(serial)
			previous = {}
		var frame: Dictionary
		if renderer.has_method("request_camera_rgb_encode"):
			renderer.call("request_camera_rgb_encode",options.w,options.q)
			frame = renderer.call("take_camera_rgb_encoded_frame")
		else:
			# Older native plugins can still use the persistent transport.
			var fallback := _get_camera_rgb_frame(serial,options.w,options.q)
			if fallback.is_empty(): continue
			frame={"body":fallback.body,"width":int(fallback.headers["X-Frame-Width"]),
				"height":int(fallback.headers["X-Frame-Height"]),"sequence":int(fallback.headers["X-Frame-Sequence"]),
				"capture_unix_ms":float(fallback.headers["X-Capture-Unix-Ms"])}
		if frame.is_empty(): continue
		var source_size := Vector2i(int(frame.width), int(frame.height))
		if renderer.has_method("get_raw_color_size"):
			var size: Vector2i = renderer.call("get_raw_color_size")
			source_size = size
			var expected_width := mini(size.x, int(options.w))
			var expected_height := roundi(float(size.y) * expected_width / maxf(1, size.x))
			if int(frame.width) != expected_width or int(frame.height) != expected_height: continue
		if Time.get_unix_time_from_system() * 1000.0 - float(frame.capture_unix_ms) > 2000.0: continue
		if frame.get("sequence",-1)==previous.get("sequence",-2): continue
		frame["profile"] = profile
		frame["headers"] = {"X-Camera-Id":serial,"X-Frame-Width":str(frame.width),
			"X-Frame-Height":str(frame.height),"X-Frame-Sequence":str(frame.sequence),
			"X-Source-Width":str(source_size.x),"X-Source-Height":str(source_size.y),
			"X-Capture-Unix-Ms":str(roundi(frame.capture_unix_ms)),
			"X-Capture-Epoch":"%d:%d" % [OS.get_process_id(),renderer.get_instance_id()]}
		_camera_rgb_frame_counter += 1
		frame["counter"] = _camera_rgb_frame_counter
		_camera_rgb_stream_frames[serial] = frame
		updated = true
	return updated

func _get_camera_rgb_frame(camera_id: String = "", width_limit: int = 3840, quality: int = 95) -> Dictionary:
	# Original sensor RGB, independent of depth validity, crop and alignment.
	# Never modify the Image owned by the capture thread.
	for renderer in _find_rgbd_renderers():
		var serial := str(renderer.call("get_direct_realsense_serial"))
		if not camera_id.is_empty() and camera_id != serial:
			continue
		if not renderer.has_method("get_raw_color_image"):
			return {}
		var source: Image = renderer.call("get_raw_color_image")
		if source == null or source.is_empty():
			return {}
		var sequence := int(renderer.call("get_frame_sequence")) if renderer.has_method("get_frame_sequence") else -1
		var key := "%s:%d:%d:%d:%d" % [serial, source.get_instance_id(), sequence, width_limit, quality]
		# Bound cache age for providers without a capture sequence.
		if _camera_rgb_cache.get("key", "") == key and Time.get_ticks_msec() - int(_camera_rgb_cache.get("time", 0)) < 1000:
			return _camera_rgb_cache["frame"]
		var image := source.duplicate() as Image
		image.convert(Image.FORMAT_RGB8)
		if image.get_width() > width_limit:
			image.resize(width_limit, maxi(1, roundi(float(image.get_height()) * width_limit / image.get_width())), Image.INTERPOLATE_LANCZOS)
		var age := float(renderer.call("get_display_frame_age_ms")) if renderer.has_method("get_display_frame_age_ms") else -1.0
		var frame := {"body": image.save_jpg_to_buffer(float(quality) / 100.0), "headers": {
			"X-Camera-Id": serial.uri_encode(),
			"X-Frame-Width": str(image.get_width()), "X-Frame-Height": str(image.get_height()),
			"X-Source-Width": str(source.get_width()), "X-Source-Height": str(source.get_height()),
			"X-Frame-Sequence": str(sequence),
			"X-Capture-Unix-Ms": str(roundi(Time.get_unix_time_from_system() * 1000.0 - age)) if age >= 0.0 else "0"}}
		_camera_rgb_cache = {"key": key, "time": Time.get_ticks_msec(), "frame": frame}
		return frame
	return {}

func _send_http_response(peer: StreamPeerTCP, status: String, content_type: String, body: PackedByteArray, extra_headers: Dictionary = {}) -> void:
	var headers := (
		"HTTP/1.1 %s\r\n" % status +
		"Content-Type: %s\r\n" % content_type +
		"Content-Length: %d\r\n" % body.size() +
		"Cache-Control: no-cache, no-store, must-revalidate\r\n" +
		"Access-Control-Allow-Origin: *\r\n" +
		"Connection: close\r\n"
	)
	for key in extra_headers:
		headers += "%s: %s\r\n" % [key, extra_headers[key]]
	headers += "\r\n"
	peer.put_data(headers.to_utf8_buffer())
	if not body.is_empty():
		peer.put_data(body)

func _capture_frame(needs_encoded := true, needs_raw := false) -> void:
	if DisplayServer.get_name().to_lower() == "headless":
		return
	_refresh_tracking_metadata()
	var viewport_texture := get_viewport().get_texture()
	if viewport_texture == null:
		return
	var image := viewport_texture.get_image()
	if image == null or image.is_empty():
		return
	_publish_captured_image(image, needs_encoded, needs_raw)

func _begin_async_frame_capture(needs_encoded: bool, needs_raw: bool) -> bool:
	if not async_high_resolution_readback or max_width < async_readback_min_width or not _async_readback_available:
		return false
	# Two pipelined readbacks cover the render-frame delay without letting a burst
	# of full-resolution resize/conversion callbacks starve camera capture.
	if _async_capture_inflight >= async_readback_max_inflight:
		return true
	var rendering_device := RenderingServer.get_rendering_device()
	if rendering_device == null or not rendering_device.has_method("texture_get_data_async"):
		_async_readback_available = false
		return false
	var texture_rid := RenderingServer.viewport_get_texture(get_viewport().get_viewport_rid())
	var rd_texture_rid := RenderingServer.texture_get_rd_texture(texture_rid)
	if not rd_texture_rid.is_valid():
		_async_readback_available = false
		return false
	var texture_format = rendering_device.texture_get_format(rd_texture_rid)
	if texture_format == null or int(texture_format.format) not in [
		RenderingDevice.DATA_FORMAT_R8G8B8A8_UNORM,
		RenderingDevice.DATA_FORMAT_R8G8B8A8_SRGB,
	]:
		_async_readback_available = false
		return false
	var width := int(texture_format.width)
	var height := int(texture_format.height)
	var callback := _on_async_frame_captured.bind(width, height, needs_encoded, needs_raw)
	var error := rendering_device.texture_get_data_async(rd_texture_rid, 0, callback)
	if error != OK:
		_async_readback_available = false
		return false
	_async_capture_inflight += 1
	return true

func _on_async_frame_captured(
	data: PackedByteArray,
	width: int,
	height: int,
	needs_encoded: bool,
	needs_raw: bool,
) -> void:
	_async_capture_inflight = maxi(0, _async_capture_inflight - 1)
	needs_encoded = needs_encoded and _has_ready_stream_clients("mjpeg")
	needs_raw = needs_raw and _has_ready_stream_clients("raw")
	if not needs_encoded and not needs_raw:
		return
	var expected_size := width * height * 4
	if data.size() < expected_size:
		_async_readback_available = false
		return
	var image := Image.create_from_data(width, height, false, Image.FORMAT_RGBA8, data)
	if image == null or image.is_empty():
		return
	_refresh_tracking_metadata()
	_publish_captured_image(image, needs_encoded, needs_raw)
	_write_stream_frame_to_clients()

func _publish_captured_image(image: Image, needs_encoded: bool, needs_raw: bool) -> void:
	if image.get_width() > max_width:
		var next_height := maxi(1, int(round(float(image.get_height()) * float(max_width) / float(image.get_width()))))
		image.resize(max_width, next_height, Image.INTERPOLATE_BILINEAR)
	if needs_raw or needs_encoded:
		if image.get_format() != Image.FORMAT_RGB8:
			image.convert(Image.FORMAT_RGB8)
	if needs_raw:
		_last_raw_frame_width = image.get_width()
		_last_raw_frame_height = image.get_height()
		_last_raw_frame_format = "rgb24"
		_last_raw_frame_bytes = image.get_data()
	if needs_encoded:
		if image.has_method("save_jpg_to_buffer"):
			_last_frame_bytes = image.save_jpg_to_buffer(float(jpeg_quality) / 100.0)
			_last_frame_content_type = "image/jpeg"
		if _last_frame_bytes.is_empty():
			_last_frame_bytes = image.save_png_to_buffer()
			_last_frame_content_type = "image/png"
	_last_frame_msec = Time.get_ticks_msec()
	_last_frame_unix_ms = Time.get_unix_time_from_system() * 1000.0
	_frame_counter += 1

func _refresh_tracking_metadata() -> void:
	if Engine.is_editor_hint():
		return
	if _tracking_source == null and not tracking_source_path.is_empty():
		_tracking_source = get_node_or_null(tracking_source_path)
	if _tracking_source == null or not _tracking_source.has_method("get_tracking_latency_metadata"):
		return
	var metadata = _tracking_source.call("get_tracking_latency_metadata")
	if metadata is not Dictionary:
		return
	_last_tracking_sent_unix_ms = float(metadata.get("sent_unix_ms", 0.0))
	_last_tracking_remote_sent_unix_ms = float(metadata.get("remote_sent_unix_ms", 0.0))
	_last_tracking_bridge_recv_unix_ms = float(metadata.get("bridge_recv_unix_ms", 0.0))
	_last_tracking_transport_age_ms = float(metadata.get("transport_age_ms", -1.0))

func _pump_rgbd_encoders(delta: float) -> bool:
	var started := Time.get_ticks_usec()
	var renderers := _find_rgbd_renderers()
	if renderers.is_empty():
		return false
	for renderer in renderers:
		if not renderer.has_method("request_rgbd_encode") or not renderer.has_method("take_rgbd_encoded_frame"):
			_rgbd_frame_accum += delta
			var fallback_interval := 1.0 / maxf(1.0, hybrid_target_fps)
			if _rgbd_frame_accum < fallback_interval:
				return false
			_rgbd_frame_accum = fmod(_rgbd_frame_accum, fallback_interval)
			return _capture_rgbd_frame()

	var active_ids: Dictionary = {}
	var publish_interval := 1.0/maxf(1.0,hybrid_target_fps)
	_rgbd_frame_accum=minf(publish_interval*2.0,_rgbd_frame_accum+delta)
	for renderer in renderers:
		var renderer_id := int(renderer.get_instance_id())
		active_ids[renderer_id] = true
		var requested_fps := _renderer_hybrid_fps(renderer, renderers.size())
		var interval := 1.0 / maxf(1.0, requested_fps)
		# Allow a single jittered acquisition to arrive early. A strict minimum
		# wall-clock interval discards real 60 Hz frames when SDK delivery jitters.
		var accumulator := minf(interval*2.0,float(_rgbd_encoder_accum.get(renderer_id,interval))+delta)
		if renderer.has_method("get_frame_sequence"):
			# Follow actual RealSense acquisitions rather than a delta
			# accumulator. A short Godot frame does not cause a duplicate copy,
			# and a long frame cannot permanently phase-shift future requests.
			var sequence := int(renderer.call("get_frame_sequence"))
			var last_sequence := int(_rgbd_last_request_sequence.get(renderer_id, -1))
			if sequence != last_sequence and accumulator>=interval:
				if bool(renderer.call("request_rgbd_encode", hybrid_max_width, hybrid_jpeg_quality)):
					_rgbd_last_request_sequence[renderer_id] = sequence
					accumulator-=interval
		else:
			if accumulator >= interval:
				accumulator-=interval
				renderer.call("request_rgbd_encode", hybrid_max_width, hybrid_jpeg_quality)
		_rgbd_encoder_accum[renderer_id] = accumulator
	for renderer_id in _rgbd_encoder_accum.keys():
		if not active_ids.has(renderer_id):
			_rgbd_encoder_accum.erase(renderer_id)
	for renderer_id in _rgbd_last_request_sequence.keys():
		if not active_ids.has(renderer_id):
			_rgbd_last_request_sequence.erase(renderer_id)
	for renderer_id in _rgbd_pending_completed.keys():
		if not active_ids.has(renderer_id): _rgbd_pending_completed.erase(renderer_id)

	var completed: Array[Dictionary] = []
	_cpu_profile["encode_queue_ms"]=(Time.get_ticks_usec()-started)*0.001
	started=Time.get_ticks_usec()
	for renderer in renderers:
		var encoded = renderer.call("take_rgbd_encoded_frame")
		if encoded is Dictionary and not encoded.is_empty():
			_rgbd_pending_completed[renderer.get_instance_id()]={"renderer":renderer,"encoded":encoded}
	if _rgbd_pending_completed.is_empty() or _rgbd_frame_accum<publish_interval:
		return false
	# Coalesce within the output cadence without waiting for another camera.
	# An extra full-frame wait can halve the rate of asynchronous acquisitions.
	for entry in _rgbd_pending_completed.values(): completed.append(entry)
	_rgbd_pending_completed.clear()
	_rgbd_frame_accum-=publish_interval
	_cpu_profile["encode_take_ms"]=(Time.get_ticks_usec()-started)*0.001
	started=Time.get_ticks_usec()
	var published := _publish_encoded_rgbd_frames(completed)
	_cpu_profile["packet_pack_ms"]=(Time.get_ticks_usec()-started)*0.001
	return published

func _renderer_hybrid_fps(renderer: Node, renderer_count: int = -1) -> float:
	var model := _renderer_model(renderer).to_lower()
	if model.contains("435"):
		return hybrid_detail_fps
	# With one camera there is no separate context/detail split. Give the lone
	# D455 the full requested detail cadence instead of leaving it at the
	# dual-camera context rate (15 Hz in the Quality web preset).
	if renderer_count == 1:
		return hybrid_detail_fps
	return hybrid_context_fps

func _publish_encoded_rgbd_frames(completed: Array[Dictionary]) -> bool:
	var camera_metadata: Array[Dictionary] = []
	var payloads: Array[PackedByteArray] = []
	var capture_unix_ms := 0.0
	for entry in completed:
		var renderer = entry.get("renderer") as Node
		var encoded = entry.get("encoded") as Dictionary
		if renderer == null or encoded == null:
			continue
		var width := int(encoded.get("width", 0))
		var height := int(encoded.get("height", 0))
		var color_jpeg: PackedByteArray = encoded.get("color_jpeg", PackedByteArray())
		var compressed_depth: PackedByteArray = encoded.get("depth_compressed", PackedByteArray())
		if width <= 0 or height <= 0 or color_jpeg.is_empty() or compressed_depth.is_empty():
			continue
		var transform := (renderer as Node3D).global_transform
		var serial := str(renderer.call("get_direct_realsense_serial")) if renderer.has_method("get_direct_realsense_serial") else ""
		var model := _renderer_model(renderer)
		var intrinsics: Vector4 = encoded.get("intrinsics", Vector4.ZERO)
		var frame_capture_unix_ms := float(encoded.get("capture_unix_ms", 0.0))
		capture_unix_ms = maxf(capture_unix_ms, frame_capture_unix_ms)
		camera_metadata.append({
			"id": serial if not serial.is_empty() else str(renderer.name),
			"model": model,
			"priority": 0 if model.to_lower().contains("455") else 1,
			"width": width,
			"height": height,
			"intrinsics": [intrinsics.x, intrinsics.y, intrinsics.z, intrinsics.w],
			"transform": _transform_array(transform),
			"sequence": int(encoded.get("sequence", 0)),
			"color_length": color_jpeg.size(),
			"depth_length": compressed_depth.size(),
			"depth_encoding": str(encoded.get("depth_encoding", "u16_mm_deflate")),
			"encode_ms": float(encoded.get("encode_ms", 0.0)),
			"requested_fps": _renderer_hybrid_fps(renderer, _find_rgbd_renderers().size()),
			"color_space": str(encoded.get("color_space", "aligned_depth")),
			"color_width": int(encoded.get("color_width", width)),
			"color_height": int(encoded.get("color_height", height)),
			"color_projection": encoded.get("color_projection", {}) if str(encoded.get("color_space", "aligned_depth")) == "original_rgb" else (renderer.call("get_raw_color_projection") if renderer.has_method("get_raw_color_projection") else {}),
		})
		payloads.append(color_jpeg)
		payloads.append(compressed_depth)
	return _publish_rgbd_packet(camera_metadata, payloads, capture_unix_ms)

func _capture_rgbd_frame() -> bool:
	var renderers := _find_rgbd_renderers()
	if renderers.is_empty():
		return false
	var camera_metadata: Array[Dictionary] = []
	var payloads: Array[PackedByteArray] = []
	for renderer in renderers:
		var depth_frame = renderer.call("get_depth_u16_frame", hybrid_max_width)
		if depth_frame is not Dictionary or depth_frame.is_empty():
			continue
		var width := int(depth_frame.get("width", 0))
		var height := int(depth_frame.get("height", 0))
		var depth_mm: PackedByteArray = depth_frame.get("depth_mm", PackedByteArray())
		var color_source = renderer.call("get_color_image")
		if width <= 0 or height <= 0 or depth_mm.is_empty() or not color_source is Image:
			continue
		var color_image := (color_source as Image).duplicate() as Image
		if color_image == null or color_image.is_empty():
			continue
		if color_image.get_width() != width or color_image.get_height() != height:
			color_image.resize(width, height, Image.INTERPOLATE_BILINEAR)
		if color_image.get_format() != Image.FORMAT_RGB8:
			color_image.convert(Image.FORMAT_RGB8)
		var color_jpeg := color_image.save_jpg_to_buffer(float(hybrid_jpeg_quality) / 100.0)
		var compressed_depth := depth_mm.compress(FileAccess.COMPRESSION_DEFLATE)
		if color_jpeg.is_empty() or compressed_depth.is_empty():
			continue
		var transform := (renderer as Node3D).global_transform
		var serial := str(renderer.call("get_direct_realsense_serial")) if renderer.has_method("get_direct_realsense_serial") else ""
		var model := _renderer_model(renderer)
		var intrinsics: Vector4 = depth_frame.get("intrinsics", Vector4.ZERO)
		camera_metadata.append({
			"id": serial if not serial.is_empty() else str(renderer.name),
			"model": model,
			"priority": 0 if model.to_lower().contains("455") else 1,
			"width": width,
			"height": height,
			"intrinsics": [intrinsics.x, intrinsics.y, intrinsics.z, intrinsics.w],
			"transform": _transform_array(transform),
			"sequence": int(depth_frame.get("sequence", 0)),
			"color_length": color_jpeg.size(),
			"depth_length": compressed_depth.size(),
			"depth_encoding": "u16_mm_deflate",
			"color_projection": renderer.call("get_raw_color_projection") if renderer.has_method("get_raw_color_projection") else {},
		})
		payloads.append(color_jpeg)
		payloads.append(compressed_depth)
	return _publish_rgbd_packet(
		camera_metadata,
		payloads,
		Time.get_unix_time_from_system() * 1000.0
	)

func _publish_rgbd_packet(
		camera_metadata: Array[Dictionary],
		payloads: Array[PackedByteArray],
		capture_unix_ms: float
	) -> bool:
	if camera_metadata.is_empty():
		return false
	var viewport_camera := get_viewport().get_camera_3d()
	if Engine.is_editor_hint() or viewport_camera == null:
		var scene_root := get_node_or_null(rgbd_root_path)
		if scene_root != null:
			var scene_camera := scene_root.get_node_or_null("Camera3D") as Camera3D
			if scene_camera != null:
				viewport_camera = scene_camera
	var viewer := {}
	if viewport_camera != null:
		var initial_transform := _transform_array(viewport_camera.global_transform)
		for camera in camera_metadata:
			if str(camera.get("model", "")).to_lower().contains("455"):
				var d455_transform = camera.get("transform", [])
				if d455_transform is Array and d455_transform.size() == 16:
					initial_transform = d455_transform
				break
		viewer = {
			"transform": initial_transform,
			"fov": viewport_camera.fov,
			"near": viewport_camera.near,
			"far": viewport_camera.far,
		}
	var metadata := {
		"version": 1,
		"capture_unix_ms": capture_unix_ms if capture_unix_ms > 0.0 else Time.get_unix_time_from_system() * 1000.0,
		"cameras": camera_metadata,
		"viewer": viewer,
		"robot": _robot_overlay_metadata(),
	}
	var metadata_bytes := JSON.stringify(metadata).to_utf8_buffer()
	var packet := "RGBD1".to_ascii_buffer()
	var metadata_length := PackedByteArray()
	metadata_length.resize(4)
	metadata_length.encode_u32(0, metadata_bytes.size())
	packet.append_array(metadata_length)
	packet.append_array(metadata_bytes)
	for payload in payloads:
		packet.append_array(payload)
	_last_rgbd_frame_bytes = packet
	_rgbd_frame_counter += 1
	return true

func _robot_overlay_metadata() -> Dictionary:
	for node in get_tree().get_nodes_in_group("robot_overlay"):
		if node.has_method("get_hybrid_render_state"):
			var state = node.call("get_hybrid_render_state")
			if state is Dictionary:
				return state
	if OS.get_environment("ROBOT_TELEOP_CAPTURE_ONLY") == "1":
		# Capture deliberately runs without a robot module. The active control
		# owner publishes these poses atomically; never open a second servo or
		# telemetry connection just to decorate the camera stream.
		var mirror_path := "user://robot_overlay_render_state.json"
		if FileAccess.file_exists(mirror_path):
			var mirror = JSON.parse_string(FileAccess.get_file_as_string(mirror_path))
			if mirror is Dictionary and mirror.get("project_root") == ProjectSettings.globalize_path("res://"):
				var age := Time.get_unix_time_from_system() - float(mirror.get("updated", 0.0))
				var robot = mirror.get("robot")
				if age >= 0.0 and age < 2.0 and robot is Dictionary:
					return robot
		# Explicit unavailability lets the browser retire old transforms when
		# the owner exits, changes project, or has not published an overlay yet.
		return {"available": false}
	return {}

func _find_rgbd_renderers() -> Array[Node]:
	var result: Array[Node] = []
	var root := get_node_or_null(rgbd_root_path)
	if root == null:
		return result
	var frame := Engine.get_process_frames()
	var root_id := root.get_instance_id()
	if frame==_renderers_cache_frame and root_id==_renderers_cache_root:
		var valid := true
		for node in _renderers_cache:
			if not is_instance_valid(node): valid=false
		if valid: return _renderers_cache
	_collect_rgbd_renderers(root, result)
	result.sort_custom(func(left: Node, right: Node) -> bool: return _renderer_priority(left) < _renderer_priority(right))
	_renderers_cache=result
	_renderers_cache_frame=frame
	_renderers_cache_root=root_id
	return result

func _collect_rgbd_renderers(node: Node, result: Array[Node]) -> void:
	if node.has_method("get_depth_u16_frame") and node.has_method("get_color_image"):
		result.append(node)
	for child in node.get_children():
		_collect_rgbd_renderers(child, result)

func _renderer_model(renderer: Node) -> String:
	var parent := renderer.get_parent()
	if parent != null:
		var id := parent.get_instance_id()
		if not _model_property_cache.has(id):
			if _model_property_cache.size()>128: _model_property_cache.clear()
			var has_model := false
			for property in parent.get_property_list():
				if str(property.get("name", ""))=="model":
					has_model=true
					break
			_model_property_cache[id]=has_model
		if bool(_model_property_cache[id]): return str(parent.get("model"))
	return "RealSense"

func _renderer_priority(renderer: Node) -> int:
	var model := _renderer_model(renderer).to_lower()
	if model.contains("455"):
		return 0
	if model.contains("435"):
		return 1
	return 10

func _transform_array(transform: Transform3D) -> Array[float]:
	return [
		transform.basis.x.x, transform.basis.x.y, transform.basis.x.z, 0.0,
		transform.basis.y.x, transform.basis.y.y, transform.basis.y.z, 0.0,
		transform.basis.z.x, transform.basis.z.y, transform.basis.z.z, 0.0,
		transform.origin.x, transform.origin.y, transform.origin.z, 1.0,
	]

func _write_stream_frame_to_clients() -> void:
	if _last_frame_bytes.is_empty() and _last_raw_frame_bytes.is_empty() and _last_rgbd_frame_bytes.is_empty() and _camera_rgb_stream_frames.is_empty():
		return
	var remove_indices: Array[int] = []
	var encoded_part_header := (
		"--%s\r\n" % BOUNDARY +
		"Content-Type: %s\r\n" % _last_frame_content_type +
		"Content-Length: %d\r\n" % _last_frame_bytes.size() +
		"X-Frame-Time-Msec: %d\r\n" % _last_frame_msec +
		"X-Capture-Unix-Ms: %.0f\r\n" % _last_frame_unix_ms +
		"X-Tracking-Sent-Unix-Ms: %.0f\r\n" % _last_tracking_sent_unix_ms +
		"X-Tracking-Remote-Sent-Unix-Ms: %.0f\r\n" % _last_tracking_remote_sent_unix_ms +
		"X-Tracking-Bridge-Recv-Unix-Ms: %.0f\r\n" % _last_tracking_bridge_recv_unix_ms +
		"X-Tracking-Transport-Age-Ms: %.1f\r\n" % _last_tracking_transport_age_ms +
		"\r\n"
	).to_utf8_buffer()
	var raw_part_header := (
		"--%s\r\n" % BOUNDARY +
		"Content-Type: application/octet-stream\r\n" +
		"Content-Length: %d\r\n" % _last_raw_frame_bytes.size() +
		"X-Frame-Width: %d\r\n" % _last_raw_frame_width +
		"X-Frame-Height: %d\r\n" % _last_raw_frame_height +
		"X-Frame-Format: %s\r\n" % _last_raw_frame_format +
		"X-Frame-Time-Msec: %d\r\n" % _last_frame_msec +
		"X-Capture-Unix-Ms: %.0f\r\n" % _last_frame_unix_ms +
		"X-Tracking-Sent-Unix-Ms: %.0f\r\n" % _last_tracking_sent_unix_ms +
		"X-Tracking-Remote-Sent-Unix-Ms: %.0f\r\n" % _last_tracking_remote_sent_unix_ms +
		"X-Tracking-Bridge-Recv-Unix-Ms: %.0f\r\n" % _last_tracking_bridge_recv_unix_ms +
		"X-Tracking-Transport-Age-Ms: %.1f\r\n" % _last_tracking_transport_age_ms +
		"\r\n"
	).to_utf8_buffer()
	var rgbd_part_header := (
		"--%s\r\n" % BOUNDARY +
		"Content-Type: application/octet-stream\r\n" +
		"Content-Length: %d\r\n" % _last_rgbd_frame_bytes.size() +
		"X-RGBD-Frame: %d\r\n" % _rgbd_frame_counter +
		"\r\n"
	).to_utf8_buffer()
	var part_footer := "\r\n".to_utf8_buffer()
	for i in _clients.size():
		var client := _clients[i]
		if not bool(client.get("streaming", false)):
			continue
		var peer := client.get("peer") as StreamPeerTCP
		if peer == null or peer.get_status() != StreamPeerTCP.STATUS_CONNECTED:
			remove_indices.append(i)
			continue
		var stream_type := str(client.get("stream_type", ""))
		var is_raw := stream_type == "raw"
		var is_rgbd := stream_type == "rgbd"
		var is_camera_rgb := stream_type == "camera_rgb"
		var rgb_options: Dictionary = client.get("camera_rgb_options", {})
		var rgb_frame: Dictionary = _camera_rgb_stream_frames.get(str(rgb_options.get("camera", "")), {})
		if is_camera_rgb and (int(client.get("last_camera_rgb_frame",-1)) == int(rgb_frame.get("counter",-2)) or rgb_frame.is_empty()):
			continue
		if is_camera_rgb and Time.get_unix_time_from_system() * 1000.0 - float(rgb_frame.get("capture_unix_ms", 0)) > 2000.0:
			continue
		if is_rgbd and int(client.get("last_rgbd_frame", -1)) == _rgbd_frame_counter:
			continue
		var part_header := rgbd_part_header if is_rgbd else (raw_part_header if is_raw else encoded_part_header)
		var frame_bytes := _last_rgbd_frame_bytes if is_rgbd else (_last_raw_frame_bytes if is_raw else _last_frame_bytes)
		if is_camera_rgb:
			frame_bytes = rgb_frame.body
			var text := "--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n" % [BOUNDARY,frame_bytes.size()]
			for key in rgb_frame.headers:
				text += "%s: %s\r\n" % [key,rgb_frame.headers[key]]
			part_header = (text+"\r\n").to_utf8_buffer()
		if frame_bytes.is_empty():
			continue
		if int(client.get("pending_segment", -1)) >= 0:
			# Keep a complete multipart frame intact. Once it drains, the client gets
			# the newest frame instead of accumulating latency in a queue.
			continue
		# Keep the pixel buffer separate so the common raw path does not copy the
		# entire multi-megabyte frame just to prepend a small multipart header.
		client["pending_header"] = part_header
		client["pending_frame"] = frame_bytes
		client["pending_footer"] = part_footer
		client["pending_segment"] = 0
		client["pending_offset"] = 0
		client["pending_started_msec"] = Time.get_ticks_msec()
		if is_rgbd:
			client["last_rgbd_frame"] = _rgbd_frame_counter
		if is_camera_rgb:
			client["last_camera_rgb_frame"] = int(rgb_frame.counter)
		if not _flush_client_output(client):
			remove_indices.append(i)
		_clients[i] = client
	for index in range(remove_indices.size() - 1, -1, -1):
		_disconnect_client(remove_indices[index])

func _flush_client_output(client: Dictionary) -> bool:
	var segment_index := int(client.get("pending_segment", -1))
	if segment_index < 0:
		return true
	var started := int(client.get("pending_started_msec", 0))
	if started > 0 and Time.get_ticks_msec() - started > STREAM_STALL_TIMEOUT_MSEC:
		return false
	var peer := client.get("peer") as StreamPeerTCP
	if peer == null or peer.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		return false
	var offset := int(client.get("pending_offset", 0))
	var sent_this_poll := 0
	var poll_started_usec := Time.get_ticks_usec()
	while segment_index < 3 and sent_this_poll < STREAM_SEND_BUDGET_BYTES:
		if Time.get_ticks_usec() - poll_started_usec >= STREAM_SEND_BUDGET_USEC:
			break
		var segment: PackedByteArray
		match segment_index:
			0:
				segment = client.get("pending_header", PackedByteArray())
			1:
				segment = client.get("pending_frame", PackedByteArray())
			_:
				segment = client.get("pending_footer", PackedByteArray())
		if offset >= segment.size():
			segment_index += 1
			offset = 0
			continue
		var remaining_budget := STREAM_SEND_BUDGET_BYTES - sent_this_poll
		var send_bytes: PackedByteArray
		if offset == 0 and segment.size() <= remaining_budget:
			# PackedByteArray is copy-on-write, so this passes the captured frame to
			# StreamPeer without allocating another full-sized temporary buffer.
			send_bytes = segment
		else:
			var chunk_end := mini(segment.size(), offset + mini(STREAM_SEND_CHUNK_BYTES, remaining_budget))
			send_bytes = segment.slice(offset, chunk_end)
		var result := peer.put_partial_data(send_bytes)
		if result.size() < 2:
			return false
		var error := int(result[0])
		var bytes_sent := int(result[1])
		if error == ERR_BUSY or bytes_sent <= 0:
			break
		if error != OK:
			return false
		offset += bytes_sent
		sent_this_poll += bytes_sent
		if offset >= segment.size():
			segment_index += 1
			offset = 0
	if segment_index >= 3:
		client["pending_header"] = PackedByteArray()
		client["pending_frame"] = PackedByteArray()
		client["pending_footer"] = PackedByteArray()
		client["pending_segment"] = -1
		client["pending_offset"] = 0
		client["pending_started_msec"] = 0
	else:
		client["pending_segment"] = segment_index
		client["pending_offset"] = offset
	return true

func _disconnect_client(index: int) -> void:
	if index < 0 or index >= _clients.size():
		return
	var peer := _clients[index].get("peer") as StreamPeerTCP
	if peer != null:
		peer.disconnect_from_host()
	_clients.remove_at(index)

func _update_status() -> void:
	var now := Time.get_ticks_msec()
	if now - _last_status_msec < 500:
		return
	_last_status_msec = now
	var listening := _server.is_listening()
	stream_status = "%s clients=%d fps=%.1f frame=%d bytes=%d" % [
		"listening" if listening else "stopped",
		_clients.size(),
		target_fps,
		_frame_counter,
		maxi(_last_frame_bytes.size(), _last_raw_frame_bytes.size()),
	]

func _has_stream_clients(stream_type: String) -> bool:
	for client in _clients:
		if bool(client.get("streaming", false)) and str(client.get("stream_type", "")) == stream_type:
			return true
	return false

func _has_ready_stream_clients(stream_type: String) -> bool:
	for client in _clients:
		if (
			bool(client.get("streaming", false))
			and str(client.get("stream_type", "")) == stream_type
			and int(client.get("pending_segment", -1)) < 0
		):
			return true
	return false
