extends SceneTree
const Stream = preload("res://godot/runtime/streaming/viewport_stream_server.gd")
var failures := 0
var checks := 0

class Source extends Node:
	var image := Image.create(320, 180, false, Image.FORMAT_RGBA8)
	var sequence := 1
	func get_direct_realsense_serial() -> String: return "1234"
	func get_raw_color_image() -> Image: return image
	func get_color_image() -> Image: return Image.create(64, 32, false, Image.FORMAT_RGB8)
	func get_frame_sequence() -> int: return sequence
	func get_display_frame_age_ms() -> float: return 10.0

class Server extends Stream:
	var source: Node
	func _find_rgbd_renderers() -> Array[Node]: return [source]

func check(ok: bool, label: String) -> void:
	checks += 1
	if not ok:
		failures += 1
		push_error(label)

func _initialize() -> void:
	var source := Source.new()
	source.image.fill(Color.BLUE)
	var server := Server.new()
	server.stream_enabled = false
	server.source = source
	var frame: Dictionary = server._get_camera_rgb_frame()
	check(not frame.is_empty(), "original RGB available")
	check(frame.headers["X-Frame-Width"] == "320" and frame.headers["X-Frame-Height"] == "180", "native dimensions without upscale")
	var decoded := Image.new()
	decoded.load_jpg_from_buffer(frame.body)
	check(decoded.get_pixel(100, 100).b > 0.98, "raw RGB survives despite empty processed color/depth")
	check(source.image.get_format() == Image.FORMAT_RGBA8, "capture image remains unmodified")
	check(server._get_camera_rgb_frame().body == frame.body, "unchanged capture reuses JPEG")
	source.image.fill(Color.RED)
	source.sequence += 1
	var changed: Dictionary = server._get_camera_rgb_frame()
	check(changed.body != frame.body and changed.headers["X-Frame-Sequence"] == "2", "new sequence invalidates JPEG")
	var small: Dictionary = server._get_camera_rgb_frame("1234", 160)
	check(small.headers["X-Frame-Width"] == "160" and small.headers["X-Frame-Height"] == "90", "optional limit preserves aspect")
	check(server._get_camera_rgb_frame("missing").is_empty(), "unknown camera never falls back")
	source.image = null
	check(server._get_camera_rgb_frame().is_empty(), "unavailable RGB never substitutes depth-aligned color")
	server.free()
	source.free()
	print("CAMERA_RGB checks=%d failures=%d" % [checks, failures])
	quit(0 if failures == 0 else 1)
