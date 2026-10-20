extends SceneTree
const Stream = preload("res://godot/runtime/streaming/viewport_stream_server.gd")
var failures := 0
var checks := 0

class Source extends Node:
	var image := Image.create(320, 180, false, Image.FORMAT_RGBA8)
	var sequence := 1
	var requests: Array[int]=[]
	var completed: Dictionary={}
	func get_direct_realsense_serial() -> String: return "1234"
	func get_raw_color_image() -> Image: return image
	func get_color_image() -> Image: return Image.create(64, 32, false, Image.FORMAT_RGB8)
	func get_frame_sequence() -> int: return sequence
	func get_display_frame_age_ms() -> float: return 10.0
	func get_depth_u16_frame(_width: int = 3840) -> Dictionary: return {}
	func request_rgbd_encode(_width: int,_quality: int) -> bool:
		requests.append(sequence)
		return true
	func take_rgbd_encoded_frame() -> Dictionary:
		var result:=completed
		completed={}
		return result

class CameraParent extends Node:
	var model := "D435"
	var inspections := 0
	func _get_property_list() -> Array[Dictionary]:
		inspections+=1
		return []

class CountingServer extends Stream:
	var visits := 0
	func _collect_rgbd_renderers(node: Node,result: Array[Node]) -> void:
		visits+=1
		super._collect_rgbd_renderers(node,result)

class Server extends Stream:
	var source: Node
	func _find_rgbd_renderers() -> Array[Node]: return [source]

class BatchServer extends Stream:
	var sources: Array[Node]=[]
	var batches: Array=[]
	func _find_rgbd_renderers() -> Array[Node]: return sources
	func _publish_encoded_rgbd_frames(completed: Array[Dictionary]) -> bool:
		batches.append(completed.map(func(entry): return entry.encoded.sequence))
		return true

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
	var parent := CameraParent.new()
	var camera := Source.new()
	var cached := CountingServer.new()
	cached.stream_enabled=false
	parent.add_child(camera)
	parent.add_child(cached)
	root.add_child(parent)
	check(cached._find_rgbd_renderers().size()==1,"renderer discovery finds the camera")
	var visits := cached.visits
	cached._find_rgbd_renderers()
	check(cached.visits==visits,"repeated queries reuse renderer discovery within a frame")
	check(cached._renderer_model(camera)=="D435","camera model discovered")
	var inspections := parent.inspections
	parent.model="D455"
	check(cached._renderer_model(camera)=="D455" and parent.inspections==inspections,"live model updates do not rebuild Inspector metadata")
	camera.free()
	check(cached._find_rgbd_renderers().is_empty(),"freed cameras invalidate the frame cache immediately")
	parent.free()
	var paced_source:=Source.new()
	var paced:=Server.new()
	paced.stream_enabled=false
	paced.hybrid_detail_fps=60
	paced.source=paced_source
	for i in 20:
		paced_source.sequence+=1
		paced._pump_rgbd_encoders(0.010 if i%2==0 else 0.0233333333)
	check(paced_source.requests.size()==20,"60 Hz pacing retains early/late acquisitions without duplicate frames")
	paced._pump_rgbd_encoders(0.1)
	check(paced_source.requests.size()==20,"pacing never re-encodes an unchanged acquisition")
	paced.free()
	paced=Server.new()
	paced.stream_enabled=false
	paced.hybrid_detail_fps=60
	paced.source=paced_source
	paced_source.requests.clear()
	for i in 20:
		paced_source.sequence+=1
		paced._pump_rgbd_encoders(0.01)
	check(paced_source.requests.size()>=11 and paced_source.requests.size()<=14,"bounded pacing still caps a faster source to the requested cadence")
	paced.free()
	paced_source.free()
	var batch:=BatchServer.new()
	batch.stream_enabled=false
	batch.hybrid_context_fps=60
	for i in 3: batch.sources.append(Source.new())
	batch.sources[0].completed={"sequence":10}
	batch._pump_rgbd_encoders(0.005)
	batch.sources[1].completed={"sequence":20}
	batch._pump_rgbd_encoders(0.006)
	check(batch.batches.is_empty(),"asynchronous camera completions wait for one coalesced packet")
	batch.sources[2].completed={"sequence":30}
	batch._pump_rgbd_encoders(0.006)
	check(batch.batches.size()==1 and batch.batches[0]==[10,20,30],"one packet retains all three asynchronously completed cameras")
	batch.sources[0].completed={"sequence":11}
	batch._pump_rgbd_encoders(0.005)
	batch.sources[0].completed={"sequence":12}
	batch._pump_rgbd_encoders(0.02)
	check(batch.batches.size()==2 and batch.batches[1]==[12],"a stalled camera cannot block newer frames and pending captures stay latest-only")
	for item in batch.sources: item.free()
	batch.free()
	print("CAMERA_RGB checks=%d failures=%d" % [checks, failures])
	quit(0 if failures == 0 else 1)
