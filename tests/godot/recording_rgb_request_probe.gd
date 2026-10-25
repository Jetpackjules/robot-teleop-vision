extends SceneTree

const StreamServer = preload("res://godot/runtime/streaming/viewport_stream_server.gd")


func _initialize() -> void:
	var requests := {}
	var small := {"camera": "one", "w": 640, "q": 80}
	var recording := {"camera": "one", "w": 3840, "q": 95}
	StreamServer.merge_original_rgb_request(requests, small)
	StreamServer.merge_original_rgb_request(requests, recording)
	StreamServer.merge_original_rgb_request(requests, small)
	assert(requests.one.w == 3840 and requests.one.q == 95)
	assert(small.w == 640 and small.q == 80)
	assert(recording.w == 3840 and recording.q == 95)
	StreamServer.merge_original_rgb_request(requests, {"camera": "two", "w": 1920, "q": 99})
	assert(requests.two.w == 1920 and requests.two.q == 99)
	assert(requests.one.w == 3840 and requests.one.q == 95)
	print("HOST_RECORDING_RGB_REQUESTS=PASS")
	quit(0)
