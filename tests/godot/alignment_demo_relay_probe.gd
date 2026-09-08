extends SceneTree
## Receives real UDP from the test's real WebSocket relay. No hardware is loaded.

var failures: Array[String] = []


func check(condition: bool, message: String) -> void:
	if not condition:
		failures.append(message)
		printerr("RELAY_PROBE_FAIL: ", message)


func _initialize() -> void:
	call_deferred("run")


func wait_for_packets(demo: Node, count: int) -> bool:
	var deadline := Time.get_ticks_msec() + 8000
	while demo.input_bridge.accepted_packets < count and Time.get_ticks_msec() < deadline:
		await process_frame
	return demo.input_bridge.accepted_packets >= count


func run() -> void:
	var scene := load("res://examples/alignment_demo/godot/AlignmentDemo.tscn") as PackedScene
	if scene == null:
		printerr("RELAY_PROBE_FAIL: Could not load the isolated demo scene")
		quit(1)
		return
	var demo = scene.instantiate()
	root.add_child(demo)
	demo.cloud.enabled = false
	await physics_frame
	check(demo.input_bridge.bound, "Explicit simulation UDP listener binds")
	check(demo.get_demo_state().input_port == demo.input_bridge.port and demo.input_bridge.port > 1023,
		"Preview reports the actual bound simulation UDP port")
	print("ALIGNMENT_RELAY_READY")
	if not await wait_for_packets(demo, 1):
		check(false, "First WebSocket-relayed UDP packet reached Godot")
		quit(1)
		return
	check(demo._head_active, "First relayed head sample is active")
	check(demo._head_displacement.length() < .00001, "First head sample establishes neutral")
	check(is_equal_approx(float(demo.arm.normalized[5]), 75.0), "First packet opens virtual gripper")
	print("ALIGNMENT_RELAY_STAGE1")
	if not await wait_for_packets(demo, 2):
		check(false, "Second WebSocket-relayed UDP packet reached Godot")
		quit(1)
		return
	check(demo._head_displacement.distance_to(Vector3(.1,.05,.05)) < .00001,
		"Centimetre head delta changes the viewpoint in metres")
	check(is_equal_approx(float(demo.arm.normalized[0]), -30.0), "Changed leader base sample drives virtual joint")
	check(is_equal_approx(float(demo.arm.normalized[5]), 20.0), "Changed leader sample closes virtual gripper")
	check(demo._external_arm_active, "Fresh leader sample has control of virtual arm")
	print("ALIGNMENT_RELAY_STAGE2")
	await create_timer(.45).timeout
	check(not demo._external_arm_active, "Missing updates stop external joint input after 300 ms")
	check(not demo._head_active, "Missing updates stop head input after 300 ms")
	check(demo.input_bridge.accepted_packets == 2, "Stale browser packet was not relayed")
	check(is_equal_approx(float(demo.arm.normalized[0]), -30.0), "Stale input preserves the last joint pose")
	print("ALIGNMENT_RELAY_PROBE ", JSON.stringify({
		"passed":failures.is_empty(), "failures":failures,
		"final":demo.get_demo_state()
	}))
	quit(0 if failures.is_empty() else 1)
