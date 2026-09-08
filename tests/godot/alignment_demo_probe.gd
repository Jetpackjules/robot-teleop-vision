extends SceneTree
## Run with --headless --fixed-fps 60 --script res://tests/godot/alignment_demo_probe.gd.

var failures: Array[String] = []

func check(condition: bool, message: String) -> void:
	if not condition:
		failures.append(message)
		printerr("FAIL: ", message)

func _initialize() -> void:
	call_deferred("run")

func packet(seq: int) -> Dictionary:
	return {"type":"alignment_demo","version":1,"source":"headless-test","seq":seq,
		"timestamp_ms":Time.get_unix_time_from_system()*1000.0,
		"arm":{"active":false},"head":{"active":false}}

func run() -> void:
	# Simulate repeated browser reconnects without waiting a minute in real time.
	# The two supplied clocks affect this isolated receiver only, never UDP data.
	var Receiver := preload("res://examples/alignment_demo/godot/simulation_input.gd")
	var receiver := Receiver.new()
	var wall := 1000000.0
	var oldest := packet(1)
	oldest.source = "sequential-session-0"
	oldest.timestamp_ms = wall
	var recent: Dictionary
	for index in range(129):
		var elapsed := index*500
		recent = packet(1)
		recent.source = "sequential-session-%d" % index
		recent.timestamp_ms = wall+elapsed
		check(receiver.accept_packet(recent,wall+elapsed,elapsed),"Sequential session %d accepted" % index)
		var active := packet(index+1)
		active.source = "active-session"
		active.timestamp_ms = wall+elapsed
		check(receiver.accept_packet(active,wall+elapsed,elapsed),"Active source remains accepted")
		check(not receiver.accept_packet(active,wall+elapsed,elapsed),"Active duplicate sequence remains rejected")
		check(receiver._sequences.size() <= 8,"Expired session history stays bounded")
	check(not receiver.accept_packet(oldest,wall+64000,64000),"Retired source's original stale packet remains rejected")
	check(not receiver.accept_packet(recent,wall+64000,64000),"Newest session duplicate remains rejected")
	check(not receiver._sequences.has("sequential-session-0"),"Inactive source was retired")
	receiver.free()
	var packed := load("res://examples/alignment_demo/godot/AlignmentDemo.tscn") as PackedScene
	check(packed != null,"Standalone scene loads")
	var demo = packed.instantiate()
	root.add_child(demo)
	demo.cloud.enabled = false
	await physics_frame
	check(not demo.input_bridge.bound,"No socket is bound without the explicit simulation port")
	check(demo.get_demo_state().input_port == 0,"Preview reports the actual unbound input port")
	var sample := packet(1)
	sample.head = {"active":true,"x":0,"y":0,"z":60,"units":"cm"}
	check(demo.apply_simulation_packet(sample),"Valid head sample accepted")
	check(not demo.apply_simulation_packet(sample),"Replay packet rejected")
	sample = packet(2)
	sample.head = {"active":true,"x":10,"y":5,"z":60,"units":"cm"}
	check(demo.apply_simulation_packet(sample),"Fresh changed head sample accepted")
	demo._update_camera()
	check(demo._head_displacement.distance_to(Vector3(.1,.05,0)) < .00001,"Head centimetres use a relative neutral origin")
	var invalid := packet(3)
	invalid.timestamp_ms -= 1500
	check(not demo.apply_simulation_packet(invalid),"Stale sample rejected")
	invalid = packet(3)
	invalid.arm = {"active":true,"normalized":[0,0,0,0,0,101]}
	check(not demo.apply_simulation_packet(invalid),"Out-of-range gripper rejected")
	invalid.arm.normalized[5] = NAN
	check(not demo.apply_simulation_packet(invalid),"Non-finite joint rejected")
	sample = packet(3)
	sample.arm = {"active":true,"normalized":[-40.4296875,130,38,78,0,25],"source":"manual"}
	check(demo.apply_simulation_packet(sample),"Manual joint sample accepted")
	check(is_equal_approx(demo.arm.normalized[5],25),"Sample drives virtual gripper")
	# Independent fixture from the existing Python SO-101 arm_pose/URDF function,
	# converted ROS->Godot and translated by this scene's virtual base position.
	check(demo.arm.tool_transform().origin.distance_to(Vector3(.1900112554,.2467666339,-.1612605944)) < .00001,"Virtual FK matches the existing SO-101 model fixture")
	check(not demo.held,"Closing far from the block cannot grasp it")
	var stopped_pose: Vector3 = demo.arm.tool_transform().origin
	check(demo.apply_simulation_packet(packet(4)),"Explicit inactive packet accepted")
	check(not demo._external_arm_active and demo.input_mode.begins_with("Keyboard"),"Inactive input returns control to keyboard")
	check(demo.arm.tool_transform().origin.distance_to(stopped_pose) < .000001,"Input deactivation preserves the virtual pose")
	check(not demo.point_cloud_mode,"Interactive workspace starts with solid geometry")
	sample = packet(5)
	sample.action = "render_cloud"
	check(demo.apply_simulation_packet(sample) and demo.point_cloud_mode,"Browser can select synthetic depth")
	sample = packet(6)
	sample.action = "view_side"
	check(demo.apply_simulation_packet(sample) and is_equal_approx(demo._orbit_yaw,-.75),"Browser can select side view")
	sample = packet(7)
	sample.action = "render_solid"
	check(demo.apply_simulation_packet(sample) and not demo.point_cloud_mode,"Browser can restore solid geometry")
	sample = packet(8)
	sample.action = "view_front"
	check(demo.apply_simulation_packet(sample) and is_zero_approx(demo._orbit_yaw),"Browser can restore front view")
	demo.reset_demo()
	demo.cloud.sample_width = 24
	demo.cloud.sample_height = 18
	demo.cloud.refresh_cloud()
	check(demo.cloud.sample_count > 100,"Synthetic fixed-sensor depth rays hit scene geometry")
	demo.start_replay()
	var saw_held := false
	var saw_lift := false
	var saw_drop := false
	for frame in range(18 * Engine.physics_ticks_per_second):
		await physics_frame
		saw_held = saw_held or demo.held
		saw_lift = saw_lift or (demo.held and demo.block.position.y > .13)
		saw_drop = saw_drop or (not demo.held and demo.block.linear_velocity.y < -.2 and demo.replay_time > 13)
		if frame % (2 * Engine.physics_ticks_per_second) == 0:
			print("PROBE_STATE ",JSON.stringify(demo.get_demo_state()))
	check(saw_held,"Replay grasps by proximity")
	check(saw_lift,"Attached block follows the moving FK tool above the table")
	check(saw_drop,"Released block falls under gravity")
	check(demo.completed,"Released block settles within the open collision bowl")
	var passed_final: Dictionary = demo.get_demo_state()
	demo.reset_demo()
	for _frame in range(60):
		await physics_frame
	check(not demo.completed,"A block resting outside the bowl is not reported as placed")
	print("ALIGNMENT_DEMO_PROBE ", JSON.stringify({"passed":failures.is_empty(),"failures":failures,"final":passed_final}))
	quit(0 if failures.is_empty() else 1)
