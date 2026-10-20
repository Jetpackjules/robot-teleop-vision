extends SceneTree
## Dedicated sensor/encoding clock. The editor reads its shared snapshots and
## retains its own UI/control services. No second robot or UDP gateway is run.

func _initialize() -> void:
	if DisplayServer.get_name() != "headless" or OS.get_environment("ROBOT_TELEOP_CAPTURE_SHM_PREFIX").is_empty():
		push_error("Camera capture service requires --headless and a unique ROBOT_TELEOP_CAPTURE_SHM_PREFIX.")
		quit(1)
		return
	OS.set_environment("ROBOT_TELEOP_CAPTURE_ONLY","1")
	OS.set_environment("ROBOT_TELEOP_MODULE","disabled")
	call_deferred("start")

func start() -> void:
	var scene:=load("res://godot/Main.tscn") as PackedScene
	if scene==null:
		quit(1)
		return
	var view:=scene.instantiate()
	for name in ["RobotModuleHost","RemoteControlGateway"]:
		var child:=view.get_node_or_null(NodePath(name))
		if child!=null: child.free()
	view.set("show_debug_panel",false)
	root.add_child(view)
