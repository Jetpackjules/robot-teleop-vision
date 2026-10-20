extends SceneTree
const View = preload("res://godot/runtime/point_cloud/unified_point_cloud_view.gd")

func _initialize() -> void:
	var pid:=int(OS.get_cmdline_user_args()[0])
	var view:=View.new()
	assert(view._capture_lease_process_alive(pid),"A live sibling/parent process owns a valid lease")
	assert(not view._capture_lease_process_alive(0),"Missing lease owner is invalid")
	var path:=ProjectSettings.globalize_path("user://capture_control_%s.bin"%OS.get_environment("ROBOT_TELEOP_CAPTURE_SHM_PREFIX"))
	DirAccess.make_dir_recursive_absolute(path.get_base_dir())
	var state:Dictionary={"cameras":{},"paused":true,"pid":pid,"updated":Time.get_unix_time_from_system(),"stamp":1}
	var file:=FileAccess.open(path,FileAccess.WRITE)
	file.store_var(state,false);file.close()
	view._capture_control_last_check=-1000
	view._sync_capture_control()
	assert(view._capture_paused,"A fresh foreign-process heartbeat pauses acquisition")
	state.updated=Time.get_unix_time_from_system()-4
	file=FileAccess.open(path,FileAccess.WRITE)
	file.store_var(state,false);file.close()
	view._capture_control_last_check=-1000
	view._sync_capture_control()
	assert(not view._capture_paused,"Lease expiry resumes acquisition even when the settings stamp is unchanged")
	DirAccess.remove_absolute(path)
	view.free()
	print("CAPTURE_CONTROL foreign_process_and_lease_expiry=passed")
	quit(0)
