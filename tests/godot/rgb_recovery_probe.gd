extends SceneTree
const View = preload("res://godot/runtime/point_cloud/unified_point_cloud_view.gd")

class Camera:
	extends MeshInstance3D
	var age := 0.0
	var rgb_age := 0.0
	var rgb_enabled := true
	var rgb_metadata := true
	var enabled := true
	var restarts := 0
	func get_display_frame_age_ms() -> float: return age
	func get_color_diagnostics() -> Dictionary:
		if not rgb_metadata: return {}
		return {"rgb_enabled":rgb_enabled,"rgb_frame_age_ms":rgb_age}
	func set_direct_realsense_enabled(value:bool) -> void: enabled=value
	func restart_direct_realsense() -> void: restarts+=1

func _initialize() -> void:
	var view := View.new()
	var camera := Camera.new()
	assert(view._capture_active_after_recovery("fresh",camera,true))
	assert(camera.restarts==0)
	camera.rgb_age=6000.0
	assert(not view._capture_active_after_recovery("rgb",camera,true))
	assert(camera.restarts==1 and not camera.enabled)
	# The same camera is rate limited, while another camera recovers independently.
	assert(view._capture_active_after_recovery("rgb",camera,true))
	assert(camera.restarts==1)
	assert(not view._capture_active_after_recovery("other",camera,true))
	assert(camera.restarts==2)
	camera.rgb_enabled=false
	assert(view._capture_active_after_recovery("depth-only",camera,true))
	camera.rgb_enabled=true
	camera.rgb_metadata=false
	assert(view._capture_active_after_recovery("older-extension",camera,true))
	assert(camera.restarts==2)
	camera.rgb_metadata=true
	camera.rgb_age=-1.0
	assert(view._capture_active_after_recovery("initializing-rgb",camera,true))
	camera.age=6000.0
	assert(not view._capture_active_after_recovery("depth",camera,true))
	assert(camera.restarts==3)
	view._capture_paused=true
	assert(view._capture_active_after_recovery("lease",camera,true))
	assert(camera.restarts==3)
	view._capture_paused=false
	assert(not view._capture_active_after_recovery("inactive",camera,false))
	assert(camera.restarts==3)
	camera.free()
	view.free()
	print("RGB_RECOVERY passed")
	quit(0)
