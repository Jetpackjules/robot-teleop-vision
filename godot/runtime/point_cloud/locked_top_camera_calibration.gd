extends "res://godot/runtime/point_cloud/realsense_rig_calibration.gd"
## Independently measure A against B and C; only A may change world pose.
const Rig = preload("res://godot/runtime/point_cloud/realsense_rig_calibration.gd")
const MAX_REFERENCE_DISAGREEMENT_M := 0.010
const MAX_REFERENCE_DISAGREEMENT_DEGREES := 1.0
var target_id := ""
var fixed_ids: Array[String] = []
var estimates: Array[Transform3D] = []
var pair_index := 0

func begin_locked(target: String, references: Array[String], previous: Dictionary) -> bool:
	if references.size() != 2 or references[0] == references[1] or target in references:
		return false
	for camera in references + [target]:
		if not previous.get(camera) is Transform3D:
			return false
	target_id = target
	fixed_ids = references.duplicate()
	cameras = references.duplicate()
	cameras.append(target)
	reference_id = references[0]
	transforms = {}
	for camera in references:
		transforms[camera] = previous[camera]
	estimates.clear()
	results.clear()
	failures.clear()
	active_pair.clear()
	pair_index = 0
	return true

func next_pair() -> Dictionary:
	active_pair = {}
	if pair_index < fixed_ids.size():
		active_pair = {"reference_camera":fixed_ids[pair_index], "target_camera":target_id}
		pair_index += 1
	return active_pair.duplicate()

func accept(result: Dictionary) -> bool:
	if active_pair.is_empty():
		return false
	var reference := str(active_pair.reference_camera)
	var solved = Rig.transform_from_payload(result)
	var valid := bool(result.get("ok",false)) and solved is Transform3D
	valid = valid and str(result.get("reference_camera",reference)) == reference
	valid = valid and str(result.get("target_camera",target_id)) == target_id
	active_pair = {}
	if not valid:
		failures.append("%s could not independently measure A: %s" % [reference,result.get("status","Invalid board pose")])
		return false
	estimates.append((transforms[reference] as Transform3D) * (solved as Transform3D))
	results[reference] = result.duplicate(true)
	if estimates.size() == 2:
		var distance := estimates[0].origin.distance_to(estimates[1].origin)
		var q0 := estimates[0].basis.get_rotation_quaternion()
		var q1 := estimates[1].basis.get_rotation_quaternion()
		var angle := rad_to_deg(q0.angle_to(q1))
		if distance > MAX_REFERENCE_DISAGREEMENT_M or angle > MAX_REFERENCE_DISAGREEMENT_DEGREES:
			failures.append("B/C disagree on A by %.1f mm / %.2f degrees; no calibration changed." % [distance*1000.0,angle])
			return false
		transforms[target_id] = Transform3D(Basis(q0.slerp(q1,0.5)),(estimates[0].origin+estimates[1].origin)*0.5)
	return true

func complete() -> bool:
	return estimates.size() == 2 and failures.is_empty() and transforms.has(target_id)

static func replace_target(registry: Dictionary, target: String, transform: Transform3D, status: String) -> Dictionary:
	var previous := Rig.resolve_registry(registry)
	if not previous.has(target) or registry.get("invalid",false):
		return {}
	var updated := registry.duplicate(true)
	var entry := Rig.payload(transform)
	entry.merge({"coordinate_space":"world","method":"native_charuco_locked_top_only","updated_at":Time.get_unix_time_from_system(),"status":status})
	updated.transforms[target] = entry
	var resolved := Rig.resolve_registry(updated)
	# Refuse legacy dependency chains where changing A would indirectly move B/C.
	for camera in previous:
		if camera != target and (not resolved.has(camera) or resolved[camera] != previous[camera]):
			return {}
	return updated
