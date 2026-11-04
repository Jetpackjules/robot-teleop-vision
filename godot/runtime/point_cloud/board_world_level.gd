extends RefCounted
## One common rigid transform: camera-to-camera and robot registration stay fixed.
const Rig = preload("res://godot/runtime/point_cloud/realsense_rig_calibration.gd")

static func solve(transforms: Dictionary, results: Dictionary) -> Dictionary:
	if transforms.size() < 2 or results.is_empty():
		return {"ok":false,"message":"No measured board pose is available."}
	var camera_center := Vector3.ZERO
	for camera in transforms:
		camera_center += (transforms[camera] as Transform3D).origin
	camera_center /= transforms.size()
	var points: Array[Vector3] = []
	var normals: Array[Vector3] = []
	for target in results:
		var result: Dictionary = results[target]
		var reference := str(result.get("reference_camera", ""))
		if not transforms.has(reference):
			return {"ok":false,"message":"Board reference camera is missing."}
		var pose = Rig.transform_from_payload({"R":result.get("reference_marker_to_depth_godot_R"),"T":result.get("reference_marker_to_depth_godot_T")})
		if not pose is Transform3D:
			return {"ok":false,"message":"The solver did not return a valid board plane."}
		var world: Transform3D = (transforms[reference] as Transform3D) * (pose as Transform3D)
		var normal := world.basis.z.normalized()
		# The cameras are above the tabletop. This also handles the opposite
		# printed-board axis conventions used by ArUco and ChArUco.
		if normal.dot(camera_center - world.origin) < 0.0:
			normal = -normal
		points.append(world.origin)
		normals.append(normal)
	var center := Vector3.ZERO
	var normal := Vector3.ZERO
	for i in points.size():
		center += points[i]
		normal += normals[i]
	center /= points.size()
	if normal.length() < 0.5:
		return {"ok":false,"message":"Board plane directions disagree."}
	normal = normal.normalized()
	for i in points.size():
		if rad_to_deg(normals[i].angle_to(normal)) > 3.0 or absf(normal.dot(points[i]-center)) > 0.015:
			return {"ok":false,"message":"Board plane changed between camera captures. Keep the board flat and still."}
	var rotation := Basis(Quaternion(normal, Vector3.UP)).orthonormalized()
	var origin := Vector3(center.x,0.0,center.z) - rotation * center
	var transform := Transform3D(rotation, origin)
	return {"ok":true,"transform":transform,"board_origin":center,"board_normal":normal,
		"correction_degrees":rad_to_deg(normal.angle_to(Vector3.UP)),"table_y":0.0}

static func correct_pose_result(result: Dictionary, reference_correction: Transform3D, target_correction: Transform3D) -> Dictionary:
	var relative = Rig.transform_from_payload(result)
	if not relative is Transform3D:
		return result
	var adjusted := result.duplicate(true)
	adjusted.merge(Rig.payload(reference_correction.affine_inverse() * (relative as Transform3D) * target_correction),true)
	var marker = Rig.transform_from_payload({"R":result.get("reference_marker_to_depth_godot_R"),"T":result.get("reference_marker_to_depth_godot_T")})
	if marker is Transform3D:
		var payload := Rig.payload(reference_correction.affine_inverse() * (marker as Transform3D))
		adjusted.reference_marker_to_depth_godot_R = payload.R
		adjusted.reference_marker_to_depth_godot_T = payload.T
	return adjusted
