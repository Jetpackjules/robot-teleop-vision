extends SceneTree
const Level = preload("res://godot/runtime/point_cloud/board_world_level.gd")
const Rig = preload("res://godot/runtime/point_cloud/realsense_rig_calibration.gd")
func _initialize():
	var normal := Vector3(0.3,1.0,-0.25).normalized()
	var board := Transform3D(Basis(Quaternion(Vector3.BACK,normal)),Vector3(0.2,0.4,-0.3))
	var cameras := {"a":Transform3D(Basis.IDENTITY,Vector3(0,2,0)),"b":Transform3D(Basis(Vector3.UP,0.3),Vector3(1,2,0)),"c":Transform3D(Basis.IDENTITY,Vector3(-1,2,0))}
	var reference: Transform3D = cameras.a
	var pose := Rig.payload(reference.affine_inverse()*board)
	var result := {"reference_camera":"a","reference_marker_to_depth_godot_R":pose.R,"reference_marker_to_depth_godot_T":pose.T}
	var solution := Level.solve(cameras,{"b":result,"c":result})
	assert(solution.ok)
	var correction: Transform3D = solution.transform
	assert((correction.basis*normal).distance_to(Vector3.UP)<0.00001)
	assert(absf((correction*board.origin).y)<0.00001)
	assert(is_equal_approx(correction.basis.determinant(),1.0))
	var before: Transform3D = reference.affine_inverse()*(cameras.b as Transform3D)
	var after: Transform3D = (correction*reference).affine_inverse()*(correction*(cameras.b as Transform3D))
	assert(before.is_equal_approx(after))
	var arm := Transform3D(Basis.IDENTITY,Vector3(0.4,0.8,0.1))
	assert((reference.affine_inverse()*arm).is_equal_approx((correction*reference).affine_inverse()*(correction*arm)))
	var bad := result.duplicate(true)
	var shifted := board
	shifted.origin += normal*0.1
	var shifted_pose := Rig.payload(reference.affine_inverse()*shifted)
	bad.reference_marker_to_depth_godot_T=shifted_pose.T
	assert(not Level.solve(cameras,{"b":result,"c":bad}).ok)
	assert(not Level.solve(cameras,{}).ok)
	bad=result.duplicate(true)
	bad.reference_marker_to_depth_godot_R=[[0,0,0],[0,0,0],[0,0,0]]
	assert(not Level.solve(cameras,{"b":bad}).ok)
	var flipped := Transform3D(board.basis*Basis(Vector3.RIGHT,PI),board.origin)
	var flip_pose := Rig.payload(reference.affine_inverse()*flipped)
	bad=result.duplicate(true)
	bad.reference_marker_to_depth_godot_R=flip_pose.R
	assert((Level.solve(cameras,{"b":bad}).transform as Transform3D).is_equal_approx(correction))
	var depth_correction := Transform3D(Basis(Vector3.RIGHT,0.01),Vector3(0.005,0.008,0.0))
	var raw_relative: Transform3D = (cameras.a as Transform3D).affine_inverse()*(cameras.b as Transform3D)
	var raw_result := Rig.payload(raw_relative)
	raw_result.merge(result)
	raw_result.merge(Rig.payload(raw_relative),true)
	var adjusted := Level.correct_pose_result(raw_result,depth_correction,Transform3D.IDENTITY)
	var corrected_reference: Transform3D = (cameras.a as Transform3D)*depth_correction
	assert((corrected_reference*(Rig.transform_from_payload(adjusted) as Transform3D)).is_equal_approx(cameras.b))
	var marker_adjusted = Rig.transform_from_payload({"R":adjusted.reference_marker_to_depth_godot_R,"T":adjusted.reference_marker_to_depth_godot_T})
	assert((corrected_reference*(marker_adjusted as Transform3D)).is_equal_approx(board))
	print("BOARD_WORLD_LEVEL_PASS: horizontal plane, zero height, rigid cameras/arm, opposite normals, invalid data")
	quit(0)
