extends SceneTree
const Locked = preload("res://godot/runtime/point_cloud/locked_top_camera_calibration.gd")
const Rig = preload("res://godot/runtime/point_cloud/realsense_rig_calibration.gd")
func _initialize():
	var previous := {"A":Transform3D(Basis(Vector3.RIGHT,0.2),Vector3(0,1,1)),"B":Transform3D(Basis(Vector3.UP,-0.4),Vector3(1,2,3)),"C":Transform3D(Basis(Vector3.UP,0.5),Vector3(-1,1,2))}
	var target := Transform3D(Basis(Vector3.RIGHT,0.4),Vector3(0.1,1.2,1.3))
	var registry := {"reference_camera":"C","transforms":{},"extra":"preserve"}
	for camera in previous:
		registry.transforms[camera] = Rig.payload(previous[camera])
		registry.transforms[camera].merge({"coordinate_space":"world","note":"metadata for "+camera})
	var refs: Array[String] = ["B","C"]
	var locked := Locked.new()
	assert(locked.begin_locked("A",refs,previous))
	assert(not locked.complete())
	for reference in refs:
		var pair := locked.next_pair()
		assert(pair.reference_camera == reference and pair.target_camera == "A")
		var result := Rig.payload(previous[reference].affine_inverse()*target)
		result["ok"] = true
		assert(locked.accept(result))
	assert(locked.complete() and locked.next_pair().is_empty())
	assert(locked.transforms.B == previous.B and locked.transforms.C == previous.C)
	assert(locked.transforms.A.is_equal_approx(target))
	var updated := Locked.replace_target(registry,"A",locked.transforms.A,"test")
	assert(not updated.is_empty())
	assert(updated.reference_camera == registry.reference_camera and updated.extra == registry.extra)
	assert(updated.transforms.B == registry.transforms.B and updated.transforms.C == registry.transforms.C)
	assert(Rig.resolve_registry(updated).B == Rig.resolve_registry(registry).B)
	assert(Rig.resolve_registry(updated).C == Rig.resolve_registry(registry).C)
	# Reject a stale/inconsistent target instead of changing the good pair.
	for difference in [Transform3D(Basis.IDENTITY,Vector3(0.04,0,0)),Transform3D(Basis(Vector3.UP,0.1),Vector3.ZERO)]:
		assert(locked.begin_locked("A",refs,previous))
		for i in range(2):
			var pair := locked.next_pair()
			var pose: Transform3D = target if i == 0 else target*difference
			var result := Rig.payload(previous[pair.reference_camera].affine_inverse()*pose)
			result["ok"] = true
			locked.accept(result)
		assert(not locked.complete() and not locked.transforms.has("A"))
	assert(locked.begin_locked("A",refs,previous))
	locked.next_pair()
	assert(not locked.accept({"ok":false,"status":"Board hidden"}) and not locked.complete())
	# Legacy B relative to A must not move indirectly when only A is saved.
	var legacy := registry.duplicate(true)
	legacy.transforms.B["coordinate_space"] = "relative"
	legacy.transforms.B["relative_to"] = "A"
	assert(Locked.replace_target(legacy,"A",target,"test").is_empty())
	assert(not locked.begin_locked("A",["B","B"],previous))
	print("LOCKED_TOP_CAMERA_PASS: two references, disagreement rejection, exact B/C entries and legacy dependency guard")
	quit()
