extends SceneTree

const Rig = preload("res://godot/runtime/point_cloud/realsense_rig_calibration.gd")
const View = preload("res://godot/runtime/point_cloud/unified_point_cloud_view.gd")
var checks := 0
var failures := 0

class TestView extends View:
	func _update_camera_renderers() -> void:
		pass
	func _restart_realsense_direct_renderers() -> void:
		pass

class ColorAnchor extends Node3D:
	var color_match_enabled := true
	var color_gain := Vector3(0.8, 0.8, 0.8)
	var color_bias := Vector3.ZERO

class ColorSurface extends MeshInstance3D:
	var depth: Image
	var rgb: Image
	var stride := 1
	var age := 0.0
	func get_depth_image() -> Image: return depth
	func get_color_image() -> Image: return rgb
	func get_current_intrinsics() -> Vector4: return Vector4(192,192,96,96)
	func get_current_grid_stride() -> int: return stride
	func get_display_frame_age_ms() -> float: return age
	func fill(gain: float, z := 1.0) -> void:
		var size := int(192 / stride)
		depth = Image.create(size,size,false,Image.FORMAT_RF)
		depth.fill(Color(z,0,0,1))
		rgb = Image.create(size,size,false,Image.FORMAT_RGBAF)
		for y in size:
			for x in size:
				rgb.set_pixel(x,y,Color((0.2+0.45*x/size)/gain,(0.3+0.3*y/size)/gain,(0.25+0.2*(x+y)/size)/gain,1))

class ColorView extends TestView:
	var ids: Array[String] = []
	var anchors: Dictionary = {}
	var disabled := ""
	var hidden_camera := ""
	func _realsense_camera_ids() -> Array[String]: return ids
	func _camera_enabled(id: String) -> bool: return id != disabled
	func _camera_diagnostic_visible(id: String) -> bool: return id != hidden_camera
	func _realsense_setting(_id: String, key: String, fallback): return "D455" if key == "model" else fallback
	func _find_realsense_camera_anchor_existing(id: String) -> Node3D: return anchors.get(id)
	func _sync_realsense_registry_from_node(_id: String) -> void: pass

func check(condition: bool, label: String) -> void:
	checks += 1
	if not condition:
		failures += 1
		push_error(label)

func response(transform: Transform3D) -> Dictionary:
	var result := Rig.payload(transform)
	result["ok"] = true
	return result

func _initialize() -> void:
	call_deferred("run")

func run() -> void:
	var ids: Array[String] = ["realsense:A", "realsense:B", "realsense:C", "realsense:D"]
	var a := Transform3D(Basis(Vector3.UP, 0.4), Vector3(1.1, 0.3, -0.2))
	var ab := Transform3D(Basis(Vector3.RIGHT, -0.2), Vector3(0.3, 0.1, 0.05))
	var bc := Transform3D(Basis(Vector3.FORWARD, 0.3), Vector3(-0.1, 0.25, 0.03))
	var cd := Transform3D(Basis(Vector3.UP, -0.1), Vector3(0.06, 0.02, -0.1))
	var previous := {ids[0]: a, ids[1]: Transform3D.IDENTITY, ids[2]: Transform3D.IDENTITY}
	var rig := Rig.new()
	check(rig.begin(ids, ids[0], previous), "four-camera rig begins")
	check(rig.next_pair().target_camera == ids[1], "first pair targets B")
	check(rig.accept(response(ab)), "A-B accepted")
	check(rig.next_pair().reference_camera == ids[0], "C tries fixed reference")
	rig.accept({"ok": false, "status": "No common view"})
	check(rig.next_pair().reference_camera == ids[1], "B bridges C when A-C fails")
	check(rig.accept(response(bc)), "B-C accepted")
	check(rig.next_pair().reference_camera == ids[0], "D tries A")
	rig.accept({"ok": false})
	check(rig.next_pair().reference_camera == ids[1], "D tries B")
	rig.accept({"ok": false})
	check(rig.next_pair().reference_camera == ids[2], "D can bridge through third camera")
	rig.accept(response(cd))
	check(rig.next_pair().is_empty() and rig.complete(), "whole rig completes")
	check((rig.transforms[ids[3]] as Transform3D).is_equal_approx(a * ab * bc * cd), "chained rotation and translation stay in fixed world frame")
	check((previous[ids[1]] as Transform3D).is_equal_approx(Transform3D.IDENTITY), "staging did not mutate current alignment")

	var view := TestView.new()
	view._camera_alignment_registry_path = "user://rig.json"
	view._realsense_alignment_ground_truth_path = "user://truth.json"
	view._alignment_transforms = previous.duplicate()
	view._native_realsense_rig = rig
	view._native_realsense_calibration_mode = "aruco"
	view._finish_native_realsense_rig()
	check(view.calibration_status.begins_with("All 4 cameras"), "runtime committed all four cameras")
	var saved: Dictionary = JSON.parse_string(FileAccess.get_file_as_string("user://rig.json"))
	var loaded := Rig.resolve_registry(saved)
	check(loaded.size() == 4, "all transforms survive reload")
	check((loaded[ids[3]] as Transform3D).is_equal_approx(a * ab * bc * cd), "reload does not double-compose a world transform")
	view._alignment_transforms.clear()
	check(view._apply_saved_realsense_alignment_ground_truth(true), "whole ArUco rig restores")
	check((view._alignment_transforms[ids[2]] as Transform3D).is_equal_approx(a * ab * bc), "ArUco restore includes C")
	var newer: Dictionary = saved.duplicate(true)
	newer["updated_at"] = Time.get_unix_time_from_system() + 10.0
	Rig.write_atomic("user://rig.json", newer)
	var latest := Transform3D(Basis.IDENTITY, Vector3(2, 3, 4))
	view._alignment_transforms[ids[2]] = latest
	check(not view._apply_saved_realsense_alignment_ground_truth(false), "startup skips older ArUco truth")
	check((view._alignment_transforms[ids[2]] as Transform3D).is_equal_approx(latest), "newer saved alignment cannot be rolled back at startup")
	check(view._apply_saved_realsense_alignment_ground_truth(true), "explicit restore can still select older truth")
	var prior := view._native_realsense_saved_prior(ids[1], ids[2])
	var expected_prior := view._packed_realsense_transform(bc)
	check(prior.size() == 12, "multi-camera prior is available")
	for i in 12:
		check(absf(prior[i] - expected_prior[i]) < 0.00001, "prior uses pair coordinates %d" % i)

	var legacy_b := Rig.payload(ab)
	legacy_b["relative_to"] = ids[0]
	var legacy_c := Rig.payload(bc)
	legacy_c["relative_to"] = ids[1]
	var world_a := Rig.payload(a)
	world_a["coordinate_space"] = "world"
	var legacy := Rig.resolve_registry({"transforms": {ids[2]: legacy_c, ids[1]: legacy_b, ids[0]: world_a}})
	check((legacy[ids[2]] as Transform3D).is_equal_approx(a * ab * bc), "out-of-order legacy pair registry resolves")
	legacy_b["relative_to"] = ids[2]
	check(Rig.resolve_registry({"transforms": {ids[1]: legacy_b, ids[2]: legacy_c}}).is_empty(), "cyclic registry is rejected")

	var pair_result := response(bc)
	pair_result.merge({"reference_camera": ids[1], "target_camera": ids[2]})
	check(view._apply_realsense_cloud_alignment_payload(pair_result), "single-pair runtime accepts valid result")
	check((view._alignment_transforms[ids[2]] as Transform3D).is_equal_approx(a * ab * bc), "single-pair runtime composes with aligned reference")
	var invalid := response(Transform3D.IDENTITY)
	invalid.R[0][0] = 2.0
	check(Rig.transform_from_payload(invalid) == null, "scaled calibration rejected")
	invalid = response(Transform3D.IDENTITY)
	invalid.T[0] = NAN
	check(Rig.transform_from_payload(invalid) == null, "nonfinite calibration rejected")

	var before := FileAccess.get_file_as_string("user://rig.json")
	var bad := Rig.new()
	bad.begin(ids, ids[0], previous)
	while not bad.next_pair().is_empty():
		bad.accept({"ok": false, "status": "Insufficient overlap"})
	view._native_realsense_rig = bad
	view._finish_native_realsense_rig()
	check(FileAccess.get_file_as_string("user://rig.json") == before, "failed rig leaves saved file untouched")
	check(view.calibration_status.contains("Previous alignment preserved"), "failed rig reports preserved state")
	view._native_realsense_rig = rig
	view._camera_alignment_registry_path = "user://missing/rig.json"
	var current := view._alignment_transforms.duplicate()
	view._finish_native_realsense_rig()
	check(view._alignment_transforms == current and view.calibration_status.contains("could not save"), "write failure cannot apply staged poses")
	view.realsense_device_registry = {"A": {"model": "D455"}, "B": {"model": "D435"}, "C": {"model": "D435"}, "D": {"model": "D455"}}
	view.render_mode = "unified_mesh"
	view.unified_camera_priority = 0
	var order: Array[String] = [ids[2], ids[3], ids[1], ids[0]]
	order.sort_custom(view._fusion_camera_precedes)
	check(order == [ids[0], ids[3], ids[1], ids[2]], "stable mesh priority includes all cameras and serial tie-breakers")
	view.render_mode = "prefer_d435"
	order.sort_custom(view._fusion_camera_precedes)
	check(order == [ids[1], ids[2], ids[0], ids[3]], "preferred camera family changes whole-rig priority")
	view._mesh_fusion_status = "Native extension needs rebuilding"
	view._update_realsense_direct_status()
	check(view.realsense_status.contains("needs rebuilding"), "stream status does not overwrite a mesh failure")
	view.free()
	check_joint_refinement_transaction()
	check_color_matching()
	print("REALSENSE_RIG_PROBE checks=%d failures=%d" % [checks, failures])
	quit(0 if failures == 0 else 1)

func check_joint_refinement_transaction() -> void:
	var view := TestView.new()
	var ids: Array[String] = ["realsense:A", "realsense:B", "realsense:C"]
	var original := {ids[0]: Transform3D.IDENTITY, ids[1]: Transform3D(Basis.IDENTITY, Vector3(0.1,0,0)), ids[2]: Transform3D(Basis.IDENTITY, Vector3(0,0.1,0))}
	var rig := Rig.new()
	rig.begin(ids, ids[0], original)
	while not rig.next_pair().is_empty(): rig.accept(response(original[rig.active_pair.target_camera]))
	view._native_realsense_rig = rig
	view._native_realsense_calibration_mode = "markerless"
	view._alignment_transforms = original.duplicate()
	view._camera_alignment_registry_path = "user://joint-rig.json"
	view._write_camera_alignment_set(original, ids[0], "previous", "previous")
	view._native_realsense_registry_before = FileAccess.get_file_as_bytes(view._camera_alignment_registry_path)
	var worker := View.JointRefinement.new()
	worker.folder = ProjectSettings.globalize_path("user://joint-worker-test")
	DirAccess.make_dir_recursive_absolute(worker.folder)
	worker.previous_registry = view._native_realsense_registry_before
	view._joint_realsense_refinement = worker
	check(view._realsense_cloud_alignment_running(), "joint stage still blocks another calibration")
	check(not view._realsense_capture_exclusive(), "joint stage releases capture for fresh frame validation")
	view._joint_realsense_refinement = null
	check(view._realsense_capture_exclusive(), "native pair stage still pauses capture")
	view._joint_realsense_refinement = worker
	var transforms := {}
	for id in ids: transforms[view._realsense_serial(id)] = Rig.payload(original[id])
	var result := {"ok": true, "selected": "refined", "reference_camera": "A", "transforms": transforms}
	var bad := result.duplicate(true)
	bad.transforms.erase("C")
	check(not view._apply_joint_realsense_result(bad), "partial joint output cannot save a subset of the rig")
	bad = result.duplicate(true)
	bad.transforms.A.T[0] = 0.02
	check(not view._apply_joint_realsense_result(bad), "joint output cannot move the fixed reference")
	bad = result.duplicate(true)
	bad.transforms.B.R[0][0] = NAN
	check(not view._apply_joint_realsense_result(bad), "nonfinite joint output rejected")
	var previous_result := result.duplicate(true)
	previous_result.selected = "previous"
	check(view._apply_joint_realsense_result(previous_result), "better existing alignment can be retained")
	check(FileAccess.get_file_as_bytes(view._camera_alignment_registry_path) == view._native_realsense_registry_before, "retaining saved alignment does not rewrite it")
	var newer: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(view._camera_alignment_registry_path))
	newer["updated_at"] = 1234
	Rig.write_atomic(view._camera_alignment_registry_path, newer)
	check(not view._apply_joint_realsense_result(result), "concurrent newer calibration cannot be overwritten")
	var file := FileAccess.open(view._camera_alignment_registry_path, FileAccess.WRITE)
	file.store_buffer(view._native_realsense_registry_before)
	file.close()
	result.transforms.B.T[1] = 0.005
	check(view._apply_joint_realsense_result(result), "validated whole joint rig saves")
	check(FileAccess.get_file_as_bytes(worker.folder + "/registry-before.json") == worker.previous_registry, "previous rig is backed up before commit")
	var saved: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(view._camera_alignment_registry_path))
	check(saved.transforms[ids[1]].method == "joint_rgbd_reprojection_refinement", "joint calibration is persisted as the new method")
	check(absf((view._alignment_transforms[ids[1]] as Transform3D).origin.y - 0.005) < 0.00001, "live poses update only after successful commit")
	view._release_native_realsense_rig()
	check(not view._realsense_cloud_alignment_running(), "joint transaction releases busy state")
	view.free()

func check_color_matching() -> void:
	var view := ColorView.new()
	for i in 6:
		var id := "realsense:" + str(i)
		view.ids.append(id)
		var surface := ColorSurface.new()
		surface.stride = 2 if i == 1 else 1
		surface.fill(1.0 + i*0.1, 2.0 if i == 3 else 1.0)
		root.add_child(surface)
		view._camera_nodes[id] = surface
		view.anchors[id] = ColorAnchor.new()
	view.anchors[view.ids[0]].color_gain = Vector3.ONE
	view.disabled = view.ids[4]
	view.hidden_camera = view.ids[5]
	check(view._live_realsense_color_ids().size() == 4, "hidden and disabled cameras excluded from color matching")
	check(view._realsense_color_reference_id(view._live_realsense_color_ids()) == view.ids[0], "all-D455 rig has a deterministic color reference")
	view.realsense_color_reference_serial = "missing"
	check(view._realsense_color_reference_id(view._live_realsense_color_ids()) == view.ids[0], "missing configured color reference falls back to live camera")
	view._match_realsense_colors_now()
	check(view.realsense_color_match_status.begins_with("Matched 2/3"), "every non-reference camera attempted, with one no-overlap rejection")
	check(view.anchors[view.ids[1]].color_gain.distance_to(Vector3.ONE*1.1) < 0.03, "color fit respects different sampling strides")
	check(view.anchors[view.ids[2]].color_gain.distance_to(Vector3.ONE*1.2) < 0.03, "third camera gets its own color fit")
	check(view.anchors[view.ids[3]].color_gain == Vector3.ONE*0.8, "failed overlap keeps previously saved correction")
	check(view.anchors[view.ids[4]].color_gain == Vector3.ONE*0.8, "disabled camera correction untouched")
	var first: ColorSurface = view._camera_nodes[view.ids[0]]
	first.age = 1500.0
	check(not view.ids[0] in view._live_realsense_color_ids(), "stale camera cannot be color reference")
	for surface in view._camera_nodes.values(): surface.free()
	for anchor in view.anchors.values(): anchor.free()
	view.free()
