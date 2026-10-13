extends RefCounted
## A transaction of camera-to-camera solves, all expressed in one fixed world frame.
## No camera, renderer or file is changed until every requested camera is connected.

var reference_id := ""
var cameras: Array[String] = []
var transforms: Dictionary = {}
var results: Dictionary = {}
var attempts: Dictionary = {}
var failures: Array[String] = []
var active_pair: Dictionary = {}

func begin(ids: Array[String], reference: String, previous: Dictionary) -> bool:
	cameras = ids.duplicate()
	reference_id = reference
	transforms.clear()
	results.clear()
	attempts.clear()
	failures.clear()
	active_pair.clear()
	if cameras.size() < 2 or reference_id not in cameras:
		return false
	transforms[reference_id] = previous.get(reference_id, Transform3D.IDENTITY)
	return true

func next_pair() -> Dictionary:
	active_pair = {}
	# Try the fixed reference first. If a camera cannot see it, an already solved
	# camera can bridge the overlap, without treating that camera as world origin.
	for target in cameras:
		if transforms.has(target):
			continue
		var references: Array[String] = [reference_id]
		for camera in cameras:
			if camera != reference_id and transforms.has(camera):
				references.append(camera)
		for reference in references:
			var key := reference + " -> " + target
			if attempts.has(key):
				continue
			attempts[key] = true
			active_pair = {"reference_camera": reference, "target_camera": target}
			return active_pair.duplicate()
	return {}

func accept(result: Dictionary) -> bool:
	if active_pair.is_empty():
		return false
	var reference := str(active_pair.reference_camera)
	var target := str(active_pair.target_camera)
	var solved = transform_from_payload(result)
	var valid := bool(result.get("ok", false)) and solved is Transform3D
	valid = valid and str(result.get("reference_camera", reference)) == reference
	valid = valid and str(result.get("target_camera", target)) == target
	if valid:
		transforms[target] = (transforms[reference] as Transform3D) * (solved as Transform3D)
		results[target] = result.duplicate(true)
	else:
		failures.append("%s -> %s: %s" % [reference, target, str(result.get("status", "Invalid camera transform"))])
	active_pair = {}
	return valid

func complete() -> bool:
	return cameras.size() >= 2 and transforms.size() == cameras.size()

func pending_cameras() -> Array[String]:
	var pending: Array[String] = []
	for camera in cameras:
		if not transforms.has(camera):
			pending.append(camera)
	return pending

static func transform_from_payload(payload: Dictionary) -> Variant:
	var r = payload.get("R")
	var t = payload.get("T")
	if not r is Array or r.size() != 3 or not t is Array or t.size() != 3:
		return null
	for row in r:
		if not row is Array or row.size() != 3:
			return null
		for value in row:
			if not (value is float or value is int) or not is_finite(float(value)):
				return null
	for value in t:
		if not (value is float or value is int) or not is_finite(float(value)):
			return null
	var basis := Basis(Vector3(r[0][0], r[1][0], r[2][0]), Vector3(r[0][1], r[1][1], r[2][1]), Vector3(r[0][2], r[1][2], r[2][2]))
	if absf(basis.determinant() - 1.0) > 0.02:
		return null
	if not (basis.transposed() * basis).is_equal_approx(Basis.IDENTITY):
		# Native numerical output can have small rounding errors, but not scale/shear.
		var orthogonal := basis.orthonormalized()
		if basis.x.distance_to(orthogonal.x) > 0.01 or basis.y.distance_to(orthogonal.y) > 0.01 or basis.z.distance_to(orthogonal.z) > 0.01:
			return null
	return Transform3D(basis.orthonormalized(), Vector3(t[0], t[1], t[2]))

static func payload(transform: Transform3D) -> Dictionary:
	var b := transform.basis
	return {"R": [[b.x.x, b.y.x, b.z.x], [b.x.y, b.y.y, b.z.y], [b.x.z, b.y.z, b.z.z]], "T": [transform.origin.x, transform.origin.y, transform.origin.z]}

static func resolve_registry(registry: Dictionary) -> Dictionary:
	var entries = registry.get("transforms", {})
	if not entries is Dictionary:
		return {}
	var resolved: Dictionary = {}
	for camera in entries:
		_resolve_entry(str(camera), entries, resolved, {})
	return resolved

static func _resolve_entry(camera: String, entries: Dictionary, resolved: Dictionary, visiting: Dictionary) -> Variant:
	if resolved.has(camera):
		return resolved[camera]
	if visiting.has(camera):
		return null
	var entry = entries.get(camera, {})
	if not entry is Dictionary:
		return null
	var transform = transform_from_payload(entry)
	if transform == null:
		return null
	visiting[camera] = true
	var reference := str(entry.get("relative_to", ""))
	if str(entry.get("coordinate_space", "")) != "world" and not reference.is_empty() and reference != camera and entries.has(reference):
		var parent = _resolve_entry(reference, entries, resolved, visiting)
		if parent == null:
			return null
		transform = (parent as Transform3D) * (transform as Transform3D)
	visiting.erase(camera)
	resolved[camera] = transform
	return transform

static func write_atomic(path: String, data: Dictionary) -> Error:
	var temporary := path + ".tmp"
	var file := FileAccess.open(temporary, FileAccess.WRITE)
	if file == null:
		return FileAccess.get_open_error()
	file.store_string(JSON.stringify(data, "\t"))
	file.flush()
	var error := file.get_error()
	file.close()
	if error != OK:
		DirAccess.remove_absolute(temporary)
		return error
	return DirAccess.rename_absolute(temporary, path)
