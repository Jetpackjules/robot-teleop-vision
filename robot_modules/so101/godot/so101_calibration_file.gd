@tool
extends RefCounted

## Portable visual calibration only. No motor goals, ports, or executable paths.
const FILE_TYPE := "so101_visual_calibration"
const SCHEMA_VERSION := 1
const MAX_BYTES := 16384
const ARRAY_FIELDS := {
	"joint_angle_directions": [6, 6, -1.0, 1.0],
	"joint_angle_offsets_degrees": [6, 6, -360.0, 360.0],
	"gripper_angle_samples_normalized": [2, 64, -100.0, 200.0],
	"gripper_angle_samples_degrees": [2, 64, -180.0, 180.0],
	"gripper_mount_correction_local": [3, 3, -0.015, 0.015],
	"gripper_visual_correction_rpy_degrees": [3, 3, -30.0, 30.0],
	"gripper_visual_correction_translation_local": [3, 3, -0.025, 0.025],
	"moving_jaw_pivot_parent": [3, 3, -0.12, 0.12],
	"moving_jaw_axis_parent": [3, 3, -1.0, 1.0],
	"moving_jaw_closed_basis_parent_row_major": [9, 9, -1.0, 1.0],
	"moving_jaw_opening_samples_degrees": [5, 5, -5.0, 140.0],
	"basis_x": [3, 3, -1.0, 1.0], "basis_y": [3, 3, -1.0, 1.0],
	"basis_z": [3, 3, -1.0, 1.0], "origin": [3, 3, -100.0, 100.0],
}
const SCALAR_FIELDS := {
	"gripper_angle_offset_degrees": [-360.0, 360.0],
	"gripper_angle_scale_degrees_per_normalized": [-5.0, 5.0],
	"gripper_angle_curvature_degrees": [-5.0, 5.0],
	"moving_jaw_visual_radial_scale": [0.75, 1.35],
	"moving_jaw_visual_axial_translation_local": [-0.03, 0.03],
}
const BOOL_FIELDS := ["force_wrist_roll_straight", "moving_jaw_calibration_enabled"]
const BASE_FIELDS := ["basis_x", "basis_y", "basis_z", "origin"]


static func numeric(value: Variant, low: float, high: float) -> bool:
	return (value is float or value is int) and is_finite(float(value)) and float(value) >= low and float(value) <= high


static func package(registration: Dictionary, version: int) -> Dictionary:
	var values := {}
	for key in ARRAY_FIELDS.keys() + SCALAR_FIELDS.keys() + BOOL_FIELDS:
		if registration.has(key):
			values[key] = registration[key]
	return {
		"type": FILE_TYPE, "schema_version": SCHEMA_VERSION,
		"robot": "so101", "kinematics_version": version,
		"exported_unix_ms": Time.get_unix_time_from_system() * 1000.0,
		"coordinate_frame": "WorldLevelAnchor local",
		"note": "Visual calibration for the same arm and motor profile. Restore the base only with matching camera alignment and physical placement.",
		"registration": values,
	}


static func validate(file: Variant, version: int) -> String:
	if not file is Dictionary or JSON.stringify(file).to_utf8_buffer().size() > MAX_BYTES:
		return "Choose a visual calibration JSON file smaller than 16 KB."
	if file.get("type") != FILE_TYPE or file.get("schema_version") != SCHEMA_VERSION or file.get("robot") != "so101":
		return "This is not a supported SO-101 visual calibration file."
	if file.get("kinematics_version") != version:
		return "This file uses a different robot model version. Export it from a matching application version."
	var values = file.get("registration")
	if not values is Dictionary:
		return "The file has no registration object."
	for key in values:
		if key not in ARRAY_FIELDS and key not in SCALAR_FIELDS and key not in BOOL_FIELDS:
			return "Unsupported calibration field: " + str(key)
	for key in ARRAY_FIELDS:
		var data = values.get(key)
		var bounds: Array = ARRAY_FIELDS[key]
		if not data is Array or data.size() < bounds[0] or data.size() > bounds[1]:
			return "Invalid array: " + key
		for value in data:
			if not numeric(value, bounds[2] - 0.000001, bounds[3] + 0.000001):
				return "Invalid numeric value: " + key
	for key in SCALAR_FIELDS:
		if not numeric(values.get(key), SCALAR_FIELDS[key][0], SCALAR_FIELDS[key][1]):
			return "Invalid numeric value: " + key
	for key in BOOL_FIELDS:
		if not values.get(key) is bool:
			return "Invalid flag: " + key
	for direction in values.joint_angle_directions:
		if absf(absf(float(direction)) - 1.0) > 0.0001:
			return "Joint directions must be +1 or -1."
	# Base pan uses the canonical telemetry convention; its placement is in the
	# rigid base transform, not an independently editable encoder-zero value.
	if absf(float(values.joint_angle_directions[0]) - 1.0) > 0.0001 or absf(float(values.joint_angle_offsets_degrees[0]) - 40.4296875) > 0.0001:
		return "The file uses an incompatible base-pan convention."
	if values.gripper_angle_samples_normalized.size() != values.gripper_angle_samples_degrees.size():
		return "Claw samples have different lengths."
	var last := -INF
	for value in values.gripper_angle_samples_normalized:
		if float(value) <= last:
			return "Claw encoder samples must be increasing."
		last = float(value)
	for key in ["gripper_mount_correction_local", "gripper_visual_correction_translation_local", "gripper_visual_correction_rpy_degrees"]:
		var data: Array = values[key]
		var limit: float = {"gripper_mount_correction_local": 0.015, "gripper_visual_correction_translation_local": 0.025, "gripper_visual_correction_rpy_degrees": 30.0}[key]
		if Vector3(data[0], data[1], data[2]).length() > limit + 0.000001:
			return "Tool correction exceeds the supported range: " + key
	var basis := Basis(_vector(values.basis_x), _vector(values.basis_y), _vector(values.basis_z))
	if not rigid_basis(basis) or basis.y.normalized().dot(Vector3.UP) < 0.25:
		return "The saved base is not a rigid upright placement."
	if values.moving_jaw_calibration_enabled:
		var axis := _vector(values.moving_jaw_axis_parent)
		var pivot := _vector(values.moving_jaw_pivot_parent)
		var b: Array = values.moving_jaw_closed_basis_parent_row_major
		var jaw_basis := Basis(Vector3(b[0], b[3], b[6]), Vector3(b[1], b[4], b[7]), Vector3(b[2], b[5], b[8]))
		if absf(axis.length() - 1.0) > 0.01 or pivot.length() > 0.12 or not rigid_basis(jaw_basis):
			return "Invalid moving-jaw geometry."
		last = -INF
		for opening in values.moving_jaw_opening_samples_degrees:
			if float(opening) < last:
				return "Claw opening samples must be increasing."
			last = float(opening)
	return ""


static func _vector(values: Array) -> Vector3:
	return Vector3(values[0], values[1], values[2])


static func rigid_basis(value: Basis) -> bool:
	return absf(value.determinant() - 1.0) < 0.001 and value.is_equal_approx(value.orthonormalized())
