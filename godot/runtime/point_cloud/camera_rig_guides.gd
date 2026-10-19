@tool
extends Node
## Transient children of camera renderers; never saved into calibration or the scene.

const COLORS := [Color(1.0, 0.22, 0.18), Color(0.2, 0.95, 0.35), Color(0.2, 0.5, 1.0)]
var _entries: Dictionary = {}

static func frustum_corners(k: Vector4, size: Vector2i, stride: int, distance: float) -> PackedVector3Array:
	var points := PackedVector3Array()
	if k.x <= 0 or k.y <= 0 or size.x < 2 or size.y < 2:
		return points
	for pixel in [Vector2.ZERO, Vector2(size.x - 1, 0), Vector2(size.x - 1, size.y - 1), Vector2(0, size.y - 1)]:
		pixel *= maxi(1, stride)
		points.append(Vector3((pixel.x - k.z) * distance / k.x, -(pixel.y - k.w) * distance / k.y, -distance))
	return points

static func _material(color: Color) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material.no_depth_test = true
	material.render_priority = 20
	material.albedo_color = color
	return material

func clear() -> void:
	for entry in _entries.values():
		if is_instance_valid(entry.root):
			entry.root.queue_free()
	_entries.clear()

func _exit_tree() -> void:
	clear()

func update_cameras(cameras: Array[Dictionary], mode: int) -> void:
	if mode <= 0:
		clear()
		return
	var present := {}
	for index in cameras.size():
		var spec := cameras[index]
		var source := spec.source as Node3D
		if not is_instance_valid(source):
			continue
		var id := source.get_instance_id()
		present[id] = true
		var color: Color = COLORS[index % COLORS.size()]
		if not _entries.has(id):
			var guide := Node3D.new()
			guide.name = "CameraLocationGuide"
			source.add_child(guide)
			var sphere := MeshInstance3D.new()
			sphere.name = "CameraPosition"
			var mesh := SphereMesh.new()
			mesh.radius = 0.022
			mesh.height = 0.044
			mesh.radial_segments = 12
			mesh.rings = 6
			sphere.mesh = mesh
			sphere.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
			guide.add_child(sphere)
			var label := Label3D.new()
			label.name = "CameraLabel"
			label.position.y = 0.055
			label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
			label.no_depth_test = true
			label.fixed_size = true
			label.font_size = 24
			label.pixel_size = 0.0015
			label.outline_size = 6
			guide.add_child(label)
			var cone := MeshInstance3D.new()
			cone.name = "CameraViewCone"
			cone.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
			guide.add_child(cone)
			_entries[id] = {"root": guide, "sphere": sphere, "label": label, "cone": cone, "signature": []}
		var entry: Dictionary = _entries[id]
		entry.label.text = str(spec.label).trim_prefix("RealSense ")
		entry.label.fixed_size = true
		entry.label.modulate = color
		entry.cone.visible = mode >= 2
		var signature := [color, spec.intrinsics, spec.size, spec.stride, spec.near, spec.far]
		if entry.signature == signature:
			continue
		entry.signature = signature
		entry.sphere.material_override = _material(color)
		entry.cone.material_override = _material(Color(color, 0.7))
		var near := frustum_corners(spec.intrinsics, spec.size, spec.stride, maxf(0.01, float(spec.near)))
		var far := frustum_corners(spec.intrinsics, spec.size, spec.stride, maxf(0.02, float(spec.far)))
		if near.is_empty() or far.is_empty():
			entry.cone.mesh = null
			continue
		var wire := ImmediateMesh.new()
		wire.surface_begin(Mesh.PRIMITIVE_LINES)
		for i in 4:
			for pair in [[near[i], near[(i + 1) % 4]], [far[i], far[(i + 1) % 4]], [Vector3.ZERO, far[i]]]:
				wire.surface_add_vertex(pair[0])
				wire.surface_add_vertex(pair[1])
		wire.surface_end()
		entry.cone.mesh = wire
	for id in _entries.keys():
		if present.has(id):
			continue
		if is_instance_valid(_entries[id].root):
			_entries[id].root.queue_free()
		_entries.erase(id)
