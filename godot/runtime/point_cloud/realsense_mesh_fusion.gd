extends RefCounted
## Shared GPU layers for every visible camera, in stable surface-priority order.
## Upload only new CPU capture images: never read textures back from the GPU.

var depth_layers: Texture2DArray
var color_layers: Texture2DArray
var camera_data: ImageTexture
var _layout := ""
var _sequences: Array[int] = []
var _metadata := PackedFloat32Array()
var error := ""

func clear() -> void:
	depth_layers = null
	color_layers = null
	camera_data = null
	_layout = ""
	_sequences.clear()
	_metadata.clear()

func update(renderers: Array[MeshInstance3D], depth_limits: Array[Vector2], depth_delta: float, max_edge: float) -> bool:
	error = ""
	if renderers.size() < 2:
		clear()
		return false
	var depths: Array[Image] = []
	var colors: Array[Image] = []
	var sequences: Array[int] = []
	var depth_size := Vector2i.ONE
	var color_size := Vector2i.ONE
	var layout := ""
	for renderer in renderers:
		var depth: Image = renderer.call("get_depth_image")
		var color: Image = renderer.call("get_color_image")
		depths.append(depth)
		colors.append(color)
		sequences.append(int(renderer.call("get_frame_sequence")))
		var ds := depth.get_size() if depth != null and not depth.is_empty() else Vector2i.ONE
		var cs := color.get_size() if color != null and not color.is_empty() else Vector2i.ONE
		depth_size = depth_size.max(ds)
		color_size = color_size.max(cs)
		layout += "%s:%s:%s;" % [renderer.get_instance_id(), ds, cs]
	var rebuild := layout != _layout or depth_layers == null or color_layers == null
	if rebuild:
		var padded_depths: Array[Image] = []
		var padded_colors: Array[Image] = []
		for index in renderers.size():
			padded_depths.append(_padded(depths[index], depth_size, Image.FORMAT_RF))
			padded_colors.append(_padded(colors[index], color_size, Image.FORMAT_RGBA8))
		var new_depth := Texture2DArray.new()
		var new_color := Texture2DArray.new()
		if new_depth.create_from_images(padded_depths) != OK or new_color.create_from_images(padded_colors) != OK:
			error = "GPU could not allocate unified-mesh layers for %d cameras." % renderers.size()
			clear()
			return false
		depth_layers = new_depth
		color_layers = new_color
		_layout = layout
	else:
		for index in renderers.size():
			if sequences[index] != _sequences[index]:
				depth_layers.update_layer(_padded(depths[index], depth_size, Image.FORMAT_RF), index)
				color_layers.update_layer(_padded(colors[index], color_size, Image.FORMAT_RGBA8), index)
	_sequences = sequences
	# Metadata lives in a texture too, avoiding fixed-size uniform/sampler arrays.
	# Each row is: world-to-camera (3), intrinsics, grid, limits, color/ready,
	# cleanup. Layer order is the tie-breaker; only earlier layers can suppress.
	var values := PackedFloat32Array()
	for index in renderers.size():
		var renderer := renderers[index]
		var inverse := renderer.global_transform.affine_inverse()
		var b := inverse.basis
		for row in [Vector4(b.x.x, b.y.x, b.z.x, inverse.origin.x), Vector4(b.x.y, b.y.y, b.z.y, inverse.origin.y), Vector4(b.x.z, b.y.z, b.z.z, inverse.origin.z)]:
			_append(values, row)
		_append(values, renderer.call("get_current_intrinsics"))
		var size := depths[index].get_size() if depths[index] != null else Vector2i.ONE
		_append(values, Vector4(size.x, size.y, max(1, int(renderer.call("get_current_grid_stride"))), 0))
		_append(values, Vector4(depth_limits[index].x, depth_limits[index].y, depth_delta, max_edge))
		var color_size_i := colors[index].get_size() if colors[index] != null else Vector2i.ONE
		var ready := size.x > 1 and size.y > 1
		if renderer.has_method("get_display_frame_age_ms"):
			ready = ready and float(renderer.call("get_display_frame_age_ms")) < 1000.0
		_append(values, Vector4(color_size_i.x, color_size_i.y, float(bool(renderer.get("color_enabled"))), float(ready)))
		_append(values, Vector4(float(bool(renderer.get("point_cleanup_enabled"))), float(renderer.get("point_cleanup_depth_delta")), float(renderer.get("point_cleanup_min_neighbors")), 0))
	if values != _metadata or camera_data == null:
		var image := Image.create_from_data(8, renderers.size(), false, Image.FORMAT_RGBAF, values.to_byte_array())
		if camera_data == null or camera_data.get_height() != renderers.size():
			camera_data = ImageTexture.create_from_image(image)
		else:
			camera_data.update(image)
		_metadata = values
	return true

static func _append(values: PackedFloat32Array, v: Vector4) -> void:
	values.append_array(PackedFloat32Array([v.x, v.y, v.z, v.w]))

static func _padded(source: Image, size: Vector2i, format: Image.Format) -> Image:
	if source != null and not source.is_empty() and source.get_size() == size and source.get_format() == format:
		return source
	var image := Image.create(size.x, size.y, false, format)
	image.fill(Color(0, 0, 0, 0))
	if source != null and not source.is_empty():
		var converted := source
		if source.get_format() != format:
			converted = source.duplicate()
			converted.convert(format)
		image.blit_rect(converted, Rect2i(Vector2i.ZERO, converted.get_size()), Vector2i.ZERO)
	return image
