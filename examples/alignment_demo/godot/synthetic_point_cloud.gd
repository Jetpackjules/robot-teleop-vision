extends MeshInstance3D
## A single FIXED virtual depth sensor raycasts the simulation's collision scene.
## Hidden surfaces are not invented when the operator moves their viewing camera.
## This is synthetic RGB-D from mesh geometry, not recorded RealSense data.

var enabled := true
var sample_width := 240
var sample_height := 180
var interval := .12
var sample_count := 0
var source_position := Vector3(-.25, .38, .38)
var source_target := Vector3(-.04, .08, -.13)
var _elapsed := 1.0


func _ready() -> void:
	layers = 2
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.vertex_color_use_as_albedo = true
	material.use_point_size = true
	material.point_size = 4.0
	material_override = material


func _physics_process(delta: float) -> void:
	if not enabled:
		return
	_elapsed += delta
	if _elapsed < interval:
		return
	_elapsed = 0
	refresh_cloud()


func refresh_cloud() -> void:
	var points := PackedVector3Array()
	var colors := PackedColorArray()
	var world := get_world_3d().direct_space_state
	var source_basis := Basis.looking_at(source_target - source_position, Vector3.UP)
	var tangent := tan(deg_to_rad(42.0) * .5)
	var aspect := float(sample_width) / float(sample_height)
	var query := PhysicsRayQueryParameters3D.new()
	query.from = source_position
	query.collision_mask = 3
	for y in range(sample_height):
		for x in range(sample_width):
			var ray := Vector3((2.0 * (x + .5) / sample_width - 1.0) * tangent * aspect, (1.0 - 2.0 * (y + .5) / sample_height) * tangent, -1)
			query.to = source_position + source_basis * ray.normalized() * 2.0
			var hit := world.intersect_ray(query)
			if hit.is_empty():
				continue
			points.append(hit.position)
			var color: Color = hit.collider.get_meta("cloud_color", Color(.75,.78,.81))
			var shade := .75 + .25 * maxf(0.0, (hit.normal as Vector3).dot(Vector3(-.4,.8,.4).normalized()))
			colors.append(Color(color.r * shade, color.g * shade, color.b * shade, 1))
	sample_count = points.size()
	if points.is_empty():
		return
	var arrays: Array = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = points
	arrays[Mesh.ARRAY_COLOR] = colors
	var point_mesh := ArrayMesh.new()
	point_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_POINTS, arrays)
	mesh = point_mesh
