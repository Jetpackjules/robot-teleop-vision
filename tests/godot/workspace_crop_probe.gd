extends "res://tests/godot/blendpcr_probe.gd"
const Crop = preload("res://godot/runtime/point_cloud/workspace_crop.gd")
const View = preload("res://godot/runtime/point_cloud/unified_point_cloud_view.gd")
var cropping := true
var minimum := Vector3(-0.15,-0.2,-1.1)
var maximum := Vector3(0.15,0.2,-0.9)
class QuietView extends View:
    func _update_camera_renderers() -> void: pass

func settle() -> void:
    for i in 8:
        renderer.update_surfaces(surfaces, viewer, 1.0, 0.035, 2)
        renderer.set_workspace_crop(cropping, minimum, maximum)
        await process_frame
        await RenderingServer.frame_post_draw

func run() -> void:
    var view := QuietView.new()
    var packet := {"workspace_crop_enabled":true,"workspace_crop_min":[-.15,-.2,-1.1],"workspace_crop_max":[.15,.2,-.9]}
    view.apply_remote_workspace_crop(packet)
    check(view.workspace_crop_enabled and view.workspace_crop_min.is_equal_approx(minimum) and view.workspace_crop_max.is_equal_approx(maximum), "remote bounds match the browser's world box")
    view.apply_remote_workspace_crop({"workspace_crop_min":[1,0,0],"workspace_crop_max":[0,0,0]})
    check(view.workspace_crop_min.is_equal_approx(minimum), "invalid remote box keeps previous valid bounds")
    view.apply_remote_workspace_crop({"workspace_crop_rotation":[18,-29,12]})
    check(view.workspace_crop_rotation.is_equal_approx(Vector3(18,-29,12)), "remote rotation persists without changing the crop dimensions")
    view.apply_remote_workspace_crop({"workspace_crop_rotation":[0,INF,0]})
    check(view.workspace_crop_rotation.is_equal_approx(Vector3(18,-29,12)), "invalid rotation preserves the existing volume")
    view.free()
    var box := Crop.box_mesh(minimum,maximum)
    check(box.get_surface_count()==1,"crop preview builds a line surface")
    var material := ShaderMaterial.new()
    material.shader=load("res://native_template.gdshader")
    Crop.apply_to_material(material,true,minimum,maximum)
    check(material.shader.code.contains("workspace_crop_world = (MODEL_MATRIX"),"native crop uses the camera's calibrated world transform")
    var shader := material.shader
    Crop.apply_to_material(material,false,minimum,maximum)
    check(material.shader==shader and not material.get_shader_parameter("workspace_crop_enabled"),"disabling crop reuses the native material")
    Crop.apply_to_material(material,true,minimum,maximum)
    check(material.shader==shader,"bounds changes do not keep recompiling shaders")
    Crop.apply_to_material(material,true,minimum,maximum,Vector3(18,-29,12))
    var inverse: Basis = material.get_shader_parameter("workspace_crop_inverse")
    check(inverse.is_equal_approx(Crop.rotation_basis(Vector3(18,-29,12)).transposed()), "native shader receives the same world-to-box rotation as the browser")
    Crop.apply_to_material(material,true,minimum,maximum)
    if DisplayServer.get_name()=="headless":
        print("WORKSPACE_CROP checks=%d failures=%d" % [checks,failures])
        quit(0 if failures==0 else 1)
        return
    viewport=SubViewport.new()
    RenderingServer.set_default_clear_color(Color.BLACK)
    viewport.size=Vector2i(128,128)
    viewport.own_world_3d=true
    viewport.render_target_update_mode=SubViewport.UPDATE_ALWAYS
    root.add_child(viewport)
    var world:=Node3D.new()
    viewport.add_child(world)
    viewer=Camera3D.new()
    viewer.projection=Camera3D.PROJECTION_ORTHOGONAL
    viewer.size=1.5
    viewer.current=true
    world.add_child(viewer)
    renderer=Renderer.new()
    world.add_child(renderer)
    var surface:=Surface.new()
    surface.native_grid=true
    surface.mesh=grid_mesh()
    surface.material_override=material
    surface.fill(1.0,Color.RED)
    world.add_child(surface)
    surfaces.append(surface)
    await sample("crop inside native BlendPCR",Color.RED)
    var pixels:=viewport.get_texture().get_image()
    check(pixels.get_pixel(95,64).r<0.1,"BlendPCR removes geometry outside exact world bounds")
    cropping=false
    Crop.apply_to_material(material,false,minimum,maximum)
    await settle()
    pixels=viewport.get_texture().get_image()
    check(pixels.get_pixel(95,64).r>0.9,"disabling crop restores the native surface")
    renderer.clear()
    cropping=true
    Crop.apply_to_material(material,true,minimum,maximum)
    for i in 4:
        await process_frame
        await RenderingServer.frame_post_draw
    pixels=viewport.get_texture().get_image()
    check(pixels.get_pixel(64,64).r>0.9 and pixels.get_pixel(95,64).r<0.1,"independent native shader applies the same box")
    print("WORKSPACE_CROP_GPU checks=%d failures=%d" % [checks,failures])
    quit(0 if failures==0 else 1)
