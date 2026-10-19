extends "res://tests/godot/blendpcr_probe.gd"
const QualityRig=preload("res://godot/runtime/point_cloud/mesh_quality_rig.gd")
const Fusion=preload("res://godot/runtime/point_cloud/realsense_mesh_fusion.gd")
var quality_rig: Node
var fusion=Fusion.new()
var use_cache:=true

class Capture extends Surface:
    func _process(_delta: float) -> void:
        # Alternate in-place depth, including frames with no new capture.
        var frame:=Engine.get_process_frames()
        if frame%3==0: return
        var image:=Image.create(image_size,image_size,false,Image.FORMAT_RF)
        image.fill(Color(1.0 if frame%2==0 else 0.0,0,0,1))
        (material_override.get_shader_parameter("depth_tex") as ImageTexture).update(image)
        frame_sequence=frame

class Consumer extends Node:
    var rig: Node
    var source: Capture
    var checks:=0
    var failures:=0
    func _process(_delta: float) -> void:
        var frame:=Engine.get_process_frames()
        if frame%3!=0:
            checks+=1
            if source.frame_sequence!=frame: failures+=1
        var active: Array[MeshInstance3D]=[source]
        rig.update(active,true)

func moving_capture(world: Node3D,native_shader: Shader) -> void:
    # The consumer is deliberately added first, as the real View precedes its
    # native capture children. Producer priority must override tree order.
    var consumer:=Consumer.new()
    world.add_child(consumer)
    var source:=Capture.new()
    source.native_grid=true
    source.image_size=16
    source.mesh=grid_mesh(16)
    source.material_override=ShaderMaterial.new()
    source.material_override.shader=native_shader
    source.fill(1.0,Color.RED)
    source.frame_sequence=2
    world.add_child(source)
    consumer.rig=quality_rig
    consumer.source=source
    var active: Array[MeshInstance3D]=[source]
    var original_priority:=source.process_priority
    quality_rig.update(active,true)
    for _frame in 12:
        await process_frame
        await RenderingServer.frame_post_draw
        var map: Image=quality_rig.get_cache(source).viewport.get_texture().get_image()
        var expected:=1.0 if source.frame_sequence%2==0 else 0.0
        check(is_equal_approx(map.get_pixel(8,8).r,expected),"intermittent in-place capture and triangle mask use the same frame")
    check(consumer.checks>0 and consumer.failures==0,"capture producer runs before view consumer")
    consumer.set_process(false)
    source.set_process(false)
    quality_rig.clear()
    check(source.process_priority==original_priority,"mode teardown restores capture priority")
    source.queue_free()
    consumer.queue_free()

func draw_rig(patch: bool) -> void:
    for frame in 8:
        var limits: Array[Vector2]=[Vector2(0.1,4),Vector2(0.1,4)]
        fusion.update(surfaces,limits,0.05,1.0)
        for i in surfaces.size():
            var mat:=surfaces[i].material_override as ShaderMaterial
            BestView.adapt(mat)
            mat.set_shader_parameter("rig_depth",fusion.depth_layers)
            mat.set_shader_parameter("rig_color",fusion.color_layers)
            mat.set_shader_parameter("rig_cameras",fusion.camera_data)
            mat.set_shader_parameter("rig_camera_index",i)
            mat.set_shader_parameter("rig_fusion_enabled",true)
            mat.set_shader_parameter("rig_best_view",true)
            mat.set_shader_parameter("rig_select_owner",patch)
        if use_cache: quality_rig.update(surfaces,patch)
        else: quality_rig.clear()
        if not patch: renderer.update_surfaces(surfaces,viewer,1.0,0.035,2,quality_rig)
        await process_frame
        await RenderingServer.frame_post_draw

func rig_sample(label: String, expected: Color, patch: bool) -> void:
    await draw_rig(patch)
    var pixel:=viewport.get_texture().get_image().get_pixel(64,64)
    check(Vector3(pixel.r,pixel.g,pixel.b).distance_to(Vector3(expected.r,expected.g,expected.b))<0.1,label+str(pixel))

func run() -> void:
    viewport=SubViewport.new()
    viewport.size=Vector2i(128,128)
    viewport.own_world_3d=true
    viewport.transparent_bg=true
    viewport.render_target_update_mode=SubViewport.UPDATE_ALWAYS
    root.add_child(viewport)
    var world:=Node3D.new()
    viewport.add_child(world)
    viewer=Camera3D.new()
    viewer.projection=Camera3D.PROJECTION_ORTHOGONAL
    viewer.size=1.2
    viewer.current=true
    world.add_child(viewer)
    var native_shader:=load("res://native_template.gdshader") as Shader
    for i in 2:
        var surface:=Surface.new()
        surface.native_grid=true
        surface.image_size=32 if i==0 else 16
        surface.mesh=grid_mesh(surface.image_size)
        surface.material_override=ShaderMaterial.new()
        surface.material_override.shader=native_shader
        world.add_child(surface)
        surfaces.append(surface)
        surface.fill(1.0,Color.GREEN if i==0 else Color.BLUE)
    quality_rig=QualityRig.new()
    world.add_child(quality_rig)
    renderer=Renderer.new()
    world.add_child(renderer)
    var asymmetric:=Image.create(32,32,false,Image.FORMAT_RF)
    asymmetric.fill(Color(0,0,0,1))
    for y in range(3,14):
        for x in range(4,19): asymmetric.set_pixel(x,y,Color(1,0,0,1))
    (surfaces[0].material_override.get_shader_parameter("depth_tex") as ImageTexture).update(asymmetric)
    surfaces[0].frame_sequence+=1
    use_cache=false
    await draw_rig(true)
    var baseline:=viewport.get_texture().get_image().get_data()
    use_cache=true
    await draw_rig(true)
    check(baseline==viewport.get_texture().get_image().get_data(),"cached geometry and peer atlas preserve every asymmetric depth-mask pixel")
    surfaces[0].fill(1.0,Color.GREEN)
    await rig_sample("cached unified best-view ownership",Color.GREEN,true)
    var old_material:=surfaces[0].material_override as ShaderMaterial
    var zero:=Image.create(32,32,false,Image.FORMAT_RF)
    zero.fill(Color(0,0,0,1))
    (old_material.get_shader_parameter("depth_tex") as ImageTexture).update(zero)
    surfaces[0].frame_sequence+=1
    await rig_sample("cached peer validity follows in-place missing depth",Color.BLUE,true)
    # Replacing the material must also replace its atlas binding, even when
    # camera IDs, order and image sizes stay exactly the same.
    surfaces[0].material_override=ShaderMaterial.new()
    surfaces[0].material_override.shader=native_shader
    surfaces[0].fill(1.0,Color.GREEN)
    await rig_sample("material replacement rebuilds atlas references",Color.GREEN,true)
    await rig_sample("shared atlas also feeds BlendPCR",Color.GREEN,false)
    renderer.clear()
    quality_rig.clear()
    for source in surfaces:
        check(not source.material_override.shader.code.contains("uniform sampler2D mesh_triangle_map"),"leaving cached modes restores native geometry shader")
        check(source.layers==1,"leaving cached modes restores layers")
    for source in surfaces: source.hide()
    await moving_capture(world,native_shader)
    print("MESH_QUALITY_RIG checks=%d failures=%d"%[checks,failures])
    quit(0 if failures==0 else 1)
