extends SceneTree
const Renderer=preload("res://godot/runtime/point_cloud/blendpcr/blendpcr_renderer.gd")
const Horizontal=preload("res://godot/runtime/point_cloud/blendpcr/weights.gdshader")
const Vertical=preload("res://godot/runtime/point_cloud/blendpcr/weights_vertical.gdshader")
var failures:=0
func _initialize(): call_deferred("run")
func run():
    var renderer:=Renderer.new()
    root.add_child(renderer)
    var input:=Image.create(13,9,false,Image.FORMAT_RGBA8)
    for y in 9:
        for x in 13: input.set_pixel(x,y,Color(float((x*7+y*11)%5)/255.0,0,0,1))
    var h: SubViewport=renderer._screen_pass(Horizontal,Vector2i(13,9))
    var v: SubViewport=renderer._screen_pass(Vertical,Vector2i(13,9))
    renderer._pass_material(h).set_shader_parameter("dominance",ImageTexture.create_from_image(input))
    renderer._pass_material(v).set_shader_parameter("horizontal_counts",h.get_texture())
    for radius in [0,1,3,10]:
        renderer._pass_material(h).set_shader_parameter("seam_radius",radius)
        renderer._pass_material(v).set_shader_parameter("seam_radius",radius)
        for frame in 5:
            await process_frame
            await RenderingServer.frame_post_draw
        var actual:=v.get_texture().get_image()
        for y in 9:
            for x in 13:
                var expected:=Vector4.ZERO
                for dy in range(-radius,radius+1):
                    for dx in range(-radius,radius+1):
                        var owner:=int(round(input.get_pixel(clampi(x+dx,0,12),clampi(y+dy,0,8)).r*255.0))-1
                        if owner>=0: expected[owner]+=1.0
                expected/=float((2*radius+1)*(2*radius+1))
                var pixel:=actual.get_pixel(x,y)
                for channel in 4:
                    if abs(pixel[channel]-expected[channel])>0.5/255.0+0.00001:
                        failures+=1
    print("BLENDPCR_WEIGHTS failures=",failures)
    quit(0 if failures==0 else 1)
