extends SceneTree
const View = preload("res://godot/runtime/point_cloud/unified_point_cloud_view.gd")
const BlendPCR = preload("res://godot/runtime/point_cloud/blendpcr/blendpcr_renderer.gd")
var checks := 0
var failures := 0
class QuietView extends View:
    func _update_camera_renderers() -> void: pass
    func _send_all_stream_commands(_force_stop := false) -> void: pass
func check(ok: bool, label: String) -> void:
    checks += 1
    if not ok:
        failures += 1
        push_error(label)
func _initialize() -> void:
    var lut := BlendPCR.compatibility_lut_image()
    for byte in 256:
        var value := lut.get_pixel(byte, 0).r
        var linear := value * (value * (value * 0.305306011 + 0.682171111) + 0.012522878)
        var recovered := 1.055 * pow(linear, 1.0 / 2.4) - 0.055
        check(absi(int(round(recovered * 255.0)) - byte) == 0, "packed metadata byte survives color conversion: %d" % byte)
    var view := QuietView.new()
    check(view.render_mode == "independent_mesh" and view.rendering_mode == 2, "existing default retained")
    check(view.unified_camera_priority == 3, "new unified views automatically choose the best local camera")
    for priority in [0,1,2,3]:
        view.unified_camera_priority = priority
        check(view.unified_camera_priority == priority, "saved camera priority retains its existing numeric value")
    view.render_mode = "prefer_d435"
    check(view.render_mode == "unified_mesh" and view.unified_camera_priority == 1, "legacy D435 mode migrates")
    view.render_mode = "prefer_d455"
    check(view.render_mode == "unified_mesh" and view.unified_camera_priority == 0, "legacy D455 mode migrates")
    for i in View.RENDERING_MODES.size():
        view.rendering_mode = i
        check(view.render_mode == View.RENDERING_MODES[i] and view.rendering_mode == i, "render selector round trip %d" % i)
        var priority := {"name": "unified_camera_priority", "usage": PROPERTY_USAGE_DEFAULT}
        view._validate_property(priority)
        check(bool(priority.usage & PROPERTY_USAGE_EDITOR) == (view.render_mode == "unified_mesh"), "camera preference only appears for Unified mesh")
        check(bool(priority.usage & PROPERTY_USAGE_STORAGE), "camera preference remains saved in every mode")
    view.diagnostic_cpu_mesh = true
    check(view.render_mode == "cpu_mesh" and not view._gpu_mesh_enabled(), "CPU fallback remains available")
    view.rendering_mode = 5
    check(not view.diagnostic_cpu_mesh and view._gpu_mesh_static_shader_enabled(), "BlendPCR selects existing GPU source geometry")
    var proxy := {"name": "rendering_mode", "usage": PROPERTY_USAGE_DEFAULT}
    view._validate_property(proxy)
    check(proxy.usage == PROPERTY_USAGE_EDITOR, "proxy cannot override stored rendering settings")
    view.free()
    print("BLENDPCR_MODES checks=%d failures=%d" % [checks,failures])
    quit(0 if failures == 0 else 1)
