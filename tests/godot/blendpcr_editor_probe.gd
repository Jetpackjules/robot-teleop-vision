@tool
extends EditorPlugin

const View = preload("res://godot/runtime/point_cloud/unified_point_cloud_view.gd")
var checks := 0
var failures := 0

class QuietView extends View:
    func _ready() -> void: set_process(false)
    func _update_camera_renderers() -> void: pass
    func _send_all_stream_commands(_force_stop = null) -> void: pass

func check(ok: bool, label: String) -> void:
    checks += 1
    if not ok:
        failures += 1
        push_error(label)

func _enter_tree() -> void:
    call_deferred("run")

func run() -> void:
    EditorInterface.set_main_screen_editor("3D")
    for frame in 4:
        await get_tree().process_frame
    var hidden := SubViewport.new()
    hidden.size = Vector2i(2, 2)
    add_child(hidden)
    var view := QuietView.new()
    hidden.add_child(view)
    var scene_camera := Camera3D.new()
    view.add_child(scene_camera)
    scene_camera.current = true
    await get_tree().process_frame
    var editor_viewport := EditorInterface.get_editor_viewport_3d(0)
    var selected := view._blendpcr_viewer()
    check(hidden.get_camera_3d() == scene_camera, "reproduce a current scene camera in the hidden viewport")
    check(scene_camera.get_viewport().get_visible_rect().size == Vector2(2, 2), "reproduce the hidden 2x2 editor viewport")
    check(selected != scene_camera, "the scene camera must not override the visible editor view")
    check(selected == editor_viewport.get_camera_3d(), "BlendPCR follows the visible editor camera")
    check(selected.get_viewport().get_visible_rect().size.x > 16, "composition keeps the visible viewport resolution")
    hidden.queue_free()
    print("BLENDPCR_EDITOR checks=%d failures=%d" % [checks, failures])
    get_tree().quit(0 if failures == 0 else 1)
