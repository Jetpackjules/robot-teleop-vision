@tool
extends Node
## GPU-only triangle validation shared by native meshes and BlendPCR. The
## atlas makes peer validation one texture lookup, without GPU readback.
const Cache=preload("res://godot/runtime/point_cloud/depth_mesh_cache.gd")
const BestView=preload("res://godot/runtime/point_cloud/best_view_fusion.gd")
const COPY="""shader_type canvas_item;
render_mode blend_disabled;
uniform sampler2D layer : filter_nearest,repeat_disable;
void fragment() { COLOR=texture(layer,UV); }
"""
var entries: Dictionary={}
var atlas: SubViewport
var _layout: Array=[]
var _copy_shader: Shader

static func _restore(entry: Dictionary) -> void:
    var source=entry.get("source") as MeshInstance3D
    if is_instance_valid(source) and source.process_priority==entry.get("capture_priority"):
        source.process_priority=entry.original_priority
    if entry.get("installed") != null and entry.material.shader==entry.installed:
        entry.material.shader=entry.original
    entry.material.set_shader_parameter("rig_quality_ready",false)

func clear() -> void:
    for entry in entries.values():
        _restore(entry)
        entry.cache.clear()
    entries.clear()
    if is_instance_valid(atlas): atlas.queue_free()
    atlas=null
    _layout.clear()

func _exit_tree(): clear()

func get_cache(source: MeshInstance3D):
    var entry=entries.get(source.get_instance_id())
    return entry.cache if entry != null else null

func update(sources: Array[MeshInstance3D], patch_original: bool) -> void:
    var keep: Array[int]=[]
    for source in sources: keep.append(source.get_instance_id())
    for id in entries.keys():
        if id not in keep:
            _restore(entries[id])
            entries[id].cache.clear()
            entries.erase(id)
    var layout: Array=[]
    var size:=Vector2i.ONE
    var ready:=sources.size()>1
    var indices: Array[int]=[]
    for source in sources:
        var mat:=source.get_active_material(0) as ShaderMaterial if source.mesh != null else null
        if mat==null:
            ready=false
            continue
        BestView.adapt(mat)
        var id:=source.get_instance_id()
        var entry: Dictionary=entries.get(id,{})
        if not entry.is_empty() and (entry.material!=mat or mat.shader!=entry.get("installed",entry.original) or entry.patch!=patch_original):
            _restore(entry)
            entry.cache.clear()
            entries.erase(id)
            entry={}
        if not Cache.supports(mat.shader):
            mat.set_shader_parameter("rig_quality_ready",false)
            ready=false
            continue
        if entry.is_empty():
            var cache:=Cache.new()
            cache.create(self,mat.shader)
            # Native capture uploads textures in _process(). It must run before
            # the view copies peer layers and checks frame sequences. Otherwise
            # an intermittent capture can change depth after UPDATE_ONCE was
            # skipped, combining new vertices with last frame's triangle mask.
            var priority:=mini(source.process_priority,get_parent().process_priority-1)
            entry={"material":mat,"original":mat.shader,"cache":cache,"patch":patch_original,
                "source":source,"original_priority":source.process_priority,"capture_priority":priority}
            source.process_priority=priority
            if patch_original:
                var shader:=Shader.new()
                shader.code=Cache.adapt(mat.shader.code)
                mat.shader=shader
                entry.installed=shader
            entries[id]=entry
        entry.cache.update(source,mat)
        if patch_original: mat.set_shader_parameter("mesh_triangle_map",entry.cache.viewport.get_texture())
        var camera_index=mat.get_shader_parameter("rig_camera_index")
        var index:=int(camera_index) if camera_index!=null else -1
        var image_size: Vector2i=entry.cache.viewport.size
        ready=ready and index>=0 and index<sources.size() and index not in indices
        indices.append(index)
        size=size.max(image_size)
        layout.append([id,index,image_size,entry.cache.viewport.get_instance_id()])
    ready=ready and layout.size()==sources.size()
    if not ready:
        for entry in entries.values(): entry.material.set_shader_parameter("rig_quality_ready",false)
        if is_instance_valid(atlas): atlas.queue_free()
        atlas=null
        _layout.clear()
        return
    var columns:=ceili(sqrt(float(sources.size())))
    var rows:=ceili(float(sources.size())/float(columns))
    var atlas_size:=size*Vector2i(columns,rows)
    if atlas==null or layout!=_layout:
        if is_instance_valid(atlas): atlas.queue_free()
        atlas=SubViewport.new()
        atlas.disable_3d=true
        atlas.transparent_bg=true
        atlas.size=atlas_size
        atlas.render_target_update_mode=SubViewport.UPDATE_ALWAYS
        add_child(atlas)
        if _copy_shader==null:
            _copy_shader=Shader.new()
            _copy_shader.code=COPY
        for item in layout:
            var rect:=ColorRect.new()
            var index:int=item[1]
            rect.position=Vector2((index%columns)*size.x,(index/columns)*size.y)
            rect.size=Vector2(item[2])
            rect.material=ShaderMaterial.new()
            rect.material.shader=_copy_shader
            rect.material.set_shader_parameter("layer",entries[item[0]].cache.viewport.get_texture())
            atlas.add_child(rect)
        _layout=layout
    for entry in entries.values():
        entry.material.set_shader_parameter("rig_quality",atlas.get_texture())
        entry.material.set_shader_parameter("rig_quality_grid",Vector4(columns,rows,size.x,size.y))
        entry.material.set_shader_parameter("rig_quality_ready",true)
