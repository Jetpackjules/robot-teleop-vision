@tool
extends Node

## Loads one robot integration without coupling the shared camera scene to it.
## Runtime selection comes from ROBOT_TELEOP_MODULE. In the editor, the ignored
## config/local.toml is used so a clean clone remains safely vision-only.

const MODULE_ROOT := "res://robot_modules"
const LOCAL_CONFIG := "res://config/local.toml"

var _module_id := "disabled"
var _module: Node = null


func _ready() -> void:
	_module_id = _selected_module_id()
	if _module_id == "disabled":
		return
	var scene_path := "%s/%s/godot/RobotModule.tscn" % [MODULE_ROOT, _module_id]
	if not ResourceLoader.exists(scene_path):
		push_error("Robot module %s has no Godot scene at %s" % [_module_id, scene_path])
		return
	var packed := load(scene_path) as PackedScene
	if packed == null:
		push_error("Could not load robot module scene: %s" % scene_path)
		return
	_module = packed.instantiate()
	_module.name = "ActiveRobotModule"
	add_child(_module)


func _selected_module_id() -> String:
	var environment := OS.get_environment("ROBOT_TELEOP_MODULE").strip_edges()
	if _safe_module_id(environment):
		return environment
	if not FileAccess.file_exists(LOCAL_CONFIG):
		return "disabled"
	var file := FileAccess.open(LOCAL_CONFIG, FileAccess.READ)
	if file == null:
		return "disabled"
	var in_robot_table := false
	for raw_line in file.get_as_text().split("\n"):
		var line := raw_line.strip_edges()
		if line.begins_with("["):
			in_robot_table = line == "[robot]"
			continue
		if not in_robot_table or not line.begins_with("adapter"):
			continue
		var parts := line.split("=", true, 1)
		if parts.size() != 2:
			continue
		var candidate := parts[1].strip_edges().trim_prefix("\"").trim_suffix("\"")
		return candidate if _safe_module_id(candidate) else "disabled"
	return "disabled"


func _safe_module_id(value: String) -> bool:
	if value.is_empty() or value.length() > 64:
		return false
	if value[0] < "a" or value[0] > "z":
		return false
	for index in value.length():
		var character := value[index]
		if not (
			character >= "a" and character <= "z"
			or character >= "0" and character <= "9"
			or character in ["-", "_"]
		):
			return false
	return true


func get_module_id() -> String:
	return _module_id
