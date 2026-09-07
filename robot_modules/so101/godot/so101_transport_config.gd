extends RefCounted

## Only non-secret, launcher-resolved wiring is read here. In particular, do not
## parse TOML/profile paths in GDScript or guess a different physical follower.
const ENVIRONMENT_NAME := "ROBOT_TELEOP_RUNTIME_CONFIG"
const SETTINGS_PATH := "res://.teleop/runtime_config.json"
const DEFAULT_PORTS := {
	"command_port": 4248,
	"status_port": 4249,
	"telemetry_port": 4250,
	"editor_telemetry_port": 4252,
}


static func resolve(environment_json: String, settings_path: String = SETTINGS_PATH) -> Dictionary:
	var contents := environment_json
	if contents.is_empty():
		if not FileAccess.file_exists(settings_path):
			# No guessed ports: an unrelated local follower may own the defaults,
			# and a clean editor must not bypass a disabled hardware configuration.
			return {"error": "Robot transport settings are not resolved. Start robot-teleop with the intended profile, then reopen this scene."}
		var file := FileAccess.open(settings_path, FileAccess.READ)
		if file == null:
			return {"error": "Cannot read resolved robot ports. Restart robot-teleop and reopen this scene."}
		contents = file.get_as_text()
	var parser := JSON.new()
	if parser.parse(contents) != OK or not parser.data is Dictionary:
		return {"error": "Invalid resolved robot configuration. Restart robot-teleop and reopen this scene."}
	return validate(parser.data)


static func validate(configuration: Dictionary) -> Dictionary:
	if configuration.get("schema_version") != 1 or configuration.get("module") != "so101":
		return {"error": "Resolved configuration does not select SO-101. Restart the launcher with the intended config, then reopen this scene."}
	if not configuration.get("enabled") is bool:
		return {"error": "Resolved robot configuration is missing its enabled flag."}
	if not configuration.enabled:
		return {"ports": DEFAULT_PORTS.duplicate(), "calibration_status_port": 4251, "enabled": false}
	var settings = configuration.get("module_settings")
	if not settings is Dictionary:
		return {"error": "Resolved SO-101 port settings are missing. Restart robot-teleop."}
	var ports: Dictionary = {}
	var occupied: Array[int] = []
	for name in DEFAULT_PORTS:
		var value = settings.get(name)
		if not _valid_port(value):
			return {"error": "Invalid SO-101 %s: expected a UDP port from 1 to 65535." % name}
		var port := int(value)
		if port in occupied:
			return {"error": "SO-101 UDP ports must be distinct; %s conflicts on %d." % [name, port]}
		ports[name] = port
		occupied.append(port)
	var calibration_port = configuration.get("calibration_status_port")
	if not _valid_port(calibration_port) or int(calibration_port) in occupied:
		return {"error": "Calibration status UDP port is invalid or conflicts with a follower port."}
	return {"ports": ports, "calibration_status_port": int(calibration_port), "enabled": configuration.enabled}


static func _valid_port(value: Variant) -> bool:
	return (value is int or value is float) and float(value) == floorf(float(value)) and float(value) >= 1.0 and float(value) <= 65535.0
