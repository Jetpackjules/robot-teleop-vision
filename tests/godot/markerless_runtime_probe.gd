extends SceneTree

func _initialize() -> void:
    if not ClassDB.class_exists("RealSensePairCalibrator"):
        push_error("RealSensePairCalibrator is missing")
        quit(1)
        return
    var calibrator = ClassDB.instantiate("RealSensePairCalibrator")
    if not calibrator.has_method("validate_markerless_runtime"):
        push_error("Stale extension: markerless runtime validation is missing")
        quit(1)
        return
    var model_path := ProjectSettings.globalize_path("res://native/realsense_shared_memory/models/superpoint_lightglue_pipeline.onnx")
    var check: Dictionary = calibrator.validate_markerless_runtime(model_path)
    print(JSON.stringify(check))
    if not bool(check.get("ok", false)):
        quit(1)
        return
    var missing: Dictionary = calibrator.validate_markerless_runtime(model_path + ".missing")
    if bool(missing.get("ok", true)):
        push_error("Missing model was incorrectly accepted")
        quit(1)
        return
    print("MARKERLESS_RUNTIME_OK: native model inference passed; missing model rejected; no hardware accessed")
    quit(0)
