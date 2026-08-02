#include "register_types.h"

#include "realsense_direct_frame_source.h"
#include "realsense_pair_calibrator.h"
#include "realsense_shared_memory_point_cloud.h"
#include "realsense_shared_memory_reader.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_realsense_shared_memory_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
    ClassDB::register_class<RealSenseDirectFrameSource>();
    ClassDB::register_class<RealSensePairCalibrator>();
    ClassDB::register_class<RealSenseSharedMemoryReader>();
    ClassDB::register_class<RealSenseSharedMemoryPointCloud>();
}

void uninitialize_realsense_shared_memory_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
}

extern "C" {
GDExtensionBool GDE_EXPORT realsense_shared_memory_library_init(
    GDExtensionInterfaceGetProcAddress p_get_proc_address,
    GDExtensionClassLibraryPtr p_library,
    GDExtensionInitialization *r_initialization
) {
    GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
    init_obj.register_initializer(initialize_realsense_shared_memory_module);
    init_obj.register_terminator(uninitialize_realsense_shared_memory_module);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
    return init_obj.init();
}
}
