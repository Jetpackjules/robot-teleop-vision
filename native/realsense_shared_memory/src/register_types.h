#pragma once

#include <godot_cpp/core/class_db.hpp>

void initialize_realsense_shared_memory_module(godot::ModuleInitializationLevel p_level);
void uninitialize_realsense_shared_memory_module(godot::ModuleInitializationLevel p_level);
