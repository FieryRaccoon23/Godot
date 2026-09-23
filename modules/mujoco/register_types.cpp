#include "register_types.h"

#include "core/config/engine.h"

// Included MuJoCo header to verify include paths during build
#include "mujoco/mujoco.h"

void initialize_mujoco_module(ModuleInitializationLevel p_level) {
	// Only initialize logic during SERVERS or SCENE initialization levels
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

	// Register custom Godot classes or singletons exposing MuJoCo here later:
	// GDREGISTER_CLASS(MyGodotMuJoCoWrapper);
}

void uninitialize_mujoco_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}

	// Cleanup custom singletons or resources here later
}