#include "register_types.h"

#include "core/config/engine.h"
#include "core/object/class_db.h"
#include "mujoco_server.h"

// Route MuJoCo's warnings (e.g. "Nan, Inf or huge value in QACC") into Godot's
// output panel instead of stdout, where they are easy to miss.
static void _mujoco_warning_handler(const char *p_msg) {
	WARN_PRINT(String("MuJoCo: ") + String::utf8(p_msg));
}

void initialize_mujoco_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	mju_user_warning = _mujoco_warning_handler;
	GDREGISTER_CLASS(MuJoCoServer);
}

void uninitialize_mujoco_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	mju_user_warning = nullptr;
}
