#include "register_types.h"

#include "core/config/engine.h"
#include "core/object/class_db.h"
#include "mujoco_server.h"

void initialize_mujoco_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
    GDREGISTER_CLASS(MuJoCoServer);
}

void uninitialize_mujoco_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
}