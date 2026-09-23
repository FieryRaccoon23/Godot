#ifndef MUJOCO_REGISTER_TYPES_H
#define MUJOCO_REGISTER_TYPES_H

#include "modules/register_module_types.h"

void initialize_mujoco_module(ModuleInitializationLevel p_level);
void uninitialize_mujoco_module(ModuleInitializationLevel p_level);

#endif // MUJOCO_REGISTER_TYPES_H