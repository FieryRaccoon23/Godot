#include "register_types.h"

#include "core/config/engine.h"
#include "needle_engine.h"

static NeedleEngine *needle_engine = nullptr;

void initialize_needle_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	GDREGISTER_CLASS(NeedleEngine);
	needle_engine = memnew(NeedleEngine);
	Engine::get_singleton()->add_singleton(Engine::Singleton("NeedleEngine", needle_engine));
}

void uninitialize_needle_module(ModuleInitializationLevel p_level) {
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	if (needle_engine) {
		memdelete(needle_engine);
		needle_engine = nullptr;
	}
}
