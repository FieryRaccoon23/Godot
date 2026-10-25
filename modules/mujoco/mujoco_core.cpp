#include "mujoco_server.h"

#include "core/config/project_settings.h"
#include "core/io/file_access.h"

#include <cmath>
#include <cstring>

using namespace MuJoCoConvert;

// =============================================================================
// Loading / lifecycle
// =============================================================================

bool MuJoCoServer::_finish_load(mjModel *p_model) {
	unload();
	model = p_model;
	data = mj_makeData(model);
	if (!data) {
		last_error = "MuJoCo: mj_makeData failed (out of memory?)";
		ERR_PRINT(last_error);
		mj_deleteModel(model);
		model = nullptr;
		return false;
	}
	// Compute positions/contacts for the initial pose so visuals are correct
	// before the first step (e.g. while the player is still in build mode).
	mj_forward(model, data);
	last_error = "";
	return true;
}

bool MuJoCoServer::load_model(const String &p_path) {
	// Prefer loading straight from disk so <include> and mesh files resolve
	// relative to the XML. This works in the editor and for user:// files.
	String abs_path = p_path;
	if (p_path.begins_with("res://") || p_path.begins_with("user://")) {
		abs_path = ProjectSettings::get_singleton()->globalize_path(p_path);
	}

	if (FileAccess::exists(abs_path)) {
		char error[1024] = "";
		CharString path_utf8 = abs_path.utf8();
		mjModel *m = mj_loadXML(path_utf8.get_data(), nullptr, error, sizeof(error));
		if (!m) {
			last_error = String("MuJoCo failed to load '") + p_path + "': " + String::utf8(error);
			ERR_PRINT(last_error);
			return false;
		}
		return _finish_load(m);
	}

	// In an exported game, res:// lives inside the .pck and is not a real file on
	// disk, so read it through Godot and compile from the string instead.
	// (Mesh files / <include> are not resolved in this path; primitives are fine.)
	Error err = OK;
	String xml = FileAccess::get_file_as_string(p_path, &err);
	if (err != OK) {
		last_error = String("MuJoCo: could not read '") + p_path + "'";
		ERR_PRINT(last_error);
		return false;
	}
	return load_model_from_string(xml);
}

bool MuJoCoServer::load_model_from_string(const String &p_xml) {
	// Requires MuJoCo 3.2+ (mjSpec API).
	char error[1024] = "";
	CharString xml_utf8 = p_xml.utf8();

	mjSpec *spec = mj_parseXMLString(xml_utf8.get_data(), nullptr, error, sizeof(error));
	if (!spec) {
		last_error = String("MuJoCo XML parse error: ") + String::utf8(error);
		ERR_PRINT(last_error);
		return false;
	}

	mjModel *m = mj_compile(spec, nullptr);
	if (!m) {
		last_error = String("MuJoCo compile error: ") + String::utf8(mjs_getError(spec));
		ERR_PRINT(last_error);
		mj_deleteSpec(spec);
		return false;
	}
	mj_deleteSpec(spec);

	// Only replace the current model once the new one compiled successfully.
	return _finish_load(m);
}

String MuJoCoServer::get_last_error() const {
	return last_error;
}

bool MuJoCoServer::is_loaded() const {
	return model != nullptr && data != nullptr;
}

void MuJoCoServer::unload() {
	if (data) {
		mj_deleteData(data);
		data = nullptr;
	}
	if (model) {
		mj_deleteModel(model);
		model = nullptr;
	}
}

String MuJoCoServer::get_version_string() const {
	return String::utf8(mj_versionString());
}

// =============================================================================
// Stepping
// =============================================================================

void MuJoCoServer::step() {
	ERR_FAIL_COND(!is_loaded());
	mj_step(model, data);
}

void MuJoCoServer::step_n(int p_steps) {
	ERR_FAIL_COND(!is_loaded());
	for (int i = 0; i < p_steps; i++) {
		mj_step(model, data);
	}
}

void MuJoCoServer::forward() {
	ERR_FAIL_COND(!is_loaded());
	mj_forward(model, data);
}

void MuJoCoServer::reset() {
	ERR_FAIL_COND(!is_loaded());
	mj_resetData(model, data);
	mj_forward(model, data);
}

void MuJoCoServer::reset_to_keyframe(int p_key_index) {
	ERR_FAIL_COND(!is_loaded());
	ERR_FAIL_INDEX(p_key_index, model->nkey);
	mj_resetDataKeyframe(model, data, p_key_index);
	mj_forward(model, data);
}

int MuJoCoServer::get_nkey() const {
	return model ? model->nkey : 0;
}

double MuJoCoServer::get_time() const {
	return data ? data->time : 0.0;
}

// =============================================================================
// Stability
// =============================================================================

bool MuJoCoServer::has_diverged() const {
	ERR_FAIL_COND_V(!is_loaded(), false);
	if (data->warning[mjWARN_BADQACC].number > 0) {
		return true;
	}
	for (int i = 0; i < model->nq; i++) {
		if (!std::isfinite(data->qpos[i])) {
			return true;
		}
	}
	for (int i = 0; i < model->nv; i++) {
		if (!std::isfinite(data->qvel[i])) {
			return true;
		}
	}
	return false;
}

void MuJoCoServer::clear_warnings() {
	ERR_FAIL_COND(!is_loaded());
	for (int i = 0; i < mjNWARNING; i++) {
		data->warning[i].number = 0;
		data->warning[i].lastinfo = 0;
	}
}

// =============================================================================
// Simulation options
// =============================================================================

double MuJoCoServer::get_timestep() const {
	ERR_FAIL_NULL_V(model, 0.0);
	return model->opt.timestep;
}

void MuJoCoServer::set_timestep(double p_timestep) {
	ERR_FAIL_NULL(model);
	ERR_FAIL_COND_MSG(p_timestep <= 0.0, "MuJoCo: timestep must be positive.");
	model->opt.timestep = p_timestep;
}

int MuJoCoServer::get_solver_iterations() const {
	ERR_FAIL_NULL_V(model, 0);
	return model->opt.iterations;
}

void MuJoCoServer::set_solver_iterations(int p_iterations) {
	ERR_FAIL_NULL(model);
	ERR_FAIL_COND(p_iterations < 1);
	model->opt.iterations = p_iterations;
}

int MuJoCoServer::get_integrator() const {
	ERR_FAIL_NULL_V(model, -1);
	return model->opt.integrator;
}

void MuJoCoServer::set_integrator(int p_integrator) {
	ERR_FAIL_NULL(model);
	ERR_FAIL_INDEX(p_integrator, 4);
	model->opt.integrator = p_integrator;
}

Vector3 MuJoCoServer::get_gravity() const {
	ERR_FAIL_NULL_V(model, Vector3());
	return to_godot(model->opt.gravity);
}

void MuJoCoServer::set_gravity(const Vector3 &p_gravity) {
	ERR_FAIL_NULL(model);
	to_mujoco(p_gravity, model->opt.gravity);
}

// =============================================================================
// Generalized coordinates
// =============================================================================

int MuJoCoServer::get_nq() const {
	return model ? model->nq : 0;
}

double MuJoCoServer::get_qpos(int p_index) const {
	ERR_FAIL_COND_V(!is_loaded(), 0.0);
	ERR_FAIL_INDEX_V(p_index, model->nq, 0.0);
	return data->qpos[p_index];
}

void MuJoCoServer::set_qpos(int p_index, double p_value) {
	ERR_FAIL_COND(!is_loaded());
	ERR_FAIL_INDEX(p_index, model->nq);
	data->qpos[p_index] = p_value;
}

PackedFloat64Array MuJoCoServer::get_qpos_array() const {
	PackedFloat64Array arr;
	ERR_FAIL_COND_V(!is_loaded(), arr);
	arr.resize(model->nq);
	if (model->nq > 0) {
		memcpy(arr.ptrw(), data->qpos, sizeof(double) * model->nq);
	}
	return arr;
}

void MuJoCoServer::set_qpos_array(const PackedFloat64Array &p_qpos) {
	ERR_FAIL_COND(!is_loaded());
	ERR_FAIL_COND_MSG(p_qpos.size() != model->nq, "MuJoCo: qpos array size must equal nq.");
	if (model->nq > 0) {
		memcpy(data->qpos, p_qpos.ptr(), sizeof(double) * model->nq);
	}
}

int MuJoCoServer::get_nv() const {
	return model ? model->nv : 0;
}

double MuJoCoServer::get_qvel(int p_index) const {
	ERR_FAIL_COND_V(!is_loaded(), 0.0);
	ERR_FAIL_INDEX_V(p_index, model->nv, 0.0);
	return data->qvel[p_index];
}

void MuJoCoServer::set_qvel(int p_index, double p_value) {
	ERR_FAIL_COND(!is_loaded());
	ERR_FAIL_INDEX(p_index, model->nv);
	data->qvel[p_index] = p_value;
}

PackedFloat64Array MuJoCoServer::get_qvel_array() const {
	PackedFloat64Array arr;
	ERR_FAIL_COND_V(!is_loaded(), arr);
	arr.resize(model->nv);
	if (model->nv > 0) {
		memcpy(arr.ptrw(), data->qvel, sizeof(double) * model->nv);
	}
	return arr;
}

void MuJoCoServer::set_qvel_array(const PackedFloat64Array &p_qvel) {
	ERR_FAIL_COND(!is_loaded());
	ERR_FAIL_COND_MSG(p_qvel.size() != model->nv, "MuJoCo: qvel array size must equal nv.");
	if (model->nv > 0) {
		memcpy(data->qvel, p_qvel.ptr(), sizeof(double) * model->nv);
	}
}

double MuJoCoServer::get_qacc(int p_index) const {
	ERR_FAIL_COND_V(!is_loaded(), 0.0);
	ERR_FAIL_INDEX_V(p_index, model->nv, 0.0);
	return data->qacc[p_index];
}

// =============================================================================
// Full state snapshot
// =============================================================================

PackedFloat64Array MuJoCoServer::get_state() const {
	PackedFloat64Array arr;
	ERR_FAIL_COND_V(!is_loaded(), arr);
	int size = mj_stateSize(model, mjSTATE_INTEGRATION);
	arr.resize(size);
	mj_getState(model, data, arr.ptrw(), mjSTATE_INTEGRATION);
	return arr;
}

bool MuJoCoServer::set_state(const PackedFloat64Array &p_state) {
	ERR_FAIL_COND_V(!is_loaded(), false);
	int size = mj_stateSize(model, mjSTATE_INTEGRATION);
	ERR_FAIL_COND_V_MSG(p_state.size() != size, false,
			"MuJoCo: state size mismatch (was it saved from a different model?).");
	mj_setState(model, data, p_state.ptr(), mjSTATE_INTEGRATION);
	mj_forward(model, data);
	return true;
}

// =============================================================================
// Bodies
// =============================================================================

int MuJoCoServer::get_nbody() const {
	return model ? model->nbody : 0;
}

int MuJoCoServer::get_body_id(const String &p_body_name) const {
	ERR_FAIL_NULL_V(model, -1);
	CharString name_utf8 = p_body_name.utf8();
	int id = mj_name2id(model, mjOBJ_BODY, name_utf8.get_data());
	if (id < 0) {
		ERR_PRINT(String("MuJoCo: no body named '") + p_body_name + "'");
	}
	return id;
}

String MuJoCoServer::get_body_name(int p_body_id) const {
	ERR_FAIL_NULL_V(model, String());
	ERR_FAIL_INDEX_V(p_body_id, model->nbody, String());
	const char *name = mj_id2name(model, mjOBJ_BODY, p_body_id);
	return name ? String::utf8(name) : String();
}

int MuJoCoServer::get_body_parent_id(int p_body_id) const {
	ERR_FAIL_NULL_V(model, -1);
	ERR_FAIL_INDEX_V(p_body_id, model->nbody, -1);
	return model->body_parentid[p_body_id];
}

double MuJoCoServer::get_body_mass(int p_body_id) const {
	ERR_FAIL_NULL_V(model, 0.0);
	ERR_FAIL_INDEX_V(p_body_id, model->nbody, 0.0);
	return model->body_mass[p_body_id];
}

Vector3 MuJoCoServer::get_body_pos(int p_body_id) const {
	ERR_FAIL_COND_V(!is_loaded(), Vector3());
	ERR_FAIL_INDEX_V(p_body_id, model->nbody, Vector3());
	return to_godot(data->xpos + 3 * p_body_id);
}

Quaternion MuJoCoServer::get_body_quat(int p_body_id) const {
	ERR_FAIL_COND_V(!is_loaded(), Quaternion());
	ERR_FAIL_INDEX_V(p_body_id, model->nbody, Quaternion());
	return quat_to_godot(data->xquat + 4 * p_body_id);
}

Transform3D MuJoCoServer::get_body_transform(int p_body_id) const {
	ERR_FAIL_COND_V(!is_loaded(), Transform3D());
	ERR_FAIL_INDEX_V(p_body_id, model->nbody, Transform3D());
	return Transform3D(Basis(quat_to_godot(data->xquat + 4 * p_body_id)),
			to_godot(data->xpos + 3 * p_body_id));
}

Vector3 MuJoCoServer::get_body_linear_velocity(int p_body_id) const {
	ERR_FAIL_COND_V(!is_loaded(), Vector3());
	ERR_FAIL_INDEX_V(p_body_id, model->nbody, Vector3());
	mjtNum vel[6]; // [angular(3), linear(3)], world orientation, at body origin
	mj_objectVelocity(model, data, mjOBJ_BODY, p_body_id, vel, 0);
	return to_godot(vel + 3);
}

Vector3 MuJoCoServer::get_body_angular_velocity(int p_body_id) const {
	ERR_FAIL_COND_V(!is_loaded(), Vector3());
	ERR_FAIL_INDEX_V(p_body_id, model->nbody, Vector3());
	mjtNum vel[6];
	mj_objectVelocity(model, data, mjOBJ_BODY, p_body_id, vel, 0);
	return to_godot(vel);
}

PackedFloat32Array MuJoCoServer::get_body_transforms_packed() const {
	PackedFloat32Array arr;
	ERR_FAIL_COND_V(!is_loaded(), arr);
	arr.resize(model->nbody * 7);
	float *w = arr.ptrw();
	for (int i = 0; i < model->nbody; i++) {
		Vector3 p = to_godot(data->xpos + 3 * i);
		Quaternion q = quat_to_godot(data->xquat + 4 * i);
		float *o = w + 7 * i;
		o[0] = p.x;
		o[1] = p.y;
		o[2] = p.z;
		o[3] = q.x;
		o[4] = q.y;
		o[5] = q.z;
		o[6] = q.w;
	}
	return arr;
}
