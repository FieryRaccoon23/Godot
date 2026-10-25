#include "mujoco_server.h"

#include <cstring>

using namespace MuJoCoConvert;

// =============================================================================
// Actuators
// =============================================================================

int MuJoCoServer::get_nu() const {
	return model ? model->nu : 0;
}

int MuJoCoServer::get_actuator_id(const String &p_name) const {
	ERR_FAIL_NULL_V(model, -1);
	CharString name_utf8 = p_name.utf8();
	int id = mj_name2id(model, mjOBJ_ACTUATOR, name_utf8.get_data());
	if (id < 0) {
		ERR_PRINT(String("MuJoCo: no actuator named '") + p_name + "'");
	}
	return id;
}

String MuJoCoServer::get_actuator_name(int p_actuator_id) const {
	ERR_FAIL_NULL_V(model, String());
	ERR_FAIL_INDEX_V(p_actuator_id, model->nu, String());
	const char *name = mj_id2name(model, mjOBJ_ACTUATOR, p_actuator_id);
	return name ? String::utf8(name) : String();
}

void MuJoCoServer::set_ctrl(int p_actuator_id, double p_value) {
	ERR_FAIL_COND(!is_loaded());
	ERR_FAIL_INDEX(p_actuator_id, model->nu);
	// MuJoCo clamps to ctrlrange itself when the actuator is ctrllimited.
	data->ctrl[p_actuator_id] = p_value;
}

double MuJoCoServer::get_ctrl(int p_actuator_id) const {
	ERR_FAIL_COND_V(!is_loaded(), 0.0);
	ERR_FAIL_INDEX_V(p_actuator_id, model->nu, 0.0);
	return data->ctrl[p_actuator_id];
}

void MuJoCoServer::set_ctrl_array(const PackedFloat64Array &p_ctrl) {
	ERR_FAIL_COND(!is_loaded());
	ERR_FAIL_COND_MSG(p_ctrl.size() != model->nu, "MuJoCo: ctrl array size must equal nu.");
	if (model->nu > 0) {
		memcpy(data->ctrl, p_ctrl.ptr(), sizeof(double) * model->nu);
	}
}

Vector2 MuJoCoServer::get_ctrl_range(int p_actuator_id) const {
	ERR_FAIL_NULL_V(model, Vector2());
	ERR_FAIL_INDEX_V(p_actuator_id, model->nu, Vector2());
	const mjtNum *r = model->actuator_ctrlrange + 2 * p_actuator_id;
	return Vector2((real_t)r[0], (real_t)r[1]);
}

double MuJoCoServer::get_actuator_force(int p_actuator_id) const {
	ERR_FAIL_COND_V(!is_loaded(), 0.0);
	ERR_FAIL_INDEX_V(p_actuator_id, model->nu, 0.0);
	return data->actuator_force[p_actuator_id];
}

// =============================================================================
// Joints
// =============================================================================

int MuJoCoServer::get_njnt() const {
	return model ? model->njnt : 0;
}

int MuJoCoServer::get_joint_id(const String &p_name) const {
	ERR_FAIL_NULL_V(model, -1);
	CharString name_utf8 = p_name.utf8();
	int id = mj_name2id(model, mjOBJ_JOINT, name_utf8.get_data());
	if (id < 0) {
		ERR_PRINT(String("MuJoCo: no joint named '") + p_name + "'");
	}
	return id;
}

String MuJoCoServer::get_joint_name(int p_joint_id) const {
	ERR_FAIL_NULL_V(model, String());
	ERR_FAIL_INDEX_V(p_joint_id, model->njnt, String());
	const char *name = mj_id2name(model, mjOBJ_JOINT, p_joint_id);
	return name ? String::utf8(name) : String();
}

int MuJoCoServer::get_joint_type(int p_joint_id) const {
	ERR_FAIL_NULL_V(model, -1);
	ERR_FAIL_INDEX_V(p_joint_id, model->njnt, -1);
	return model->jnt_type[p_joint_id];
}

int MuJoCoServer::get_joint_body_id(int p_joint_id) const {
	ERR_FAIL_NULL_V(model, -1);
	ERR_FAIL_INDEX_V(p_joint_id, model->njnt, -1);
	return model->jnt_bodyid[p_joint_id];
}

int MuJoCoServer::get_joint_qpos_addr(int p_joint_id) const {
	ERR_FAIL_NULL_V(model, -1);
	ERR_FAIL_INDEX_V(p_joint_id, model->njnt, -1);
	return model->jnt_qposadr[p_joint_id];
}

int MuJoCoServer::get_joint_dof_addr(int p_joint_id) const {
	ERR_FAIL_NULL_V(model, -1);
	ERR_FAIL_INDEX_V(p_joint_id, model->njnt, -1);
	return model->jnt_dofadr[p_joint_id];
}

Vector2 MuJoCoServer::get_joint_range(int p_joint_id) const {
	ERR_FAIL_NULL_V(model, Vector2());
	ERR_FAIL_INDEX_V(p_joint_id, model->njnt, Vector2());
	const mjtNum *r = model->jnt_range + 2 * p_joint_id;
	return Vector2((real_t)r[0], (real_t)r[1]);
}

double MuJoCoServer::get_joint_value(int p_joint_id) const {
	ERR_FAIL_COND_V(!is_loaded(), 0.0);
	ERR_FAIL_INDEX_V(p_joint_id, model->njnt, 0.0);
	int type = model->jnt_type[p_joint_id];
	ERR_FAIL_COND_V_MSG(type != mjJNT_HINGE && type != mjJNT_SLIDE, 0.0,
			"MuJoCo: get_joint_value only supports hinge and slide joints; use get_joint_qpos_addr for free/ball.");
	return data->qpos[model->jnt_qposadr[p_joint_id]];
}

double MuJoCoServer::get_joint_velocity(int p_joint_id) const {
	ERR_FAIL_COND_V(!is_loaded(), 0.0);
	ERR_FAIL_INDEX_V(p_joint_id, model->njnt, 0.0);
	int type = model->jnt_type[p_joint_id];
	ERR_FAIL_COND_V_MSG(type != mjJNT_HINGE && type != mjJNT_SLIDE, 0.0,
			"MuJoCo: get_joint_velocity only supports hinge and slide joints; use get_joint_dof_addr for free/ball.");
	return data->qvel[model->jnt_dofadr[p_joint_id]];
}

// =============================================================================
// Equality constraints (grabbing / attaching)
// =============================================================================

int MuJoCoServer::get_neq() const {
	return model ? model->neq : 0;
}

int MuJoCoServer::get_equality_id(const String &p_name) const {
	ERR_FAIL_NULL_V(model, -1);
	CharString name_utf8 = p_name.utf8();
	int id = mj_name2id(model, mjOBJ_EQUALITY, name_utf8.get_data());
	if (id < 0) {
		ERR_PRINT(String("MuJoCo: no equality constraint named '") + p_name + "'");
	}
	return id;
}

void MuJoCoServer::set_equality_active(int p_eq_id, bool p_active) {
	ERR_FAIL_COND(!is_loaded());
	ERR_FAIL_INDEX(p_eq_id, model->neq);
	data->eq_active[p_eq_id] = p_active ? 1 : 0;
}

bool MuJoCoServer::is_equality_active(int p_eq_id) const {
	ERR_FAIL_COND_V(!is_loaded(), false);
	ERR_FAIL_INDEX_V(p_eq_id, model->neq, false);
	return data->eq_active[p_eq_id] != 0;
}

bool MuJoCoServer::snap_equality_to_current_pose(int p_eq_id) {
	ERR_FAIL_COND_V(!is_loaded(), false);
	ERR_FAIL_INDEX_V(p_eq_id, model->neq, false);
	ERR_FAIL_COND_V_MSG(model->eq_objtype[p_eq_id] != mjOBJ_BODY, false,
			"MuJoCo: snap_equality_to_current_pose only supports body-based weld/connect.");

	const int b1 = model->eq_obj1id[p_eq_id];
	const int b2 = model->eq_obj2id[p_eq_id];
	mjtNum *ed = model->eq_data + mjNEQDATA * p_eq_id;

	const mjtNum *p1 = data->xpos + 3 * b1;
	const mjtNum *p2 = data->xpos + 3 * b2;
	const mjtNum *R1 = data->xmat + 9 * b1;
	const mjtNum *R2 = data->xmat + 9 * b2;
	const mjtNum *q1 = data->xquat + 4 * b1;
	const mjtNum *q2 = data->xquat + 4 * b2;

	switch (model->eq_type[p_eq_id]) {
		case mjEQ_WELD: {
			// eq_data: [anchor(3), relpose pos(3), relpose quat(4), torquescale]
			// Hold body2 exactly where it is now relative to body1.
			mjtNum diff[3];
			mju_sub3(diff, p2, p1);
			ed[0] = ed[1] = ed[2] = 0;
			mju_mulMatTVec3(ed + 3, R1, diff);
			mjtNum q1_inv[4];
			mju_negQuat(q1_inv, q1);
			mju_mulQuat(ed + 6, q1_inv, q2);
			return true;
		}
		case mjEQ_CONNECT: {
			// eq_data: [anchor in body1 frame(3), anchor in body2 frame(3)]
			// Keep body1's anchor, recompute body2's so the points coincide now.
			mjtNum a1_world[3], diff[3];
			mju_mulMatVec3(a1_world, R1, ed);
			mju_addTo3(a1_world, p1);
			mju_sub3(diff, a1_world, p2);
			mju_mulMatTVec3(ed + 3, R2, diff);
			return true;
		}
		default:
			ERR_FAIL_V_MSG(false, "MuJoCo: snap_equality_to_current_pose only supports weld and connect.");
	}
}

// =============================================================================
// Sensors
// =============================================================================

int MuJoCoServer::get_nsensor() const {
	return model ? model->nsensor : 0;
}

int MuJoCoServer::get_sensor_id(const String &p_name) const {
	ERR_FAIL_NULL_V(model, -1);
	CharString name_utf8 = p_name.utf8();
	int id = mj_name2id(model, mjOBJ_SENSOR, name_utf8.get_data());
	if (id < 0) {
		ERR_PRINT(String("MuJoCo: no sensor named '") + p_name + "'");
	}
	return id;
}

int MuJoCoServer::get_sensor_dim(int p_sensor_id) const {
	ERR_FAIL_NULL_V(model, 0);
	ERR_FAIL_INDEX_V(p_sensor_id, model->nsensor, 0);
	return model->sensor_dim[p_sensor_id];
}

PackedFloat64Array MuJoCoServer::get_sensor_data(int p_sensor_id) const {
	PackedFloat64Array arr;
	ERR_FAIL_COND_V(!is_loaded(), arr);
	ERR_FAIL_INDEX_V(p_sensor_id, model->nsensor, arr);
	// Raw MuJoCo values: vector sensors are in MuJoCo's frame (Z-up).
	int adr = model->sensor_adr[p_sensor_id];
	int dim = model->sensor_dim[p_sensor_id];
	arr.resize(dim);
	if (dim > 0) {
		memcpy(arr.ptrw(), data->sensordata + adr, sizeof(double) * dim);
	}
	return arr;
}

// =============================================================================
// Applied forces
// =============================================================================

void MuJoCoServer::apply_force(int p_body_id, const Vector3 &p_force, const Vector3 &p_torque) {
	ERR_FAIL_COND(!is_loaded());
	ERR_FAIL_INDEX(p_body_id, model->nbody);
	mjtNum *x = data->xfrc_applied + 6 * p_body_id; // [force(3), torque(3)], world frame, at CoM
	to_mujoco(p_force, x);
	to_mujoco(p_torque, x + 3);
}

void MuJoCoServer::clear_applied_forces() {
	ERR_FAIL_COND(!is_loaded());
	memset(data->xfrc_applied, 0, sizeof(mjtNum) * 6 * model->nbody);
}
