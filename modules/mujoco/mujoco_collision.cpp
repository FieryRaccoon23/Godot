#include "mujoco_server.h"

using namespace MuJoCoConvert;

// =============================================================================
// Geoms (static properties)
// =============================================================================

int MuJoCoServer::get_ngeom() const {
	return model ? model->ngeom : 0;
}

int MuJoCoServer::get_geom_id(const String &p_geom_name) const {
	ERR_FAIL_NULL_V(model, -1);
	CharString name_utf8 = p_geom_name.utf8();
	int id = mj_name2id(model, mjOBJ_GEOM, name_utf8.get_data());
	if (id < 0) {
		ERR_PRINT(String("MuJoCo: no geom named '") + p_geom_name + "'");
	}
	return id;
}

String MuJoCoServer::get_geom_name(int p_geom_id) const {
	ERR_FAIL_NULL_V(model, String());
	ERR_FAIL_INDEX_V(p_geom_id, model->ngeom, String());
	const char *name = mj_id2name(model, mjOBJ_GEOM, p_geom_id);
	return name ? String::utf8(name) : String();
}

int MuJoCoServer::get_geom_type(int p_geom_id) const {
	ERR_FAIL_NULL_V(model, -1);
	ERR_FAIL_INDEX_V(p_geom_id, model->ngeom, -1);
	return model->geom_type[p_geom_id]; // mjtGeom: 0=plane 2=sphere 3=capsule 4=ellipsoid 5=cylinder 6=box 7=mesh
}

int MuJoCoServer::get_geom_body_id(int p_geom_id) const {
	ERR_FAIL_NULL_V(model, -1);
	ERR_FAIL_INDEX_V(p_geom_id, model->ngeom, -1);
	return model->geom_bodyid[p_geom_id];
}

Vector3 MuJoCoServer::get_geom_size(int p_geom_id) const {
	ERR_FAIL_NULL_V(model, Vector3());
	ERR_FAIL_INDEX_V(p_geom_id, model->ngeom, Vector3());
	// Raw MuJoCo size; meaning depends on type:
	//   sphere: x = radius
	//   capsule/cylinder: x = radius, y = half-length (along local Z in MuJoCo,
	//                     which is local Y in Godot)
	//   box: x/y/z half-extents in MuJoCo's local frame. For a Godot BoxMesh
	//        use size = 2 * Vector3(x, z, y).
	const mjtNum *s = model->geom_size + 3 * p_geom_id;
	return Vector3((real_t)s[0], (real_t)s[1], (real_t)s[2]);
}

Vector3 MuJoCoServer::get_geom_friction(int p_geom_id) const {
	ERR_FAIL_NULL_V(model, Vector3());
	ERR_FAIL_INDEX_V(p_geom_id, model->ngeom, Vector3());
	// x = sliding, y = torsional, z = rolling
	const mjtNum *f = model->geom_friction + 3 * p_geom_id;
	return Vector3((real_t)f[0], (real_t)f[1], (real_t)f[2]);
}

Color MuJoCoServer::get_geom_rgba(int p_geom_id) const {
	ERR_FAIL_NULL_V(model, Color());
	ERR_FAIL_INDEX_V(p_geom_id, model->ngeom, Color());
	const float *c = model->geom_rgba + 4 * p_geom_id;
	return Color(c[0], c[1], c[2], c[3]);
}

// =============================================================================
// Geoms (world pose, per step)
// =============================================================================

Vector3 MuJoCoServer::get_geom_pos(int p_geom_id) const {
	ERR_FAIL_COND_V(!is_loaded(), Vector3());
	ERR_FAIL_INDEX_V(p_geom_id, model->ngeom, Vector3());
	return to_godot(data->geom_xpos + 3 * p_geom_id);
}

Quaternion MuJoCoServer::get_geom_quat(int p_geom_id) const {
	ERR_FAIL_COND_V(!is_loaded(), Quaternion());
	ERR_FAIL_INDEX_V(p_geom_id, model->ngeom, Quaternion());
	mjtNum q[4];
	mju_mat2Quat(q, data->geom_xmat + 9 * p_geom_id);
	return quat_to_godot(q);
}

PackedFloat32Array MuJoCoServer::get_geom_transforms_packed() const {
	PackedFloat32Array arr;
	ERR_FAIL_COND_V(!is_loaded(), arr);
	arr.resize(model->ngeom * 7);
	float *w = arr.ptrw();
	mjtNum mq[4];
	for (int i = 0; i < model->ngeom; i++) {
		Vector3 p = to_godot(data->geom_xpos + 3 * i);
		mju_mat2Quat(mq, data->geom_xmat + 9 * i);
		Quaternion q = quat_to_godot(mq);
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

// =============================================================================
// Contacts (per step)
// =============================================================================

int MuJoCoServer::get_contact_count() const {
	return data ? data->ncon : 0;
}

Vector3 MuJoCoServer::get_contact_pos(int p_contact_index) const {
	ERR_FAIL_COND_V(!is_loaded(), Vector3());
	ERR_FAIL_INDEX_V(p_contact_index, data->ncon, Vector3());
	return to_godot(data->contact[p_contact_index].pos);
}

Vector3 MuJoCoServer::get_contact_normal(int p_contact_index) const {
	ERR_FAIL_COND_V(!is_loaded(), Vector3());
	ERR_FAIL_INDEX_V(p_contact_index, data->ncon, Vector3());
	// First row of the contact frame; points from geom1 towards geom2.
	return to_godot(data->contact[p_contact_index].frame);
}

double MuJoCoServer::get_contact_depth(int p_contact_index) const {
	ERR_FAIL_COND_V(!is_loaded(), 0.0);
	ERR_FAIL_INDEX_V(p_contact_index, data->ncon, 0.0);
	// Negative = penetrating, positive = separated (within margin).
	return data->contact[p_contact_index].dist;
}

int MuJoCoServer::get_contact_geom1(int p_contact_index) const {
	ERR_FAIL_COND_V(!is_loaded(), -1);
	ERR_FAIL_INDEX_V(p_contact_index, data->ncon, -1);
	return data->contact[p_contact_index].geom[0];
}

int MuJoCoServer::get_contact_geom2(int p_contact_index) const {
	ERR_FAIL_COND_V(!is_loaded(), -1);
	ERR_FAIL_INDEX_V(p_contact_index, data->ncon, -1);
	return data->contact[p_contact_index].geom[1];
}

double MuJoCoServer::get_contact_normal_force(int p_contact_index) const {
	ERR_FAIL_COND_V(!is_loaded(), 0.0);
	ERR_FAIL_INDEX_V(p_contact_index, data->ncon, 0.0);
	mjtNum f[6];
	mj_contactForce(model, data, p_contact_index, f);
	return f[0];
}

Vector3 MuJoCoServer::get_contact_force(int p_contact_index) const {
	ERR_FAIL_COND_V(!is_loaded(), Vector3());
	ERR_FAIL_INDEX_V(p_contact_index, data->ncon, Vector3());
	mjtNum f[6]; // contact frame: [normal, tangent1, tangent2, torques...]
	mj_contactForce(model, data, p_contact_index, f);
	const mjtNum *fr = data->contact[p_contact_index].frame;
	mjtNum world[3];
	for (int k = 0; k < 3; k++) {
		world[k] = f[0] * fr[k] + f[1] * fr[3 + k] + f[2] * fr[6 + k];
	}
	return to_godot(world);
}

// =============================================================================
// Raycast
// =============================================================================

Dictionary MuJoCoServer::raycast(const Vector3 &p_origin, const Vector3 &p_direction, double p_max_distance, bool p_include_static, int p_exclude_body_id) const {
	Dictionary result;
	ERR_FAIL_COND_V(!is_loaded(), result);
	ERR_FAIL_COND_V_MSG(p_direction.length_squared() == 0, result, "MuJoCo: raycast direction is zero.");

	Vector3 dir = p_direction.normalized();
	mjtNum pnt[3], vec[3];
	to_mujoco(p_origin, pnt);
	to_mujoco(dir, vec);

	int geom_id = -1;
	// geomgroup = nullptr -> test all geom groups.
	// MuJoCo 3.5.0 added a trailing `normal` output argument to mj_ray.
	// (mjVERSION_HEADER is e.g. 340 for 3.4.0 and 3005000 for 3.5.0.)
#if mjVERSION_HEADER >= 3005000
	mjtNum dist = mj_ray(model, data, pnt, vec, nullptr,
			p_include_static ? 1 : 0, p_exclude_body_id, &geom_id, nullptr);
#else
	mjtNum dist = mj_ray(model, data, pnt, vec, nullptr,
			p_include_static ? 1 : 0, p_exclude_body_id, &geom_id);
#endif

	if (geom_id < 0 || dist < 0 || dist > p_max_distance) {
		return result; // miss
	}

	result["geom_id"] = geom_id;
	result["body_id"] = model->geom_bodyid[geom_id];
	result["distance"] = dist;
	result["position"] = p_origin + dir * (real_t)dist;
	return result;
}
