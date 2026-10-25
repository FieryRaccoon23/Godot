#include "mujoco_server.h"

using namespace MuJoCoConvert;

// Note: these work for ropes made with <flexcomp> (a 1D flex). A rope made with
// the "cable" <composite> is a chain of ordinary bodies; read it with the body
// functions (e.g. get_body_transforms_packed) instead.

int MuJoCoServer::get_flex_id(const String &p_flex_name) const {
	ERR_FAIL_NULL_V(model, -1);
	CharString name_utf8 = p_flex_name.utf8();
	int id = mj_name2id(model, mjOBJ_FLEX, name_utf8.get_data());
	if (id < 0) {
		ERR_PRINT(String("MuJoCo: no flex named '") + p_flex_name + "'");
	}
	return id;
}

int MuJoCoServer::get_flex_vertex_count(int p_flex_id) const {
	ERR_FAIL_NULL_V(model, 0);
	ERR_FAIL_INDEX_V(p_flex_id, model->nflex, 0);
	return model->flex_vertnum[p_flex_id];
}

Vector3 MuJoCoServer::get_flex_vertex_pos(int p_flex_id, int p_vertex_index) const {
	ERR_FAIL_COND_V(!is_loaded(), Vector3());
	ERR_FAIL_INDEX_V(p_flex_id, model->nflex, Vector3());
	ERR_FAIL_INDEX_V(p_vertex_index, model->flex_vertnum[p_flex_id], Vector3());
	int global_vert = model->flex_vertadr[p_flex_id] + p_vertex_index;
	return to_godot(data->flexvert_xpos + 3 * global_vert);
}

PackedVector3Array MuJoCoServer::get_flex_vertex_positions(int p_flex_id) const {
	PackedVector3Array arr;
	ERR_FAIL_COND_V(!is_loaded(), arr);
	ERR_FAIL_INDEX_V(p_flex_id, model->nflex, arr);
	int count = model->flex_vertnum[p_flex_id];
	int base = model->flex_vertadr[p_flex_id];
	arr.resize(count);
	Vector3 *w = arr.ptrw();
	for (int i = 0; i < count; i++) {
		w[i] = to_godot(data->flexvert_xpos + 3 * (base + i));
	}
	return arr;
}

double MuJoCoServer::get_flex_radius(int p_flex_id) const {
	ERR_FAIL_NULL_V(model, 0.0);
	ERR_FAIL_INDEX_V(p_flex_id, model->nflex, 0.0);
	return model->flex_radius[p_flex_id];
}

int MuJoCoServer::get_flex_edge_count(int p_flex_id) const {
	ERR_FAIL_NULL_V(model, 0);
	ERR_FAIL_INDEX_V(p_flex_id, model->nflex, 0);
	return model->flex_edgenum[p_flex_id];
}

Vector2i MuJoCoServer::get_flex_edge(int p_flex_id, int p_edge_index) const {
	ERR_FAIL_NULL_V(model, Vector2i());
	ERR_FAIL_INDEX_V(p_flex_id, model->nflex, Vector2i());
	ERR_FAIL_INDEX_V(p_edge_index, model->flex_edgenum[p_flex_id], Vector2i());
	int base = 2 * (model->flex_edgeadr[p_flex_id] + p_edge_index);
	// Local vertex indices within this flex.
	return Vector2i(model->flex_edge[base + 0], model->flex_edge[base + 1]);
}

double MuJoCoServer::get_flex_edge_length(int p_flex_id, int p_edge_index) const {
	ERR_FAIL_COND_V(!is_loaded(), 0.0);
	ERR_FAIL_INDEX_V(p_flex_id, model->nflex, 0.0);
	ERR_FAIL_INDEX_V(p_edge_index, model->flex_edgenum[p_flex_id], 0.0);
	int global_edge = model->flex_edgeadr[p_flex_id] + p_edge_index;
	return data->flexedge_length[global_edge];
}

double MuJoCoServer::get_flex_edge_rest_length(int p_flex_id, int p_edge_index) const {
	ERR_FAIL_NULL_V(model, 0.0);
	ERR_FAIL_INDEX_V(p_flex_id, model->nflex, 0.0);
	ERR_FAIL_INDEX_V(p_edge_index, model->flex_edgenum[p_flex_id], 0.0);
	int global_edge = model->flex_edgeadr[p_flex_id] + p_edge_index;
	return model->flexedge_length0[global_edge];
}
