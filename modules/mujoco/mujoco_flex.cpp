#include "mujoco_server.h"

// Bindings for flex/cable (rope) methods are registered in mujoco_server.cpp's
// _bind_methods(), since GDCLASS requires a single _bind_methods per class —
// see the ClassDB::bind_method calls added there for these four methods.

int MuJoCoServer::get_flex_id(const String &p_flex_name) const {
    ERR_FAIL_NULL_V(model, -1);
    CharString name_utf8 = p_flex_name.utf8();
    int id = mj_name2id(model, mjOBJ_FLEX, name_utf8.get_data());
    if (id < 0) {
        ERR_PRINT(String("MuJoCo: no flex named '") + p_flex_name + String("'"));
    }
    return id;
}

int MuJoCoServer::get_flex_vertex_count(int p_flex_id) const {
    ERR_FAIL_NULL_V(model, 0);
    ERR_FAIL_INDEX_V(p_flex_id, model->nflex, 0);
    return model->flex_vertnum[p_flex_id];
}

Vector3 MuJoCoServer::get_flex_vertex_pos(int p_flex_id, int p_vertex_index) const {
    ERR_FAIL_NULL_V(data, Vector3());
    ERR_FAIL_INDEX_V(p_flex_id, model->nflex, Vector3());
    ERR_FAIL_INDEX_V(p_vertex_index, model->flex_vertnum[p_flex_id], Vector3());

    int global_vert = model->flex_vertadr[p_flex_id] + p_vertex_index;

    double x = data->flexvert_xpos[3 * global_vert + 0];
    double y = data->flexvert_xpos[3 * global_vert + 1];
    double z = data->flexvert_xpos[3 * global_vert + 2];

    // Same Y/Z swap as get_body_pos, for MuJoCo Z-up -> Godot Y-up
    return Vector3((float)x, (float)z, (float)y);
}

float MuJoCoServer::get_flex_radius(int p_flex_id) const {
    ERR_FAIL_NULL_V(model, 0.0f);
    ERR_FAIL_INDEX_V(p_flex_id, model->nflex, 0.0f);
    return (float)model->flex_radius[p_flex_id];
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
    int v0 = model->flex_edge[base + 0]; // local vertex index within this flex
    int v1 = model->flex_edge[base + 1];

    return Vector2i(v0, v1);
}

float MuJoCoServer::get_flex_edge_length(int p_flex_id, int p_edge_index) const {
    ERR_FAIL_NULL_V(data, 0.0f);
    ERR_FAIL_INDEX_V(p_flex_id, model->nflex, 0.0f);
    ERR_FAIL_INDEX_V(p_edge_index, model->flex_edgenum[p_flex_id], 0.0f);

    int global_edge = model->flex_edgeadr[p_flex_id] + p_edge_index;
    return (float)data->flexedge_length[global_edge];
}

float MuJoCoServer::get_flex_edge_rest_length(int p_flex_id, int p_edge_index) const {
    ERR_FAIL_NULL_V(model, 0.0f);
    ERR_FAIL_INDEX_V(p_flex_id, model->nflex, 0.0f);
    ERR_FAIL_INDEX_V(p_edge_index, model->flex_edgenum[p_flex_id], 0.0f);

    int global_edge = model->flex_edgeadr[p_flex_id] + p_edge_index;
    return (float)model->flexedge_length0[global_edge];
}