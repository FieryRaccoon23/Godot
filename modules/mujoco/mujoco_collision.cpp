#include "mujoco_server.h"

// Bindings for collision methods are registered in mujoco_server.cpp's
// _bind_methods(), since GDCLASS requires a single _bind_methods per class —
// see the ClassDB::bind_method calls added there for these four methods.

int MuJoCoServer::get_ngeom() const {
    return model ? model->ngeom : 0;
}

int MuJoCoServer::get_geom_id(const String &p_geom_name) const {
    ERR_FAIL_NULL_V(model, -1);
    CharString name_utf8 = p_geom_name.utf8();
    int id = mj_name2id(model, mjOBJ_GEOM, name_utf8.get_data());
    if (id < 0) {
        ERR_PRINT(String("MuJoCo: no geom named '") + p_geom_name + String("'"));
    }
    return id;
}

int MuJoCoServer::get_geom_type(int p_geom_id) const {
    ERR_FAIL_NULL_V(model, -1);
    ERR_FAIL_INDEX_V(p_geom_id, model->ngeom, -1);
    return model->geom_type[p_geom_id]; // see mjtGeom: 0=plane 2=sphere 3=capsule 6=box etc.
}

int MuJoCoServer::get_geom_body_id(int p_geom_id) const {
    ERR_FAIL_NULL_V(model, -1);
    ERR_FAIL_INDEX_V(p_geom_id, model->ngeom, -1);
    return model->geom_bodyid[p_geom_id];
}

Vector3 MuJoCoServer::get_geom_size(int p_geom_id) const {
    ERR_FAIL_NULL_V(model, Vector3());
    ERR_FAIL_INDEX_V(p_geom_id, model->ngeom, Vector3());
    // meaning of xyz depends on geom_type: sphere uses only x (radius);
    // capsule/cylinder use x=radius, y=half-length; box uses x/y/z half-extents
    return Vector3(
        (float)model->geom_size[3 * p_geom_id + 0],
        (float)model->geom_size[3 * p_geom_id + 1],
        (float)model->geom_size[3 * p_geom_id + 2]);
}

Vector3 MuJoCoServer::get_geom_friction(int p_geom_id) const {
    ERR_FAIL_NULL_V(model, Vector3());
    ERR_FAIL_INDEX_V(p_geom_id, model->ngeom, Vector3());
    // x = sliding friction, y = torsional friction, z = rolling friction
    return Vector3(
        (float)model->geom_friction[3 * p_geom_id + 0],
        (float)model->geom_friction[3 * p_geom_id + 1],
        (float)model->geom_friction[3 * p_geom_id + 2]);
}

// --- Dynamic per-step contact data ---

int MuJoCoServer::get_contact_count() const {
    return data ? data->ncon : 0;
}

Vector3 MuJoCoServer::get_contact_pos(int p_contact_index) const {
    ERR_FAIL_NULL_V(data, Vector3());
    ERR_FAIL_INDEX_V(p_contact_index, data->ncon, Vector3());
    const mjContact &c = data->contact[p_contact_index];
    return Vector3((float)c.pos[0], (float)c.pos[2], (float)c.pos[1]); // Z-up -> Y-up swap
}

Vector3 MuJoCoServer::get_contact_normal(int p_contact_index) const {
    ERR_FAIL_NULL_V(data, Vector3());
    ERR_FAIL_INDEX_V(p_contact_index, data->ncon, Vector3());
    const mjContact &c = data->contact[p_contact_index];
    // frame[0..2] is the contact normal (first row of the contact frame)
    return Vector3((float)c.frame[0], (float)c.frame[2], (float)c.frame[1]);
}

float MuJoCoServer::get_contact_depth(int p_contact_index) const {
    ERR_FAIL_NULL_V(data, 0.0f);
    ERR_FAIL_INDEX_V(p_contact_index, data->ncon, 0.0f);
    // negative = penetrating, positive = separated (rarely seen since contacts
    // are only generated near/at touching distance)
    return (float)data->contact[p_contact_index].dist;
}

int MuJoCoServer::get_contact_geom1(int p_contact_index) const {
    ERR_FAIL_NULL_V(data, -1);
    ERR_FAIL_INDEX_V(p_contact_index, data->ncon, -1);
    return data->contact[p_contact_index].geom[0];
}

int MuJoCoServer::get_contact_geom2(int p_contact_index) const {
    ERR_FAIL_NULL_V(data, -1);
    ERR_FAIL_INDEX_V(p_contact_index, data->ncon, -1);
    return data->contact[p_contact_index].geom[1];
}