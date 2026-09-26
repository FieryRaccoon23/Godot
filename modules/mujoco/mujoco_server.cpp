#include "mujoco_server.h"

void MuJoCoServer::_bind_methods() {
    ClassDB::bind_method(D_METHOD("load_model", "xml_path"), &MuJoCoServer::load_model);
    ClassDB::bind_method(D_METHOD("step"), &MuJoCoServer::step);
    ClassDB::bind_method(D_METHOD("reset"), &MuJoCoServer::reset);
    ClassDB::bind_method(D_METHOD("unload"), &MuJoCoServer::unload);

    ClassDB::bind_method(D_METHOD("get_nq"), &MuJoCoServer::get_nq);
    ClassDB::bind_method(D_METHOD("get_qpos", "index"), &MuJoCoServer::get_qpos);
    ClassDB::bind_method(D_METHOD("set_qpos", "index", "value"), &MuJoCoServer::set_qpos);

    ClassDB::bind_method(D_METHOD("get_nv"), &MuJoCoServer::get_nv);
    ClassDB::bind_method(D_METHOD("get_qvel", "index"), &MuJoCoServer::get_qvel);
    ClassDB::bind_method(D_METHOD("set_qvel", "index", "value"), &MuJoCoServer::set_qvel);

    ClassDB::bind_method(D_METHOD("get_qacc", "index"), &MuJoCoServer::get_qacc);

    ClassDB::bind_method(D_METHOD("get_time"), &MuJoCoServer::get_time);

    ClassDB::bind_method(D_METHOD("get_body_id", "body_name"), &MuJoCoServer::get_body_id);
    ClassDB::bind_method(D_METHOD("get_body_pos", "body_id"), &MuJoCoServer::get_body_pos);
    ClassDB::bind_method(D_METHOD("get_body_quat", "body_id"), &MuJoCoServer::get_body_quat);
}

MuJoCoServer::MuJoCoServer() {}
MuJoCoServer::~MuJoCoServer() { unload(); }

bool MuJoCoServer::load_model(const String &p_xml_path) {
    unload();
    char error[1000] = "";
    CharString path_utf8 = p_xml_path.utf8();
    model = mj_loadXML(path_utf8.get_data(), nullptr, error, 1000);
    if (!model) {
        ERR_PRINT(String("MuJoCo failed to load model: ") + String(error));
        return false;
    }
    data = mj_makeData(model);
    return true;
}

void MuJoCoServer::step() {
    if (model && data) {
        mj_step(model, data);
    }
}

void MuJoCoServer::reset() {
    if (model && data) {
        mj_resetData(model, data);
    }
}

void MuJoCoServer::unload() {
    if (data) { mj_deleteData(data); data = nullptr; }
    if (model) { mj_deleteModel(model); model = nullptr; }
}

int MuJoCoServer::get_nq() const {
    return model ? model->nq : 0;
}

float MuJoCoServer::get_qpos(int p_index) const {
    ERR_FAIL_NULL_V(data, 0.0f);
    ERR_FAIL_INDEX_V(p_index, model->nq, 0.0f);
    return (float)data->qpos[p_index];
}

void MuJoCoServer::set_qpos(int p_index, float p_value) {
    ERR_FAIL_NULL(data);
    ERR_FAIL_INDEX(p_index, model->nq);
    data->qpos[p_index] = p_value;
}

float MuJoCoServer::get_time() const {
    return data ? (float)data->time : 0.0f;
}

int MuJoCoServer::get_body_id(const String &p_body_name) const {
    ERR_FAIL_NULL_V(model, -1);
    CharString name_utf8 = p_body_name.utf8();
    int id = mj_name2id(model, mjOBJ_BODY, name_utf8.get_data());
    if (id < 0) {
        ERR_PRINT(String("MuJoCo: no body named '") + p_body_name + String("'"));
    }
    return id;
}

Vector3 MuJoCoServer::get_body_pos(int p_body_id) const {
    ERR_FAIL_NULL_V(data, Vector3());
    ERR_FAIL_INDEX_V(p_body_id, model->nbody, Vector3());

    // MuJoCo world positions are in xpos[3*body_id .. +2], MuJoCo uses X-forward,
    // Z-up by convention (same as Godot's Y-up requires a swap: MuJoCo Z -> Godot Y)
    double x = data->xpos[3 * p_body_id + 0];
    double y = data->xpos[3 * p_body_id + 1];
    double z = data->xpos[3 * p_body_id + 2];

    // Swap Y/Z to go from MuJoCo's Z-up convention to Godot's Y-up
    return Vector3((float)x, (float)z, (float)y);
}

Quaternion MuJoCoServer::get_body_quat(int p_body_id) const {
    ERR_FAIL_NULL_V(data, Quaternion());
    ERR_FAIL_INDEX_V(p_body_id, model->nbody, Quaternion());

    // MuJoCo stores quaternions as (w, x, y, z) in xquat
    double w = data->xquat[4 * p_body_id + 0];
    double x = data->xquat[4 * p_body_id + 1];
    double y = data->xquat[4 * p_body_id + 2];
    double z = data->xquat[4 * p_body_id + 3];

    // Same axis swap as position (Y<->Z), and Godot's Quaternion constructor
    // takes (x, y, z, w) order, not MuJoCo's (w, x, y, z)
    return Quaternion((float)x, (float)z, (float)y, (float)w);
}

int MuJoCoServer::get_nv() const {
    return model ? model->nv : 0;
}

float MuJoCoServer::get_qvel(int p_index) const {
    ERR_FAIL_NULL_V(data, 0.0f);
    ERR_FAIL_INDEX_V(p_index, model->nv, 0.0f);
    return (float)data->qvel[p_index];
}

void MuJoCoServer::set_qvel(int p_index, float p_value) {
    ERR_FAIL_NULL(data);
    ERR_FAIL_INDEX(p_index, model->nv);
    data->qvel[p_index] = p_value;
}

float MuJoCoServer::get_qacc(int p_index) const {
    ERR_FAIL_NULL_V(data, 0.0f);
    ERR_FAIL_INDEX_V(p_index, model->nv, 0.0f);
    return (float)data->qacc[p_index];
}