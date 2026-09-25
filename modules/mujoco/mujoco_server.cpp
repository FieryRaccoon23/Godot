#include "mujoco_server.h"

void MuJoCoServer::_bind_methods() {
    ClassDB::bind_method(D_METHOD("load_model", "xml_path"), &MuJoCoServer::load_model);
    ClassDB::bind_method(D_METHOD("step"), &MuJoCoServer::step);
    ClassDB::bind_method(D_METHOD("reset"), &MuJoCoServer::reset);
    ClassDB::bind_method(D_METHOD("unload"), &MuJoCoServer::unload);

    ClassDB::bind_method(D_METHOD("get_nq"), &MuJoCoServer::get_nq);
    ClassDB::bind_method(D_METHOD("get_qpos", "index"), &MuJoCoServer::get_qpos);
    ClassDB::bind_method(D_METHOD("set_qpos", "index", "value"), &MuJoCoServer::set_qpos);

    ClassDB::bind_method(D_METHOD("get_time"), &MuJoCoServer::get_time);
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