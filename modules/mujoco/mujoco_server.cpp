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

    ClassDB::bind_method(D_METHOD("get_flex_id", "flex_name"), &MuJoCoServer::get_flex_id);
    ClassDB::bind_method(D_METHOD("get_flex_vertex_count", "flex_id"), &MuJoCoServer::get_flex_vertex_count);
    ClassDB::bind_method(D_METHOD("get_flex_vertex_pos", "flex_id", "vertex_index"), &MuJoCoServer::get_flex_vertex_pos);
    ClassDB::bind_method(D_METHOD("get_flex_radius", "flex_id"), &MuJoCoServer::get_flex_radius);
    ClassDB::bind_method(D_METHOD("get_flex_edge_count", "flex_id"), &MuJoCoServer::get_flex_edge_count);
    ClassDB::bind_method(D_METHOD("get_flex_edge", "flex_id", "edge_index"), &MuJoCoServer::get_flex_edge);
    ClassDB::bind_method(D_METHOD("get_flex_edge_length", "flex_id", "edge_index"), &MuJoCoServer::get_flex_edge_length);
    ClassDB::bind_method(D_METHOD("get_flex_edge_rest_length", "flex_id", "edge_index"), &MuJoCoServer::get_flex_edge_rest_length);
}

MuJoCoServer::MuJoCoServer() {}
MuJoCoServer::~MuJoCoServer() { unload(); }