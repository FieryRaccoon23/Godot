#include "mujoco_server.h"

// All bindings live here because a GDCLASS has exactly one _bind_methods().
// Implementations are split across mujoco_core.cpp, mujoco_collision.cpp,
// mujoco_control.cpp and mujoco_flex.cpp.

void MuJoCoServer::_bind_methods() {
	// --- Loading / lifecycle ---
	ClassDB::bind_method(D_METHOD("load_model", "path"), &MuJoCoServer::load_model);
	ClassDB::bind_method(D_METHOD("load_model_from_string", "xml"), &MuJoCoServer::load_model_from_string);
	ClassDB::bind_method(D_METHOD("get_last_error"), &MuJoCoServer::get_last_error);
	ClassDB::bind_method(D_METHOD("is_loaded"), &MuJoCoServer::is_loaded);
	ClassDB::bind_method(D_METHOD("unload"), &MuJoCoServer::unload);
	ClassDB::bind_method(D_METHOD("get_version_string"), &MuJoCoServer::get_version_string);

	// --- Stepping ---
	ClassDB::bind_method(D_METHOD("step"), &MuJoCoServer::step);
	ClassDB::bind_method(D_METHOD("step_n", "steps"), &MuJoCoServer::step_n);
	ClassDB::bind_method(D_METHOD("forward"), &MuJoCoServer::forward);
	ClassDB::bind_method(D_METHOD("reset"), &MuJoCoServer::reset);
	ClassDB::bind_method(D_METHOD("reset_to_keyframe", "key_index"), &MuJoCoServer::reset_to_keyframe);
	ClassDB::bind_method(D_METHOD("get_nkey"), &MuJoCoServer::get_nkey);
	ClassDB::bind_method(D_METHOD("get_time"), &MuJoCoServer::get_time);

	// --- Stability ---
	ClassDB::bind_method(D_METHOD("has_diverged"), &MuJoCoServer::has_diverged);
	ClassDB::bind_method(D_METHOD("clear_warnings"), &MuJoCoServer::clear_warnings);

	// --- Simulation options ---
	ClassDB::bind_method(D_METHOD("get_timestep"), &MuJoCoServer::get_timestep);
	ClassDB::bind_method(D_METHOD("set_timestep", "timestep"), &MuJoCoServer::set_timestep);
	ClassDB::bind_method(D_METHOD("get_solver_iterations"), &MuJoCoServer::get_solver_iterations);
	ClassDB::bind_method(D_METHOD("set_solver_iterations", "iterations"), &MuJoCoServer::set_solver_iterations);
	ClassDB::bind_method(D_METHOD("get_integrator"), &MuJoCoServer::get_integrator);
	ClassDB::bind_method(D_METHOD("set_integrator", "integrator"), &MuJoCoServer::set_integrator);
	ClassDB::bind_method(D_METHOD("get_gravity"), &MuJoCoServer::get_gravity);
	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &MuJoCoServer::set_gravity);

	// --- Generalized coordinates ---
	ClassDB::bind_method(D_METHOD("get_nq"), &MuJoCoServer::get_nq);
	ClassDB::bind_method(D_METHOD("get_qpos", "index"), &MuJoCoServer::get_qpos);
	ClassDB::bind_method(D_METHOD("set_qpos", "index", "value"), &MuJoCoServer::set_qpos);
	ClassDB::bind_method(D_METHOD("get_qpos_array"), &MuJoCoServer::get_qpos_array);
	ClassDB::bind_method(D_METHOD("set_qpos_array", "qpos"), &MuJoCoServer::set_qpos_array);
	ClassDB::bind_method(D_METHOD("get_nv"), &MuJoCoServer::get_nv);
	ClassDB::bind_method(D_METHOD("get_qvel", "index"), &MuJoCoServer::get_qvel);
	ClassDB::bind_method(D_METHOD("set_qvel", "index", "value"), &MuJoCoServer::set_qvel);
	ClassDB::bind_method(D_METHOD("get_qvel_array"), &MuJoCoServer::get_qvel_array);
	ClassDB::bind_method(D_METHOD("set_qvel_array", "qvel"), &MuJoCoServer::set_qvel_array);
	ClassDB::bind_method(D_METHOD("get_qacc", "index"), &MuJoCoServer::get_qacc);

	// --- State snapshot ---
	ClassDB::bind_method(D_METHOD("get_state"), &MuJoCoServer::get_state);
	ClassDB::bind_method(D_METHOD("set_state", "state"), &MuJoCoServer::set_state);

	// --- Bodies ---
	ClassDB::bind_method(D_METHOD("get_nbody"), &MuJoCoServer::get_nbody);
	ClassDB::bind_method(D_METHOD("get_body_id", "body_name"), &MuJoCoServer::get_body_id);
	ClassDB::bind_method(D_METHOD("get_body_name", "body_id"), &MuJoCoServer::get_body_name);
	ClassDB::bind_method(D_METHOD("get_body_parent_id", "body_id"), &MuJoCoServer::get_body_parent_id);
	ClassDB::bind_method(D_METHOD("get_body_mass", "body_id"), &MuJoCoServer::get_body_mass);
	ClassDB::bind_method(D_METHOD("get_body_pos", "body_id"), &MuJoCoServer::get_body_pos);
	ClassDB::bind_method(D_METHOD("get_body_quat", "body_id"), &MuJoCoServer::get_body_quat);
	ClassDB::bind_method(D_METHOD("get_body_transform", "body_id"), &MuJoCoServer::get_body_transform);
	ClassDB::bind_method(D_METHOD("get_body_linear_velocity", "body_id"), &MuJoCoServer::get_body_linear_velocity);
	ClassDB::bind_method(D_METHOD("get_body_angular_velocity", "body_id"), &MuJoCoServer::get_body_angular_velocity);
	ClassDB::bind_method(D_METHOD("get_body_transforms_packed"), &MuJoCoServer::get_body_transforms_packed);

	// --- Geoms ---
	ClassDB::bind_method(D_METHOD("get_ngeom"), &MuJoCoServer::get_ngeom);
	ClassDB::bind_method(D_METHOD("get_geom_id", "geom_name"), &MuJoCoServer::get_geom_id);
	ClassDB::bind_method(D_METHOD("get_geom_name", "geom_id"), &MuJoCoServer::get_geom_name);
	ClassDB::bind_method(D_METHOD("get_geom_type", "geom_id"), &MuJoCoServer::get_geom_type);
	ClassDB::bind_method(D_METHOD("get_geom_body_id", "geom_id"), &MuJoCoServer::get_geom_body_id);
	ClassDB::bind_method(D_METHOD("get_geom_size", "geom_id"), &MuJoCoServer::get_geom_size);
	ClassDB::bind_method(D_METHOD("get_geom_friction", "geom_id"), &MuJoCoServer::get_geom_friction);
	ClassDB::bind_method(D_METHOD("get_geom_rgba", "geom_id"), &MuJoCoServer::get_geom_rgba);
	ClassDB::bind_method(D_METHOD("get_geom_pos", "geom_id"), &MuJoCoServer::get_geom_pos);
	ClassDB::bind_method(D_METHOD("get_geom_quat", "geom_id"), &MuJoCoServer::get_geom_quat);
	ClassDB::bind_method(D_METHOD("get_geom_transforms_packed"), &MuJoCoServer::get_geom_transforms_packed);

	// --- Contacts ---
	ClassDB::bind_method(D_METHOD("get_contact_count"), &MuJoCoServer::get_contact_count);
	ClassDB::bind_method(D_METHOD("get_contact_pos", "contact_index"), &MuJoCoServer::get_contact_pos);
	ClassDB::bind_method(D_METHOD("get_contact_normal", "contact_index"), &MuJoCoServer::get_contact_normal);
	ClassDB::bind_method(D_METHOD("get_contact_depth", "contact_index"), &MuJoCoServer::get_contact_depth);
	ClassDB::bind_method(D_METHOD("get_contact_geom1", "contact_index"), &MuJoCoServer::get_contact_geom1);
	ClassDB::bind_method(D_METHOD("get_contact_geom2", "contact_index"), &MuJoCoServer::get_contact_geom2);
	ClassDB::bind_method(D_METHOD("get_contact_normal_force", "contact_index"), &MuJoCoServer::get_contact_normal_force);
	ClassDB::bind_method(D_METHOD("get_contact_force", "contact_index"), &MuJoCoServer::get_contact_force);

	ClassDB::bind_method(D_METHOD("raycast", "origin", "direction", "max_distance", "include_static", "exclude_body_id"),
			&MuJoCoServer::raycast, DEFVAL(1000.0), DEFVAL(true), DEFVAL(-1));

	// --- Actuators ---
	ClassDB::bind_method(D_METHOD("get_nu"), &MuJoCoServer::get_nu);
	ClassDB::bind_method(D_METHOD("get_actuator_id", "name"), &MuJoCoServer::get_actuator_id);
	ClassDB::bind_method(D_METHOD("get_actuator_name", "actuator_id"), &MuJoCoServer::get_actuator_name);
	ClassDB::bind_method(D_METHOD("set_ctrl", "actuator_id", "value"), &MuJoCoServer::set_ctrl);
	ClassDB::bind_method(D_METHOD("get_ctrl", "actuator_id"), &MuJoCoServer::get_ctrl);
	ClassDB::bind_method(D_METHOD("set_ctrl_array", "ctrl"), &MuJoCoServer::set_ctrl_array);
	ClassDB::bind_method(D_METHOD("get_ctrl_range", "actuator_id"), &MuJoCoServer::get_ctrl_range);
	ClassDB::bind_method(D_METHOD("get_actuator_force", "actuator_id"), &MuJoCoServer::get_actuator_force);

	// --- Joints ---
	ClassDB::bind_method(D_METHOD("get_njnt"), &MuJoCoServer::get_njnt);
	ClassDB::bind_method(D_METHOD("get_joint_id", "name"), &MuJoCoServer::get_joint_id);
	ClassDB::bind_method(D_METHOD("get_joint_name", "joint_id"), &MuJoCoServer::get_joint_name);
	ClassDB::bind_method(D_METHOD("get_joint_type", "joint_id"), &MuJoCoServer::get_joint_type);
	ClassDB::bind_method(D_METHOD("get_joint_body_id", "joint_id"), &MuJoCoServer::get_joint_body_id);
	ClassDB::bind_method(D_METHOD("get_joint_qpos_addr", "joint_id"), &MuJoCoServer::get_joint_qpos_addr);
	ClassDB::bind_method(D_METHOD("get_joint_dof_addr", "joint_id"), &MuJoCoServer::get_joint_dof_addr);
	ClassDB::bind_method(D_METHOD("get_joint_range", "joint_id"), &MuJoCoServer::get_joint_range);
	ClassDB::bind_method(D_METHOD("get_joint_value", "joint_id"), &MuJoCoServer::get_joint_value);
	ClassDB::bind_method(D_METHOD("get_joint_velocity", "joint_id"), &MuJoCoServer::get_joint_velocity);

	// --- Equality constraints ---
	ClassDB::bind_method(D_METHOD("get_neq"), &MuJoCoServer::get_neq);
	ClassDB::bind_method(D_METHOD("get_equality_id", "name"), &MuJoCoServer::get_equality_id);
	ClassDB::bind_method(D_METHOD("set_equality_active", "eq_id", "active"), &MuJoCoServer::set_equality_active);
	ClassDB::bind_method(D_METHOD("is_equality_active", "eq_id"), &MuJoCoServer::is_equality_active);
	ClassDB::bind_method(D_METHOD("snap_equality_to_current_pose", "eq_id"), &MuJoCoServer::snap_equality_to_current_pose);

	// --- Sensors ---
	ClassDB::bind_method(D_METHOD("get_nsensor"), &MuJoCoServer::get_nsensor);
	ClassDB::bind_method(D_METHOD("get_sensor_id", "name"), &MuJoCoServer::get_sensor_id);
	ClassDB::bind_method(D_METHOD("get_sensor_dim", "sensor_id"), &MuJoCoServer::get_sensor_dim);
	ClassDB::bind_method(D_METHOD("get_sensor_data", "sensor_id"), &MuJoCoServer::get_sensor_data);

	// --- Applied forces ---
	ClassDB::bind_method(D_METHOD("apply_force", "body_id", "force", "torque"), &MuJoCoServer::apply_force, DEFVAL(Vector3()));
	ClassDB::bind_method(D_METHOD("clear_applied_forces"), &MuJoCoServer::clear_applied_forces);

	// --- Flex ---
	ClassDB::bind_method(D_METHOD("get_flex_id", "flex_name"), &MuJoCoServer::get_flex_id);
	ClassDB::bind_method(D_METHOD("get_flex_vertex_count", "flex_id"), &MuJoCoServer::get_flex_vertex_count);
	ClassDB::bind_method(D_METHOD("get_flex_vertex_pos", "flex_id", "vertex_index"), &MuJoCoServer::get_flex_vertex_pos);
	ClassDB::bind_method(D_METHOD("get_flex_vertex_positions", "flex_id"), &MuJoCoServer::get_flex_vertex_positions);
	ClassDB::bind_method(D_METHOD("get_flex_radius", "flex_id"), &MuJoCoServer::get_flex_radius);
	ClassDB::bind_method(D_METHOD("get_flex_edge_count", "flex_id"), &MuJoCoServer::get_flex_edge_count);
	ClassDB::bind_method(D_METHOD("get_flex_edge", "flex_id", "edge_index"), &MuJoCoServer::get_flex_edge);
	ClassDB::bind_method(D_METHOD("get_flex_edge_length", "flex_id", "edge_index"), &MuJoCoServer::get_flex_edge_length);
	ClassDB::bind_method(D_METHOD("get_flex_edge_rest_length", "flex_id", "edge_index"), &MuJoCoServer::get_flex_edge_rest_length);
}

MuJoCoServer::MuJoCoServer() {}

MuJoCoServer::~MuJoCoServer() {
	unload();
}
