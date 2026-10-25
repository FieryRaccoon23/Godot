#ifndef MUJOCO_SERVER_H
#define MUJOCO_SERVER_H

#include "core/object/class_db.h"
#include "core/object/ref_counted.h"
#include "core/variant/dictionary.h"
#include "mujoco/mujoco.h"

// -----------------------------------------------------------------------------
// Coordinate conversion between MuJoCo (right-handed, Z-up) and Godot
// (right-handed, Y-up). This is a proper rotation of -90 degrees about X:
//
//   Godot (x, y, z) = MuJoCo (x, z, -y)
//   MuJoCo (x, y, z) = Godot (x, -z, y)
//
// Simply swapping Y and Z is a mirror, not a rotation: it flips handedness, so
// the world appears mirrored and joint rotations spin the wrong way.
//
// Note on local sizes: because geom frames are rotated too, a MuJoCo box with
// half-extents (sx, sy, sz) is (sx, sz, sy) in its Godot-local frame, and a
// MuJoCo capsule/cylinder (local Z axis) lies along the Godot-local Y axis,
// which matches Godot's CapsuleMesh/CylinderMesh.
// -----------------------------------------------------------------------------
// The bulk copy helpers assume MuJoCo's default double-precision build.
static_assert(sizeof(mjtNum) == sizeof(double), "MuJoCo must be built with double-precision mjtNum.");

namespace MuJoCoConvert {

inline Vector3 to_godot(const mjtNum *p_v) {
	return Vector3((real_t)p_v[0], (real_t)p_v[2], (real_t)-p_v[1]);
}

inline void to_mujoco(const Vector3 &p_v, mjtNum *r_out) {
	r_out[0] = p_v.x;
	r_out[1] = -p_v.z;
	r_out[2] = p_v.y;
}

// MuJoCo quaternions are (w, x, y, z); Godot's constructor is (x, y, z, w).
inline Quaternion quat_to_godot(const mjtNum *p_q) {
	return Quaternion((real_t)p_q[1], (real_t)p_q[3], (real_t)-p_q[2], (real_t)p_q[0]);
}

inline void quat_to_mujoco(const Quaternion &p_q, mjtNum *r_out) {
	r_out[0] = p_q.w;
	r_out[1] = p_q.x;
	r_out[2] = -p_q.z;
	r_out[3] = p_q.y;
}

} // namespace MuJoCoConvert

class MuJoCoServer : public RefCounted {
	GDCLASS(MuJoCoServer, RefCounted);

private:
	mjModel *model = nullptr;
	mjData *data = nullptr;
	String last_error;

	bool _finish_load(mjModel *p_model);

protected:
	static void _bind_methods();

public:
	// --- Loading / lifecycle (mujoco_core.cpp) ---
	// Accepts res://, user:// and absolute paths. Works in exported games too.
	bool load_model(const String &p_path);
	// Load an MJCF string built in code (e.g. a player-designed robot).
	bool load_model_from_string(const String &p_xml);
	String get_last_error() const;
	bool is_loaded() const;
	void unload();
	String get_version_string() const;

	// --- Stepping ---
	void step();
	void step_n(int p_steps);
	void forward(); // update positions/contacts without advancing time
	void reset();
	void reset_to_keyframe(int p_key_index);
	int get_nkey() const;
	double get_time() const;

	// --- Stability ---
	// True if MuJoCo detected bad accelerations (the sim "exploded") or the state
	// contains NaN/inf. MuJoCo auto-resets on bad acceleration, so check after
	// each step_n() and stop the run if this returns true.
	bool has_diverged() const;
	void clear_warnings();

	// --- Simulation options ---
	double get_timestep() const;
	void set_timestep(double p_timestep);
	int get_solver_iterations() const;
	void set_solver_iterations(int p_iterations);
	int get_integrator() const; // mjtIntegrator: 0=Euler 1=RK4 2=implicit 3=implicitfast
	void set_integrator(int p_integrator);
	Vector3 get_gravity() const; // Godot coordinates
	void set_gravity(const Vector3 &p_gravity);

	// --- Generalized coordinates (raw MuJoCo order and frame, not converted) ---
	int get_nq() const;
	double get_qpos(int p_index) const;
	void set_qpos(int p_index, double p_value);
	PackedFloat64Array get_qpos_array() const;
	void set_qpos_array(const PackedFloat64Array &p_qpos);

	int get_nv() const;
	double get_qvel(int p_index) const;
	void set_qvel(int p_index, double p_value);
	PackedFloat64Array get_qvel_array() const;
	void set_qvel_array(const PackedFloat64Array &p_qvel);

	double get_qacc(int p_index) const;

	// --- Full state snapshot (instant restart, rewind, replays) ---
	PackedFloat64Array get_state() const;
	bool set_state(const PackedFloat64Array &p_state);

	// --- Bodies (mujoco_core.cpp) ---
	int get_nbody() const;
	int get_body_id(const String &p_body_name) const;
	String get_body_name(int p_body_id) const;
	int get_body_parent_id(int p_body_id) const;
	double get_body_mass(int p_body_id) const;
	Vector3 get_body_pos(int p_body_id) const;
	Quaternion get_body_quat(int p_body_id) const;
	Transform3D get_body_transform(int p_body_id) const;
	Vector3 get_body_linear_velocity(int p_body_id) const;
	Vector3 get_body_angular_velocity(int p_body_id) const;
	// All bodies in one call: 7 floats per body [px, py, pz, qx, qy, qz, qw],
	// already in Godot coordinates. Index i*7 is body i (body 0 is the world).
	PackedFloat32Array get_body_transforms_packed() const;

	// --- Geoms + contacts (mujoco_collision.cpp) ---
	int get_ngeom() const;
	int get_geom_id(const String &p_geom_name) const;
	String get_geom_name(int p_geom_id) const;
	int get_geom_type(int p_geom_id) const;
	int get_geom_body_id(int p_geom_id) const;
	Vector3 get_geom_size(int p_geom_id) const; // raw MuJoCo size, see note at top
	Vector3 get_geom_friction(int p_geom_id) const;
	Color get_geom_rgba(int p_geom_id) const;
	Vector3 get_geom_pos(int p_geom_id) const;
	Quaternion get_geom_quat(int p_geom_id) const;
	// 7 floats per geom, same layout as get_body_transforms_packed().
	PackedFloat32Array get_geom_transforms_packed() const;

	int get_contact_count() const;
	Vector3 get_contact_pos(int p_contact_index) const;
	Vector3 get_contact_normal(int p_contact_index) const;
	double get_contact_depth(int p_contact_index) const;
	int get_contact_geom1(int p_contact_index) const;
	int get_contact_geom2(int p_contact_index) const;
	double get_contact_normal_force(int p_contact_index) const;
	Vector3 get_contact_force(int p_contact_index) const; // world-space, Godot coords

	// Returns {} on miss, else {"geom_id", "body_id", "distance", "position"}.
	// p_exclude_body_id: body to ignore (e.g. the ray's own robot), -1 for none.
	Dictionary raycast(const Vector3 &p_origin, const Vector3 &p_direction, double p_max_distance, bool p_include_static, int p_exclude_body_id) const;

	// --- Actuators, joints, equality, sensors, forces (mujoco_control.cpp) ---
	int get_nu() const;
	int get_actuator_id(const String &p_name) const;
	String get_actuator_name(int p_actuator_id) const;
	void set_ctrl(int p_actuator_id, double p_value);
	double get_ctrl(int p_actuator_id) const;
	void set_ctrl_array(const PackedFloat64Array &p_ctrl);
	Vector2 get_ctrl_range(int p_actuator_id) const;
	double get_actuator_force(int p_actuator_id) const;

	int get_njnt() const;
	int get_joint_id(const String &p_name) const;
	String get_joint_name(int p_joint_id) const;
	int get_joint_type(int p_joint_id) const; // mjtJoint: 0=free 1=ball 2=slide 3=hinge
	int get_joint_body_id(int p_joint_id) const;
	int get_joint_qpos_addr(int p_joint_id) const;
	int get_joint_dof_addr(int p_joint_id) const;
	Vector2 get_joint_range(int p_joint_id) const;
	// Convenience for hinge/slide joints (angle in radians or slide distance).
	double get_joint_value(int p_joint_id) const;
	double get_joint_velocity(int p_joint_id) const;

	int get_neq() const;
	int get_equality_id(const String &p_name) const;
	void set_equality_active(int p_eq_id, bool p_active);
	bool is_equality_active(int p_eq_id) const;
	// Weld/connect constraints store a target pose computed at load time, so
	// activating one mid-run yanks the bodies to that pose. Call this right
	// before set_equality_active(id, true) to "grab" things where they are now.
	bool snap_equality_to_current_pose(int p_eq_id);

	int get_nsensor() const;
	int get_sensor_id(const String &p_name) const;
	int get_sensor_dim(int p_sensor_id) const;
	PackedFloat64Array get_sensor_data(int p_sensor_id) const;

	// Persistent until changed or cleared. Godot coordinates, applied at body CoM.
	void apply_force(int p_body_id, const Vector3 &p_force, const Vector3 &p_torque);
	void clear_applied_forces();

	// --- Flex (mujoco_flex.cpp) ---
	int get_flex_id(const String &p_flex_name) const;
	int get_flex_vertex_count(int p_flex_id) const;
	Vector3 get_flex_vertex_pos(int p_flex_id, int p_vertex_index) const;
	PackedVector3Array get_flex_vertex_positions(int p_flex_id) const;
	double get_flex_radius(int p_flex_id) const;
	Vector2i get_flex_edge(int p_flex_id, int p_edge_index) const;
	int get_flex_edge_count(int p_flex_id) const;
	double get_flex_edge_length(int p_flex_id, int p_edge_index) const;
	double get_flex_edge_rest_length(int p_flex_id, int p_edge_index) const;

	MuJoCoServer();
	~MuJoCoServer();
};

#endif // MUJOCO_SERVER_H
