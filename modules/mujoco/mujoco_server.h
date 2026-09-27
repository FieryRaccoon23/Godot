#ifndef MUJOCO_SERVER_H
#define MUJOCO_SERVER_H

#include "core/object/class_db.h"
#include "core/object/ref_counted.h"
#include "mujoco/mujoco.h"

class MuJoCoServer : public RefCounted {
    GDCLASS(MuJoCoServer, RefCounted);

private:
    mjModel *model = nullptr;
    mjData *data = nullptr;

protected:
    static void _bind_methods();

public:
    bool load_model(const String &p_xml_path);
    void step();
    void reset();
    void unload();

    int get_nq() const;   // number of position coords
    float get_qpos(int p_index) const;
    void set_qpos(int p_index, float p_value);

    int get_nv() const;   // number of velocity DOFs
    float get_qvel(int p_index) const;
    void set_qvel(int p_index, float p_value);

    float get_qacc(int p_index) const;  // read-only: computed by the solver each step

    float get_time() const;

    // Geom shape + friction (static)
    int get_ngeom() const;
    int get_geom_id(const String &p_geom_name) const;
    int get_geom_type(int p_geom_id) const;
    int get_geom_body_id(int p_geom_id) const;
    Vector3 get_geom_size(int p_geom_id) const;
    Vector3 get_geom_friction(int p_geom_id) const;

    // Contacts (dynamic, per-step)
    int get_contact_count() const;
    Vector3 get_contact_pos(int p_contact_index) const;
    Vector3 get_contact_normal(int p_contact_index) const;
    float get_contact_depth(int p_contact_index) const;
    int get_contact_geom1(int p_contact_index) const;
    int get_contact_geom2(int p_contact_index) const;

     // Body lookup + transform access
    int get_body_id(const String &p_body_name) const;
    Vector3 get_body_pos(int p_body_id) const;
    Quaternion get_body_quat(int p_body_id) const;

    // Flex/cable (rope) access
    int get_flex_id(const String &p_flex_name) const;
    int get_flex_vertex_count(int p_flex_id) const;
    Vector3 get_flex_vertex_pos(int p_flex_id, int p_vertex_index) const;
    float get_flex_radius(int p_flex_id) const;
    Vector2i get_flex_edge(int p_flex_id, int p_edge_index) const;
    int get_flex_edge_count(int p_flex_id) const;
    float get_flex_edge_length(int p_flex_id, int p_edge_index) const;
    float get_flex_edge_rest_length(int p_flex_id, int p_edge_index) const;

    MuJoCoServer();
    ~MuJoCoServer();
};

#endif // MUJOCO_SERVER_H