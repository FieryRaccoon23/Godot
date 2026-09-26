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

     // Body lookup + transform access
    int get_body_id(const String &p_body_name) const;
    Vector3 get_body_pos(int p_body_id) const;
    Quaternion get_body_quat(int p_body_id) const;

    MuJoCoServer();
    ~MuJoCoServer();
};

#endif // MUJOCO_SERVER_H