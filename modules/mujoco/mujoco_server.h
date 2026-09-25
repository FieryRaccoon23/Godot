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

    float get_time() const;

    MuJoCoServer();
    ~MuJoCoServer();
};

#endif // MUJOCO_SERVER_H