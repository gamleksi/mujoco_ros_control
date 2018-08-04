#include <mujoco_ros_control/RobotHWMujoco.h>

RobotHWMujoco::RobotHWMujoco(const mjModel &m) {
    assert(m.njnt >= 0);
    const auto n = (size_t) m.njnt;
    cmd.resize(n, 0.0);
    pos.resize(n, 0.0);
    vel.resize(n, 0.0);
    eff.resize(n, 0.0);
    qadr.resize(n, 0);
    vadr.resize(n, 0);

    for (size_t i = 0; i < n; ++i) {
        const auto joint_type = m.jnt_type[i];
        if (joint_type == mjJNT_FREE || joint_type == mjJNT_BALL) {
            continue;
        }

        const auto joint_name = mj_id2name(&m, mjOBJ_JOINT, i);
        qadr[i] = (size_t) m.jnt_qposadr[i];
        vadr[i] = (size_t) m.jnt_dofadr[i];

        hardware_interface::JointStateHandle state_handle_a(joint_name, &pos[i], &vel[i], &eff[i]);
        jnt_state_interface.registerHandle(state_handle_a);

        hardware_interface::JointHandle eff_handle_a(jnt_state_interface.getHandle(joint_name), &cmd[i]);
        jnt_eff_interface.registerHandle(eff_handle_a);
    }

    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_eff_interface);
}

void RobotHWMujoco::read(const mjData &d) {
    for (size_t i = 0; i < qadr.size(); ++i) {
        pos[i] = d.qpos[qadr[i]];
        vel[i] = d.qvel[vadr[i]];
        eff[i] = d.qfrc_applied[vadr[i]];
    }
}

void RobotHWMujoco::write(mjData &d) {
    for (size_t i = 0; i < vadr.size(); ++i) {
        d.qfrc_applied[vadr[i]] = cmd[i];
    }
}
