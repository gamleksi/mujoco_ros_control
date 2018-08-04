/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 8/2/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 *
 *     Register Joint State interface and Joint Effort interface for every joint in MuJoCo model.
 *
 */

#ifndef MUJOCO_ROS_CONTROL_ROBOTHWMUJOCO_H
#define MUJOCO_ROS_CONTROL_ROBOTHWMUJOCO_H

#include <mujoco.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

class RobotHWMujoco : public hardware_interface::RobotHW {
public:

    /** @brief Register interfaces for each joint in a MuJoCo model. */
    explicit RobotHWMujoco(const mjModel &m);

    /** @brief From mjdata extract position,velocity and acceleration into the internal state */
    void read(const mjData &d);

    /** @brief Write to mjData computed command */
    void write(mjData &d);

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::EffortJointInterface jnt_eff_interface;

    std::vector<double> cmd, pos, vel, eff;
    /** @brief Defines position in the array of MuJoCo data, qadr for qpos array and vadr for qvel, qacc, and xfrc_applied */
    std::vector<size_t> qadr, vadr;
};

#endif //MUJOCO_ROS_CONTROL_ROBOTHWMUJOCO_H
