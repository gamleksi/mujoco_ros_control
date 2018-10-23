/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 8/2/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <mujoco.h>
#include <mujoco_ros_control/RobotHWMujoco.h>
#include <controller_manager/controller_manager.h>

std::unique_ptr<RobotHWMujoco> hw;
std::unique_ptr<controller_manager::ControllerManager> cm;
std::unique_ptr<ros::Time> start_time;

void cb_controller(const mjModel *m, mjData *d) {
    hw->read(*d);
    const float duration = d->time;

    cm->update(*start_time + ros::Duration(duration), ros::Duration(m->opt.timestep));
    hw->write(*d);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mujoco_control");
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    const auto default_model_path = ros::package::getPath("mujoco_ros_control") + "/model/simple_robot.xml";
    const auto model_path = node.param("model", default_model_path);

    const auto key_path = std::string(getenv("HOME")) + "/.mujoco/mjkey.txt";

    if (!mj_activate(key_path.c_str())) {
        ROS_ERROR_STREAM("Cannot activate mujoco with key: " << key_path);
        return -1;
    }

    char error[1000] = "";
    auto m = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
    if (!m) {
        ROS_ERROR_STREAM("Cannot load model: " << model_path);
        ROS_ERROR_STREAM(error);
        return -1;
    }

    auto d = mj_makeData(m);
    if (!d) {
        ROS_ERROR_STREAM("Cannot make data structure for model.");
        return -1;
    }

    hw.reset(new RobotHWMujoco(*m));
    cm.reset(new controller_manager::ControllerManager(hw.get(), node));
    start_time.reset(new ros::Time());
    *start_time = ros::Time::now();

    mjcb_control = cb_controller;

    const auto timestep = ros::Duration(m->opt.timestep);
    const auto timer = node.createTimer(timestep, [&](const ros::TimerEvent &e) {
        mj_step(m, d);
    });

    ros::waitForShutdown();

    mj_deleteModel(m);
    mj_deleteData(d);
    mj_deactivate();

    return 0;
}
