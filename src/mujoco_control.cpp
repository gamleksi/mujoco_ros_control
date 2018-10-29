/**
 * Copyright (c) Aalto  - All Rights Reserved
 * Created on: 8/2/18
 *     Author: Vladimir Petrik <vladimir.petrik@aalto.fi>
 */

#include "stdio.h"
#include <mutex>
#include <GLFW/glfw3.h>
#include <mujoco.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

#include <mujoco_ros_control/RobotHWMujoco.h>
#include <controller_manager/controller_manager.h>

std::unique_ptr<RobotHWMujoco> hw;
std::unique_ptr<controller_manager::ControllerManager> cm;
std::unique_ptr<ros::Time> start_time;


// MuJoCo model and data
mjModel *m = 0;
mjData *d = 0;

// MuJoCo visualization
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

void cb_controller(const mjModel *m, mjData *d) {
    hw->read(*d);
    const float duration = d->time;

    cm->update(*start_time + ros::Duration(duration), ros::Duration(m->opt.timestep));
    hw->write(*d);
}

void initVisual() {
    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, 200);

    // center and scale view
    cam.lookat[0] = m->stat.center[0];
    cam.lookat[1] = m->stat.center[1];
    cam.lookat[2] = m->stat.center[2];
    cam.distance = 1.5 * m->stat.extent;
}

void initGl() {

    if (!glfwInit())
        mju_error("Could not initialize GLFW");

    // create invisible window, single-buffered
    glfwWindowHint(GLFW_VISIBLE, 0);
    glfwWindowHint(GLFW_DOUBLEBUFFER, 0); // GLFW_FALSE);
    GLFWwindow *window = glfwCreateWindow(800, 800, "Invisible window", NULL, NULL);
    if (!window)
        mju_error("Could not create GLFW window");
    // make context current
    glfwMakeContextCurrent(window);
}


void render(mjrRect &viewport) {
    std::cout << "Render" << std::endl;
    // update abstract scene
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    std::cout << "Render1" << std::endl;
    // render scene in offscreen buffer
    mjr_render(viewport, &scn, &con);
    std::cout << "Render3" << std::endl;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "mujoco_control");
    ros::NodeHandle node("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    const auto default_model_path = ros::package::getPath("mujoco_ros_control") + "/model/simple_robot.urdf";
    const auto model_path = node.param("model", default_model_path);

    const auto key_path = std::string(getenv("HOME")) + "/.mujoco/mjkey.txt";

    if (!mj_activate(key_path.c_str())) {
        ROS_ERROR_STREAM("Cannot activate mujoco with key: " << key_path);
        return -1;
    }

    char error[1000] = "";
    m = mj_loadXML(model_path.c_str(), nullptr, error, 1000);

    if (!m) {
        ROS_ERROR_STREAM("Cannot load model: " << model_path);
        ROS_ERROR_STREAM(error);
        return -1;
    }

    d = mj_makeData(m);
    if (!d) {
        ROS_ERROR_STREAM("Cannot make data structure for model.");
        return -1;
    }

    // run one computation to initialize all fields
    initGl();
    mj_step(m, d);
    initVisual();

    hw.reset(new RobotHWMujoco(*m));
    cm.reset(new controller_manager::ControllerManager(hw.get(), node));

    start_time.reset(new ros::Time());
    *start_time = ros::Time::now();

    mjcb_control = cb_controller;

    const auto timestep = ros::Duration(m->opt.timestep);
    std::cout << m->opt.timestep << std::endl;

//    auto sim_timer = node.createTimer(timestep, [&](const ros::TimerEvent &e) {
//       std::cout << "Simulation" << std::endl;
//       mj_step(m, d);
//    });

    // Render
    mjr_setBuffer(mjFB_OFFSCREEN, &con);
    if (con.currentBuffer != mjFB_OFFSCREEN)
        printf("Warning: offscreen rendering not supported, using default/window framebuffer\n");

    // get size of active renderbuffer
    mjrRect viewport = mjr_maxViewport(&con);
    int W = viewport.width;
    int H = viewport.height;

    std::mutex mutex;
    const auto render_timer = node.createTimer(timestep * 180, [&mutex, &viewport](const ros::TimerEvent &e) {
        std::lock_guard<std::mutex> guard(mutex);
        render(viewport);
    });

    ros::waitForShutdown();

    //free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteModel(m);
    mj_deleteData(d);
    mj_deactivate();

    return 0;
}
