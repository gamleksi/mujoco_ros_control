### Introduction
The _ros_control_ interface for the _MuJoCo_ simulator.
The code parses a given model and register a control interface for each _slide_ or _hinge_ joint.
The _ros_control_ effort based controllers could be then loaded as shown in example _start_simple_robot.launch_.
It provides trajectory based interface e.g. for MoveIt.  

### Installation
#### MuJoCo
Download MuJoCo simulator from http://www.mujoco.org and put the simulator code as well as your MuJoCo key into the folder _~/.mujoco/_, i.e.:
```
ls ~/.mujoco
mjkey.txt  mjpro150

ls ~/.mujoco/mjpro150/
bin  doc  include  model  sample
```

#### Ros package install
Put this package into your ros workspace, in case you do not have a workspace yet:

```
mkdir -p ~/ros/src && cd ~/ros/src
git clone git@version.aalto.fi:robotic_manipulation/mujoco_ros_control.git
cd ..
colcon build
```

#### Run tests
All tests should pass if the installation was successful.
```
colcon build --cmake-target tests && colcon test --packages-select  mujoco_ros_control && colcon test-result --all
...
Summary: X tests, 0 errors, 0 failures, 0 skipped
where X >= 2
```


### Simple 1 DOF Example:

```
roslaunch mujoco_ros_control start_simple_robot.launch
roslaunch mujoco_ros_control rviz.launch
```

### Simulation of Kuka lwr/lbr robot provided in separate repository:
```
github... TBD
``` 

### Resources
- https://version.aalto.fi/gitlab/robotic_manipulation/mujoco_ros_control
- http://www.mujoco.org/
- https://colcon.readthedocs.io/