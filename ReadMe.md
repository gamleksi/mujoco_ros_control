# Mujoco Ros Control

Built by [Intelligent Robotics group](http://irobotics.aalto.fi).

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
mjkey.txt  mjpro200

ls ~/.mujoco/mjpro200/
bin  doc  include  model  sample
```

### Resources
- http://www.mujoco.org/
- https://colcon.readthedocs.io/
