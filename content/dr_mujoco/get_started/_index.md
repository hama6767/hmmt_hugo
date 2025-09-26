+++
date = '2025-02-10T10:50:46+02:00'
draft = false
title = 'Get started'
math = true
weight = 1
+++

## Requirements
Ubuntu 22.04, 24.04

ROS2 Humble, Jazzy

Mujoco more than 3.3.5

It should work with other versions but they have not been tested.

## Installation
Install mujoco

```
pip install mujoco
```

Clone simulator repository from here.
https://github.com/Centre-for-Biorobotics/dr_mujoco

Put it on your ros2 workspace and then colcon build. 

You can launch demo script as

```
ros2 launch dr_mujoco default.launch.py 
```

If you see the red box on the simulator, it is success!

![Image](https://github.com/user-attachments/assets/74ee62a0-6b53-417b-b833-69c395442b1c)


This simulator is for differential drive robot. **But mujoco model is not included in this repository** due to the class usage. You have to make your own mujoco model of differential drive robot.