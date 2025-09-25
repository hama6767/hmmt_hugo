+++
alwaysopen = false
date = '2025-09-24T10:50:30+02:00'
title = 'DR mujoco'
collapsibleMenu=true
weight = 10
+++

The documentation page for DR mujoco which simulate differential drive robot for educational purpose.

![Image](https://github.com/user-attachments/assets/f4cb7bf3-b1aa-4886-a3c5-4155851b7869)

## MuJoCo & ROS 2 Simulator Overview

This simulator is a bridge program designed to connect the advanced physics engine **MuJoCo** with the **ROS 2** robotics framework.

It allows you to test your ROS 2 including control, navigation, and perception algorithms—in a realistic, physics-based environment.

## Key Features

### Seamless ROS 2 Integration (The Core Feature)

The simulator's most important feature is its complete integration into the ROS 2 ecosystem.

  * **Uses Standard Topics**:

      * It accepts robot movement commands via the standard **/cmd\_vel** topic, using the `geometry_msgs/msg/Twist` message type.
      * It publishes the robot's position and velocity status as an **/odom** topic, using the `nav_msgs/msg/Odometry` message type.

  * **Publishes Sensor Data as ROS 2 Messages**:

      * Video from the robot's camera is published as a `sensor_msgs/msg/Image` message, which can be viewed in RViz or used as input for computer vision nodes.
      * Wheel encoder tick counts are published as `std_msgs/msg/Int32` messages, allowing for more realistic odometry calculation tests.

  * **Compatible with Existing ROS 2 Tools**:

      * You can use the entire ROS 2 ecosystem you're already familiar with, such as visualizing odometry and sensor data in `RViz2` or inspecting the node and topic connections with `rqt_graph`.

### Realistic Physics Simulation

Thanks to the fast and accurate MuJoCo physics engine, the simulator faithfully reproduces contact, friction, and inertia. This helps to reduce the "Sim-to-Real Gap," making the transition from simulation to a physical robot smoother.

### Customizable Robot Models

The robot in the simulation is defined using MuJoCo's **XML format**, giving you the freedom to design and test your own creations. You can quickly modify wheel sizes, body mass, sensor placements, and more to prototype different robot designs.

## How It Works

This simulator acts as a bidirectional "translator" between your ROS 2 nodes and the MuJoCo engine.

```
 [Your ROS 2 Node]   <-- ROS 2 Topics -->   [This Simulator]   <-- MuJoCo API -->   [MuJoCo Physics Engine]
(e.g., Teleop Node)    (/cmd_vel, /odom)     (ROS 2 <=> MuJoCo)
```


{{% children containerstyle="div" style="h3" description=true %}}
