+++
date = '2025-02-10T10:50:46+02:00'
draft = false
title = 'ICRA25'
math = true
weight = 2
+++

## Cross-platform Learning-based Fault Tolerant Surfacing Controller for Underwater Robots
Yuya Hamamatsu, Walid Remmas, Jaan Rebane, Maarja Kruusmaa, Asko Ristolainen

[Tallinn University of Technology](https://taltech.ee/en/biorobotics)

{{% button href="https://github.com/Centre-for-Biorobotics/eeUVsim_Gazebo" style="grey" icon="fa-fw fab fa-github" %}}Code{{% /button %}}


### Abstract
In this paper, we propose a novel cross-platform fault-tolerant surfacing controller for underwater robots, based on reinforcement learning (RL). Unlike conventional approaches, which require explicit identification of malfunctioning actuators, our method allows the robot to surface using only the remaining operational actuators without needing to pinpoint the failures. The proposed controller learns a robust policy capable of handling diverse failure scenarios across different actuator configurations. Moreover, we introduce a transfer learning mechanism that shares a part of the control policy across various underwater robots with different actuators, thus improving learning efficiency and generalization across platforms. To validate our approach, we conduct simulations on three different types of underwater robots: a hovering-type AUV, a torpedo shaped AUV, and a turtle-shaped robot (U-CAT). Additionally, real-world experiments are performed, successfully transferring the learned policy from simulation to a physical U-CAT in a controlled environment. Our RL-based controller demonstrates superior performance in terms of stability and success rate compared to a baseline controller, achieving an 85.7 percent success rate in real-world tests compared to 57.1 percent with a baseline controller. This research provides a scalable and efficient solution for fault-tolerant control for diverse underwater platforms, with potential applications in real-world aquatic missions.


![Image](https://github.com/user-attachments/assets/4d24b549-fcd9-42a9-9c12-cb3337fc30df)

## Video
{{< youtube pHw0qLb1rkI >}}

## Overview

Our study proposes a reinforcement learning (RL)-based fault-tolerant surfacing controller for underwater robots that does not require explicit identification of faulty actuators. Instead, it enables the robot to surface using only the remaining operational actuators. Additionally, we introduce cross-platform transfer learning to enhance learning efficiency across different robotic platforms.

In this study, we focus on three types of underwater robots: a hovering-type AUV equipped with eight thrusters, a torpedo-shaped AUV with four rudders and a single thruster, and a turtle-shaped robot (U-CAT) that utilizes four oscillating fins for propulsion. To achieve this, we employ Proximal Policy Optimization (PPO) with Long Short-Term Memory (LSTM) networks. This reinforcement learning approach allows the policy to handle sequential decision-making under uncertain conditions. The agent receives sensor observations, including vertical velocity from a pressure sensor, attitude represented by quaternions, and angular velocity derived from an Inertial Measurement Unit (IMU). Based on these observations, the agent determines control actions tailored to each robot’s actuator configuration.

The training procedure follows a structured approach where each episode begins with a randomly assigned set of actuator failures. The agent must learn to surface without direct knowledge of which actuators are malfunctioning. The policy is optimized based on a reward function that includes vertical velocity, uprightness stability, and a goal reward for successfully reaching the surface. The PPO algorithm ensures that policy updates remain stable by limiting drastic changes, while the LSTM component captures temporal dependencies crucial for effective control.

![Image](https://github.com/hama6767/pubdata/raw/main/Peek%202024-09-23%2013-52.gif)

To further improve learning efficiency, we introduce cross-platform transfer learning by sharing the first LSTM layer between different robots. Since the observation space remains consistent across platforms, this shared layer allows knowledge transfer while maintaining the adaptability required for different actuator configurations. Experimental results show that transferring the first LSTM layer significantly accelerates learning across robots. However, when additional layers are transferred, performance improvements are inconsistent, likely due to the actuator-specific characteristics embedded in the deeper network layers.

Real-world validation is conducted using U-CAT in a controlled pool environment. The trained RL controller achieves an 85.7% success rate in surfacing trials, outperforming a baseline PID controller, which achieves only 57.1%. The RL controller exhibits superior stability, minimizing variations in attitude while ensuring consistent surfacing behavior. Analysis of failed trials reveals that unsuccessful attempts with the RL model result from insufficient thrust rather than instability, indicating that further optimization of thrust allocation could enhance performance.

![Image](https://github.com/hama6767/pubdata/raw/main/Peek%202024-09-23%2015-22.gif)


### cite
 {{% tab title="bibtex" %}}
```c
@inproceedings{hamamatsu2025icra,
    title={Cross-platform Learning-based Fault Tolerant Surfacing Controller for Underwater Robots}, 
    author={Hamamatsu, Yuya and Remmas, Walid and Rebane, Jaan and Kruusmaa, Maarja and Asko, Ristolainen},
    booktitle={2025 IEEE International Conference on Robotics and Automation (ICRA)},
    year={2025},
    organization={IEEE}
}
```
{{% /tab %}}