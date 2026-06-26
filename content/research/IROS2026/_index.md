+++
date = '2026-06-26T12:00:00+02:00'
draft = false
title = 'IROS26'
math = true
weight = 3
+++

## Layout-independent actuation allocator for marine robots
Yuya Hamamatsu, Maarja Kruusmaa, Asko Ristolainen

[Tallinn University of Technology](https://taltech.ee/en/biorobotics)

{{% button href="#" style="grey" icon="fa-fw fab fa-github" %}}Code (Coming soon){{% /button %}}

{{% button href="#" icon="fa-solid fa-file-pdf" style="warning" %}}arXiv{{% /button %}}

### Abstract
In this study, we propose a layout-independent control allocator capable of zero-shot deployment across diverse actuator configurations. The proposed method utilizes a learning pipeline that integrates a Graph Neural Network (GNN) and a Transformer to represent the robot's geometric layout as a graph, along with a Mixture Density Network (MDN) to predict multi-modal control command distributions. Furthermore, by incorporating a differentiable physics surrogate model, we achieve command refinement during inference to minimize target wrench tracking error and energy consumption. A single generalized model using randomly generated actuator layout data demonstrated high trajectory tracking performance on different actuator layout robots outside the training distribution. Additionally, in real-world pool experiments, our approach achieved performance nearly equivalent to conventional controllers designed to specific layouts.

![Image](https://github.com/user-attachments/assets/87fa9e0c-db3d-4c04-b43a-2b4a1ef39308)

## Video
{{< youtube 0Ut5Zu3zmBE>}}

## Overview

This study introduces a layout-independent control allocator for underwater robots that enables zero-shot deployment across various actuator configurations without requiring platform-specific redesigns.

The approach represents the robot's fin placement and orientation as a graph using a Graph Neural Network (GNN) and a Transformer. A Mixture Density Network (MDN) then predicts optimal control command distributions. Additionally, a differentiable physics surrogate model refines these commands during inference, minimizing both trajectory tracking errors and energy consumption.

Evaluated through both simulations and real-world pool experiments using a single model trained on randomly generated layouts, the system achieved tracking performance comparable to traditional layout-specific controllers. It also demonstrated robust fault tolerance in scenarios where one or two fins failed, and successfully reduced mean power consumption by 11.9% compared to baseline methods.