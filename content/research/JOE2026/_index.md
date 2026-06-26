+++
date = '2026-06-26T16:25:22+03:00'
draft = false
title = 'JOE2026'
math = true
weight = 2
+++

## Strouhal-Aware Model Predictive Control for Efficient Multi-Fin Flapping Locomotion
Yuya Hamamatsu, Zixi Chen, Maarja Kruusmaa, Asko Ristolainen

[Tallinn University of Technology](https://taltech.ee/en/biorobotics)

Vrije Universiteit Brussel

{{% button href="#" style="grey" icon="fa-fw fab fa-github" %}}Code (Coming soon){{% /button %}}

{{% button href="#" icon="fa-solid fa-file-pdf" style="warning" %}}arXiv{{% /button %}}

### Abstract
Efficient flapping propulsion hinges on operating within a narrow Strouhal number window, a principle nature has converged upon for maximum thrust-to-power ratio. We translate this bioinspired empirical rule into real-time control, demonstrating it on an autonomous underwater vehicle driven by four soft fins. The proposed Strouhal-aware Model Predictive Control (MPC) enhances a quasi-steady hydrodynamic model with an explicit penalty for Strouhal deviation, solving the resulting nonconvex problem via a two-stage sampling and gradient optimization that runs onboard at 25 Hz. Pool and field trials show that the controller keeps each fin within the optimal Strouhal corridor (0.25-0.35) while precisely tracking commanded forces. This results in a mean reduction in mechanical power of 8.8% to 32% throughout the cruising range of 0.1 to 0.3 m/s. The proposed method also allows for a velocity of 0.4 m/s, which is unattainable for a baseline of the conventional inverse model. The results confirm that embedding first-principle flow physics into an MPC objective yields tangible endurance gains without sacrificing agility, offering a generic pathway to energy-aware locomotion in next-generation multifin robots.

![Image](https://github.com/user-attachments/assets/7620d9c8-fa4d-4682-9aa3-b86b29b03f9a)

<!-- ## Video
{{< youtube INSERT_VIDEO_ID_HERE >}} -->

## Overview

This study introduces a novel control framework for multi-fin underwater robots that integrates a bioinspired principle of efficient flapping propulsion to significantly improve endurance. The efficiency of flapping locomotion relies heavily on maintaining a specific Strouhal number (typically between 0.25 and 0.35).

The proposed Strouhal-aware Model Predictive Control (MPC) explicitly penalizes deviations from this optimal Strouhal corridor. To handle the highly nonconvex optimization landscape in real-time, the framework employs a two-stage hybrid solver running onboard at 25 Hz. It first uses a Sampling-based MPC (SMPC) to find a near-global warm start, followed by a gradient-based Nonlinear MPC (NMPC) for precise local refinement.

Validated on the U-CAT robot, which is propelled by four flexible silicone fins, the controller successfully balanced thrust tracking and energy efficiency. Pool experiments demonstrated that the proposed method reduced mean mechanical power consumption by 8.8% to 32% across cruising speeds of 0.1 to 0.3 m/s when compared to a conventional inverse-model baseline. Furthermore, field trials conducted at Rummu Lake confirmed the controller's robustness and stable trajectory tracking capabilities in dynamic, real-world aquatic environments.