var relearn_searchindex = [
  {
    "breadcrumb": "Y.HMMT \u003e Research",
    "content": "Underwater Soft Fin Flapping Motion with Deep Neural Network Based Surrogate Model Yuya Hamamatsu*, Pavlo Kupyn* ** , Roza Gkliva*, Asko Ristolainen*, Maarja Kruusmaa*\n*Tallinn University of Technology, **Vilnius University\nAbstract This study presents a novel framework for precise force control of fin-actuated underwater robots by integrating a deep neural network (DNN)-based surrogate model with reinforcement learning (RL). To address the complex interactions with the underwater environment and the high experimental costs, a DNN surrogate model acts as a simulator for enabling efficient training for the RL agent. Additionally, grid-switching control is applied to select optimized models for specific force reference ranges, improving control accuracy and stability. Experimental results show that the RL agent, trained in the surrogate simulation, generates complex thrust motions and achieves precise control of a real soft fin actuator. This approach provides an efficient control solution for fin-actuated robots in challenging underwater environments.\nVideo Overview The surrogate model consists of two neural networks: PosNet, a 1D convolutional neural network (CNN) that predicts the actual motion response of the actuator to a given command, and ForceNet, an LSTM-based network that estimates the generated force based on the predicted motion. By decoupling motor response prediction and force generation, this architecture enables more accurate modeling of dynamic interactions in the underwater environment. PosNet learns a mapping from motor commands to movement states, capturing uncertainties and nonlinearities in actuator behavior, while ForceNet models the time-dependent force dynamics influenced by the soft fin’s deformation and hydrodynamic forces. The networks are trained using supervised learning with real-world force and motion data, optimizing mean squared error (MSE) loss for accurate predictions.\nThe RL agent operates within this surrogate simulation environment to learn optimal control policies for generating target forces. The policy outputs a target angle and angular velocity, which are processed through the surrogate model to estimate the resulting force. The reward function combines force accuracy with stability constraints using the Sobolev norm, which penalizes both force error and abrupt variations in force output. This formulation ensures that the learned policy not only tracks the desired force but also maintains smooth transitions, preventing instability due to sudden force fluctuations. Proximal Policy Optimization (PPO) with an LSTM-based policy network is employed to handle the sequential dependencies in force control, enabling the RL agent to leverage temporal context for improved decision-making.\n$$| F |{W^{1,2}} \\approx \\left( \\sum{i=0}^{n-1} |F^i - F^r|^2 + \\sum_{i=0}^{n-2} |F^{i+1} - F^i|^2 \\right)^{1/2}$$\nTo further enhance performance, a grid-switching RL approach is introduced, where multiple policies are trained for different force reference ranges, and the appropriate policy is selected dynamically based on the current target. This strategy prevents degradation in policy performance when extrapolating to force conditions that were not well-represented in training, thereby improving control robustness across a wide range of forces.\nExperiments validate the effectiveness of this approach by comparing single-policy RL control with grid-switching RL control. The grid-switching model consistently achieves lower force-tracking error, demonstrating improved precision in force generation. These results highlight the benefits of integrating a surrogate model with RL for learning efficient and accurate force control strategies in underwater soft-fin actuation.\nCite ​ bibtex @inproceedings{hamamatsu2025underwater, title={Underwater Soft Fin Flapping Motion with Deep Neural Network Based Surrogate Model}, author={Hamamatsu, Yuya and Kupyn, Pavlo and Gkliva, Roza and Asko, Ristolainen and Kruusmaa, Maarja}, booktitle={IEEE-RAS International Conference on Soft Robotics (RoboSoft)}, year={2025}, organization={IEEE} }",
    "description": "Underwater Soft Fin Flapping Motion with Deep Neural Network Based Surrogate Model Yuya Hamamatsu*, Pavlo Kupyn* ** , Roza Gkliva*, Asko Ristolainen*, Maarja Kruusmaa*\n*Tallinn University of Technology, **Vilnius University\nAbstract This study presents a novel framework for precise force control of fin-actuated underwater robots by integrating a deep neural network (DNN)-based surrogate model with reinforcement learning (RL). To address the complex interactions with the underwater environment and the high experimental costs, a DNN surrogate model acts as a simulator for enabling efficient training for the RL agent. Additionally, grid-switching control is applied to select optimized models for specific force reference ranges, improving control accuracy and stability. Experimental results show that the RL agent, trained in the surrogate simulation, generates complex thrust motions and achieves precise control of a real soft fin actuator. This approach provides an efficient control solution for fin-actuated robots in challenging underwater environments.",
    "tags": [],
    "title": "Robosoft2025",
    "uri": "/research/robosoft2025/index.html"
  },
  {
    "breadcrumb": "",
    "content": "Hi, I’m guitar player. My interest is heavy metal\nCV Todo\nResearch Project Fig Conf/journal Title ICRA 2025 Cross-platform Learning-based Fault Tolerant Surfacing Controller for Underwater Robots RoboSoft 2025 Underwater Soft Fin Flapping Motion with Deep Neural Network Based Surrogate Model",
    "description": "Hi, I’m guitar player. My interest is heavy metal\nCV Todo\nResearch Project Fig Conf/journal Title ICRA 2025 Cross-platform Learning-based Fault Tolerant Surfacing Controller for Underwater Robots RoboSoft 2025 Underwater Soft Fin Flapping Motion with Deep Neural Network Based Surrogate Model",
    "tags": [],
    "title": "Y.HMMT",
    "uri": "/index.html"
  },
  {
    "breadcrumb": "Y.HMMT",
    "content": "",
    "description": "",
    "tags": [],
    "title": "Categories",
    "uri": "/categories/index.html"
  },
  {
    "breadcrumb": "Y.HMMT",
    "content": "",
    "description": "",
    "tags": [],
    "title": "Tags",
    "uri": "/tags/index.html"
  }
]
