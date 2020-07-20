# Applying Multi-Agent RL to SLAM with Graph Pose for Sampled-Data MPC and CPN of Autonomous Drone Swarms

#### Project website: [Capstone 2020](https://yuejinyz.github.io/2020CapstoneProject.github.io/)

## Abstract

Developing methods that allow drones to autonomously navigate in different environ-ments has been a topic of extensive research in recent years. One research topic of interestfor interest in autonomous drone navigation is to explore the maneuverability and capability of drones to navigate inaccessible environments and situations that might be too risky for human access. Using a swarm of drones that autonomously navigate a post-catastrophe scenario in order to optimally map the disaster zone, i.e independently and efficiently identify and map the structural damage across a geographic site, has been aproblem less explored. Detection and mapping changes across a post-catastrophe site en-ables a more robust estimation of structural damage. This project attempted to exploreand simulate a reinforcement learning approach to enable drones to perform task assign-ment and scheduling in order to efficiently maximize coverage for identifying and map-ping structural changes within the post-catastrophe environment. The primary objective of the simulation was to focus on the exploration of _ad-hoc_ decentralized task assignment and scheduling by one or more drone(s) at the edge with minimal connectivity aside from local communication between nearest neighbors. Other workstreams in the project explored satellite and aerial imagery, _seismic structural damage equation models and generative adversarial networks_ (GANs) related to the Port-au-Prince 2010 Haiti earthquake site as a use case and attempt to explore methods that might be utilized to identify structural changes fromsatellite images, using generative synthetic data and estimated fragility equations in orderto address uncertainty and ambiguity in the detection of discrepancies in edges related to damage which could aid the drones in the uncertain areas.~

## Contents

- [A2C algorithm - RL](A2C)
- [Deep Q-learning - RL](DQL_model)
- [GAN for generating data](GAN)
- [2D simulation](swarmsim-2d)
- [3D simulation](swarsim-3d)

For a detailed explanation, refer the project website
