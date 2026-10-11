# Adaptive VFH+ Mobile Robot Navigation

An adaptive Vector Field Histogram (A-VFH+) local navigation and obstacle avoidance algorithm designed for differential drive mobile robots in MATLAB, featuring dynamic weight adjustment to mitigate local minima and cyclic oscillations.

## Key Features
- **Adaptive Weighting (β-Indicator):** Dynamically suppresses target attraction weight near traps to enable smooth U-turn escapes.
- **Differential Drive Kinematics:** Full low-level kinematic implementation mapping steering commands to wheel velocities.
- **Real-Time Simulation:** Visual testing environment with configurable obstacle clutter.

## Repository Structure
- `VFHPlusController.m`: Class definitions and adaptive VFH+ core navigation logic.
- `run_VFHplus.m`: Main simulation loop, robot kinematic updates, and environment setups.

## How to Run
1. Open both files in MATLAB.
2. Run `run_VFHplus.m` to execute the simulation and generate obstacle performance metrics.
