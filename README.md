# Quadrotor Simulation

This repository contains a quadrotor simulation in MATLAB and Simulink, with formation control integrating Adaptive Integral Fast Terminal Sliding Mode Control and Artificial Potential Field guidance system.

## Installation and Usage

Follow these steps to run the simulation:

1. **Clone the repository**
   ```sh
   git clone https://github.com/raulpab/quadrotor_simulation.git
   cd quadrotor_simulation
   ```

2. **Run the simulation in MATLAB**
   - Open MATLAB and navigate to the cloned folder.
   - Run the following script to define the UAV trajectory:
     ```matlab
     trajectory_uav_
     ```

3. **Run the model in Simulink**
   - Open the `FORMATION_CONTROL_trajectory.slx` file in Simulink.
   - Run the simulation.

4. **Generate graphics**
   - Run the following script in MATLAB:
     ```matlab
     Graphics
     ```

## Requirements
- MATLAB with Simulink
- Control tools and dynamic system simulation

## Author
[Pablo Raul](https://github.com/raulpab)
