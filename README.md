# isaac-sim-franka

This repository handles the useful scripts and configurations for interacting with Isaac Sim. The main goal is to provide a simple way to interact with the Franka Emika Panda robot in the Isaac Sim environment.

## Requirements

| Package / SDK    | Version  |
| ---------------- | -------- |
| Nvidia Omniverse | 2023.2.3 |
| Isaac Sim        | 4.0.0    |
| Python           | 3.10     |
| ROS2             | Humble   |
| Moveit           | Humble   |

## Setup experiment

1. Clone this repository in your workspace
2. Install the required dependencies
   1. Make sure you have virtualenv installed
   2. Create a virtual environment
   3. Install the required dependencies
      ```bash
      pip install -r requirements.txt
      ```

## Run experiment

1. Start Isaac Sim
   1. Run the bash script to start the Isaac Sim environment
      ```bash
      ./isaac --isaac-dir <path-to-isaac-sim>
      ```
2. Start **ROS2**
   1. To start the demo launch file,
      ```bash
      # From the root of the workspace,
      cd ros_ws/
      colcon build
      source ./install/setup.bash

      # Now start the demo launch file
      ros2 launch franka_demo demo.launch.py
      ```
