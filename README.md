# indoor Flight Arena ROS Environment for Reinforcement Learning

This repository contains the ROS environment necessary for launching and controlling the **drone** robot in a Gazebo simulation. For Gazebo and Rviz based RL simulation training programs. Built a sensor and simulation environment started by sdf and urdf commands.
## Table of Contents
- [Overview](#overview)
- [Installation](#installation)
- [Usage](#usage)
  - [Launching the Environment](#launching-the-environment)

## Overview

This environment, including the ROS map and robot configuration, is based on the [RL robot navigation]([https://github.com/reiniscimurs/DRL-robot-navigation](https://github.com/Kelsey-666/Simulation.git)) repository.

The environment supports launching the **drone** robot in Gazebo or Rviz. It includes all necessary configurations and packages to bring up, describe, and control the robot.
### Cranfield University Indoor Flight Centre Refined Scenario Modelling

### I. Overall scene effects

![Screenshot 2025-03-27 151904](https://github.com/user-attachments/assets/1508fb0c-3bcf-4293-b9c8-971681a3629f)
![Screenshot 2025-03-21 145248](https://github.com/user-attachments/assets/f5de7ec9-e35d-4cb2-b2b1-136d5e81efaf)
![A_Inside_overview](https://github.com/user-attachments/assets/13d1869b-fa0b-4109-a198-16793a4f206e)

### II. Sensor modelling effects

#### 1.Sensor performance

Camera [Noise-free image (160*160)]

![A_Camera_without_noise](https://github.com/user-attachments/assets/75fd6bd7-2ebd-4a15-92a1-634641148bdd)

Camera [noisy (Gaussian noise 0.7/0.007) image (160*160, image cropped)]

![A_Camera_with_noise](https://github.com/user-attachments/assets/e5222dde-c6e0-4387-adf5-22c2d6a8384a)

#### 2. Radar

Radar [noiseless visualisation of radar lines]

![A_Lidar_without_noise](https://github.com/user-attachments/assets/8e2bcfd8-607d-43a2-994c-f0b23231c7ab)

Radar [noisy (Gaussian 0.7/0.007) visual radar line (position detection error)]

![A_Lidar_with_noise](https://github.com/user-attachments/assets/d4e31dc0-8b25-4cbf-8508-c043ee82ef99)

### III. Special modelling notes

#### 1.Glass material

![A_Glsaa VisualLidar_performance](https://github.com/user-attachments/assets/31184e24-1e83-4314-b790-ef1948097fe6)
![A_Glass_performance](https://github.com/user-attachments/assets/71d2b9d8-2386-4ae3-baea-56ddcb85dc25)

Note: The glass part has no collision model, only visual model, and its transparency is 0.2. At the same time, the transmittance to radar light is 0.85, and the reflectivity is 0.15. This is achieved by the following key code:

`        <material>Gazebo/Glass</material>`
`         <laser_retro>0</laser_retro>`
`         <sensor name="laser_transparency">`
`         <transmissivity>0.85</transmissivity>`
`         <reflectivity>0.15</reflectivity>`
`        </sensor>`

#### 2.Drone collision volume handling

![A_UAV_coliision_model](https://github.com/user-attachments/assets/3201c3f0-0e0c-4e70-8339-c7e7db5bf38f)

Note: Considering that the UAV needs a geo-fence to prevent possible physical collisions, the collision volume of the UAV is constructed as a cylinder with an additional radius of 0.5m.

## Installation

### Prerequisites

- **[ROS Noetic](http://wiki.ros.org/noetic/Installation)**: Ensure that ROS Noetic is installed and sourced.
- **Gazebo 11**: Required for simulation.
- **Python 3.8+**: For using Gymnasium-based reinforcement learning environments.

### Setting Up the Workspace

1. Clone this repository:
    ```bash
    git clone https://github.com/Kelsey-666/Simulation.git
    cd Simulation
    ```

2. Build the Catkin workspace:
    ```bash
    catkin_make_isolated
    ```

3. Source the workspace:
    ```bash
    source devel_isolated/setup.bash
    ```

## Usage

### Launching the Environment

To launch the simulation environment for the drone robot:
```bash
source catkin_ws/devel_isolated/setup.bash
roslaunch Simulation a_drone.launch
```
