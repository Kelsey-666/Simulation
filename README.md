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


### II. Sensor modelling effects

#### 1.Sensor performance

Camera [Noise-free image (160*160)]

![Screenshot 2025-03-26 172302](C:\Users\yuanx\Pictures\Screenshots\Screenshot 2025-03-26 172302.png)

Camera [noisy (Gaussian noise 0.7/0.007) image (160*160, image cropped)]

![Screenshot 2025-03-27 162118](C:\Users\yuanx\Pictures\Screenshots\Screenshot 2025-03-27 162118.png)

#### 2. Radar

Radar [noiseless visualisation of radar lines]

![Screenshot 2025-03-27 154940](C:\Users\yuanx\Pictures\Screenshots\Screenshot 2025-03-27 154940.png)

Radar [noisy (Gaussian 0.7/0.007) visual radar line (position detection error)]

![Screenshot 2025-03-27 162027](C:\Users\yuanx\Pictures\Screenshots\Screenshot 2025-03-27 162027.png)

### III. Special modelling notes

#### 1.Glass material

![Screenshot 2025-03-27 160456](C:\Users\yuanx\Pictures\Screenshots\Screenshot 2025-03-27 160456.png)

Note: The glass part has no collision model, only visual model, and its transparency is 0.2. At the same time, the transmittance to radar light is 0.85, and the reflectivity is 0.15. This is achieved by the following key code:

`        <material>Gazebo/Glass</material>
          <laser_retro>0</laser_retro>
          <sensor name="laser_transparency">
          <transmissivity>0.85</transmissivity>
          <reflectivity>0.15</reflectivity> 
        </sensor>`

#### 2.Drone collision volume handling

![Screenshot 2025-03-27 153221](C:\Users\yuanx\Pictures\Screenshots\Screenshot 2025-03-27 153221.png)

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
