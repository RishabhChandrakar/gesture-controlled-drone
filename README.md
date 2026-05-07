# gesture-controlled-drone
**Gesture Controlled Drone** is a a vision based human gesture-controlled drone system with autonomous yaw alignment, designed for intuitive and real-time human–drone interaction. The system allows a user to control drone motion using natural body gestures, while ensuring that the drone continuously orients itself toward the user.


To run this project in minutes, check [Quick Start](#1-Quick-Start). Check other sections for more detailed information.

Please kindly star :star: this project if it helps you. We take great efforts to develope and maintain it :grin::grin:.


## Table of Contents

* [Quick Start](#1-Quick-Start)
* [Algorithms and Papers](#2-Algorithms-and-Papers)       <!-- * [Run Simulations](#4-run-simulations) -->
* [Validation and Testing](#2-Validation-and-Testing)
* [Use in Your Application](#3-use-in-your-application)
<!-- * [Known issues](#known-issues) -->

## 1. Quick Start

This project has been tested on Ubuntu 22.04(ROS Humble) .

Firstly, you should install the following required libraries:

Eigen3
Ompl
Octopmap

```
  sudo apt-get update && sudo apt-get install -y \
  libeigen3-dev \
  ros-humble-octomap-ros \
  ros-humble-ompl
```

Then simply clone and compile our package (using ssh here):

```
git clone git@github.com:RishabhChandrakar/minco-trajectory-planner.git
cd minco-trajectory-planner
colcon build
```

You may check the detailed [instruction](#3-setup-and-config) to setup the project. 

After compilation you can start a simulation (run in a new terminals): 
```
source install/setup.bash && ros2 launch planner planner_simulation.launch
```

## 2. Architecture

The system combines:

- **Human pose estimation**
- **Gesture interpretation**
- **State-machine-based control logic**
- **Closed-loop yaw control**

The complete architecture consists of two primary ROS2 packages:

- **Object Tracker**
- **Human Tracking Controls**

### 1. Object Tracker Package  

This package handles the vision part and gesture recognition.

It consists of two files:

**process_frame** - It extracts key human body landmarks from monocular camera input using **MediaPipe** and then recognises the gesture using **geometric and joint-angle** relationships without requiring any heavy deep-learning models.

**tracker_node** - It integrates perception output with a **state-machine-based interaction logic** to ensure stable gesture recognition , safe state transitions , robust interaction.

It publishes:

- **Lateral motion commands** (`direction + intensity`)  
  Topic: ```bash /lateral_command
  
- **Human position offset**   
  Topic: ```bash /waist_angle

### 2. Human Tracking Controls Package  

This package is responsible for complete flight management, the whole architecture consists of following parts :

**Closed-Loop Tracking and Control** - It includes a **PID-based yaw controller** driven by the human position offset, enabling the drone to continuously align itself toward the user.

**Lateral Motion Control**
- Left/right drone motion
- Implemented using **body-frame to world-frame transformation**

**Position Hold**
- Activated automatically when no valid gesture is detected
- Helps maintain stable hover behavior

### Safety Mechanisms

To prevent unstable drone behavior during perception loss or ambiguous gesture input , following safety measures have been implemented .

- **Vision-timeout fallback to hover**
- **Threshold-based engagement/disengagement of tracking**
- Stable control transitions using finite-state logic

---

## 3. Validation and Testing

The complete pipeline has been validated in both:

### Simulation Environment
- **Gazebo**
- **ROS2**
- **ArduPilot SITL**

### Real Hardware Platform
- **F450 Quadrotor**
- **Raspberry Pi 4**
- **Raspberry Pi Camera**

## 4. Use in Your Application



