# gesture-controlled-drone
**Gesture Controlled Drone** is a a vision based human gesture-controlled drone system with autonomous yaw alignment, designed for intuitive and real-time human–drone interaction. The system allows a user to control drone motion using natural body gestures, while ensuring that the drone continuously orients itself toward the user.


To run this project in minutes, check [Quick Start](#1-Quick-Start). Check other sections for more detailed information.

Please kindly star :star: this project if it helps you. We take great efforts to develope and maintain it :grin::grin:.


## Table of Contents

* [Quick Start](#1-Quick-Start)
* [Algorithms and Papers](#2-Algorithms-and-Papers)       <!-- * [Run Simulations](#4-run-simulations) -->
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

<!-- You will find the random map and the drone in ```Rviz```. You can select goals for the drone to reach using the ```2D Nav Goal``` tool. A sample simulation is showed [here](#demo1). -->


## 2. Algorithms and Papers

The system combines:

- **Human pose estimation**
- **Gesture interpretation**
- **State-machine-based control logic**
- **Closed-loop yaw control**

Designed for **low-latency interaction** and **stable flight behavior**, the system runs efficiently on lightweight hardware using a **monocular camera**.

All submodules such as **human pose estimation**, **gesture recognition**, and **closed-loop yaw control** are implemented within the `minco-trajectory-planner`.

The complete architecture is organized into two primary ROS2 packages:

---

# Object Tracker Package  
### *(Perception + Interaction Logic)*

This package handles the complete vision pipeline and gesture interpretation.

It consists of two main files:

---

### `process_frame`

This module:

- Extracts human body landmarks from monocular camera input using **MediaPipe**
  - Shoulders
  - Elbows
  - Wrists
  - Hips

- Performs gesture recognition using lightweight geometric relationships:
  - Joint-angle-based calculations
  - Real-time inference
  - No heavy deep-learning models required

This enables efficient and low-latency gesture understanding on lightweight hardware.

---

### `tracker_node`

This module integrates perception output with a **state-machine-based interaction logic** to ensure:

- Stable gesture recognition
- Safe state transitions
- Robust interaction behavior

It publishes:

- **Lateral motion commands** (`direction + intensity`)  
  Topic:
  ```bash
  /lateral_command

# Human Tracking Controls Package  
### *(Control + Flight Execution)*

This package is responsible for:

- **Drone control**
- **Flight management**
- **Closed-loop human tracking**

The complete controller is implemented using **ROS2** and **MAVROS**.

---

## Flight Controller Architecture

The system implements a **finite-state flight controller** with multiple operational stages:

- **Mode Switching** (`GUIDED`)
- **Arming**
- **Takeoff**
- **Hover**
- **Active Tracking**
- **Landing**

This state-machine-based design ensures stable transitions and safer autonomous operation.

---

## Closed-Loop Tracking and Control

The package implements a **PID-based yaw controller** driven by the human position offset, enabling the drone to continuously align itself toward the user.

Gesture-based commands are converted into velocity control inputs, including:

### Lateral Motion Control
- Left/right drone motion
- Implemented using **body-frame to world-frame transformation**

### Altitude Stabilization
- Maintains stable flight height
- Implemented using **proportional control**

### Position Hold
- Activated automatically when no valid gesture is detected
- Helps maintain stable hover behavior

---

## Safety Mechanisms

The controller incorporates multiple safety features to improve flight reliability:

- **Vision-timeout fallback to hover**
- **Threshold-based engagement/disengagement of tracking**
- Stable control transitions using finite-state logic

These mechanisms prevent unstable drone behavior during perception loss or ambiguous gesture input.

---

## Validation and Testing

The complete pipeline has been validated in both:

### Simulation Environment
- **Gazebo**
- **ROS2**
- **ArduPilot SITL**

### Real Hardware Platform
- **F450 Quadrotor**
- **Raspberry Pi 4**

## 3. Use in Your Application

<!-- If you have successfully run the simulation and want to use __minco-trajectory-planner__ in your project,
please explore the files kino_replan.launch or topo_replan.launch.
Important parameters that may be changed in your usage are contained and documented.

Note that in our configuration, the size of depth image is 640x480. 
For higher map fusion efficiency we do downsampling (in kino_algorithm.xml, skip_pixel = 2).
If you use depth images with lower resolution (like 256x144), you might disable the downsampling by setting skip_pixel = 1. Also, the _depth_scaling_factor_ is set to 1000, which may need to be changed according to your device.

Finally, for setup problem, like compilation error caused by different versions of ROS/Eigen, please first refer to existing __issues__, __pull request__, and __Google__ before raising a new issue. Insignificant issue will receive no reply.

-->

## 4. Parameters

The ROS implementation exposes several parameters:

|Parameter|Definition|Default|
|---|---|---|
|`enable_magnetometer`|If true, magnetometer readings are included in the state estimation update.|`false`|
|`mag_calib/bias`|Bias of the magnetometer.|`[0,0,0]`|
|`mag_calib/scale`|Scale of the magnetometer.|`[1,1,1]`|
|`mag_calib/reference`|World frame reference vector of the magnetometer.|`[0,0,0]`|
|`process_scale_factor`|'Fudge factor' to multiply by incoming gyro covariances.|1|
|`gyro_bias_thresh`|Threshold of motion below which we may estimate gyro bias.|0.01 rad/s|

When using the node, you should remap `~imu` and `~field` to the appropriate topics. See `attitude_eskf.launch` for an example.