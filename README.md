# gesture-controlled-drone
** Gesture Controlled Drone ** is a a vision based human gesture-controlled drone system with autonomous yaw alignment, designed for intuitive and real-time human–drone interaction. The system allows a user to control drone motion using natural body gestures, while ensuring that the drone continuously orients itself toward the user.


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

*Human pose estimation
*Gesture interpretation
*State-machine-based control logic
*Closed-loop yaw control

Designed for low-latency interaction and stable flight behavior, the system runs efficiently on lightweight hardware using a monocular camera.

All sub modules such as human pose estimation, gesture recognition, closed-loop yaw control are implemented in __minco-trajectory-planner__:

The whole architecture is organized into two primary ROS2 packages:


Object Tracker Package (Perception + Interaction Logic)

This package handles the complete vision pipeline and gesture interpretation, and it consists of two files :

- __process_frame__
It extracts key human body landmarks (shoulders, elbows, wrists, hips) from monocular camera input using MediaPipe.
Gesture recognition is performed using simple geometric relationships (joint angles), enabling lightweight and real-time computation without depending on any heavy learning models.

- __tracker_node__
It Integrates perception output with a state-machine-based interaction logic to ensure stable gesture recognition and safe transitions.
It publishes:
Lateral motion commands (direction + intensity) in the topic `/lateral_command` 
Human position offset (waist center) for tracking in the topic `/waist_angle`

This module effectively converts raw visual input into structured control signals.


Human Tracking Controls Package (Control + Flight Execution)

This package is responsible for drone control, flight management, and closed-loop tracking, implemented using ROS2 and MAVROS.

Implements a finite-state flight controller with stages such as:
*Mode switching (GUIDED)
*Arming
*Takeoff
*Hover
*Active tracking
*Landing

It implementd a PID controller for yaw alignment, driven by the human position offset, enabling the drone to continuously face the user.
Converts gesture-based commands into velocity control inputs, including:

Lateral motion (left/right) using body-frame to world-frame transformation
Altitude stabilization using proportional control
Position hold when no gesture is active

Incorporates safety mechanisms, such as:
Vision timeout fallback to hover
Threshold-based engagement/disengagement of tracking

The full pipeline is validated in simulation (Gazebo–ROS–ArduPilot SITL) as well as on hardware with an F450 quadrotor running on a Raspberry Pi 4.



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