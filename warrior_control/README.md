# Warrior Control

A ROS 2 control package implementing swerve drive kinematics for a three-wheel omnidirectional warrior robot.

## Overview

This module provides inverse and forward kinematics solutions for a three-wheel swerve drive system, enabling omnidirectional movement with independent wheel steering and velocity control.

### Features
- ✅ Inverse kinematics: Convert body velocity ($v_x$, $v_y$, $\omega_z$) to wheel commands
- ✅ Forward kinematics: Calculate odometry from wheel states
- ✅ Configurable wheel geometry and constraints
- ✅ Real-time velocity control interface

## Principle

The swerve drive kinematics is derived from rigid body dynamics principles.

### Rigid Body Dynamics

<p align="center">
 <img src="./docs/rigid_body_dynamics.png" width="840" alt="Rigid body dynamics diagram"/> 
 <br><b>Fig 1. Rigid Body Dynamics</b>
</p>

For a rigid body moving in 2D space, the velocity at any point can be expressed as:

$$\mathbf{v}_i = \mathbf{v}_{center} + \boldsymbol{\omega} \times \mathbf{r}_i$$

where:
- $\mathbf{v}_i$: velocity of wheel $i$
- $\mathbf{v}_{center}$: linear velocity of robot center
- $\boldsymbol{\omega}$: angular velocity of the robot
- $\mathbf{r}_i$: position vector from center to wheel $i$

### Inverse Kinematics

<p align="center">
 <img src="./docs/inverse_kinematics.png" width="840" alt="Inverse kinematics for three-wheel swerve"/> 
 <br><b>Fig 2. Inverse Kinematics for Warrior</b>
</p>

Given desired body velocities $(v_x, v_y, \omega_z)$, the module calculates:
- Wheel steering angles $\theta_f, \theta_l, \theta_r$
- Wheel driving velocities $v_f, v_l, v_r$


## Usage

### Dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Build
```bash
colcon build --packages-select warrior_control
source install/setup.bash
```

### Launch
```bash
ros2 launch warrior_control warrior_control.launch.py
```

### Configuration

Edit `config/warrior_params.yaml` to configure:
- Wheel positions relative to robot center
- Maximum wheel velocities and accelerations
- Control loop frequency

### API

**Subscribed Topics:**
- `/cmd_vel` (geometry_msgs/Twist): Desired robot velocity

**Published Topics:**
- `/odom` (nav_msgs/Odometry): Robot odometry

## 📝 References

[1] Wheeled Mobile Robot Kinematics. Available at: https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html

[2] Using Inverse Kinematics to become a Master-Swerver. Available at: https://abhinavwastaken.medium.com/using-inverse-kinematics-to-become-a-master-swerver-1026759d81b0

[3] Lynch, K. M. and Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.