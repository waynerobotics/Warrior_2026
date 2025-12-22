# Warrior Control

---

A ROS 2 control package implementing swerve drive kinematics for a three-wheel omnidirectional warrior robot.

## Overview

This module provides inverse and forward kinematics solutions for a three-wheel swerve drive system, enabling omnidirectional movement with independent wheel steering and velocity control.

### Features
- ✅ Inverse kinematics: Convert body velocity (vx, vy, ω) to wheel commands
- ✅ Forward kinematics: Calculate odometry from wheel states
- ✅ Configurable wheel geometry and constraints
- ✅ Real-time velocity control interface

## Principle

The swerve drive kinematics is derived from rigid body dynamics principles.

### Rigid Body Dynamics

<p align="center">
 <img src="./docs/rigid_body_dynamics.png" height="540" alt="Rigid body dynamics diagram"/> 
 <br><b>Fig 1. Rigid Body Dynamics</b>
</p>

For a rigid body moving in 2D space, the velocity at any point can be expressed as:

\[
\mathbf{v}_i = \mathbf{v}_{center} + \boldsymbol{\omega} \times \mathbf{r}_i
\]

Where:
- \[\mathbf{v}_i\]: Velocity of wheel \[i\]
- \[\mathbf{v}_{center}\]: Linear velocity of robot center
- \[\boldsymbol{\omega}\]: Angular velocity of the robot
- \[\mathbf{r}_i\]: Position vector from center to wheel \[i\]

### Inverse Kinematics

<p align="center">
 <img src="./docs/inverse_kinematics.png" height="540" alt="Inverse kinematics for three-wheel swerve"/> 
 <br><b>Fig 2. Inverse Kinematics for Warrior</b>
</p>

Given desired body velocities \[(v_x, v_y, \omega)\], the module calculates:
- Wheel steering angles \[\theta_i\]
- Wheel driving velocities \[v_i\]

For each wheel \[i\]:
```
v_ix = v_x - ω * r_iy
v_iy = v_y + ω * r_ix

θ_i = atan2(v_iy, v_ix)
v_i = sqrt(v_ix² + v_iy²)
```

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
- `/wheel_states` (custom_msgs/WheelStates): Commanded wheel angles and velocities
- `/odom` (nav_msgs/Odometry): Robot odometry

## 📝 References

[1] Wheeled Mobile Robot Kinematics. Available at: https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html

[2] Using Inverse Kinematics to become a Master-Swerver. Available at: https://abhinavwastaken.medium.com/using-inverse-kinematics-to-become-a-master-swerver-1026759d81b0

[3] Ether, J. (2014). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.