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

### Rigid Body Kinematics

The kinematics of the swerve drive system are derived based on rigid body motion principles.

<div align="center">
<table>
  <tr>
    <td align="center"><b>Fig 1. Schematic of Rigid Body Kinematics</b></td>
    <td align="center"><b>Fig 2. Warrior Robot Kinematics</b></td>
  </tr>
  <tr>
    <td align="center"><img src="./docs/rigid_body_kinematics.png" height="430"/></td>
    <td align="center"><img src="./docs/inverse_kinematics.png" height="430"/></td>
  </tr>
</table>
</div>

As illustrated in `Fig. 1`, for two points `A` and `B` on the same rigid body, both points share the same translational motion while point `B` simultaneously rotates about point `A`. Therefore, the velocity relationship between the two points can be expressed as:

$$
\mathbf{v}_B =
\mathbf{v}_A +
\boldsymbol{\omega}
\times
\mathbf{r}_{AB}
$$

where:

- $\mathbf{v}_A$: translational velocity of point `A`
- $\mathbf{v}_B$: velocity of point `B`
- $\boldsymbol{\omega}$: angular velocity of point `B` relative to point `A`
- $\mathbf{r}_{AB}$: position vector from point `A` to point `B`

### Warrior Robot Kinematics

Based on the rigid body kinematics described above, the kinematic model of the Warrior robot can be derived. As in `Fig 2`, given the desired base twist $[\mathbf{v}_b,\ \boldsymbol{\omega}]$, the velocity relationship between the robot base and each swerve module is expressed as:

$$
\mathbf{v}_i = \mathbf{v}_b + \boldsymbol{\omega} \times \mathbf{r}_{bi}, \qquad i \in \\{1,2,3\\}
$$


where $v_b = [v_{bx}, v_{by}, 0]^T$ denotes the linear velocity of the robot base, $\boldsymbol{\omega} = [0,0,\omega_z]^T$ is the angular velocity of the base, and $v_i = [v_{ix}, v_{iy}, 0]^T$ is the velocity of the $i$-th swerve module. The vector $r_{bi} = [r_{ix}, r_{iy}, 0]^T$ represents the position vector from the robot base to the corresponding swerve module. Accordingly, the kinematic relationship can be expressed in matrix form as:

$$
\begin{bmatrix}
v_{ix} \\
v_{iy}
\end{bmatrix} =
\begin{bmatrix}
1 & 0 & -r_{iy} \\
0 & 1 & r_{ix}
\end{bmatrix}
\begin{bmatrix}
v_{bx} \\
v_{by} \\
\omega_z
\end{bmatrix},
\qquad
i \in \\{1,2,3\\}
$$

By inverse kinematics, the desired driving velocity and steering angle for each swerve module are given by:

$$
\|\mathbf{v}_i\|_2 =
\sqrt{v_{ix}^2 + v_{iy}^2},
\qquad
i \in \\{1,2,3\\}
$$

$$
\theta_i =
\mathrm{atan2}(v_{iy}, v_{ix}),
\qquad
i \in \\{1,2,3\\}
$$

The corresponding wheel angular velocity for each separate driving motor can then be computed as:

$$
\omega_i^{wheel} =
\frac{\|\mathbf{v}_i\|_2}{R_{i}},
\qquad
i \in \\{1,2,3\\}
$$

where $\|\mathbf{v}_i\|_2$ denotes the desired linear velocity of the $i$-th wheel, $\omega_i^{wheel}$ is the desired angular velocity of the driving wheel, $R_i$ is the wheel radius, and $\theta_i$ represents the desired steering angle of the corresponding steering motor.

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