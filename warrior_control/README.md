# Warrior Control

A ROS 2 control package implementing swerve drive kinematics for a three-wheel
omnidirectional warrior robot. See the [workspace README](../README.md) for how
this fits the whole pipeline.

## Overview

This module provides inverse and forward kinematics solutions for a three-wheel
swerve drive system, enabling omnidirectional movement with independent wheel
steering and velocity control. It ships as a `ros2_control` controller plugin
(`swerve_drive_controller/SwerveDriveController`,
[src/swerve_drive_controller.cpp](src/swerve_drive_controller.cpp)) plus a
header-only IK library ([include/warrior_control/swerve_ik.hpp](include/warrior_control/swerve_ik.hpp)).

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

Based on the rigid body kinematics described above, the kinematics of the Warrior robot can be derived. As in `Fig 2`, given the desired base twist $\xi_b = [v_{bx},\ v_{by},\ \omega_z]^T$, the velocity relationship between the robot base and each swerve module is expressed as:

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


## Control Flow
The overall and control pipeline are summarized below:

The robot first receives the desired base twist $\mathbf{\xi}_b$. Through inverse kinematics (IK), the base motion is then transformed into swerve module commands, including the desired steering angle $\mathbf{\theta}_i$ and wheel speed $\omega_i$ for each module. The steering motors track the desired angles using PD control, while the driving motors regulate the wheel speeds through PWM-based velocity control.

```mermaid
graph LR
    A["Base Twist​"] -->|IK| B["Swerve Command"]

    B -->|θ_i| C["Steer Angle"]
    B -->|ω_i| D["Wheel Speed"]

    C -->|PD| E["Steering Motor"]
    D -->|PWM| F["Driving Motor"]
```

## Implementation

The controller's per-tick data flow inside `update()`:

```mermaid
flowchart LR
    CV[/cmd_vel\nTwistStamped/] --> CL[clamp to cmd_vel_limit]
    CL --> SM[smoother\naccel/decel limits]
    SM --> IK[SwerveIK]
    IK -->|steer θ/position| SCMD[steer cmd ifaces]
    IK -->|drive ω/velocity| DCMD[drive cmd ifaces]
    STATE[steer position\nstate ifaces] --> IK
    DCMD --> KF[Kalman EKF] --> ODOM[/odom/]
```

The [swerve_ik.hpp](include/warrior_control/swerve_ik.hpp) library adds two
practical refinements on top of the textbook IK above:

```mermaid
graph LR
    A[Base twist vx,vy,wz] -->|IK| B[per-wheel vᵢ]
    B -->|atan2| C[θᵢ steer]
    B -->|‖vᵢ‖/R| D[ωᵢ drive]
    C --> E[flip + slip guard]
```

- **Flip optimization** — if the steering delta exceeds 90°, add π and negate the
  drive speed (shortest reorientation).
- **Slip guard** — drive speed scaled by `cos(angle_error)`; zeroed if the error
  exceeds 90°. Below `0.01` m/s & rad/s the module holds its current angle at zero speed.

### ros2_control interfaces

| | Steer joints | Drive joints |
|---|---|---|
| **command** | `position` | `velocity` |
| **state** | `position` | `position`, `velocity` |

Joints (from config): `{front,left,right}_steer_joint`, `{front,left,right}_drive_joint`
— must match the HW plugin in
[warrior_description/.../robot.ros2_control.xacro](../warrior_description/xacro/ros2_control/robot.ros2_control.xacro).

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

Normally invoked via `warrior_bringup` (`main.launch.py` / swerve teleop), not
directly. To run the controller stack on its own:

```bash
# Real robot — controller_manager + swerve_drive_controller + rviz:
ros2 launch warrior_control swerve_drive.real.launch.py

# Simulation — Gazebo + bridge + swerve_drive_controller:
ros2 launch warrior_control swerve_drive.gazebo.launch.py
```

### Configuration

Edit [config/warrior_controllers_real.yaml](config/warrior_controllers_real.yaml)
(real) or [config/warrior_controllers_sim.yaml](config/warrior_controllers_sim.yaml)
(sim) to configure:
- Wheel radius (`wheel_radius`), module positions (`wheel_to_center`) and bearings (`alpha`, 0/120/240°)
- Velocity limits (`cmd_vel_limit`) and the accel/decel `smoother`
- Odometry EKF process/measurement noise (`kf.*`)

### API

**Subscribed Topics:**
- `/cmd_vel` (`geometry_msgs/TwistStamped`): desired base twist (hardcoded topic name)
- `/odom_gt` (`nav_msgs/Odometry`): simulation ground truth (republished as `/odom` with a z offset)

**Published Topics:**
- `/odom` (`nav_msgs/Odometry`): EKF-fused odometry (topic set by the `odom_topic` param)

## 📝 References

[1] Wheeled Mobile Robot Kinematics. Available at: https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html

[2] Using Inverse Kinematics to become a Master-Swerver. Available at: https://abhinavwastaken.medium.com/using-inverse-kinematics-to-become-a-master-swerver-1026759d81b0

[3] Lynch, K. M. and Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.
</content>
