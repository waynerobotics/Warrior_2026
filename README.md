# Warrior 2026

ROS 2 stack for Wayne Robotics' 2026 IGVC entry. Targets **ROS 2 Humble** +
**Gazebo Fortress** on **Ubuntu 22.04**.

## Quick start

### Installation
Follow the warrior_scripts Readme for installation instructions

### Launch
Follow the warrior_bringup Readme for launch instructions

## System layout

```mermaid
flowchart LR
    subgraph INPUT["Input"]
        JOY[warrior_joy]
        NAV[warrior_navigation]
    end

    subgraph BRAIN["Estimation & Control"]
        LOC[warrior_localization]
        GPS[warrior_gps]
        CTRL[warrior_control]
    end

    subgraph PLANT["Plant"]
        SIM[warrior_simulation]
        HW[warrior_hardware]
    end

    DESC[warrior_description]
    BRINGUP[warrior_bringup]
    MSGS[warrior_msgs]

    JOY -->|cmd_vel| CTRL
    NAV -->|cmd_vel| CTRL
    CTRL -->|joint cmds| HW
    CTRL -->|joint cmds| SIM
    HW -->|joint states, sensors| LOC
    SIM -->|joint states, sensors| LOC
    GPS --> LOC
    LOC -->|odom, tf| NAV
    DESC -.->|URDF| SIM
    DESC -.->|URDF| CTRL
    BRINGUP -.->|launches| BRAIN
    BRINGUP -.->|launches| PLANT
    BRINGUP -.->|launches| INPUT
    MSGS -.->|types| BRAIN
    MSGS -.->|types| PLANT
```

## Packages

| Package | What it does | README |
|---|---|---|
| [warrior_bringup](warrior_bringup/) | Top-level launchers (sim, real, turtlebot variants) + systemd unit | [README](warrior_bringup/README.md) |
| [warrior_control](warrior_control/) | Swerve / diff drive controllers, kinematics | [README](warrior_control/README.md) |
| [warrior_description](warrior_description/) | URDF / xacro / meshes | [README](warrior_description/README.md) |
| [warrior_gps](warrior_gps/) | GPS + AprilTag fusion nodes | — |
| [warrior_hardware](warrior_hardware/) | `ros2_control` HW interface + USB driver for swerve Arduinos and SPARK MAXes | [README](warrior_hardware/README.md) · [TEST_PLAN](warrior_hardware/TEST_PLAN.md) |
| [warrior_joy](warrior_joy/) | Gamepad → cmd_vel | [README](warrior_joy/README.md) |
| [warrior_localization](warrior_localization/) | EKF, SLAM, sensor fusion | — |
| [warrior_msgs](warrior_msgs/) | Custom message definitions | — |
| [warrior_navigation](warrior_navigation/) | Costmaps, path planning, Nav2 integration | — |
| [warrior_scripts](warrior_scripts/) | Install scripts + dev helpers | [README](warrior_scripts/README.md) |
| [warrior_simulation/warrior_gazebo](warrior_simulation/warrior_gazebo/) | Gazebo worlds + SDF models | [README](warrior_simulation/warrior_gazebo/README.md) |

## Where things live

- **Launch anything** → [warrior_bringup](warrior_bringup/README.md)
- **Install / first-time setup** → [warrior_scripts](warrior_scripts/README.md)
- **Robot kinematics & math** → [warrior_control](warrior_control/README.md)
- **Sim worlds** → [warrior_gazebo](warrior_simulation/warrior_gazebo/README.md)
- **Real-hw control flow** → [warrior_hardware](warrior_hardware/README.md)
