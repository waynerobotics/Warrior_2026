# User Guide

## Warrior System

This package provides a topic-bridge interface for the low-level control layer within the [ros2_control](https://control.ros.org/humble/doc/ros2_control/doc/index.html) framework. Similar to a typical hardware resource manager implementation, the `read()` and `write()` methods are executed at a fixed frequency to communicate with the physical robot hardware.

Unlike traditional implementations that directly interface with hardware drivers in C++, this package introduces ROS topic-based communication as an intermediate layer. Although this design may introduce a small amount of additional latency, it provides significantly greater flexibility for hardware communication and integration. For example, it enables serial communication implementations in Python rather than being restricted to C++ only.

### System Layer Diagram

---

The flowchart below illustrates how the topic-bridged system communicates with the swerve controller and the hardware serial interface.

```mermaid
flowchart LR

    %% =========================
    %% Communication Layer
    %% =========================
    subgraph COMM["Communication Layer"]
        direction TB

        FB["/warrior_swerve_state"]
        CMD["/warrior_swerve_command"]
    end


    %% =========================
    %% ros2_control Core
    %% =========================
    subgraph CORE["ros2_control Core"]
        direction TB

        SYS([warrior_robot_ros2_control])

        CTRL["Warrior Swerve Controller"]
    end


    %% =========================
    %% Hardware Layer
    %% =========================
    subgraph HWLAYER["Hardware Layer"]
        direction TB

        HW["Hardware Serial Interface"]
    end


    %% =========================
    %% Data Flow
    %% =========================
    FB ==>|"read()"| SYS

    SYS ==>|"write()"| CMD

    CTRL -. "command interfaces" .-> SYS
    SYS -. "state interfaces" .-> CTRL

    CMD ==>|"tx"| HW
    HW ==>|"rx"| FB


    %% =========================
    %% Node Styles
    %% =========================
    classDef hardware fill:#1e293b,stroke:#64748b,color:#e2e8f0,stroke-width:2px;
    classDef state fill:#1e3a5f,stroke:#3b82f6,color:#dbeafe,stroke-width:2px;
    classDef command fill:#163826,stroke:#22c55e,color:#dcfce7,stroke-width:2px;
    classDef system fill:#3b1f1f,stroke:#ef4444,color:#fee2e2,stroke-width:4px;
    classDef controller fill:#312e81,stroke:#8b5cf6,color:#ede9fe,stroke-width:2px;


    %% =========================
    %% Subgraph Styles
    %% =========================
    style COMM fill:#111827,stroke:#374151,stroke-width:2px,color:#d1d5db
    style CORE fill:#18181b,stroke:#52525b,stroke-width:2px,color:#e4e4e7
    style HWLAYER fill:#0f172a,stroke:#334155,stroke-width:2px,color:#cbd5e1


    %% =========================
    %% Apply Classes
    %% =========================
    class HW hardware;
    class FB state;
    class CMD command;
    class SYS system;
    class CTRL controller;
```

### ros2-control Xacro Configuration

---

The ros2_control xacro configuration file is located at: [robot.ros2_control.xacro](../../warrior_description/xacro/ros2_control/robot.ros2_control.xacro).

Currently, each swerve module supports the following hardware interfaces:

- **Steering Motor**
  - State interface: `position`
  - Command interface: `position`
- **Driving Motor**
  - State interface: `velocity`
  - Command interface: `velocity`
  
Additional hardware interfaces will be added in future updates to support smoother and more advanced robot control capabilities.


```xml
        <ros2_control name="warrior_robot_ros2_control" type="system">
            <hardware>
                <plugin>warrior_system/SwerveTopicBridge</plugin>
                <param name="joint_commands_topic">/warrior_joint_commands</param>
                <param name="joint_states_topic">/warrior_joint_states</param>
            </hardware>
            <joint name="left_steer_joint">
                <command_interface name="position"/>
                <state_interface name="position"/>
            </joint>
            <joint name="left_drive_joint">
                <command_interface name="velocity"/>
                <state_interface name="velocity"/>
            </joint>
            <joint name="right_steer_joint">
                <command_interface name="position"/>
                <state_interface name="position"/>
            </joint>
            <joint name="right_drive_joint">
                <command_interface name="velocity"/>
                <state_interface name="velocity"/>
            </joint>
            <joint name="front_steer_joint">
                <command_interface name="position"/>
                <state_interface name="position"/>
            </joint>
            <joint name="front_drive_joint">
                <command_interface name="velocity"/>
                <state_interface name="velocity"/>
            </joint>
        </ros2_control>
```