# Warrior Robot Launch Guide

This document explains how to launch all Warrior robot configurations using the centralized launch system.

## Quick Start

### Simulated Swerve Drive (IGVC Competition World)
```bash
ros2 launch warrior_bringup swerve_sim.launch.py
```

### Simulated Differential Drive (Empty World)
```bash
ros2 launch warrior_bringup diff_sim.launch.py
```

### Real Warrior Hardware
```bash
ros2 launch warrior_bringup warrior_real.launch.py
```

### Simulated TurtleBot3 (with GPS)
```bash
ros2 launch warrior_bringup turtlebot_sim.launch.py
```

### Real TurtleBot3 Hardware
```bash
ros2 launch warrior_bringup turtlebot_real.launch.py
```

---

## Master Launcher: `main.launch.py`

The `main.launch.py` file is the central entry point for all robot launch configurations. It provides a unified interface with intelligent routing based on arguments.

### Usage

```bash
ros2 launch warrior_bringup main.launch.py robot_type:=<type> [world_name:=<world>] [use_sim_time:=<true|false>]
```

### Arguments

| Argument | Type | Default | Options | Description |
|----------|------|---------|---------|-------------|
| `robot_type` | string | `swerve_sim` | `swerve_sim`, `diff_sim`, `warrior_real`, `turtlebot_sim`, `turtlebot_real` | Type of robot to launch |
| `world_name` | string | `competition.world` | See below | Gazebo world file name (ignored for real hardware) |
| `use_sim_time` | bool | `true` | `true`, `false` | Use simulation time |
| `namespace` | string | (empty) | any string | Robot namespace (for multi-robot setups) |

### World Options

Supported worlds depend on robot type:

**Swerve Drive (`swerve_sim`):**
- `competition.world` (default) - IGVC competition arena
- `empty.world` - Minimal empty world
- Any other world in `warrior_description/worlds/`

**Differential Drive (`diff_sim`):**
- `empty.world` (default) - Minimal empty world
- `competition.world` - IGVC competition arena
- Any other world in `warrior_description/worlds/`

**TurtleBot3 (`turtlebot_sim`):**
- `turtlebot3_world_gps.world` (default) - TurtleBot3 world with GPS support

---

## Convenience Launchers

For common use cases, convenience wrapper launchers are provided. These pre-set the `robot_type` and sensible defaults:

### `swerve_sim.launch.py`
Launches simulated swerve-drive Warrior in competition world.

**Usage:**
```bash
# Use defaults (competition world)
ros2 launch warrior_bringup swerve_sim.launch.py

# Override world
ros2 launch warrior_bringup swerve_sim.launch.py world_name:=empty.world
```

### `diff_sim.launch.py`
Launches simulated differential-drive Warrior in empty world.

**Usage:**
```bash
# Use defaults (empty world)
ros2 launch warrior_bringup diff_sim.launch.py

# Override world
ros2 launch warrior_bringup diff_sim.launch.py world_name:=competition.world
```

### `warrior_real.launch.py`
Launches real Warrior hardware with ROS 2 control stack.

**Usage:**
```bash
ros2 launch warrior_bringup warrior_real.launch.py
```

**Prerequisites:**
- Hardware is properly connected and powered on
- Arduino firmware flashed and responding (if using arduino_drive)
- ROS 2 drivers running for any additional hardware (GPS, IMU, etc.)

### `turtlebot_sim.launch.py`
Launches simulated TurtleBot3 Burger with GPS localization.

**Usage:**
```bash
ros2 launch warrior_bringup turtlebot_sim.launch.py
```

### `turtlebot_real.launch.py`
Launches real TurtleBot3 Burger hardware.

**Usage:**
```bash
ros2 launch warrior_bringup turtlebot_real.launch.py
```

**Prerequisites:**
- TurtleBot3 Burger hardware is powered on and accessible
- TurtleBot3 ROS 2 driver packages are installed (`turtlebot3_bringup`)

---

## Launch Matrix: Supported Combinations

### Simulated Swerve (Full Sensor Suite)
```bash
# Default (competition world)
ros2 launch warrior_bringup swerve_sim.launch.py

# Alternative worlds
ros2 launch warrior_bringup swerve_sim.launch.py world_name:=empty.world
ros2 launch warrior_bringup main.launch.py robot_type:=swerve_sim world_name:=custom.world
```

**Includes:** Gazebo simulation, full sensor bridges (camera, lidar, IMU), RViz visualization, teleop_twist_keyboard

**Add-ons (run separately):**
- Joystick control: `ros2 launch warrior_joy joy_swerve.launch.py`
- Localization: `ros2 launch warrior_localization ekf.launch.py`
- Navigation: `ros2 launch warrior_navigation nav_utils/costmap.launch.py`

### Simulated Differential Drive
```bash
# Default (empty world)
ros2 launch warrior_bringup diff_sim.launch.py

# Alternative worlds
ros2 launch warrior_bringup diff_sim.launch.py world_name:=competition.world
```

**Includes:** Gazebo simulation, limited sensor bridges (camera only), RViz visualization, teleop_twist_keyboard

**Add-ons (run separately):**
- Joystick control: `ros2 launch warrior_joy joy_2stick.launch.py`

### Simulated TurtleBot3 with GPS
```bash
ros2 launch warrior_bringup turtlebot_sim.launch.py
```

**Includes:** Gazebo simulation, TurtleBot3 Burger model, GPS bridge, sensor interfaces

**Add-ons (run separately):**
- Joystick control: `ros2 launch warrior_joy joy_turtle.launch.py`
- SLAM: `ros2 launch warrior_localization online_async_launch.py`
- Navigation with path following: `ros2 launch warrior_navigation goal_follower.launch.py use_sim:=true`

### Real Warrior Hardware
```bash
ros2 launch warrior_bringup warrior_real.launch.py
```

**Includes:** ROS 2 control stack, robot state publisher, RViz visualization, teleop_twist_keyboard

**Add-ons (run separately):**
- Joystick control: `ros2 launch warrior_joy joy_2stick.launch.py` or `joy_swerve.launch.py`
- Optional: `ros2 launch warrior_localization ekf.launch.py use_sim_time:=false`

### Real TurtleBot3 Hardware
```bash
ros2 launch warrior_bringup turtlebot_real.launch.py
```

**Includes:** TurtleBot3 native bringup, hardware drivers

**Add-ons (run separately):**
- Joystick control: `ros2 launch warrior_joy joy_turtle.launch.py`
- SLAM: `ros2 launch warrior_localization online_async_launch.py use_sim_time:=false`
- Navigation: `ros2 launch warrior_navigation goal_follower.launch.py use_sim:=false`

---

## Advanced: Multi-Robot Setup with Namespaces

To run multiple robots simultaneously, use the `namespace` argument:

```bash
# Terminal 1: Swerve robot 1
ros2 launch warrior_bringup swerve_sim.launch.py namespace:=/robot1

# Terminal 2: TurtleBot robot 2
ros2 launch warrior_bringup turtlebot_sim.launch.py namespace:=/robot2
```

Topics and services will be namespaced accordingly (e.g., `/robot1/cmd_vel`, `/robot2/odom`).

---

## Joystick Control (Run Separately)

Joystick launchers are designed to be run independently in separate terminals and can be combined with any robot launch. Choose the appropriate joystick type for your robot:

### Swerve Drive Joystick
```bash
ros2 launch warrior_joy joy_swerve.launch.py
```

### Differential Drive (Dual-Stick) Joystick
```bash
ros2 launch warrior_joy joy_2stick.launch.py
```

### TurtleBot3 Joystick
```bash
ros2 launch warrior_joy joy_turtle.launch.py
```

**Note:** These are standalone launchers and don't need to be included in the main robot launch. You can mix and match joystick types with robot types as needed.

---

## Optional: Localization, Navigation & Path Planning

These modules work with most robot configurations and are launched separately:

### Basic Localization (EKF)
```bash
# Simulation
ros2 launch warrior_localization ekf.launch.py use_sim_time:=true

# Real hardware
ros2 launch warrior_localization ekf.launch.py use_sim_time:=false
```

### Advanced Localization (SLAM + Dual EKF + GPS)
```bash
ros2 launch warrior_localization dual_ekf_navsat.launch.py use_sim_time:=true
```

### GPS-Specific Localization (AprilTag + GPS Fusion)
```bash
ros2 launch warrior_gps gps_localization.launch.py mode:=indoor
```

### Cost Maps (for planning)
```bash
ros2 launch warrior_navigation nav_utils/costmap.launch.py use_sim_time:=true
```

### Complex Path Following (with SLAM, EKF, and custom path follower)
```bash
ros2 launch warrior_navigation goal_follower.launch.py use_sim:=true
```

### Nav2-Based Navigation
```bash
ros2 launch warrior_navigation nav2_goal_follower.launch.py use_sim:=true
```

---

## Troubleshooting

### "Package not found" Error
```
Resource not found: warrior_control

Solution: Make sure the workspace is properly sourced:
source install/setup.bash
```

### "World file not found" Error
```
[ERROR] Error: Cannot find file [/path/to/worlds/my_world.world]

Solution: Check that the world file exists in warrior_description/worlds/
List available worlds:
ls src/warrior/warrior_description/worlds/
```

### Gazebo Not Launching (Black Window)
```
Solution 1: Verify you have Gazebo installed:
which gz-sim

Solution 2: Set up Gazebo paths properly:
source /opt/ros/<distro>/setup.bash

Solution 3: Check sim_time - verify use_sim_time:=true is used for simulation
```

### Teleop Not Responding
```
Solution 1: Check that teleop is running:
ros2 node list | grep teleop

Solution 2: Verify cmd_vel remapping is correct for your robot:
- Swerve: /swerve_drive_controller/cmd_vel
- Diff:   /diff_drive_controller/cmd_vel
- TurtleBot: /cmd_vel

Solution 3: Launch teleop separately if desired:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Real Hardware Not Responding
```
Solution 1: Check hardware connections and power
Solution 2: Verify drivers are running:
ros2 node list

Solution 3: Check hardware diagnostics:
ros2 topic list
ros2 topic echo /diagnostics
```

---

## Architecture Overview

```
main.launch.py (master entry point)
├─ robot_type == "swerve_sim"
│  └─> warrior_control/swerve_drive.gazebo.launch.py
├─ robot_type == "diff_sim"
│  └─> warrior_control/diff_drive.gazebo.launch.py
├─ robot_type == "warrior_real"
│  └─> warrior_bringup/warrior.launch.py
├─ robot_type == "turtlebot_sim"
│  └─> warrior_navigation/nav_utils/turtlebot3_world_gps.launch.py
└─ robot_type == "turtlebot_real"
   └─> turtlebot3_bringup/robot.launch.py (with TURTLEBOT3_MODEL=burger)

Convenience wrappers (pre-configured shortcuts):
├─ swerve_sim.launch.py     -> main.launch.py robot_type:=swerve_sim
├─ diff_sim.launch.py       -> main.launch.py robot_type:=diff_sim
├─ warrior_real.launch.py   -> main.launch.py robot_type:=warrior_real
├─ turtlebot_sim.launch.py  -> main.launch.py robot_type:=turtlebot_sim
└─ turtlebot_real.launch.py -> main.launch.py robot_type:=turtlebot_real
```

---

## Migration Guide: From Old Launchers

### Old Way vs New Way

| Task | Old Command | New Command |
|------|------------|------------|
| Launch swerve sim | `ros2 launch warrior_control swerve_drive.gazebo.launch.py` | `ros2 launch warrior_bringup swerve_sim.launch.py` |
| Launch swerve with custom world | N/A (had to edit file) | `ros2 launch warrior_bringup swerve_sim.launch.py world_name:=empty.world` |
| Launch diff drive | `ros2 launch warrior_control diff_drive.gazebo.launch.py` | `ros2 launch warrior_bringup diff_sim.launch.py` |
| Launch real robot | `ros2 launch warrior_bringup warrior.launch.py` | `ros2 launch warrior_bringup warrior_real.launch.py` |
| Launch TurtleBot3 | `ros2 launch warrior_navigation nav_utils/turtlebot3_world_gps.launch.py` | `ros2 launch warrior_bringup turtlebot_sim.launch.py` |

**Old launchers are still available** for backward compatibility but are now marked as deprecated. The new system provides better organization and easier configuration.

---

## Configuration File Organization

Configuration files remain in their respective packages to maintain package independence:

- `warrior_control/config/` - Robot control parameters (ROS 2 control, joint limits)
- `warrior_bringup/config/` - Gazebo bridge configurations
- `warrior_navigation/config/` - Navigation and costmap parameters
- `warrior_localization/config/` - EKF, SLAM, and fusion parameters
- `warrior_gps/config/` - GPS localization parameters

This structure ensures each package is self-contained and can be updated independently.

---

## Next Steps

1. **Test each robot type** to ensure launches work properly
2. **Configure sensors** in RViz for your use case
3. **Set up joystick** if using teleoperation
4. **Enable localization** for autonomous navigation
5. **Launch navigation nodes** when ready for path planning

For detailed configuration of individual components, see the README files in each package.
