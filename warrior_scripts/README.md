# Warrior 2026 — Setup

ROS 2 Humble + Gazebo Fortress on Ubuntu 22.04 (WSL2).

## Install

```bash
sudo bash installWarriorDependencies.sh
source ~/.bashrc
ros_rebuild
```

`installWarriorDependencies.sh` adds the ROS 2 apt repo and installs Humble
desktop, Gazebo Fortress (`ros-humble-ros-gz`), `ros2_control`, and the
bashrc lines (`source /opt/ros/humble/setup.bash`, `sorce`, `ros_rebuild`).

`ros_rebuild` is an alias for `cd ~/ros2_ws && colcon build && source install/setup.bash`.

## Launch Gazebo

```bash
bash launch_gazebo.sh
```
