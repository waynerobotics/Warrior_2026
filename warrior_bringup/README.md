# Warrior Bringup

## User Guide

1. Install all the dependent packages:

```bash
sudo apt update && sudo apt install \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-controller-interface \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-joint-state-publisher-gui
```


2. Colcon build:
```bash 
colcon build --package-up-to warrior_bringup --symlink-install
```

