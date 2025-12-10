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

2. Add source for Gazebo:

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update && sudo apt install gz-harmonic \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-gz-ros2-control


```


2. Colcon build:
```bash 
colcon build --package-up-to warrior_bringup --symlink-install
```

