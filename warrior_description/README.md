# Warrior Description

## User Guide

1. First colcon build in your workspace, e.g.,

```bash
cd ~/ros2_ws && colcon build --package-up-to warrior_description --symlink-install
```

2. To visualize the warrior robot in Rviz, run:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch warrior_description display.launch.py
```


## Visualization

The Visualization of the Warrior robot is displayed as below:

<p align="center">
<img src="./docs/rviz.png" alt="robot" height="800">
</p>

