#!/bin/bash
set -e
cd ~/ros2_ws
colcon build --packages-up-to warrior_bringup --symlink-install
