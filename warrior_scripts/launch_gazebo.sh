#!/bin/bash
# Launch Gazebo Fortress with the warrior empty world via ros2.
# LIBGL_ALWAYS_SOFTWARE=1 dodges a WSLg/OGRE-Next crash in the Sensors plugin.
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
WARRIOR_GAZEBO_SHARE="$(ros2 pkg prefix warrior_gazebo)/share/warrior_gazebo"
export IGN_GAZEBO_RESOURCE_PATH="${WARRIOR_GAZEBO_SHARE}/models:${WARRIOR_GAZEBO_SHARE}/worlds${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
WORLD="${WARRIOR_GAZEBO_SHARE}/worlds/competition.world"
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$WORLD"
