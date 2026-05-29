#!/usr/bin/env bash

set -e

# ------------------------------------------
# Check Setup
# ------------------------------------------
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$HOME/ros2_ws/install/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ERROR: ROS 2 setup not found at $ROS_SETUP" >&2
    exit 1
fi

if [[ ! -f "$WS_SETUP" ]]; then
    echo "ERROR: Workspace setup not found at $WS_SETUP — run 'colcon build' first" >&2
    exit 1
fi

# shellcheck source=/opt/ros/jazzy/setup.bash
source "$ROS_SETUP"
# shellcheck source=/home/fire/ros2_ws/install/setup.bash
source "$WS_SETUP"

# Must match ROS_DOMAIN_ID used in the interactive shell / rqt / ros2 CLI
export ROS_DOMAIN_ID=30

# ------------------------------------------
# Run one
# ------------------------------------------

# Real
# exec ros2 launch warrior_bringup warrior_real.launch.py

#  Tele-op
exec ros2 launch warrior_bringup warrior_swerve_teleop.launch.py

# Gazebo
# export LIBGL_ALWAYS_SOFTWARE=1
# source /opt/ros/humble/setup.bash
# source ~/ros2_ws/install/setup.bash
# WARRIOR_GAZEBO_SHARE="$(ros2 pkg prefix warrior_gazebo)/share/warrior_gazebo"
# export IGN_GAZEBO_RESOURCE_PATH="${WARRIOR_GAZEBO_SHARE}/models:${WARRIOR_GAZEBO_SHARE}/worlds${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
# WORLD_NAME="${1:-competition.world}"
# ros2 launch warrior_bringup swerve_sim.launch.py world_name:="$WORLD_NAME"