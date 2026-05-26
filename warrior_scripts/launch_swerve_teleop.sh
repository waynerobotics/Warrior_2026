#!/usr/bin/env bash
# launch_swerve_teleop.sh
# Launches the Warrior swerve teleop stack:
#   joy_node → joy_swerve → twist_to_motor → motor_manager
#
# Usage:
#   ./launch_swerve_teleop.sh          # run interactively
#   Run on startup via systemd — see README.md

set -e

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

exec ros2 launch warrior_bringup warrior_swerve_teleop.launch.py
