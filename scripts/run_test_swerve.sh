#!/usr/bin/env bash
# Source ROS 2 + this workspace's install overlay, then launch the multi-wheel
# swerve test. Extra args are forwarded to `ros2 launch`.
#
#   ./scripts/run_test_swerve.sh
#   ./scripts/run_test_swerve.sh wheels:=2,3       # subset of wheels
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE_ROOT=$(cd "$SCRIPT_DIR/../../.." && pwd)

source /opt/ros/jazzy/setup.bash

if [[ ! -f "$WORKSPACE_ROOT/install/setup.bash" ]]; then
    echo "Workspace not built at $WORKSPACE_ROOT/install. Run:" >&2
    echo "  (cd $WORKSPACE_ROOT && colcon build --symlink-install)" >&2
    exit 1
fi
source "$WORKSPACE_ROOT/install/setup.bash"

exec ros2 launch warrior_serial test_swerve_module.launch.py "$@"
