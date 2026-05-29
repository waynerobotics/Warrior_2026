#!/bin/bash
# Usage:
#   ./pub_swerve_cmd.sh [swerve_id] [steer_rad] [drive_rad_s] [rate_hz]
#
#   ./pub_swerve_cmd.sh front  0.785  32.0  10
#   ./pub_swerve_cmd.sh right  0.47   32.0   5
#   ./pub_swerve_cmd.sh left   0.57   32.0  10

set -e

SWERVE_ID="${1:-left}"
STEER_RAD="${2:-0.0}"
DRIVE_RAD_S="${3:-0.0}"
RATE="${4:-10}"

echo "Publishing to /warrior_swerve_command"
echo "  swerve_id           : $SWERVE_ID"
echo "  steer_position_rad  : $STEER_RAD"
echo "  drive_velocity_rad_s: $DRIVE_RAD_S"
echo "  rate                : ${RATE}Hz"
echo "---"

ros2 topic pub --rate "$RATE" /warrior_swerve_command warrior_msgs/msg/SwerveCmd \
  "{swerve_id: '$SWERVE_ID', steer_position_rad: $STEER_RAD, drive_velocity_rad_s: $DRIVE_RAD_S, stamp: {sec: 0, nanosec: 0}}"