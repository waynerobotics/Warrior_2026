#!/usr/bin/env bash
set -euo pipefail

# Launch Gazebo Fortress with the warrior swerve robot in the competition world.
# LIBGL_ALWAYS_SOFTWARE=1 dodges a WSLg/OGRE-Next crash in the Sensors plugin.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WARRIOR_REPO="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORLD_NAME="${1:-competition.world}"

PIXI_EXECUTABLE="${PIXI_EXECUTABLE:-}"
if [[ -z "${PIXI_EXECUTABLE}" ]]; then
    PIXI_EXECUTABLE="$(command -v pixi || true)"
fi
if [[ -z "${PIXI_EXECUTABLE}" && -x "${HOME}/.pixi/bin/pixi" ]]; then
    PIXI_EXECUTABLE="${HOME}/.pixi/bin/pixi"
fi
if [[ -z "${PIXI_EXECUTABLE}" ]]; then
    echo "ERROR: pixi was not found; install it or set PIXI_EXECUTABLE" >&2
    exit 1
fi
if [[ ! -f "${WARRIOR_REPO}/install/pixi/setup.bash" ]]; then
    echo "ERROR: Pixi ROS overlay is missing; run 'pixi run build' in ${WARRIOR_REPO}" >&2
    exit 1
fi

cd "${WARRIOR_REPO}"
exec "${PIXI_EXECUTABLE}" run --locked bash -c '
    source install/pixi/setup.bash
    export LIBGL_ALWAYS_SOFTWARE=1
    WARRIOR_GAZEBO_SHARE="$(ros2 pkg prefix warrior_gazebo)/share/warrior_gazebo"
    export IGN_GAZEBO_RESOURCE_PATH="${WARRIOR_GAZEBO_SHARE}/models:${WARRIOR_GAZEBO_SHARE}/worlds${IGN_GAZEBO_RESOURCE_PATH:+:${IGN_GAZEBO_RESOURCE_PATH}}"
    exec ros2 launch warrior_bringup swerve_sim.launch.py world_name:="$1"
' -- "${WORLD_NAME}"
