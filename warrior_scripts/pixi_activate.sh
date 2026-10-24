#!/usr/bin/env bash
# Sourced by Pixi whenever the Warrior environment is activated.

if [[ -n "${PIXI_PROJECT_ROOT:-}" ]]; then
    WARRIOR_PIXI_ROOT="${PIXI_PROJECT_ROOT}"
elif [[ -n "${PIXI_WORKSPACE_ROOT:-}" ]]; then
    WARRIOR_PIXI_ROOT="${PIXI_WORKSPACE_ROOT}"
else
    WARRIOR_PIXI_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

# Pixi uses a dedicated overlay so an older apt/Docker-era build is not loaded
# automatically. Do not activate an apt-installed ROS in the same shell.
if [[ -f "${WARRIOR_PIXI_ROOT}/install/pixi/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WARRIOR_PIXI_ROOT}/install/pixi/setup.bash"
fi

export WARRIOR_WORKSPACE="${WARRIOR_PIXI_ROOT}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${WARRIOR_PIXI_ROOT}/log/ros}"
mkdir -p "${ROS_LOG_DIR}"
unset WARRIOR_PIXI_ROOT

