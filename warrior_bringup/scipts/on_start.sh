#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WARRIOR_REPO="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WARRIOR_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"

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
export WARRIOR_DOMAIN_ID
exec "${PIXI_EXECUTABLE}" run --locked bash -c '
    source install/pixi/setup.bash
    export ROS_DOMAIN_ID="${WARRIOR_DOMAIN_ID}"
    exec ros2 launch warrior_bringup warrior_swerve_teleop.launch.py
'
