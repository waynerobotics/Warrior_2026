#!/usr/bin/env bash
# Isaac owns its Python ABI and GPU libraries; ROS runs in a separate process.
set -euo pipefail
robot_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${robot_script_dir}/run_isaac.sh" robot "$@"
