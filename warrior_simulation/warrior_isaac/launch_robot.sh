#!/usr/bin/env bash
# Isaac owns its Python ABI and GPU libraries; ROS runs in a separate process.
set -euo pipefail
robot_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
robot_isaac_root="${ISAAC_SIM_PATH:-${HOME}/isaacsim}"
if [[ ! -x "${robot_isaac_root}/python.sh" ]]; then
    echo "Set ISAAC_SIM_PATH to an Isaac Sim 5.1 installation containing python.sh." >&2
    exit 2
fi
exec env -u PYTHONPATH -u LD_LIBRARY_PATH -u PYTHONHOME -u PYTHONEXE \
    -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
    "${robot_isaac_root}/python.sh" "${robot_script_dir}/run_robot.py" "$@"
