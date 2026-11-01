#!/usr/bin/env bash
# Keep Isaac's Python and shared libraries separate from the Pixi ROS environment.
set -euo pipefail
course_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
course_isaac_root="${ISAAC_SIM_PATH:-${HOME}/isaacsim}"
if [[ ! -x "${course_isaac_root}/python.sh" ]]; then
    echo "Set ISAAC_SIM_PATH to an Isaac Sim installation containing python.sh." >&2
    exit 2
fi
exec env -u PYTHONPATH -u LD_LIBRARY_PATH -u PYTHONHOME -u PYTHONEXE \
    -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
    "${course_isaac_root}/python.sh" "${course_script_dir}/run_course.py" "$@"
