#!/usr/bin/env bash
# Keep Isaac's Python and shared libraries separate from the Pixi ROS environment.
set -euo pipefail
course_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${course_script_dir}/run_isaac.sh" course "$@"
