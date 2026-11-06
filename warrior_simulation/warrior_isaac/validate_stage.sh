#!/usr/bin/env bash
# Use this project's Isaac OpenUSD cache without launching Kit or a GPU.
set -euo pipefail
course_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "${course_script_dir}/run_isaac.sh" validate "$@"
