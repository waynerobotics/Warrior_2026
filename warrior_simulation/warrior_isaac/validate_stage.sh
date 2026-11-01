#!/usr/bin/env bash
# Use Isaac Sim's bundled OpenUSD without launching Kit or requiring a GPU.
set -euo pipefail
course_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
course_isaac_root="${ISAAC_SIM_PATH:-${HOME}/isaacsim}"
if [[ ! -x "${course_isaac_root}/python.sh" ]]; then
    echo "Set ISAAC_SIM_PATH to an Isaac Sim installation containing python.sh." >&2
    exit 2
fi
course_usd_libs=""
for course_candidate in "${course_isaac_root}"/extscache/omni.usd.libs-*; do
    if [[ -d "${course_candidate}/pxr" && -d "${course_candidate}/bin" ]]; then
        course_usd_libs="${course_candidate}"
        break
    fi
done
if [[ -z "${course_usd_libs}" ]]; then
    echo "Bundled OpenUSD library not found; run validate_stage.py using a Python with pxr installed." >&2
    exit 2
fi
exec env -u PYTHONHOME -u PYTHONEXE -u CONDA_PREFIX -u CONDA_DEFAULT_ENV \
    PYTHONPATH="${course_usd_libs}" LD_LIBRARY_PATH="${course_usd_libs}/bin" \
    "${course_isaac_root}/python.sh" "${course_script_dir}/validate_stage.py" "$@"
