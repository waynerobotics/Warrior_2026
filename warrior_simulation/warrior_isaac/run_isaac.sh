#!/usr/bin/env bash
# Enter only this project's Isaac environment, including when called from ROS.
set -euo pipefail
runtime_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime_project_root="$(cd -- "${runtime_script_dir}/../.." && pwd)"
runtime_pixi="$(type -P pixi || true)"
if [[ -z "${runtime_pixi}" ]]; then
    echo "Pixi is required. Install Pixi, then run 'pixi run install-isaac' in this project." >&2
    exit 2
fi

# An enclosing `pixi run` may have sourced the Python 3.12 ROS workspace. Its
# paths and activation markers must not reach the separate Python 3.11 runtime.
# Keep display, GPU-selection, and caller-provided NVIDIA EULA settings intact.
while IFS= read -r runtime_variable; do
    case "${runtime_variable}" in
        ROS_DOMAIN_ID) ;;
        PYTHON*|LD_*|CONDA*|_CONDA*|_CE_CONDA|_CE_M|MAMBA*|VIRTUAL_ENV*|\
        PIXI_PROJECT_*|PIXI_WORKSPACE_*|PIXI_ENVIRONMENT_*|PIXI_IN_SHELL|PIXI_DEFAULT_ENVIRONMENT|\
        AMENT_*|COLCON_*|ROS_*|RMW_IMPLEMENTATION|CMAKE_PREFIX_PATH|\
        QT_PLUGIN_PATH|QT_QPA_PLATFORM_PLUGIN_PATH|QML2_IMPORT_PATH|\
        CARB_APP_PATH|EXP_PATH|ISAAC_PATH|ISAAC_SIM_PATH|ISAAC_JUPYTER_PYTHON_PACKAGE|\
        PXR_PLUGINPATH_NAME|PXR_AR_DEFAULT_SEARCH_PATH|USD_INSTALL_ROOT|\
        OMNI_EXTENSIONS_PATH|OMNI_KIT_APP_DIR|BASH_ENV|ENV)
            unset "${runtime_variable}"
            ;;
    esac
done < <(compgen -e)
export PYTHONNOUSERSITE=1
export PATH="$(dirname -- "${runtime_pixi}"):/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

# The absolute manifest makes launches independent of the caller's directory.
# --locked prevents a runtime invocation from changing the reviewed lockfile;
# -x selects the executable even if a task called `python` is added later.
exec "${runtime_pixi}" run --manifest-path "${runtime_project_root}/pixi.toml" \
    -e isaac --locked -x python "${runtime_script_dir}/runtime.py" "$@"
