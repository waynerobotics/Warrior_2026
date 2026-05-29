#!/usr/bin/env bash
# build.sh — configure and compile multitask_infer on Jetson / Linux
#
# Usage:
#   ./build.sh                          # Release build
#   ./build.sh Debug                    # Debug build
#   ./build.sh Release /opt/tensorrt    # Explicit TensorRT root
#
# All extra arguments after the first two are forwarded to cmake.

set -euo pipefail

BUILD_TYPE="${1:-Release}"
TRT_ROOT="${2:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

CMAKE_EXTRA_ARGS=()
if [[ -n "${TRT_ROOT}" ]]; then
    CMAKE_EXTRA_ARGS+=("-DTENSORRT_ROOT=${TRT_ROOT}")
fi
# Forward any remaining arguments (e.g. -DCMAKE_VERBOSE_MAKEFILE=ON)
shift 2 2>/dev/null || true
CMAKE_EXTRA_ARGS+=("$@")

echo "==> Build type : ${BUILD_TYPE}"
echo "==> Build dir  : ${BUILD_DIR}"
[[ -n "${TRT_ROOT}" ]] && echo "==> TRT root   : ${TRT_ROOT}"

mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake "${SCRIPT_DIR}" \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    "${CMAKE_EXTRA_ARGS[@]}"

make -j"$(nproc)"

echo ""
echo "==> Binary: ${BUILD_DIR}/multitask_infer"
echo ""
echo "Example usage:"
echo "  ${BUILD_DIR}/multitask_infer --engine /path/to/model.trt"
echo "  ${BUILD_DIR}/multitask_infer --engine model.trt --camera 0 --save out.mp4"
echo "  ${BUILD_DIR}/multitask_infer --engine model.trt --source /dev/video2 --score 0.25 --morph"
