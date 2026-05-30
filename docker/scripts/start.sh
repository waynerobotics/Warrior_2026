#!/bin/bash

ARCH=$(uname -m)
IMAGE_NAME=warrior-igvc-2026

case "${ARCH}" in
    x86_64)
        PLATFORM="x86_64"
        GPU_ARGS="--gpus all"
        ;;
    aarch64|arm64)
        if [ -f /etc/nv_tegra_release ]; then
            PLATFORM="jetson"
            GPU_ARGS="--runtime=nvidia"
        else
            PLATFORM="arm64"
            GPU_ARGS=""
        fi
        ;;
    *)
        echo "[ERROR] Unsupported architecture: ${ARCH}"
        exit 1
        ;;
esac

docker run -it --rm \
  --user wrclub \
  ${GPU_ARGS} \
  --net host \
  --pid host \
  --ipc host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/input:/dev/input \
  ${IMAGE_NAME}:${PLATFORM}