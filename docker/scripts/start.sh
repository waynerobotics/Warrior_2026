#!/bin/bash

ARCH=$(uname -m)
IMAGE_NAME=warrior-igvc-2026

case "${ARCH}" in
    x86_64)
        PLATFORM="x86_64"
        ;;
    aarch64|arm64)
        # Jetson
        if [ -f /etc/nv_tegra_release ]; then
            PLATFORM="jetson"
        else
            # Standard ARM64
            PLATFORM="arm64"
        fi
        ;;
    *)
        echo "[ERROR] Unsupported architecture: ${ARCH}"
        exit 1
        ;;
esac

docker run -it --rm \
  --user $(id -u):$(id -g) \
  --gpus all \
  --net host \
  --pid host \
  --ipc host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device=/dev/input:/dev/input \
  ${IMAGE_NAME}:${PLATFORM} 