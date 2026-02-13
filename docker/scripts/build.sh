#!/bin/bash
set -e

# ======================================================
# warrior Docker Build Script
# - Auto arch detection (x86 / jetson)
# - Auto Dockerfile & tag selection
# - Auto ssh-agent handling
# ======================================================

# -------- Detect Architecture --------
ARCH=$(uname -m)

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

echo "[INFO] Detected architecture: ${ARCH} -> ${PLATFORM}"

# -------- Image Names --------
IMAGE_NAME=warrior-igvc-2026
IMAGE_TAG=${PLATFORM}

# -------- Dockerfiles --------
DOCKERFILE=../dockerfile/${PLATFORM}.Dockerfile

# -------- Enable BuildKit --------
export DOCKER_BUILDKIT=1

# -------- Start SSH agent --------
SSH_KEY="$HOME/.ssh/id_rsa"
SSH_OPTION=""

if [ -f "$SSH_KEY" ]; then
    echo "[INFO] Starting ssh-agent and adding key $SSH_KEY"
    eval "$(ssh-agent -s)"
    ssh-add "$SSH_KEY"
    echo "[DEBUG] SSH_AUTH_SOCK=$SSH_AUTH_SOCK"

    if [ -n "$SSH_AUTH_SOCK" ] && [ -S "$SSH_AUTH_SOCK" ]; then
        chmod 666 "$SSH_AUTH_SOCK"
        echo "[DEBUG] Fixed SSH socket permissions"
    fi

    SSH_OPTION="--ssh default"
else
    echo "[INFO] No SSH key found at $SSH_KEY, building WITHOUT SSH"
fi

# -------- Sanity Check --------
if [ ! -f "${DOCKERFILE}" ]; then
    echo "[ERROR] Dockerfile not found: ${DOCKERFILE}"
    exit 1
fi


# ======================================================
# Build Base Image
# ======================================================
echo "=============================="
echo "Building Warrior Docker Image:"
echo "  ${IMAGE_NAME}:${IMAGE_TAG}"
echo "  Dockerfile: ${DOCKERFILE}"
echo "=============================="

docker build \
    ${SSH_OPTION} \
    -f ${DOCKERFILE} \
    -t ${IMAGE_NAME}:${IMAGE_TAG} .


# ======================================================
# Done
# ======================================================
echo "=============================="
echo "Docker Build Completed ✅"
echo "Platform:   ${PLATFORM}"
echo "Docker Image: ${IMAGE_NAME}:${IMAGE_TAG}"
echo "=============================="