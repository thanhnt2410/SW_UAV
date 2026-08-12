#!/usr/bin/env bash
set -euo pipefail

IMAGE="${SW_UAV_IMAGE:-sw-uav:latest}"
NAME="${SW_UAV_CONTAINER:-sw-uav}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

xhost +local:docker >/dev/null 2>&1 || true
xhost +local:root >/dev/null 2>&1 || true

if ! sudo docker ps -a --format '{{.Names}}' | grep -Fxq "${NAME}"; then
    sudo docker run -dit \
        --gpus all \
        --net=host \
        --ipc=host \
        -e DISPLAY="${DISPLAY:-:0}" \
        -e QT_X11_NO_MITSHM=1 \
        -e QT_XCB_GL_INTEGRATION=xcb_glx \
        -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video,display \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "${ROOT_DIR}:/app" \
        -w /app \
        --name "${NAME}" \
        "${IMAGE}" \
        bash >/dev/null
elif [ "$(sudo docker inspect -f '{{.State.Running}}' "${NAME}")" != "true" ]; then
    sudo docker start "${NAME}" >/dev/null
fi

sudo docker exec "${NAME}" git config --global --add safe.directory /app/dependencies/PX4-Autopilot
sudo docker exec -it "${NAME}" bash
