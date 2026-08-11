# #!/usr/bin/env bash
# set -e
# set -o pipefail

# cd /app

# mkdir -p "${XDG_RUNTIME_DIR:-/tmp/runtime-root}" "${YOLO_CONFIG_DIR:-/tmp/Ultralytics}"
# chmod 700 "${XDG_RUNTIME_DIR:-/tmp/runtime-root}" || true

# if [ -f /opt/ros/humble/setup.bash ]; then
#     source /opt/ros/humble/setup.bash
# fi

# source /opt/conda/etc/profile.d/conda.sh
# conda activate uav

# git config --global --add safe.directory /app/dependencies/PX4-Autopilot || true
# export SW_UAV_DOCKER=1

# # ./swam_uav.sh
# # mkdir -p /app/logs/docker
# # python src/main.py 2>&1 | tee /app/logs/docker/app.log
tail -f /dev/null