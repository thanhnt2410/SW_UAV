#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f /.dockerenv ]; then
    export SW_UAV_DOCKER=1
fi

pkill -f px4
pkill -f gz
pkill -f gazebo
pkill -f ruby

if [ "${SW_UAV_DOCKER:-}" = "1" ]; then
    git config --global --add safe.directory "${SCRIPT_DIR}/dependencies/PX4-Autopilot" || true
fi

python3 "${SCRIPT_DIR}/px4_extensions/battery_power/install.py" \
    --px4-dir "${SCRIPT_DIR}/dependencies/PX4-Autopilot" \
    --config "${SCRIPT_DIR}/config/uav_config.yaml"

cd "${SCRIPT_DIR}/dependencies/PX4-Autopilot"

if [ "${SW_UAV_DOCKER:-}" = "1" ]; then
    LOG_DIR="${SCRIPT_DIR}/logs/docker"
    mkdir -p "${LOG_DIR}"

    echo "[docker] PX4/Gazebo logs:"
    echo "[docker]   ${LOG_DIR}/px4-main.log"
    echo "[docker]   ${LOG_DIR}/px4-uav-1.log ... ${LOG_DIR}/px4-uav-5.log"

    make px4_sitl gz_x500 > "${LOG_DIR}/px4-main.log" 2>&1 &
    sleep 10
    for i in {1..5}
    do
        PX4_GZ_STANDALONE=1 \
        PX4_SYS_AUTOSTART=4001 \
        PX4_GZ_MODEL=x500 \
        PX4_GZ_MODEL_POSE="$((i*3)),0,0.3,0,0,0" \
        ./build/px4_sitl_default/bin/px4 -i "$i" > "${LOG_DIR}/px4-uav-${i}.log" 2>&1 &
        sleep 1
    done
    exit 0
fi

gnome-terminal -- bash -c "make px4_sitl gz_x500; exec bash"
sleep 10
for i in {1..5}
do
    gnome-terminal -- bash -c "
    PX4_GZ_STANDALONE=1 \
    PX4_SYS_AUTOSTART=4001 \
    PX4_GZ_MODEL=x500 \
    PX4_GZ_MODEL_POSE="$((i*3)),0,0.3,0,0,0" \
    ./build/px4_sitl_default/bin/px4 -i $i;
    sleep 1
    exec bash"
done
