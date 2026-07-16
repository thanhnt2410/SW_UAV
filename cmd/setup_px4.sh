#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_DIR="${ROOT_DIR}/dependencies/PX4-Autopilot"
PX4_REVISION="44c128aade5984f4824225145ab8b58000fcd6dd"

echo '=========== Setting up pinned PX4 with Gazebo battery telemetry ==========='

mkdir -p "${ROOT_DIR}/dependencies"

if [ ! -d "${PX4_DIR}/.git" ]; then
    echo '1. Cloning PX4'
    git clone https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}"
    git -C "${PX4_DIR}" checkout "${PX4_REVISION}"
    git -C "${PX4_DIR}" submodule update --init --recursive
else
    echo '1. Using existing PX4 checkout (no local files are deleted)'
fi

echo '2. Running default Ubuntu setup script...'
# Chạy script mặc định để cài các thư viện lõi (compiler, Python deps...)
bash "${PX4_DIR}/Tools/setup/ubuntu.sh"

echo '3. Installing GStreamer dependencies...'
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y

echo '4. Installing MotorPowerSystem and the PX4-Gazebo battery bridge...'
python3 "${ROOT_DIR}/px4_extensions/battery_power/install.py" \
    --px4-dir "${PX4_DIR}" \
    --config "${ROOT_DIR}/config/uav_config.yaml"

echo '5. Building PX4 SITL and Gazebo plugins...'
make -C "${PX4_DIR}" px4_sitl

echo '=========== Done ==========='
