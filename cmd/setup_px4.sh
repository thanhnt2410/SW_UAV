echo '===========Setting up PX4==========='
cd dependencies/
rm -rf PX4-Autopilot/

echo 'Cloning PX4'
git clone https://github.com/PX4/PX4-Autopilot.git --recursive

bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
git submodule update --init --recursive

echo 'Building PX4'
cd PX4-Autopilot/

make clean
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y

make px4_sitl_default gazebo-classic

cd ..
echo 'Done'



#!/bin/bash
echo '=========== Setting up PX4 (1.14) with Gazebo Harmonic ==========='

cd dependencies/
rm -rf PX4-Autopilot/

echo '1. Cloning PX4 - Branch release/1.14'
git clone -b release/1.14 https://github.com/PX4/PX4-Autopilot.git --recursive

cd PX4-Autopilot/

echo '2. Running default Ubuntu setup script...'
# Chạy script mặc định để cài các thư viện lõi (compiler, Python deps...)
bash ./Tools/setup/ubuntu.sh

echo '3. Installing GStreamer dependencies...'
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y

make px4_sitl
# Lệnh build mới: Dùng 'gz' thay vì 'gazebo-classic', x500 là model drone mặc định

cd ..
echo '=========== Done ==========='