
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