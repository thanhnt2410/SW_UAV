# Swarm UAVs project

## Hardware requirements:

Ubuntu 22.04 with minimum 16GB RAM and 60GB available ROM, and external GPU (optional)

ROS2-Humble and python 3.10

## Setups:

### 0. [Miniconda](https://docs.anaconda.com/free/miniconda/miniconda-install/)

```
bash cmd/setup_miniconda.sh
```
### 1. Install conda environment (uav)

```
conda env create -f environment.yml
conda activate uav
#pip install mavsdk asyncio --force
```
### 2. Gazebo ROS2:

To install [Gazebo Harmonic (Gazebo Sim)](https://gazebosim.org/docs/harmonic/install_ubuntu/)

Follow this instruction to install ROS (Optional): [Install ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)</br>

### 3. [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot.git)

```bash
bash cmd/setup_px4.sh
```

#### Gazebo battery and motor-power extension

`setup_px4.sh` tự cài `MotorPowerSystem`, nối dữ liệu pin vào `GZBridge` và
build PX4. Thông số pin được chỉnh trong `config/uav_config.yaml` tại hai mục
`gazebo_battery_model` và `px4_parameters`; không cần sửa source PX4.
Chi tiết: [ENERGY_SIMULATION.md](px4_extensions/battery_power/ENERGY_SIMULATION.md).

Sau khi đổi cấu hình, chạy lại `./swam_uav.sh` để tự đồng bộ và build. Kiểm tra
dữ liệu bằng:

```bash
gz topic -e -n 1 -t /model/x500/motor_power
gz topic -e -n 1 -t /model/x500/battery/linear_battery/state
```

### 7. [QGroundControl Ground Control Station](https://github.com/mavlink/qgroundcontrol/releases) (Optional)

## Using Docker

Docker image đã bao gồm Ubuntu 22.04, Gazebo Harmonic, môi trường build PX4,
Python và các thư viện của ứng dụng. Host không cần cài CUDA Toolkit hoặc
cuDNN. Nếu dùng GPU NVIDIA, host vẫn cần NVIDIA driver và NVIDIA Container
Toolkit.

### 1. Prerequisites

Cài Docker Engine bằng script (chỉ hỗ trợ Ubuntu 22.04):

```bash
bash cmd/install_docker.sh
```

Sau khi cài đặt, đăng xuất rồi đăng nhập lại hoặc chạy `newgrp docker`. Với GPU
NVIDIA, kiểm tra Docker có truy cập được GPU:

```bash
docker run --rm --gpus all \
  nvidia/cuda:12.4.1-cudnn-runtime-ubuntu22.04 \
  nvidia-smi
```

### 2. Build image

Chạy từ thư mục gốc của repository:

```bash
docker build --progress=plain \
  -t sw-uav:latest \
  -f docker/Dockerfile .
```

Lần build đầu có thể mất nhiều thời gian và cần nhiều dung lượng trống. Các
lần build tiếp theo sẽ sử dụng Docker cache.

### 3. Open the development container

```bash
chmod +x docker/shell.sh
./docker/shell.sh
```

Script mount toàn bộ repository vào `/app`, dùng network của host và chuyển
X11/GPU vào container. Các thay đổi source trong container cũng xuất hiện ngay
trên host.

Script mặc định chạy với `--gpus all`. Trên máy không có GPU NVIDIA, bỏ tùy
chọn này và các biến môi trường `NVIDIA_*` trong `docker/shell.sh` trước khi
chạy.

### 4. Run inside the container

Build PX4 SITL lần đầu:

```bash
cd /app/dependencies/PX4-Autopilot
make px4_sitl gz_x500
```

Chạy giao diện trực tiếp:

```bash
python src/app.py
```

Hoặc chạy entry point chính với biến báo cho ứng dụng biết nó đang ở trong
container:

```bash
SW_UAV_DOCKER=1 python src/main.py
```

Không cần cài `gnome-terminal` trong container. Nếu chạy `python src/main.py`
mà không đặt `SW_UAV_DOCKER=1`, chương trình sẽ đi vào nhánh dành cho host và
báo `gnome-terminal: not found`.

### 5. Run the complete simulation

Chạy script điều phối trên **host**, không chạy bên trong container:

```bash
./swam_uav.sh
```

Script sẽ sử dụng image `sw-uav:latest`, chuẩn bị PX4 và mở các terminal cho mô
phỏng. Có thể đổi tên image/container bằng các biến `SW_UAV_IMAGE` và
`SW_UAV_CONTAINER`.


## Run program without Docker

### 1. Run all:
Terminal 1
```
./swam_uav.sh 
```
Terminal 2
```
conda activate uav
python src/main.py
```

### 2. Run only UI

```
python src/app.py
```

```
python src/interface_base.py
```

```
python src/interface_map.py

```
## Debug

1. Check opening ports
TCP

```

    netstat -ltnp

```

UDP

```

    netstat -lunp

```

UARTs

```

     ls /dev/tty*

````
2. Debug programs
```Interface
   gdb --agrs python src/app.py
````

## Collaborators:
