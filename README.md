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

Follow this instruction to install ROS: [Install ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)</br>

Then, to install [Gazebo Harmonic (Gazebo Sim)](https://gazebosim.org/docs/harmonic/install_ubuntu/)


### 3. [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot.git)

```
bash cmd/setup_px4.sh
```

And check the results by:

```
./swam_uav.sh 
```

### 4. [MavSDK Python](https://github.com/mavlink/MAVSDK-Python.git)

```
bash cmd/setup_mavsdk.sh
```

### 5. [MavLink Router](https://github.com/intel/mavlink-router.git)

```
bash cmd/setup_mavrouter.sh
```

### 7. [QGroundControl Ground Control Station](https://github.com/mavlink/qgroundcontrol/releases) (Optional)


## Run program

### 1. Run all:
Terminal 1
```
./swam_uav.sh 
```
Terminal 2
```
conda activate uav
export QT_XCB_GL_INTEGRATION=none  #avoid crash
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

