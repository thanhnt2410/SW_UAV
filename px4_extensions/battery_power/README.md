# PX4 Gazebo battery extension

Tài liệu đầy đủ về cài đặt, mô hình pin và công thức tính năng lượng:
[ENERGY_SIMULATION.md](ENERGY_SIMULATION.md).

This directory is the repository-owned source of the simulated x500 battery
telemetry extension. It packages two pieces that otherwise live only in an
ignored `dependencies/PX4-Autopilot` working tree:

- `MotorPowerSystem`, which converts rotor speed to motor power, current,
  voltage, charge, and state of charge in Gazebo;
- a PX4 `GZBridge` subscription that publishes Gazebo voltage/current through
  PX4 `battery_status`, MAVLink, and MAVSDK.

`cmd/setup_px4.sh` calls `install.py` before building PX4. The installer is
idempotent and reads all electrical model values from
`config/uav_config.yaml:gazebo_battery_model`.

`swam_uav.sh` also runs the installer before launching PX4. Therefore, after
changing `gazebo_battery_model`, the next swarm launch updates `model.sdf` and
the normal PX4/Gazebo build step rebuilds anything affected by the change.

Manual installation into an existing PX4 checkout:

```bash
python3 px4_extensions/battery_power/install.py \
  --px4-dir dependencies/PX4-Autopilot \
  --config config/uav_config.yaml
make -C dependencies/PX4-Autopilot px4_sitl
```

After starting `gz_x500`, useful checks are:

```bash
gz topic -e -n 1 -t /model/x500/motor_power
gz topic -e -n 1 -t /model/x500/battery/linear_battery/state
```
