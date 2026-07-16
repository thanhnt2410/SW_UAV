# Mô phỏng và tính năng lượng tiêu thụ UAV

Tài liệu này mô tả cách cài đặt extension pin Gazebo, nguồn gốc dữ liệu pin và
cách dự án tính năng lượng tiêu thụ của UAV.

## 1. Kiến trúc dữ liệu

```text
Tốc độ rotor trong Gazebo
          ↓
MotorPowerSystem
          ↓ voltage, current, charge, percentage
/model/<uav>/battery/linear_battery/state
          ↓
PX4 GZBridge → battery_status
          ↓
MAVLink BATTERY_STATUS
          ↓
MAVSDK telemetry.battery()
          ↓
main_controller.py → Energy (Wh)
```

`MotorPowerSystem` sinh dữ liệu điện áp và dòng điện. `GZBridge` đưa hai giá
trị này vào PX4. PX4 ước lượng dung lượng đã dùng và phần trăm pin, sau đó gửi
qua MAVLink cho MAVSDK.

## 2. Cài đặt trên máy mới

Từ thư mục gốc của dự án, chạy:

```bash
bash cmd/setup_px4.sh
```

Script sẽ:

1. Clone đúng PX4 revision mà extension đã được kiểm thử.
2. Cài source `MotorPowerSystem` vào PX4.
3. Patch CMake và `GZBridge`.
4. Thêm plugin vào model `x500` theo cấu hình YAML.
5. Build PX4 SITL và `libMotorPowerSystem.so`.

Extension được đóng gói trong repository nên không cần sửa thủ công file bên
trong `dependencies/PX4-Autopilot`.

Khởi chạy swarm:

```bash
./swam_uav.sh
```

Script này chạy lại installer trước khi khởi động. Vì installer có tính
idempotent, chạy nhiều lần không tạo code hoặc cấu hình trùng lặp.

## 3. Cấu hình model pin Gazebo

Các thông số nằm trong `config/uav_config.yaml`:

```yaml
gazebo_battery_model:
  model: "x500"
  battery_name: "linear_battery"
  joint_names:
    - "rotor_0_joint"
    - "rotor_1_joint"
    - "rotor_2_joint"
    - "rotor_3_joint"
  motor_constant: 8.54858e-06
  moment_constant: 0.016
  rotor_velocity_slowdown: 10.0
  motor_efficiency: 0.82
  static_power: 8.0
  capacity: 5.0
  initial_charge: 5.0
  open_circuit_voltage_constant_coef: 16.8
  open_circuit_voltage_linear_coef: -2.4
  resistance: 0.02
  smooth_current_tau: 0.5
```

Ý nghĩa chính:

| Tham số | Đơn vị | Ý nghĩa |
|---|---:|---|
| `motor_efficiency` | 0–1 | Hiệu suất chuyển đổi điện năng sang cơ năng |
| `static_power` | W | Tải cố định của flight controller và cảm biến |
| `capacity` | Ah | Dung lượng danh định của pin |
| `initial_charge` | Ah | Điện lượng ban đầu khi bắt đầu mô phỏng |
| `open_circuit_voltage_constant_coef` | V | Điện áp hở mạch khi pin đầy |
| `open_circuit_voltage_linear_coef` | V | Độ thay đổi điện áp từ đầy đến cạn |
| `resistance` | Ω | Nội trở toàn bộ pack pin |
| `smooth_current_tau` | s | Hằng số thời gian của bộ lọc dòng điện |

Cấu hình mặc định tương ứng pin LiPo 4S 5000 mAh:

- điện áp đầy: 16.8 V;
- điện áp cạn theo model: 14.4 V;
- dung lượng: 5 Ah;
- tải tĩnh: 8 W.

Sau khi đổi các giá trị trên, chạy lại `./swam_uav.sh` để cập nhật model và
build phần thay đổi.

## 4. Cấu hình bộ ước lượng pin PX4

Các tham số PX4 cũng nằm trong `config/uav_config.yaml`:

```yaml
px4_parameters:
  simulation:
    BAT1_SOURCE: 0
    BAT1_N_CELLS: 4
    BAT1_CAPACITY: 4500.0
    BAT1_V_CHARGED: 4.05
    BAT1_V_EMPTY: 3.60
    BAT1_R_INTERNAL: 0.005
  real: {}
```

| Tham số | Ý nghĩa |
|---|---|
| `BAT1_SOURCE` | Nguồn dữ liệu pin; `0` là power module/Gazebo bridge |
| `BAT1_N_CELLS` | Số cell nối tiếp |
| `BAT1_CAPACITY` | Dung lượng PX4 sử dụng để ước lượng, đơn vị mAh |
| `BAT1_V_CHARGED` | Điện áp một cell khi đầy |
| `BAT1_V_EMPTY` | Điện áp một cell khi cạn |
| `BAT1_R_INTERNAL` | Nội trở của một cell, đơn vị Ω |

`capacity` của Gazebo dùng Ah, còn `BAT1_CAPACITY` dùng mAh. Model hiện dùng
pin 5000 mAh nhưng PX4 dùng 4500 mAh, tương ứng 90% dung lượng danh định có thể
sử dụng.

Sau khi UAV kết nối, `DroneService` tự đọc và áp dụng các tham số trên qua
MAVSDK. Có thể ghi đè riêng cho một UAV bằng mục `px4_parameters` của UAV đó.

Không đặt tham số pin mô phỏng vào mục `real` nếu chưa hiệu chỉnh pin và power
module thật.

## 5. Công thức công suất động cơ

Với mỗi rotor, plugin lấy tốc độ góc thực:

```text
ω = |joint_velocity| × rotor_velocity_slowdown
```

Lực đẩy:

```text
T = motor_constant × ω²
```

Mô-men:

```text
τ = moment_constant × T
```

Công suất điện của tất cả động cơ:

```text
P_motor = Σ(|τ × ω| / motor_efficiency)
```

Tổng công suất lấy từ pin:

```text
P_total = P_motor + static_power
```

## 6. Mô hình dòng điện, điện áp và dung lượng

Dòng điện thô:

```text
I_raw = P_total / V
```

Dòng được làm mượt theo `smooth_current_tau`, sau đó điện lượng còn lại được
cập nhật:

```text
Q_remaining = Q_remaining - I × Δt / 3600
SOC = Q_remaining / Q_capacity
```

Điện áp hở mạch:

```text
V_oc = V_full + V_delta × (1 - SOC)
```

Điện áp đầu cực sau khi tính sụt áp trên nội trở:

```text
V = V_oc - R_internal × I
```

Plugin công bố:

- `voltage`: điện áp đầu cực, V;
- `current`: dòng tiêu thụ, A;
- `charge`: điện lượng còn lại, Ah;
- `capacity`: dung lượng pin, Ah;
- `percentage`: trạng thái sạc, %.

## 7. Dữ liệu MAVSDK

Ứng dụng đọc:

```python
async for battery in system.telemetry.battery():
    voltage_v = battery.voltage_v
    current_a = battery.current_battery_a
    consumed_ah = battery.capacity_consumed_ah
    remaining = battery.remaining_percent
```

| Trường MAVSDK | Nguồn/ý nghĩa |
|---|---|
| `voltage_v` | Điện áp pack từ Gazebo qua PX4 |
| `current_battery_a` | Dòng tiêu thụ từ Gazebo qua PX4 |
| `capacity_consumed_ah` | PX4 tích phân dòng điện theo thời gian |
| `remaining_percent` | PX4 ước lượng dung lượng còn lại, giá trị 0–1 |

Lệnh sau chỉ đặt tần số nhận dữ liệu khoảng 10 Hz, không thay đổi đặc tính pin:

```python
await system.telemetry.set_rate_battery(10.0)
```

## 8. Công thức Energy trong ứng dụng

Năng lượng được tích phân từ điện áp và dòng điện:

```text
Energy_Wh = Σ(V_i × I_i × Δt_i / 3600)
```

Trong đó:

- `V_i`: điện áp tại mẫu thứ `i`, V;
- `I_i`: dòng điện tại mẫu thứ `i`, A;
- `Δt_i`: thời gian giữa hai mẫu, giây;
- chia 3600 để đổi từ watt-giây sang watt-giờ.

Code tương ứng trong `src/main_controller.py`:

```python
dt_s = min(max(now - last_sample_time, 0.0), 2.0)
energy_wh += voltage_v * current_a * dt_s / 3600.0
```

Khoảng lấy mẫu được giới hạn tối đa 2 giây để một khoảng mất telemetry dài
không làm năng lượng tăng sai lệch quá lớn.

Với nhiều UAV, năng lượng của một lượt chạy là tổng năng lượng của toàn đội:

```text
Energy_team = Σ Energy_uav
```

Nếu chạy thuật toán nhiều lần, giá trị hiển thị trong bảng là trung bình của
các lượt.

Nếu một UAV không cung cấp mẫu điện áp/dòng hợp lệ, ứng dụng dùng tải dự phòng
120 W:

```text
Energy_fallback = 120 × flight_time_seconds / 3600
```

Ở chế độ chỉ lập kế hoạch (`planning_only`), UAV không bay nên Energy bằng 0.

Energy cũng tham gia vào điểm đánh giá thuật toán:

```text
Score = Cost + 0.1 × Time + 10 × Energy
```

Score càng thấp càng tốt.

## 9. Kiểm tra hoạt động

Sau khi `gz_x500` chạy, kiểm tra công suất động cơ:

```bash
gz topic -e -n 1 -t /model/x500/motor_power
```

Kiểm tra trạng thái pin:

```bash
gz topic -e -n 1 -t /model/x500/battery/linear_battery/state
```

Kết quả pin cần có `voltage > 0` và `current >= 0`.

Kiểm tra plugin đã được build:

```bash
find dependencies/PX4-Autopilot/build -name 'libMotorPowerSystem.so'
```

Cài hoặc đồng bộ extension thủ công:

```bash
python3 px4_extensions/battery_power/install.py \
  --px4-dir dependencies/PX4-Autopilot \
  --config config/uav_config.yaml

make -C dependencies/PX4-Autopilot px4_sitl
```

## 10. Xử lý lỗi thường gặp

### Không có topic `motor_power`

- Kiểm tra `libMotorPowerSystem.so` đã được build.
- Chạy lại installer và build PX4.
- Kiểm tra đang sử dụng model `x500`.

### Có `motor_power` nhưng không có battery state

- Kiểm tra `battery_name` phải là `linear_battery`.
- Kiểm tra block `MotorPowerSystem` đã được thêm vào `model.sdf`.
- Khởi động lại Gazebo sau khi đổi YAML.

### MAVSDK trả về `NaN` cho dòng điện

- Kiểm tra topic battery state có trường `current`.
- Kiểm tra `GZBridge` đã được patch và build lại.
- Kiểm tra PX4 đang nhận `battery_status`.

### Energy luôn dùng fallback 120 W

Điều này có nghĩa ứng dụng không nhận được ít nhất hai mẫu V/I hợp lệ cho UAV.
Kiểm tra lần lượt topic Gazebo, `battery_status` của PX4 và telemetry MAVSDK.

## 11. Các file quan trọng

| File | Chức năng |
|---|---|
| `config/uav_config.yaml` | Cấu hình model pin và tham số PX4 |
| `px4_extensions/battery_power/install.py` | Cài plugin và patch bridge/model |
| `px4_extensions/battery_power/motor_power/` | Source plugin Gazebo |
| `cmd/setup_px4.sh` | Cài PX4 và extension trên máy mới |
| `swam_uav.sh` | Đồng bộ extension và chạy swarm |
| `src/services/drone_service.py` | Áp dụng tham số PX4 và đọc pin |
| `src/main_controller.py` | Tích phân năng lượng và tổng hợp kết quả |

