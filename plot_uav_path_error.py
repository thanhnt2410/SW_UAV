import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import glob

def latlon_to_xy(lat, lon, lat_ref, lon_ref):
    """Chuyển đổi Lat/Lon sang hệ tọa độ XY cục bộ (đơn vị: mét)"""
    R = 6378137.0  # Bán kính Trái Đất (mét)
    lat_rad = np.radians(lat)
    lat_ref_rad = np.radians(lat_ref)
    
    dx = np.radians(lon - lon_ref) * R * np.cos(lat_ref_rad)
    dy = np.radians(lat - lat_ref) * R
    return dx, dy

def point_to_segment_distance(p, a, b):
    """Tính khoảng cách ngắn nhất từ điểm P đến đoạn thẳng AB"""
    ab = b - a
    ap = p - a
    
    # Tránh lỗi chia cho 0 nếu A trùng B
    if np.dot(ab, ab) == 0:
        return np.linalg.norm(ap)
        
    # Tính điểm hình chiếu t trên đoạn thẳng (giới hạn từ 0 đến 1)
    t = np.dot(ap, ab) / np.dot(ab, ab)
    t = np.clip(t, 0.0, 1.0)
    
    projection = a + t * ab
    return np.linalg.norm(p - projection)

def main():
    # 1. Khai báo đường dẫn
    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    num_uavs = 5
    lat_ref, lon_ref = None, None
    colors = ['b', 'g', 'r', 'c', 'm', 'y']
    
    plt.rcParams.update({'font.size': 11})

    # 1. Tìm các UAV có dữ liệu hợp lệ trước
    valid_uavs = []
    uav_files = {}

    for i in range(1, num_uavs + 1):
        plan_file = os.path.join(base_dir, f"src/logs/points/reduced_point{i}.txt")
        history_files = glob.glob(os.path.join(base_dir, f"src/logs/drone_current_pos/uav_{i}_history_*.txt"))
        
        if os.path.exists(plan_file) and history_files:
            history_file = max(history_files, key=os.path.getctime)
            plan_data = pd.read_csv(plan_file, header=None, names=["lat", "lon"])
            history_data = pd.read_csv(history_file, header=None, names=["time", "lat", "lon", "alt"])
            
            if len(plan_data) > 0 and len(history_data) > 0:
                valid_uavs.append(i)
                uav_files[i] = (plan_data, history_data)
            else:
                print(f"Bỏ qua UAV {i}: Dữ liệu trống.")
        else:
            print(f"Bỏ qua UAV {i}: Không tìm thấy file plan hoặc history.")

    if not valid_uavs:
        print("Không có dữ liệu hợp lệ của bất kỳ UAV nào để vẽ.")
        return
        

    for row_idx, uav_id in enumerate(valid_uavs):
        print(f"Đang xử lý dữ liệu cho UAV {uav_id}...")
        plan_data, history_data = uav_files[uav_id]

        # 3. Lấy điểm tham chiếu
        if lat_ref is None:
            lat_ref = plan_data['lat'].iloc[0]
            lon_ref = plan_data['lon'].iloc[0]

        plan_xy = np.array([latlon_to_xy(row['lat'], row['lon'], lat_ref, lon_ref) 
                            for _, row in plan_data.iterrows()])
        history_xy = np.array([latlon_to_xy(row['lat'], row['lon'], lat_ref, lon_ref) 
                               for _, row in history_data.iterrows()])

        # --- Lọc đoạn lịch sử bay thực sự (bám theo Waypoints) ---
        # Tìm vị trí (index) mà UAV tiến gần điểm plan đầu tiên nhất
        dists_to_start = np.linalg.norm(history_xy - plan_xy[0], axis=1)
        start_idx = np.argmin(dists_to_start)
        
        # Tìm vị trí (index) mà UAV tiến gần điểm plan cuối cùng nhất (tính từ sau khi bắt đầu)
        dists_to_end = np.linalg.norm(history_xy[start_idx:] - plan_xy[-1], axis=1)
        end_idx = start_idx + np.argmin(dists_to_end)
        
        mission_history_xy = history_xy[start_idx:end_idx+1]

        # 4. Tính toán lỗi
        errors = []
        for p in mission_history_xy:
            min_dist = float('inf')
            for j in range(len(plan_xy) - 1):
                d = point_to_segment_distance(p, plan_xy[j], plan_xy[j+1])
                if d < min_dist:
                    min_dist = d
            errors.append(min_dist)

        # 5. Tạo cửa sổ (Figure) riêng cho từng UAV
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        fig.suptitle(f"UAV {uav_id} Performance Analysis", fontsize=14, fontweight='bold')
        c = colors[(uav_id - 1) % len(colors)]
        
        # --- Biểu đồ 1: Quỹ đạo (Trajectory) ---
        ax1.plot(plan_xy[:, 0], plan_xy[:, 1], f'{c}--o', label=f"Plan UAV {uav_id}", linewidth=1.5, markersize=4, alpha=0.5)
        ax1.plot(history_xy[:, 0], history_xy[:, 1], f'{c}-', label=f"Transit Path UAV {uav_id}", linewidth=1.0, alpha=0.2) # Đường bay phụ (mờ)
        ax1.plot(mission_history_xy[:, 0], mission_history_xy[:, 1], f'{c}-', label=f"Mission Path UAV {uav_id}", linewidth=2.0, alpha=0.9) # Đường bay chính (đậm)
        ax1.plot(mission_history_xy[0, 0], mission_history_xy[0, 1], f'{c}o', markersize=6) # Điểm bắt đầu tính error
        ax1.plot(mission_history_xy[-1, 0], mission_history_xy[-1, 1], f'{c}x', markersize=6) # Điểm kết thúc tính error
        
        # --- Biểu đồ 2: Lỗi vị trí (Position Error) ---
        ax2.plot(errors, f'{c}-', linewidth=1.5, label=f"Error UAV {uav_id} (Avg: {np.mean(errors):.2f}m)")

        # --- Định dạng cửa sổ ---
        ax1.set_xlabel("Local X (meters)")
        ax1.set_ylabel("Local Y (meters)")
        ax1.set_title(f"Trajectory UAV {uav_id}")
        ax1.legend(fontsize='small')
        ax1.grid(True, linestyle=':', alpha=0.7)
        ax1.set_aspect('equal', 'box')

        ax2.set_xlabel("Time Step (Index)")
        ax2.set_ylabel("Error (meters)")
        ax2.set_title(f"Position Error UAV {uav_id}")
        ax2.legend(fontsize='small')
        ax2.grid(True, linestyle=':', alpha=0.7)

        plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()
