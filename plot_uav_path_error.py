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
    plan_file = os.path.join(base_dir, "src/logs/points/reduced_point1.txt")
    
    # Tự động tìm file history mới nhất của UAV 1
    history_files = glob.glob(os.path.join(base_dir, "src/logs/drone_current_pos/uav_1_history_*.txt"))
    if not history_files:
        print("Không tìm thấy file history nào! Hãy chắc chắn UAV đã bay nhiệm vụ.")
        return
    history_file = max(history_files, key=os.path.getctime)
    print(f"Đang đọc và vẽ biểu đồ từ file: {os.path.basename(history_file)}")

    # 2. Đọc dữ liệu
    plan_data = pd.read_csv(plan_file, header=None, names=["lat", "lon"])
    history_data = pd.read_csv(history_file, header=None, names=["time", "lat", "lon", "alt"])

    # 3. Lấy điểm tham chiếu (Điểm bắt đầu của Plan) để tính XY
    lat_ref = plan_data['lat'].iloc[0]
    lon_ref = plan_data['lon'].iloc[0]

    # Chuyển đổi toàn bộ sang hệ tọa độ XY (mét)
    plan_xy = np.array([latlon_to_xy(row['lat'], row['lon'], lat_ref, lon_ref) 
                        for _, row in plan_data.iterrows()])
    history_xy = np.array([latlon_to_xy(row['lat'], row['lon'], lat_ref, lon_ref) 
                           for _, row in history_data.iterrows()])

    # 4. Tính toán lỗi vị trí (Cross-Track Error)
    errors = []
    for p in history_xy:
        min_dist = float('inf')
        # Tìm khoảng cách ngắn nhất từ điểm hiện tại đến các đoạn thẳng của Plan
        for i in range(len(plan_xy) - 1):
            d = point_to_segment_distance(p, plan_xy[i], plan_xy[i+1])
            if d < min_dist:
                min_dist = d
        errors.append(min_dist)

    # 5. Cấu hình và vẽ biểu đồ
    plt.rcParams.update({'font.size': 11})
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

    # --- Biểu đồ 1: Quỹ đạo (Trajectory) ---
    ax1.plot(plan_xy[:, 0], plan_xy[:, 1], 'k--o', label="Plan Path (Waypoints)", linewidth=2, markersize=6)
    ax1.plot(history_xy[:, 0], history_xy[:, 1], 'b-', label="UAV Actual Path", linewidth=1.5, alpha=0.8)
    # Đánh dấu điểm Bắt đầu và Kết thúc của UAV
    ax1.plot(history_xy[0, 0], history_xy[0, 1], 'go', label="Start", markersize=8)
    ax1.plot(history_xy[-1, 0], history_xy[-1, 1], 'rx', label="End", markersize=8)
    
    ax1.set_xlabel("Local X (meters)")
    ax1.set_ylabel("Local Y (meters)")
    ax1.set_title("UAV Trajectory Comparison")
    ax1.legend()
    ax1.grid(True, linestyle=':', alpha=0.7)
    ax1.set_aspect('equal', 'box') # Đảm bảo tỷ lệ khung hình thật để không méo

    # --- Biểu đồ 2: Lỗi vị trí (Position Error) ---
    ax2.plot(errors, 'r-', linewidth=1.5, label="Cross-Track Error")
    ax2.set_xlabel("Time Step (Index)")
    ax2.set_ylabel("Error (meters)")
    ax2.set_title(f"Position Tracking Error\nAvg Error: {np.mean(errors):.2f}m | Max: {np.max(errors):.2f}m")
    ax2.legend()
    ax2.grid(True, linestyle=':', alpha=0.7)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
