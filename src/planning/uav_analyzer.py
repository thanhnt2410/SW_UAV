"""
# uav_coverage.py
# ================

# Module tính toán độ bao phủ (coverage) của UAV trên một vùng khảo sát,
# dựa trên quỹ đạo bay thực tế (flight_path) và kích thước vùng quét
# (footprint) của cảm biến gắn trên UAV.

# Quy trình xử lý:
#     1. Chuyển tọa độ GPS (lat, lon) sang hệ tọa độ phẳng UTM (mét) bằng pyproj.
#     2. Dựng polygon vùng khảo sát bằng shapely.
#     3. Với mỗi đoạn bay (cặp waypoint liên tiếp), dựng một dải hình chữ nhật
#        (rectangle strip) rộng bằng footprint_size dọc theo đoạn bay đó -
#        đây là cách mô phỏng chính xác vùng quét hình vuông/chữ nhật,
#        thay vì dùng buffer() mặc định (bo tròn hai đầu).
#     4. Hợp nhất (unary_union) toàn bộ các dải quét.
#     5. Lấy phần giao với polygon khảo sát để loại bỏ phần quét nằm ngoài vùng.
#     6. Tính diện tích và tỷ lệ bao phủ.

# Yêu cầu thư viện:
#     pip install pyproj shapely
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any

from pyproj import Transformer, CRS
from pyproj.aoi import AreaOfInterest
from pyproj.database import query_utm_crs_info
from shapely.geometry import Polygon, LineString, MultiPolygon, mapping
from shapely.ops import unary_union, transform as shapely_transform
from shapely import BufferCapStyle, BufferJoinStyle
import numpy as np

 
GPSPoint = Tuple[float, float]  # (latitude, longitude)
XYPoint = Tuple[float, float]   # (x, y) mét trong hệ UTM
 
 
class UAVAnalyzer:
    """
    Class duy nhất, gộp toàn bộ chức năng: chuyển đổi tọa độ GPS <-> mét
    (UTM), dựng vùng quét (footprint) theo quỹ đạo bay, tính diện tích/tỷ lệ
    bao phủ so với vùng khảo sát, và vẽ trực quan kết quả.
 
    Ví dụ sử dụng:
        >>> analyzer = UAVAnalyzer(
        ...     area_gps=[(21.028, 105.804), (21.029, 105.804),
        ...               (21.029, 105.806), (21.028, 105.806)],
        ...     flight_path=[(21.0281, 105.8045), (21.0285, 105.8045),
        ...                  (21.0285, 105.8055)],
        ...     footprint_size=20.0,
        ... )
        >>> result = analyzer.compute_coverage()
        >>> print(analyzer.summary(result))
        >>> analyzer.visualize(save_path="coverage.png", show=False)
    """
 
    # ------------------------------------------------------------------- #
    # Khởi tạo
    # ------------------------------------------------------------------- #
    def __init__(
        self,
        area_gps: List[GPSPoint],
        flight_path: List[GPSPoint],
        footprint_size: float = 20.0,
    ) -> None:
        """
        Args:
            area_gps: Danh sách đỉnh GPS (lat, lon) tạo thành polygon vùng
                khảo sát. Cần tối thiểu 3 điểm.
            flight_path: Danh sách điểm GPS (lat, lon) UAV đã bay qua,
                theo thứ tự thời gian. Cần tối thiểu 1 điểm.
            footprint_size: Cạnh của vùng quét hình vuông (m). Mặc định 20 m
                (tương ứng footprint 20x20 m).
 
        Raises:
            ValueError: Nếu area_gps có ít hơn 3 điểm, flight_path rỗng,
                hoặc footprint_size <= 0.
        """
        if len(area_gps) < 3:
            raise ValueError("area_gps cần tối thiểu 3 điểm để tạo polygon.")
        if not flight_path:
            raise ValueError("flight_path không được rỗng.")
        if footprint_size <= 0:
            raise ValueError("footprint_size phải > 0.")
 
        self.area_gps = area_gps
        self.flight_path = flight_path
        self.footprint_size = footprint_size
        self._half_width = footprint_size / 2.0
 
        # Xác định UTM CRS phù hợp dựa trên centroid của area_gps, và
        # dựng sẵn 2 Transformer để dùng lại nhiều lần (tránh khởi tạo lại
        # mỗi lần chuyển đổi 1 điểm, khá tốn chi phí).
        self._utm_crs: CRS = self._compute_utm_crs(area_gps)
        self._wgs84_crs: CRS = CRS.from_epsg(4326)
        self._to_utm = Transformer.from_crs(
            self._wgs84_crs, self._utm_crs, always_xy=True
        )
        self._to_wgs84 = Transformer.from_crs(
            self._utm_crs, self._wgs84_crs, always_xy=True
        )
 
    # ------------------------------------------------------------------- #
    # Chuyển đổi tọa độ GPS <-> mét (UTM)
    # ------------------------------------------------------------------- #
    @staticmethod
    def _compute_utm_crs(points: List[GPSPoint]) -> CRS:
        """Tự động xác định UTM CRS phù hợp dựa trên centroid của các điểm GPS."""
        lats = [p[0] for p in points]
        lons = [p[1] for p in points]
        center_lat = sum(lats) / len(lats)
        center_lon = sum(lons) / len(lons)
 
        utm_crs_list = query_utm_crs_info(
            datum_name="WGS 84",
            area_of_interest=AreaOfInterest(
                west_lon_degree=center_lon,
                south_lat_degree=center_lat,
                east_lon_degree=center_lon,
                north_lat_degree=center_lat,
            ),
        )
        if not utm_crs_list:
            raise RuntimeError(
                "Không thể xác định UTM zone phù hợp cho khu vực đã cho."
            )
        return CRS.from_epsg(utm_crs_list[0].code)
 
    @property
    def utm_crs(self) -> CRS:
        """CRS UTM đang được sử dụng để chiếu tọa độ (tiện để debug/log)."""
        return self._utm_crs
 
    def _gps_to_xy(self, point: GPSPoint) -> XYPoint:
        """Chuyển 1 điểm GPS (lat, lon) sang tọa độ phẳng (x, y) mét."""
        lat, lon = point
        x, y = self._to_utm.transform(lon, lat)
        return (x, y)
 
    def _gps_list_to_xy(self, points: List[GPSPoint]) -> List[XYPoint]:
        """Chuyển danh sách điểm GPS sang danh sách tọa độ phẳng (x, y)."""
        return [self._gps_to_xy(p) for p in points]
 
    def _xy_to_gps(self, point: XYPoint) -> GPSPoint:
        """Chuyển 1 điểm tọa độ phẳng (x, y) ngược lại về GPS (lat, lon)."""
        x, y = point
        lon, lat = self._to_wgs84.transform(x, y)
        return (lat, lon)
 
    # ------------------------------------------------------------------- #
    # Dựng vùng quét (footprint) hình chữ nhật cho từng đoạn bay
    # ------------------------------------------------------------------- #
    def _build_segment_strip(self, p1: XYPoint, p2: XYPoint) -> Optional[Polygon]:
        """
        Dựng dải hình chữ nhật quét dọc theo đoạn thẳng p1 -> p2.
 
        Cách làm: tính vector hướng bay và vector pháp tuyến (vuông góc),
        kéo dài 2 đầu đoạn thẳng thêm half_width (mô phỏng flat-cap để
        footprint tại waypoint được phủ đủ), rồi offset 2 bên theo pháp
        tuyến +-half_width để ra 4 đỉnh hình chữ nhật.
 
        Trả về None nếu p1 trùng p2 (đoạn có độ dài 0) - trường hợp này
        được xử lý bằng cách dựng 1 ô vuông tại điểm đó thay thế.
        """
        x1, y1 = p1
        x2, y2 = p2
        dx, dy = x2 - x1, y2 - y1
        length = (dx ** 2 + dy ** 2) ** 0.5
 
        if length == 0:
            return self._build_point_square(p1)
 
        ux, uy = dx / length, dy / length      # vector đơn vị dọc hướng bay
        nx, ny = -uy, ux                        # vector pháp tuyến (vuông góc)
 
        h = self._half_width
        ex1, ey1 = x1 - ux * h, y1 - uy * h
        ex2, ey2 = x2 + ux * h, y2 + uy * h
 
        corners = [
            (ex1 + nx * h, ey1 + ny * h),
            (ex2 + nx * h, ey2 + ny * h),
            (ex2 - nx * h, ey2 - ny * h),
            (ex1 - nx * h, ey1 - ny * h),
        ]
        return Polygon(corners)
 
    def _build_point_square(self, p: XYPoint) -> Polygon:
        """Dựng 1 ô vuông footprint_size x footprint_size tại 1 điểm."""
        x, y = p
        h = self._half_width
        return Polygon(
            [(x - h, y - h), (x + h, y - h), (x + h, y + h), (x - h, y + h)]
        )
 
    def _build_path_footprint(self, path_xy: List[XYPoint]) -> Polygon:
        """
        Dựng vùng quét hợp nhất cho toàn bộ quỹ đạo bay: lặp qua từng cặp
        điểm liên tiếp, dựng hình chữ nhật quét, rồi unary_union() gộp tất
        cả lại (đảm bảo không đếm trùng diện tích ở những đoạn chồng lấn).
        """
        if not path_xy:
            raise ValueError("path_xy không được rỗng.")
 
        if len(path_xy) == 1:
            return self._build_point_square(path_xy[0])
 
        strips = []
        for p1, p2 in zip(path_xy[:-1], path_xy[1:]):
            strip = self._build_segment_strip(p1, p2)
            if strip is not None and strip.is_valid and not strip.is_empty:
                strips.append(strip)
 
        if not strips:
            raise ValueError("Không dựng được vùng quét nào từ flight_path.")
 
        return unary_union(strips)
 
    # ------------------------------------------------------------------- #
    # Dựng các hình học chính (public, tách riêng để dễ kiểm thử)
    # ------------------------------------------------------------------- #
    def build_survey_polygon(self) -> Polygon:
        """Dựng polygon vùng khảo sát (hệ tọa độ mét) từ area_gps."""
        xy_points = self._gps_list_to_xy(self.area_gps)
        polygon = Polygon(xy_points)
        if not polygon.is_valid:
            polygon = polygon.buffer(0)  # sửa lỗi tự cắt (self-intersection) nếu có
        return polygon
 
    def build_flight_linestring(self) -> LineString:
        """Dựng LineString quỹ đạo bay (hệ tọa độ mét) từ flight_path."""
        xy_points = self._gps_list_to_xy(self.flight_path)
        return LineString(xy_points)
 
    def build_coverage_footprint(self) -> Polygon:
        """Dựng vùng quét hợp nhất (union) của toàn bộ quỹ đạo bay."""
        xy_points = self._gps_list_to_xy(self.flight_path)
        return self._build_path_footprint(xy_points)
 
    # ------------------------------------------------------------------- #
    # Tính toán coverage
    # ------------------------------------------------------------------- #
    def compute_coverage(self) -> Dict[str, Any]:
        """
        Thực hiện toàn bộ quy trình tính toán và trả về kết quả dạng dict.
 
        Returns:
            Dict gồm:
                - "coverage_percent": float, tỷ lệ bao phủ (%).
                - "covered_area_m2": float, diện tích đã quét (m^2).
                - "mission_area_m2": float, diện tích vùng khảo sát (m^2).
                - "covered_polygon_geojson": dict GeoJSON của vùng đã quét
                  (tọa độ GPS lat/lon), dùng để trực quan hóa sau này.
        """
        survey_polygon = self.build_survey_polygon()
        coverage_footprint = self.build_coverage_footprint()
 
        covered_polygon = survey_polygon.intersection(coverage_footprint)
 
        mission_area = survey_polygon.area
        covered_area = covered_polygon.area
        coverage_percent = (
            covered_area / mission_area * 100.0 if mission_area > 0 else 0.0
        )

        # --- Integrate cost calculation ---
        path_xy = self._gps_list_to_xy(self.flight_path)
        cost_metrics = self._calculate_cost_metrics(np.array(path_xy), survey_polygon)
 
        return {
            "coverage_percent": coverage_percent,
            "covered_area_m2": covered_area,
            "mission_area_m2": mission_area,
            "covered_polygon_geojson": self._polygon_xy_to_geojson(covered_polygon),
            **cost_metrics # Merge the cost dictionary
        }
 
    def _calculate_cost_metrics(self, xy_path, xy_polygon, weights=(1.0, 0.02, 0.5)):
        """
        Calculates various cost metrics for a given flight path.
        This is the integrated version of the old `calculate_cost_for_path`.
        """
        if xy_path is None or len(xy_path) < 2:
            return {
                "cost": float('inf'), "distance_m": 0.0, "turns": 0,
                "swept_area_m2": 0.0, "overlap_ratio": 0.0
            }

        # Distance and Turns
        diffs = np.diff(xy_path, axis=0)
        seg_lengths = np.linalg.norm(diffs, axis=1)
        j_length = np.sum(seg_lengths)
        headings = np.arctan2(diffs[:, 1], diffs[:, 0])
        d_heads = np.abs((np.diff(headings) + np.pi) % (2 * np.pi) - np.pi)
        j_turn = np.sum(d_heads)
        turn_count = np.sum(d_heads > np.radians(1.0))

        # Overlap and Swept Area
        polygon_area = xy_polygon.area
        path_line = LineString(xy_path)
        sweep_region = path_line.buffer(
            self._half_width, # Use half_width from the class
            cap_style=BufferCapStyle.square,
            join_style=BufferJoinStyle.mitre
        )
        
        total_strip_area = 0.0
        for i in range(len(xy_path)-1):
            segment = LineString([xy_path[i], xy_path[i+1]])
            total_strip_area += segment.buffer(self._half_width).area
        
        overlap_area = max(total_strip_area - sweep_region.area, 0.0)
        overlap_ratio = overlap_area / max(polygon_area, 1e-9)
        
        # Total Cost
        j_total = weights[0] * j_turn + weights[1] * j_length + weights[2] * overlap_ratio

        return {
            "cost": j_total,
            "distance_m": j_length,
            "turns": turn_count,
            "overlap_ratio": overlap_ratio,
        }

    @staticmethod
    def summary(result: Dict[str, Any]) -> str:
        """Trả về chuỗi tóm tắt kết quả compute_coverage(), dễ đọc để log/in ra."""
        return (
            f"Mission area : {result['mission_area_m2']:,.2f} m^2\n"
            f"Covered area : {result['covered_area_m2']:,.2f} m^2\n"
            f"Coverage     : {result['coverage_percent']:.2f} %\n"
            f"Distance     : {result.get('distance_m', 0):,.2f} m\n"
            f"Turns        : {result.get('turns', 0)}\n"
            f"Cost         : {result.get('cost', 0):.2f}"
        )
 
    def _polygon_xy_to_geojson(self, geom) -> Dict[str, Any]:
        if geom.is_empty:
            return {"type": "Polygon", "coordinates": []}
 
        def _to_gps(x: float, y: float) -> Tuple[float, float]:
            lat, lon = self._xy_to_gps((x, y))
            return (lon, lat)  # GeoJSON quy ước thứ tự (lon, lat)
 
        geom_gps = shapely_transform(_to_gps, geom)
        return mapping(geom_gps)
 
    # ------------------------------------------------------------------- #
    # Trực quan hóa (matplotlib)
    # ------------------------------------------------------------------- #
    @staticmethod
    def _iter_polygons(geom):
        if geom.is_empty:
            return
        if isinstance(geom, MultiPolygon):
            for poly in geom.geoms:
                yield poly
        elif isinstance(geom, Polygon):
            yield geom
 
    def visualize(
        self,
        title: str = "UAV Coverage",
        show_waypoints: bool = True,
        figsize: Tuple[float, float] = (9, 7),
        save_path: Optional[str] = None,
        show: bool = True,
    ):
        try:
            import matplotlib.pyplot as plt
            from matplotlib.patches import Polygon as MplPolygon
            from matplotlib.patches import Patch as MplLegendPatch
            from matplotlib.lines import Line2D
        except ImportError as exc:
            raise ImportError(
                "Cần cài matplotlib để vẽ: pip install matplotlib"
            ) from exc
 
        survey_polygon = self.build_survey_polygon()
        flight_line = self.build_flight_linestring()
        coverage_footprint = self.build_coverage_footprint()
        covered_polygon = survey_polygon.intersection(coverage_footprint)
 
        fig, ax = plt.subplots(figsize=figsize)
 
        # 1) Vùng đã quét (tô xanh lá), xử lý cả lỗ hổng bên trong nếu có
        for poly in self._iter_polygons(covered_polygon):
            x, y = poly.exterior.xy
            ax.add_patch(
                MplPolygon(list(zip(x, y)), closed=True, facecolor="#2ecc71",
                          edgecolor="none", alpha=0.55, label="_nolegend_")
            )
            for interior in poly.interiors:
                xi, yi = interior.xy
                ax.add_patch(
                    MplPolygon(list(zip(xi, yi)), closed=True,
                              facecolor="white", edgecolor="none", alpha=1.0)
                )
 
        # 2) Vùng khảo sát (đường viền nét đứt màu xanh dương)
        sx, sy = survey_polygon.exterior.xy
        ax.plot(sx, sy, color="#2980b9", linewidth=2, linestyle="--")
 
        # 3) Quỹ đạo bay (đường liền màu đỏ cam) + waypoint
        fx, fy = flight_line.xy
        ax.plot(fx, fy, color="#e67e22", linewidth=1.5)
        if show_waypoints:
            ax.scatter(fx, fy, color="#c0392b", s=15, zorder=5)
 
        legend_handles = [
            Line2D([0], [0], color="#2980b9", linewidth=2, linestyle="--",
                   label="Vùng khảo sát"),
            Line2D([0], [0], color="#e67e22", linewidth=1.5, label="Quỹ đạo bay"),
        ]
        if show_waypoints:
            legend_handles.append(
                Line2D([0], [0], marker="o", color="none",
                       markerfacecolor="#c0392b", markersize=6, label="Waypoint")
            )
        legend_handles.append(
            MplLegendPatch(facecolor="#2ecc71", edgecolor="none", alpha=0.55,
                            label="Vùng đã quét")
        )
 
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title(title)
        ax.legend(handles=legend_handles, loc="best")
        ax.grid(True, linestyle=":", alpha=0.4)
        fig.tight_layout()
 
        if save_path:
            fig.savefig(save_path, dpi=150)
        if show:
            plt.show()
 
        return fig
 
 
# --------------------------------------------------------------------------- #
# Chạy thử nhanh (demo) khi thực thi trực tiếp file này
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    area = [
        (21.0280, 105.8040),
        (21.0290, 105.8040),
        (21.0290, 105.8060),
        (21.0280, 105.8060),
    ]
 
    flight_path = [
        (21.02805, 105.80405),
        (21.02895, 105.80405),
        (21.02895, 105.80425),
        (21.02805, 105.80425),
        (21.02805, 105.80445),
        (21.02895, 105.80445),
        (21.02895, 105.80465),
        (21.02805, 105.80465),
        (21.02805, 105.80485),
        (21.02895, 105.80485),
        (21.02895, 105.80505),
        (21.02805, 105.80505),
        (21.02805, 105.80525),
        (21.02895, 105.80525),
        (21.02895, 105.80545),
        (21.02805, 105.80545),
        (21.02805, 105.80565),
        (21.02895, 105.80565),
        (21.02895, 105.80585),
        (21.02805, 105.80585),
        (21.02805, 105.80595),
        (21.02895, 105.80595),
    ]
 
    analyzer = UAVAnalyzer(
        area_gps=area,
        flight_path=flight_path,
        footprint_size=20.0,
    )
    result = analyzer.compute_coverage()
 
    print(analyzer.summary(result))
 
    geojson = result["covered_polygon_geojson"]
    geom_type = geojson.get("type", "?")
    coords = geojson.get("coordinates", [])
    if geom_type == "Polygon":
        n_holes = max(len(coords) - 1, 0)
        print(f"\nGeoJSON type: Polygon | 1 vùng liền mạch, {n_holes} lỗ hổng bên trong")
    elif geom_type == "MultiPolygon":
        print(f"\nGeoJSON type: MultiPolygon | {len(coords)} mảnh vùng quét tách rời nhau")
    else:
        print(f"\nGeoJSON type: {geom_type}")
 
    analyzer.visualize(
        title="UAV Coverage Demo",
        save_path="uav_coverage_demo.png",
        show=False,
    )
    print("\nĐã lưu hình vẽ vào uav_coverage_demo.png")