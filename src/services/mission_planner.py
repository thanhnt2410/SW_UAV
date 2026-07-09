from typing import List, Tuple

from config_loader import ConfigLoader
from planning.grid import (
    calculate_grid_size_from_hfov_and_vfov,
    calculate_overlapped_grid_size,
)
from planning.polygon_ops import find_parallel_polygon_intersection


class MissionPlanner:
    """
    Orchestrates mission planning by combining fleet configuration with planning algorithms.
    """

    def __init__(self, config: ConfigLoader):
        self.config = config

    def get_grid_sizes_for_fleet(self) -> List[Tuple[float, float]]:
        """
        Calculates the effective grid size (width, height) for each UAV in the fleet
        based on their camera configuration and desired overlap.
        This replaces the hardcoded `calculate_grid_size` from map_helpers.
        """
        grid_sizes = []
        h_overlap = self.config.stream["survey"]["horizontal_overlap"]
        v_overlap = self.config.stream["survey"]["vertical_overlap"]

        for uav_conf in self.config.uav["uavs"]:
            h_fov = uav_conf.get("h_fov", 90.0)
            v_fov = uav_conf.get("v_fov", 52.0)
            alt = uav_conf.get("init_alt", 10.0)

            grid_width, grid_height = calculate_grid_size_from_hfov_and_vfov(h_fov, v_fov, alt)

            eff_width, eff_height = calculate_overlapped_grid_size(
                grid_width, grid_height, h_overlap, v_overlap
            )
            grid_sizes.append((eff_width, eff_height))

        return grid_sizes

    def generate_lawnmower_waypoints(
        self, area_vertices: list, grid_size: tuple, uav_index: int
    ) -> list:
        """
        Generates lawnmower pattern waypoints for a single survey area.
        This is a refactoring of the old `generate_waypoints` from map_helpers.
        The unused `i` parameter is removed.
        """
        area_min_y = min(v[1] for v in area_vertices)
        area_max_y = max(v[1] for v in area_vertices)

        _, default_grid_height = grid_size

        number_of_rows = int((area_max_y - area_min_y) / default_grid_height) + 1

        new_grid_height = (area_max_y - area_min_y) / number_of_rows
        intersection_points, _, _ = find_parallel_polygon_intersection(
            area_vertices, new_grid_height, number_of_rows
        )

        points = []
        if not intersection_points:
            return points

        return intersection_points
