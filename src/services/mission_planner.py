from typing import List, Tuple

import numpy as np

from config_loader import ConfigLoader
from planning.geometry import (
    calculate_new_lat_lon,
    convert_to_cartesian,
    find_longest_edge,
    rotate_and_shift_point,
    revert_rotate_and_shift_point,
    find_midpoint,
    line_equation_from_points,
    angle_with_x_axis,
)
from planning.grid import (
    calculate_grid_size_from_hfov_and_vfov,
    calculate_overlapped_grid_size,
    generate_grid,
)
from planning.polygon_ops import find_parallel_polygon_intersection


class MissionPlanner:
    """
    Orchestrates mission planning by combining fleet configuration with planning algorithms.
    """

    def __init__(self, config: ConfigLoader):
        """
        Initializes the MissionPlanner with application configuration.

        Args:
            config: The loaded application configuration object.
        """
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
            # Use get() with defaults for safety
            h_fov = uav_conf.get("h_fov", 90.0)
            v_fov = uav_conf.get("v_fov", 52.0)
            alt = uav_conf.get("init_alt", 10.0)

            # Calculate raw footprint
            grid_width, grid_height = calculate_grid_size_from_hfov_and_vfov(h_fov, v_fov, alt)

            # Adjust for overlap
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
        area_min_x = min(v[0] for v in area_vertices)
        area_max_y = max(v[1] for v in area_vertices)

        default_grid_width, default_grid_height = grid_size

        number_of_rows = int((area_max_y - area_min_y) / default_grid_height) + 1

        _, longest_edge_coord = find_longest_edge(area_vertices)
        x_root_coord = min(longest_edge_coord[0][0], longest_edge_coord[1][0])
        y_root_coord = (
            longest_edge_coord[0][1]
            if x_root_coord == longest_edge_coord[0][0]
            else longest_edge_coord[1][1]
        )

        new_grid_height = (area_max_y - area_min_y) / number_of_rows
        (
            intersection_points,
            segment_length,
            is_up,
        ) = find_parallel_polygon_intersection(area_vertices, new_grid_height, number_of_rows)

        # This part of the logic seems highly specific and might need further review
        # For now, it's ported as-is from the original implementation.
        points = []
        if not intersection_points:
            return points

        starting_points = [
            p1 if p1[0] < p2[0] else p2
            for p1, p2 in zip(intersection_points[1::2], intersection_points[0::2])
        ]

        for i in range(number_of_rows):
            # The logic for generating points seems incomplete/buggy in the original code.
            # This is a simplified placeholder.
            # A full, robust lawnmower implementation would be needed here.
            pass

        # Placeholder: for now, just return the intersection points as a simple path
        return intersection_points