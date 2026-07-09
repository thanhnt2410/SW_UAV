"""Survey-area sizing, splitting, and grid-point generation."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

import numpy as np
from shapely.geometry import LineString as ShapelyLineString, Polygon as ShapelyPolygon
from shapely.ops import split as shapely_split

from .geometry import (
    EARTH_RADIUS,
    convert_degrees_to_radius,
    convert_to_cartesian,
    convert_to_lat_lon,
    find_longest_edge,
    find_midpoint,
    calculate_new_lat_lon,
    line_equation_from_points,
    angle_with_x_axis,
    rotate_and_shift_point,
    revert_rotate_and_shift_point,
    perpendicular_line_equation,
    divide_line_into_segments,
)
from .polygon_ops import (
    ray_casting_point_in_polygon,
    does_line_intersect_polygon,
    divide_points,
    split_area,
    find_parallel_polygon_intersection,
)
from scipy.spatial import ConvexHull

Point = Tuple[float, float]


def calculate_grid_size_from_hfov_and_vfov(h_fov: float, v_fov: float, uav_alt: float) -> Tuple[float, float]:
    """Compute the camera ground footprint in meters."""
    import math

    grid_width = 2 * uav_alt * math.tan(math.radians(h_fov / 2))
    grid_height = 2 * uav_alt * math.tan(math.radians(v_fov / 2))
    return grid_width, grid_height


def calculate_overlapped_grid_size(
    grid_width: float, grid_height: float, h_overlap: float, v_overlap: float
) -> Tuple[float, float]:
    """Compute effective scan spacing after overlap."""
    overlapped_grid_width = grid_width - h_overlap * grid_width
    overlapped_grid_height = grid_height - v_overlap * grid_height
    return overlapped_grid_width, overlapped_grid_height


def area_of_polygon(vertices: List[Point]) -> Dict[str, float]:
    """Calculate the area of a closed lat/lon polygon."""
    area = 0.0
    n = len(vertices)
    if n < 3:
        return {"m2": 0.0, "km2": 0.0, "ha": 0.0}

    for i in range(n - 1):
        p1 = vertices[i]
        p2 = vertices[i + 1]
        area += convert_degrees_to_radius(p2[1] - p1[1]) * (
            2 + math.sin(convert_degrees_to_radius(p1[0])) + math.sin(convert_degrees_to_radius(p2[0]))
        )
    area = area * EARTH_RADIUS**2 / 2

    return {
        "m2": abs(area),
        "km2": abs(area) / 1e6,
        "ha": abs(area) / 1e4,
    }


def remove_duplicate_pts(vertices: List[Point]) -> List[Point]:
    """Remove duplicate points from a list of vertices, preserving order."""
    seen: List[Point] = []
    for v in vertices:
        if v not in seen:
            seen.append(v)
    return seen


def split_polygon_into_areas_old(vertices, number_of_parts):
    """Split a polygon into a number of parts."""
    positions = vertices
    number_of_part = number_of_parts

    min_lat = min(positions, key=lambda x: x[0])[0]
    min_lon = min(positions, key=lambda x: x[1])[1]
    cartesian_coordinates = convert_to_cartesian(positions)

    _, longest_edge_point = find_longest_edge(cartesian_coordinates)

    midpoint = find_midpoint(longest_edge_point[0], longest_edge_point[1])
    new = calculate_new_lat_lon(min_lat, min_lon, midpoint[1], midpoint[0])

    slope, intercept = line_equation_from_points(longest_edge_point[0], longest_edge_point[1])
    angle = angle_with_x_axis(slope)

    new_point = rotate_and_shift_point(
        midpoint[0],
        midpoint[1],
        (-angle),
        midpoint[0],
        midpoint[1],
        (-midpoint[0]),
        (-midpoint[1]),
    )

    perp_slope, perp_intercept = perpendicular_line_equation(midpoint, slope)

    intersect_point = does_line_intersect_polygon(
        midpoint, perp_slope, perp_intercept, cartesian_coordinates
    )
    new = calculate_new_lat_lon(min_lat, min_lon, intersect_point[1], intersect_point[0])

    perpendicular_points = divide_line_into_segments(
        midpoint[0], midpoint[1], intersect_point[0], intersect_point[1], number_of_part
    )
    per_GPS_list = []
    for point in perpendicular_points:
        new = calculate_new_lat_lon(min_lat, min_lon, point[1], point[0])
        per_GPS_list.append(new)

    div_GPS_list = []
    div_points = divide_points(perpendicular_points, cartesian_coordinates, perp_slope, slope)
    for point in div_points:
        new = calculate_new_lat_lon(min_lat, min_lon, point[1], point[0])
        div_GPS_list.append(new)

    rotated_div_points = []
    for point in div_points:
        new_point = rotate_and_shift_point(
            point[0], point[1], (-angle), midpoint[0], midpoint[1], (-midpoint[0]), (-midpoint[1])
        )
        rotated_div_points.append(new_point)

    rotated_perpendicular_points = []
    for point in perpendicular_points:
        new_point = rotate_and_shift_point(
            point[0], point[1], (-angle), midpoint[0], midpoint[1], (-midpoint[0]), (-midpoint[1])
        )
        rotated_perpendicular_points.append(new_point)

    rotated_cartesian_coordinates = []
    for point in cartesian_coordinates:
        new_point = rotate_and_shift_point(
            point[0], point[1], (-angle), midpoint[0], midpoint[1], (-midpoint[0]), (-midpoint[1])
        )
        rotated_cartesian_coordinates.append(new_point)

    rotate_polygon = []
    rotate_polygon = rotated_div_points + rotated_cartesian_coordinates

    rotated_area = split_area(rotate_polygon, rotated_perpendicular_points)
    final_area = []

    for i in range(len(rotated_area)):
        area = rotated_area[i]
        unrotated_area = []
        for point in area:
            new_point = revert_rotate_and_shift_point(
                point[0],
                point[1],
                (-angle),
                midpoint[0],
                midpoint[1],
                (-midpoint[0]),
                (-midpoint[1]),
                clockwise=True,
            )
            unrotated_area.append(new_point)
        per_GPS_list = []
        for point in unrotated_area:
            new = calculate_new_lat_lon(min_lat, min_lon, point[1], point[0])
            per_GPS_list.append(new)

        points = np.array(per_GPS_list)

        hull = ConvexHull(points)

        hull_vertices = points[hull.vertices]

        points = [tuple(point) for point in hull_vertices]
        final_area.append(points)

    return final_area, rotated_area, angle, midpoint, min_lat, min_lon


def calculate_grid_size():
    uav_num = 5
    h_fov = (90, 90, 100, 100, 100)
    v_fov = (52, 52, 52, 52, 52)
    uav_alt = (10, 10, 10, 10, 10)
    h_overlap = 0
    v_overlap = 0
    grid_size = []
    for i in range(uav_num):
        grid_width, grid_height = calculate_grid_size_from_hfov_and_vfov(h_fov[i], v_fov[i], uav_alt[i])
        overlapped_grid_width, overlapped_grid_height = calculate_overlapped_grid_size(grid_width, grid_height, h_overlap, v_overlap)
        grid_size.append((overlapped_grid_width, overlapped_grid_height))
    return grid_size


def generate_waypoints(area_vertices, grid_size, i):
    print("===================================================================================")
    area_min_x = min(v[0] for v in area_vertices)
    area_max_x = max(v[0] for v in area_vertices)
    area_min_y = min(v[1] for v in area_vertices)
    area_max_y = max(v[1] for v in area_vertices)
    print("vertices: ", area_vertices)
    print("min_x, max_x, min_y, max_y: ", area_min_x, area_max_x, area_min_y, area_max_y)
    area_width = area_max_x - area_min_x
    area_height = area_max_y - area_min_y
    print("Area width, height: ", area_width, area_height)

    default_grid_width = grid_size[0]
    default_grid_height = grid_size[1]
    print("Grid width, height: ", default_grid_width, default_grid_height)

    number_of_rows = int(area_height/default_grid_height) + 1
    print("Number of rows: ", number_of_rows)

    longest_edge_length, longest_edge_coord = find_longest_edge(area_vertices)
    print("Coord, longest_edge_length: ", longest_edge_coord, longest_edge_length)
    if longest_edge_coord[0][0] < longest_edge_coord[1][0]:
        x_root_coord = longest_edge_coord[0][0]
        y_root_coord = longest_edge_coord[0][1]
    else:
        x_root_coord = longest_edge_coord[1][0]
        y_root_coord = longest_edge_coord[1][1]

    new_grid_height = area_height / number_of_rows
    intersection_points, segment_length, is_up = find_parallel_polygon_intersection(area_vertices, new_grid_height, number_of_rows)
    new_grid_width = []
    root_grid = (longest_edge_length - default_grid_width) / (int(area_width / default_grid_width))
    new_grid_width.insert(0, root_grid)
    for i in range(len(segment_length)):
        if not(segment_length[i] < default_grid_width):
            row_grid_width = (segment_length[i] - default_grid_width) / (int(segment_length[i] / default_grid_width))
        else:
            row_grid_width = segment_length[i]
        new_grid_width.append(row_grid_width)

    starting_points = []
    for i in range(1, len(intersection_points), 2):
        p1 = intersection_points[i]
        p2 = intersection_points[i - 1]
        if p1[0] < p2[0]:
            starting_points.append(p1)
        else:
            starting_points.append(p2)
    print("Starting points: ", starting_points)

    print("New grid width: ", new_grid_width)
    print("New grid height: ", new_grid_height)

    points = []
    segment_length.insert(0, longest_edge_length)
    for i in range(number_of_rows):
        for j in range(int(segment_length[i]/default_grid_width) + 1):
            if is_up:
                if 0 == i:
                    x = x_root_coord + default_grid_width / 2 + (j * new_grid_width[i])
                    y = y_root_coord + new_grid_height / 2
                else:
                    x = starting_points[i-1][0] + default_grid_width/2 + (j * new_grid_width[i])
                    y = starting_points[i-1][1] + new_grid_height/2
            else:
                if 0 == i:
                    x = x_root_coord + default_grid_width / 2 + (j * new_grid_width[i])
                    y = y_root_coord - new_grid_height / 2
                else:
                    x = starting_points[i-1][0] + default_grid_width/2 + (j * new_grid_width[i])
                    y = starting_points[i-1][1] - new_grid_height/2
            points.append((x, y))
    print("Generated points: ", points)
    return points


def split_grids(rotated_area, angle, midpoint, min_lat, min_lon, grid_size, n_areas):
    if n_areas in (0, 1):
        area = rotated_area[0]
        cartesian_coordinates = convert_to_cartesian(area)
        points = np.array(cartesian_coordinates)
        hull = ConvexHull(points)
        hull_vertices = points[hull.vertices]
        points = [tuple(point) for point in hull_vertices]
        grid_points = generate_grid(points, grid_size)
        unrotated_area = [revert_rotate_and_shift_point(p[0], p[1], -angle, midpoint[0], midpoint[1], -midpoint[0], -midpoint[1], clockwise=True) for p in grid_points]
        per_GPS_list = [calculate_new_lat_lon(min_lat, min_lon, p[1], p[0]) for p in unrotated_area]
        return per_GPS_list
    else:
        areas_dot = []
        for i, area in enumerate(rotated_area):
            points = np.array(area)
            hull = ConvexHull(points)
            hull_vertices = points[hull.vertices]
            points = [tuple(point) for point in hull_vertices]
            grid_size_list = calculate_grid_size()
            grid_points = generate_waypoints(points, grid_size_list[i], i)
            areas_dot.append(grid_points)

        grid_GPS = []
        for i, area in enumerate(areas_dot):
            unrotated_area = [revert_rotate_and_shift_point(p[0], p[1], -angle, midpoint[0], midpoint[1], -midpoint[0], -midpoint[1], clockwise=True) for p in area]
            per_GPS_list = [calculate_new_lat_lon(min_lat, min_lon, p[1], p[0]) for p in unrotated_area]
            grid_GPS.append(per_GPS_list)

        print("Grid GPS: ", grid_GPS)
        return grid_GPS


def generate_grid(vertices: List[Point], spacing_m: float) -> List[Point]:
    """Generate Cartesian grid points inside a polygon."""
    min_x = min(v[0] for v in vertices)
    max_x = max(v[0] for v in vertices)
    min_y = min(v[1] for v in vertices)
    max_y = max(v[1] for v in vertices)

    points: List[Point] = []
    for i in range(int((max_y - min_y) / spacing_m) + 1):
        for j in range(int((max_x - min_x) / spacing_m) + 1):
            x = min_x + (j * spacing_m)
            y = min_y + (i * spacing_m)
            if ray_casting_point_in_polygon((x, y), vertices):
                points.append((x, y))

    return points
