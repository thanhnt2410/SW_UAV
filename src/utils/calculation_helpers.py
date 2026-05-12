import math
import random
import numpy as np
from shapely.geometry import LineString, Point, Polygon
from sympy import Point, Polygon

EARTH_RADIUS = 6378137  # meters


def convert_degrees_to_radius(degrees):
    return degrees * math.pi / 180


def haversine(lat1, lon1, lat2, lon2):
    distance_lat = math.radians(lat2 - lat1)
    distance_lon = math.radians(lon2 - lon1)
    a = math.sin(distance_lat / 2) * math.sin(distance_lat / 2) + math.cos(
        math.radians(lat1)
    ) * math.cos(math.radians(lat2)) * math.sin(distance_lon / 2) * math.sin(distance_lon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = EARTH_RADIUS * c
    return distance


def convert_to_cartesian(positions):
    # Find the minimum latitude and longitude to use as the origin
    min_lat = min(positions, key=lambda x: x[0])[0]
    min_lon = min(positions, key=lambda x: x[1])[1]

    # Create a list to hold Cartesian coordinates
    cartesian_coords = []

    # Convert each position to Cartesian coordinates
    for lat, lon in positions:
        # Distance from the minimum longitude to the current point's longitude (x-coordinate)
        x = haversine(min_lat, min_lon, min_lat, lon)
        # Distance from the minimum latitude to the current point's latitude (y-coordinate)
        y = haversine(min_lat, min_lon, lat, min_lon)

        # Append the calculated Cartesian coordinates to the list
        cartesian_coords.append((x, y))

    return cartesian_coords


def convert_to_lat_lon(ref_point, distance):
    """Convert a distance in meters to a latitude and longitude offset from a reference point."""
    lat_offset, lon_offset = ref_point
    distance_north, distance_east = distance

    delta_lat = distance_north / EARTH_RADIUS
    point_lat = lat_offset + math.degrees(delta_lat)

    r = EARTH_RADIUS * math.cos(math.radians(lat_offset))
    delta_lon = distance_east / r
    point_lon = lon_offset + math.degrees(delta_lon)

    return (point_lat, point_lon)


def is_polygon_convex(points):
    """Check if a polygon defined by a list of points is convex."""
    polygon = Polygon(*points)
    return polygon.is_convex()


def distance_between_points(p1, p2):
    """
    Calculate the distance between two points in meters.
    Args:
        p1: A tuple (x, y) representing the first point.
        p2: A tuple (x, y) representing the second point.
    returns:
        Float: The distance between the two points in meters.
    """
    return haversine(p1[0], p1[1], p2[0], p2[1])


def find_slope_intercept(p1, p2):
    """Find the slope and intercept of a line defined by two points."""
    x = [p1[0], p2[0]]
    y = [p1[1], p2[1]]
    slope, intercept = np.polyfit(x, y, 1)
    return slope, intercept


def find_perpendicular_slope_intercept(slope, point):
    """Find the slope and intercept of a line perpendicular to a line defined by a slope and a point."""
    x, y = point
    perpendicular_slope = -1 / slope
    perpendicular_intercept = y - perpendicular_slope * x
    return perpendicular_slope, perpendicular_intercept


def find_intersection(p1, p2, slope, intercept):
    """
    Find the intersection point of a line segment and a line.

    Parameters:
        p1 (tuple): The first point of the line segment (x1, y1).
        p2 (tuple): The second point of the line segment (x2, y2).
        slope (float): The slope of the other line.
        intercept (float): The y-intercept of the other line.

    Returns:
        list: The intersection point [x_intersect, y_intersect] if it exists and lies within the segment, otherwise None.
    """

    x1, y1 = p1
    x2, y2 = p2
    if x1 == x2:
        x_intersect = x1
        y_intersect = slope * x_intersect + intercept
    elif y1 == y2:
        y_intersect = y1
        x_intersect = (y_intersect - intercept) / slope
    else:
        slope_edge = (y2 - y1) / (x2 - x1)
        intercept_edge = y1 - slope_edge * x1
        if slope == slope_edge:
            return None
        x_intersect = (intercept_edge - intercept) / (slope - slope_edge)
        y_intersect = slope * x_intersect + intercept
    if min(x1, x2) <= x_intersect <= max(x1, x2) and min(y1, y2) <= y_intersect <= max(y1, y2):
        return [x_intersect, y_intersect]
    return None


def distance_between_cartesian_points(p1, p2):
    """Calculate the Euclidean distance between two points in Cartesian coordinates."""
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def find_longest_edge(points):
    """Find the longest edge of a quadrilateral."""
    max_length = 0
    longest_edge = None

    for i in range(len(points)):
        p1, p2 = points[i], points[(i + 1) % len(points)]
        length = np.linalg.norm(np.array(p1) - np.array(p2))
        if length > max_length:
            max_length = length
            longest_edge = (p1, p2)

    return max_length, longest_edge

def find_longest_edge_lat_lon(points):
    """Find the longest edge of a quadrilateral."""
    max_length = 0
    longest_edge = None

    for i in range(len(points)):
        p1, p2 = points[i], points[(i + 1) % len(points)]
        length = distance_between_points(p1, p2)
        if length > max_length:
            max_length = length
            longest_edge = (p1, p2)

    return max_length, longest_edge

def find_segment_points(edge, N=2):
    """Find N equal segment points on an edge."""
    (x1, y1), (x2, y2) = edge
    segment_points = []
    for i in range(1, N):
        x = x1 + (x2 - x1) * i / N
        y = y1 + (y2 - y1) * i / N
        segment_points.append((x, y))
    return segment_points


def perpendicular_lines_at_points(edge, points=[], N=2, length=1000):
    """
    Generate perpendicular lines at specified points along a given edge.
    Parameters:
        edge (tuple): A tuple containing two points (x1, y1) and (x2, y2) that define the edge.
        points (list, optional): A list of points (x, y) where perpendicular lines should be generated.
                                If not provided, points will be generated along the edge.
        N (int, optional): Number of points to generate along the edge if `points` is not provided. Default is 2.
        length (int, optional): Length of the perpendicular lines to be generated. Default is 1000.
    Returns:
        list: A list of tuples, each containing the slope and intercept of the perpendicular lines.
    """

    (x1, y1), (x2, y2) = edge
    perp_lines = []

    if len(points) == 0:
        points = find_segment_points(edge, N)

    for point in points:
        x, y = point

        if x2 - x1 == 0:  # Vertical edge
            perp_line = LineString([(x - length, y), (x + length, y)])
        elif y2 - y1 == 0:  # Horizontal edge
            perp_line = LineString([(x, y - length), (x, y + length)])
        else:
            slope = (y2 - y1) / (x2 - x1)
            perp_slope = -1 / slope
            perp_line = LineString(
                [(x - length, y - length * perp_slope), (x + length, y + length * perp_slope)]
            )
        perp_lines.append(find_slope_intercept(*perp_line.coords[:]))

    return perp_lines


def find_polygon_line_intersections(cartesian_polygon, line):
    """Find intersections of a line with a polygon."""

    intersection_points = []
    cartesian_coordinates = np.array(cartesian_polygon)
    slope, intercept = line

    for i in range(len(cartesian_coordinates)):
        p1 = cartesian_coordinates[i]
        p2 = cartesian_coordinates[(i + 1) % len(cartesian_coordinates)]
        intersection = find_intersection(p1, p2, slope, intercept)

        if intersection:
            intersection_points.append(intersection)

    return intersection_points


def is_between_lines(point, line1, line2):
    """Check if a point lies between two lines."""
    x, y = point
    slope1, intercept1 = line1
    slope2, intercept2 = line2
    return (
        min(slope1 * x + intercept1, slope2 * x + intercept2)
        <= y
        <= max(slope1 * x + intercept1, slope2 * x + intercept2)
    )


def is_left_of_line(point, line):
    """Check if a point lies to the left of a line."""
    x, y = point
    slope, intercept = line
    return y >= slope * x + intercept


def calculate_angle(a, b, c):
    """Calculate angle at b given points a, b, c, including reflex angles."""
    ba = (a[0] - b[0], a[1] - b[1])
    bc = (c[0] - b[0], c[1] - b[1])

    # Dot product
    dot_prod = ba[0] * bc[0] + ba[1] * bc[1]
    # Magnitude of vector AB and BC
    mag_ab = math.sqrt(ba[0] ** 2 + ba[1] ** 2)
    mag_bc = math.sqrt(bc[0] ** 2 + bc[1] ** 2)

    if mag_ab == 0 or mag_bc == 0:
        # Return an angle of 0 or handle it as needed
        return 0

    cosine_angle = (dot_prod) / (mag_ab * mag_bc)
    angle = math.acos(cosine_angle)
    # Calculate the cross product of vectors ba and bc
    cross_product = ba[0] * bc[1] - ba[1] * bc[0]
    # Convert the angle from radians to degrees
    angle_degrees = math.degrees(angle)
    # If the cross product is negative, the angle is reflex, so we adjust the angle
    if cross_product > 0:
        angle_degrees = 360 - angle_degrees
    return angle_degrees


def heron_formula(a, b, c):
    """Apply Heron's formula to calculate the area of a triangle given its side lengths."""
    s = (a + b + c) / 2
    area = math.sqrt(s * (s - a) * (s - b) * (s - c))
    return area


# ==================================================================================================
def find_longest_edge2(cartesian_coords):
    """
    Find the vertices that form the longest edge in a polygon given its vertices in Cartesian coordinates.

    :param cartesian_coords: A list of tuples (x, y) representing the Cartesian coordinates of the polygon vertices.
    :return: A tuple containing:
        - the length of the longest edge,
        - the coordinates of the start vertex of the longest edge,
        - the coordinates of the end vertex of the longest edge.
    """
    num_vertices = len(cartesian_coords)
    longest_edge_length = 0
    longest_edge_vertices = (None, None)

    for i in range(num_vertices):
        # Current vertex
        x1, y1 = cartesian_coords[i]
        # Next vertex, with wrap-around using modulo to close the polygon
        x2, y2 = cartesian_coords[(i + 1) % num_vertices]

        # Calculate the distance between the current vertex and the next vertex
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        # Check if this is the longest edge found so far
        if distance > longest_edge_length:
            longest_edge_length = distance
            longest_edge_vertices = ((x1, y1), (x2, y2))

    return longest_edge_length, longest_edge_vertices


def find_midpoint(point1, point2):
    # Extract coordinates from the points
    x1, y1 = point1
    x2, y2 = point2

    # Calculate the midpoint coordinates
    mid_x = (x1 + x2) / 2.0
    mid_y = (y1 + y2) / 2.0

    return (mid_x, mid_y)


def line_equation_from_points(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    # Check for vertical line
    if x1 == x2:
        # The line is vertical, slope is undefined, and the line equation is x = x1
        return None, x1

    # Check for horizontal line
    elif y1 == y2:
        # The line is horizontal, slope is 0, and the line equation is y = y1
        return 0, y1

    else:
        # For non-vertical and non-horizontal lines, calculate slope and intercept
        slope = (y2 - y1) / (x2 - x1)
        intercept = y1 - slope * x1
        return slope, intercept


def angle_with_x_axis(slope):
    if slope is None:  # Vertical line
        angle_degrees = 90
    else:
        angle_radians = math.atan(slope)
        angle_degrees = math.degrees(angle_radians)

    return angle_degrees


def perpendicular_line_equation(midpoint, slope, tolerance=1e-6):
    """
    Calculate the slope and y-intercept of the line perpendicular to the line with the given slope,
    passing through the given midpoint.

    :param midpoint: A tuple (x, y) representing the midpoint through which the perpendicular line passes.
    :param slope: The slope of the original line. Can be a number, zero, or None for vertical lines.
    :param tolerance: The tolerance range for slope comparisons to account for floating-point inaccuracies.
    :return: A tuple (perpendicular slope, y-intercept) representing the perpendicular line.
             Returns (None, y-coordinate of midpoint) for vertical lines and
             (0, x-coordinate of midpoint) for horizontal lines.
    """
    mx, my = midpoint

    # Check if the original slope is near zero (horizontal line)
    if slope is not None:
        if -tolerance < slope < tolerance:
            # The perpendicular line would be vertical
            return None, my

    # Check if the original slope is undefined (vertical line)
    elif slope is None:
        # The perpendicular line would be horizontal
        return 0, my

    # Otherwise, calculate the perpendicular slope and intercept
    perp_slope = -1 / slope
    perp_intercept = my - perp_slope * mx
    return perp_slope, perp_intercept


def calculate_new_lat_lon(origin_lat, origin_lon, distance_north, distance_east):
    """Calculate new latitude and longitude from origin given distances north and east."""
    R = 6378000  # Radius of Earth in meters
    delta_lat = distance_north / R  # Change in latitude in radians
    new_lat = origin_lat + math.degrees(delta_lat)  # New latitude in degrees

    # Adjust for change in longitude, which depends on latitude
    r = R * math.cos(math.radians(new_lat))  # Effective radius at new latitude
    delta_lon = distance_east / r  # Change in longitude in radians
    new_lon = origin_lon + math.degrees(delta_lon)  # New longitude in degrees

    return (new_lat, new_lon)


def divide_line_into_segments(x1, y1, x2, y2, n):
    points = []
    # We skip 0 and n because they correspond to the endpoints.
    for i in range(1, n):
        t = i / n
        xt = (1 - t) * x1 + t * x2
        yt = (1 - t) * y1 + t * y2
        points.append((xt, yt))

    return points


def perpendicular_line_intersect_polygon(slope, intercept, vertices):
    across_points = []
    for i in range(len(vertices)):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % len(vertices)]

        # Check if the edge is vertical
        if x1 == x2:
            if slope is None:
                # Both the line and edge are vertical, so check if they are the same line
                if x1 == intercept:  # Using intercept as x since the line is vertical
                    # Overlapping vertical lines intersect at any point along their span
                    overlap_range = [max(min(y1, y2), intercept), min(max(y1, y2), intercept)]
                    if overlap_range[0] != overlap_range[1]:
                        # Return a representative point from the overlapping segment
                        point = (x1, sum(overlap_range) / 2)
                        across_points.append(point)
            else:
                # The edge is vertical while the line is not, calculate y using line's equation
                intersect_y = slope * x1 + intercept
                if min(y1, y2) <= intersect_y <= max(y1, y2):
                    point = (x1, intersect_y)
                    across_points.append(point)

        # Check if the edge is non-vertical
        elif x2 != x1:
            m_e = (y2 - y1) / (x2 - x1)
            b_e = y1 - m_e * x1
            if slope is None:
                # The line is vertical, calculate x using edge's equation
                intersect_x = intercept  # Using intercept as x since the line is vertical
                intersect_y = m_e * intercept + b_e
                if min(x1, x2) <= intersect_x <= max(x1, x2):
                    point = (intersect_x, intersect_y)
                    across_points.append(point)
            elif slope != m_e:
                # Ensure the lines are not parallel and calculate the intersection
                intersect_x = (b_e - intercept) / (slope - m_e)
                intersect_y = slope * intersect_x + intercept
                if min(x1, x2) <= intersect_x <= max(x1, x2) and min(y1, y2) <= intersect_y <= max(
                    y1, y2
                ):
                    point = (intersect_x, intersect_y)
                    across_points.append(point)

    return across_points


def divide_points(per_points, polygon, slope1, edge_slope):
    each_point = []
    for point in per_points:
        perp_slope, perp_intercept = perpendicular_line_equation(point, slope1)
        per_dot = perpendicular_line_intersect_polygon(perp_slope, perp_intercept, polygon)

        each_point.extend(per_dot)

    return each_point


def rotate_and_shift_point(
    x, y, angle, pivot_x, pivot_y, shift_x=0, shift_y=0, units="DEGREES", clockwise=False
):
    """
    Rotates a point around a pivot point either clockwise or counterclockwise and then applies a shift.

    :param x: The x-coordinate of the point to be rotated.
    :param y: The y-coordinate of the point to be rotated.
    :param angle: The angle of rotation. Positive angles will result in counterclockwise rotation,
                  and negative angles will result in clockwise rotation if `clockwise` is set to True.
    :param pivot_x: The x-coordinate of the pivot point.
    :param pivot_y: The y-coordinate of the pivot point.
    :param shift_x: The distance to shift along the x-axis after rotation.
    :param shift_y: The distance to shift along the y-axis after rotation.
    :param units: The units of the angle: 'DEGREES' (default) or 'RADIANS'.
    :param clockwise: If True, the rotation will be clockwise, otherwise counterclockwise.
    :return: A tuple containing the rotated and shifted x and y coordinates.
    """

    # Convert angle from degrees to radians if specified in degrees
    if units.upper() == "DEGREES":
        angle = math.radians(angle)

    # If clockwise rotation is desired, negate the angle
    if clockwise:
        angle = -angle

    # Translate point to pivot
    x -= pivot_x
    y -= pivot_y

    # Apply rotation
    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)
    x_rotated = (x * cos_theta) - (y * sin_theta)
    y_rotated = (x * sin_theta) + (y * cos_theta)

    # Translate point back from pivot and apply additional shift
    x_final = x_rotated + pivot_x + shift_x
    y_final = y_rotated + pivot_y + shift_y

    return x_final, y_final


def revert_rotate_and_shift_point(
    x, y, angle, pivot_x, pivot_y, shift_x=0, shift_y=0, units="DEGREES", clockwise=False
):
    """
    Reverts the rotation and shift applied to a point. First subtracts the shift, then rotates the point back.

    :param x: The x-coordinate of the point that was rotated and shifted.
    :param y: The y-coordinate of the point that was rotated and shifted.
    :param angle: The original angle of rotation used to rotate the point.
    :param pivot_x: The x-coordinate of the pivot point used in the original rotation.
    :param pivot_y: The y-coordinate of the pivot point used in the original rotation.
    :param shift_x: The x-distance of the shift to revert.
    :param shift_y: The y-distance of the shift to revert.
    :param units: The units of the angle: 'DEGREES' (default) or 'RADIANS'.
    :param clockwise: If the original rotation was clockwise, set True; otherwise, set False.
    :return: A tuple containing the x and y coordinates of the point after reverting the rotation and shift.
    """

    # Subtract the shift
    x -= shift_x
    y -= shift_y

    # Convert angle from degrees to radians if specified in degrees
    if units.upper() == "DEGREES":
        angle = math.radians(angle)

    # Reverse the angle direction for the rotation back
    if not clockwise:
        angle = -angle

    # Translate point to pivot
    x -= pivot_x
    y -= pivot_y

    # Apply rotation in the opposite direction
    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)
    x_reverted = (x * cos_theta) + (y * sin_theta)
    y_reverted = (-x * sin_theta) + (y * cos_theta)

    # Translate point back from pivot
    x_final = x_reverted + pivot_x
    y_final = y_reverted + pivot_y

    return x_final, y_final


def does_line_intersect_polygon(mid, slope, intercept, vertices, tolerance=1e-6):
    for i in range(len(vertices)):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % len(vertices)]

        # Check for vertical edge (x1 == x2)
        if x1 == x2:
            if slope is None:  # The line is also vertical, check if they are the same line
                if abs(x1 - mid[0]) <= tolerance:  # The line coincides with the edge
                    continue
                else:  # Parallel lines, no intersection
                    return None
            else:
                intersect_x = x1
                intersect_y = slope * intersect_x + intercept
                if (
                    min(y1, y2) <= intersect_y <= max(y1, y2)
                    and abs(intersect_x - mid[0]) > tolerance
                    and abs(intersect_y - mid[1]) > tolerance
                ):
                    return intersect_x, intersect_y

        else:  # Non-vertical edge
            edge_slope = (y2 - y1) / (x2 - x1)
            edge_intercept = y1 - edge_slope * x1

            # The line is vertical, use its x-intercept (which is actually the x coordinate)
            if slope is None:
                intersect_x = intercept
                intersect_y = edge_slope * intersect_x + edge_intercept
                if (
                    min(x1, x2) <= intersect_x <= max(x1, x2)
                    and abs(intersect_y - mid[1]) > tolerance
                ):
                    return intersect_x, intersect_y

            elif slope != edge_slope:  # The line is not vertical and not parallel to the edge
                intersect_x = (edge_intercept - intercept) / (slope - edge_slope)
                intersect_y = slope * intersect_x + intercept

                if (
                    min(x1, x2) <= intersect_x <= max(x1, x2)
                    and min(y1, y2) <= intersect_y <= max(y1, y2)
                    and (
                        abs(intersect_x - mid[0]) > tolerance
                        or abs(intersect_y - mid[1]) > tolerance
                    )
                ):
                    return intersect_x, intersect_y
    return None


def split_area(area, perp, tolerance=1e-6):
    area_list = []

    # Helper function to check if y is approximately less than or equal to perp_y
    def y_leq_with_tolerance(y, perp_y):
        return abs(y) <= abs(perp_y) + tolerance

    # Helper function to check if y is approximately greater than or equal to perp_y
    def y_geq_with_tolerance(y, perp_y):
        return abs(y) >= abs(perp_y) - tolerance

    # Case when only one perpendicular point is provided
    if len(perp) == 1:
        below_or_equal, above_or_equal = [], []
        perp_y = perp[0][1]

        for point in area:
            if y_leq_with_tolerance(point[1], perp_y):
                below_or_equal.append(point)
            if y_geq_with_tolerance(point[1], perp_y):
                above_or_equal.append(point)

        area_list.append(below_or_equal)
        area_list.append(above_or_equal)

    # Case when more than one perpendicular point is provided
    else:
        for i, perp_point in enumerate(perp):
            one_area = []
            perp_y = perp_point[1]

            if i == 0:
                for point in area:
                    if y_leq_with_tolerance(point[1], perp_y):
                        one_area.append(point)

            elif i == len(perp) - 1:
                previous_perp_y = perp[i - 1][1]
                for point in area:
                    if y_geq_with_tolerance(point[1], previous_perp_y) and y_leq_with_tolerance(
                        point[1], perp_y
                    ):
                        one_area.append(point)
                area_list.append(one_area)
                one_area = []
                for point in area:
                    if y_geq_with_tolerance(point[1], perp_y):
                        one_area.append(point)

            else:
                previous_perp_y = perp[i - 1][1]
                for point in area:
                    if y_geq_with_tolerance(point[1], previous_perp_y) and y_leq_with_tolerance(
                        point[1], perp_y
                    ):
                        one_area.append(point)

            area_list.append(one_area)

    return area_list


def ray_casting_point_in_polygon(point, polygon):
    """
    Determine if a point is inside a polygon using the Ray Casting method in a Cartesian coordinate system.

    :param point: A tuple (x, y) representing the point to check.
    :param polygon: A list of tuples [(x, y), ...] representing the polygon vertices.
    :return: True if the point is inside the polygon; False otherwise.
    """
    x, y = point
    inside = False

    n = len(polygon)
    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside


import math

import numpy as np

# from calculation_helpers import *
from scipy.spatial import ConvexHull, Delaunay

x_coords = []
y_coords = []


def read_points_from_file(filename):
    try:
        with open(filename, "r") as file:
            lines = file.readlines()
            x_coords = []
            y_coords = []
            for line in lines:
                x, y = map(float, line.strip().split(", "))
                x_coords.append(x)
                y_coords.append(y)
            return x_coords, y_coords
    except FileNotFoundError:
        return None, None


def find_polygon_edges(positions):
    # Convert the list of positions to a NumPy array
    points = np.array(positions)

    # Calculate the convex hull
    print(points)
    hull = ConvexHull(points)

    # Extract the vertices of the convex hull
    hull_vertices = points[hull.vertices]

    # Convert the vertices back to a list of tuples
    edge_points = [tuple(point) for point in hull_vertices]

    # Find the rest of the points that are not on the edges
    rest_points = [point for point in positions if point not in edge_points]

    return edge_points, rest_points


def point_on_line(point_a, point_c, point_b, margin_of_error=0.0000001):
    """
    Check if point_c lies on the line defined by point_a and point_b
    within a specified margin of error.
    """
    # Calculate distances between the points
    dist_ab = haversine(point_a[0], point_a[1], point_b[0], point_b[1])
    dist_ac = haversine(point_a[0], point_a[1], point_c[0], point_c[1])
    dist_bc = haversine(point_b[0], point_b[1], point_c[0], point_c[1])

    # Check if the sum of distances AC and BC is approximately equal to AB
    return math.isclose(dist_ab, dist_ac + dist_bc, rel_tol=margin_of_error)


def check_and_move_points(list_A, list_B):
    """
    Update list A (edges) and list B (rest) by moving points from list B to list A if
    they are determined to be on an edge within a tolerance range.
    """
    updated_list_A = list_A[:]
    updated_list_B = list_B[:]

    for point in list_B:
        for i in range(len(list_A) - 1):
            if point_on_line(list_A[i], point, list_A[i + 1]):
                updated_list_A.append(point)
                updated_list_B.remove(point)
                break  # Move to the next point after finding its place

    return updated_list_A, updated_list_B


def reorder_list(start_point, list1):
    distances = []
    # Calculate the distance from start_point to each point in list1
    for point in list1:
        distance = haversine(start_point[0], start_point[1], point[0], point[1])
        distances.append((point, distance))

    # Find the nearest point and its index
    # TÌM ĐIỂM GẦN NHẤT VỚI DRONE
    nearest_point, _ = min(distances, key=lambda x: x[1])
    nearest_index = list1.index(nearest_point)

    # Reorder the list starting with the nearest point
    reordered_list = list1[int(nearest_index) :] + list1[: int(nearest_index)]

    return reordered_list


def split_at_farthest_point(start_position, reordered_points, list_2):
    # Calculate the distance of each point from the start position
    distances = [
        haversine(start_position[0], start_position[1], point[0], point[1])
        for point in reordered_points
    ]

    # Find the index of the farthest point
    farthest_index = distances.index(max(distances))  # TÌM ĐIỂM XA NHẤT VỚI DRONE

    # Split the reordered list at the farthest point, including the farthest point in the first part
    keep_points = reordered_points[: farthest_index + 1]
    move_to_list_2 = reordered_points[farthest_index + 1 :]
    list_2.extend(move_to_list_2)

    # Duplicate the last point in keep_points
    last_point = keep_points[-1]

    # Put it at the beginning of list_2
    list_2.insert(0, last_point)

    return keep_points, list_2


def find_nearest_to_A(points, point_A):
    # Calculate distances to point A and find the minimum
    distances = [haversine(point[0], point[1], point_A[0], point_A[1]) for point in points]
    min_distance_index = distances.index(min(distances))
    return points[min_distance_index]


def find_shortest_path(start_position, points_list):
    # Calculate the distance from point A to all other points
    distances_to_A = [
        haversine(start_position[0], start_position[1], point[0], point[1])
        for point in points_list
    ]

    # Identify the nearest point to point A (to be the last point)
    nearest_to_A_index = distances_to_A.index(min(distances_to_A))
    # Remove and save the nearest point
    point_nearest_to_A = points_list.pop(nearest_to_A_index)

    # Start with the beginning point, already placed from keep_points
    path = [points_list.pop(0)]

    # Use a modified nearest neighbor algorithm to construct the path
    while points_list:
        last_point = path[-1]
        nearest_index = min(
            range(len(points_list)),
            key=lambda i: haversine(
                last_point[0], last_point[1], points_list[i][0], points_list[i][1]
            ),
        )
        nearest_point = points_list.pop(nearest_index)
        path.append(nearest_point)

    # Append the point nearest to A at the end of the path
    path.append(point_nearest_to_A)

    return path


def sort_polygon_vertices(vertices):
    # Calculate centroid of the polygon
    centroid_x = sum(x for x, y in vertices) / len(vertices)
    centroid_y = sum(y for x, y in vertices) / len(vertices)
    centroid = (centroid_x, centroid_y)

    # Define a function to calculate the angle between centroid and vertex
    def angle_from_centroid(vertex):
        return math.atan2(vertex[1] - centroid_y, vertex[0] - centroid_x)

    # Sort vertices by the angle from centroid
    sorted_vertices = sorted(vertices, key=angle_from_centroid, reverse=True)

    return sorted_vertices


# def find_path(points, point_A):
#     # TÌM CẠNH CỦA ĐA GIÁC (BƯỚC NÀY TÌM ĐƯỢC GÓC CỦA ĐA GIÁC)
#     List1, List2 = find_polygon_edges(points)

#     # for i, point in enumerate(List1):
#     #     print(f"{point}")

#     # TÌM CẠNH CỦA ĐA GIÁC (BƯỚC NÀY TÌM ĐƯỢC CÁC ĐIỂM NẰM TRÊN CẠNH ĐA GIÁC)
#     list_A, list_B = check_and_move_points(List1, List2)

#     # Sắp xếp các điểm trên cạnh của đa giác theo thứ tự
#     list_A = sort_polygon_vertices(list_A)

#     # Sắp xếp các điểm trên cạnh của đa giác bắt đầu từ điểm gần drone nhất
#     list_A = reorder_list(point_A, list_A)

#     # Tách đường đi của Drone thành 2 phần, phần 1 đi từ điểm gần drone nhất đến điểm xã drone nhất theo cạnh của đa giác
#     list_A, list_B = split_at_farthest_point(point_A, list_A, list_B)
#     # phần 2 đi đường zig-zag từ điểm xa drone nhất về điểm gần drone thứ 2

#     # Assuming list_2 and point_A are defined
#     # tìm đường zig-zag ngắn nhất cho phần đường đi thứ 2
#     path_list_2 = find_shortest_path(point_A, list_B.copy())

#     list_A.pop()  # bỏ đi điểm cuối cùng của phần thứ nhất vì trùng với điểm đầu phần thứ 2
#     list_A.extend(path_list_2)  # nối 2 phần

#     return list_A

#HaoNV35 add 13/10/2025.
def find_zigzag_path(points, uav_init_point):
    # points_on_row
    i = 1
    j = 0
    n = 100
    sub_list = [[] for i in range(n)]
    sub_list[0].append(points[0])
    while i < len(points) - 1:
        if point_on_line(points[i-1], points[i], points[i+1]):
            sub_list[j].append(points[i])
            i += 1
        else:
            sub_list[j].append(points[i])
            j += 1
            sub_list[j].append(points[i+1])
            i += 2
    sub_list[j].append(points[-1])

    points_on_row = [[] for i in range(j+1)] 
    for i in range(j+1):
        points_on_row[i] = sub_list[i]
        print(f"points_on_row{i+1}: ", points_on_row[i])

    start_distance = distance_between_points(uav_init_point, points_on_row[0][0])
    end_distance = distance_between_points(uav_init_point, points_on_row[0][-1])
    if start_distance > end_distance:
        for i in range(len(points_on_row)):
            if i % 2 == 0:
                points_on_row[i].reverse()
    else:
        for i in range(len(points_on_row)):
            if i % 2 != 0:
                points_on_row[i].reverse()
    final_path = []
    for i in range(len(points_on_row)):
        final_path.extend(points_on_row[i])
    start_point = final_path[0]
    return final_path, start_point
# Duc start
def distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])
def angle_between(v1, v2):
    # Tính góc giữa hai vector (theo radian)
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    mag1 = math.sqrt(v1[0]**2 + v1[1]**2)
    mag2 = math.sqrt(v2[0]**2 + v2[1]**2)
    if mag1 == 0 or mag2 == 0:
        return 0
    cos_ang = dot / (mag1 * mag2)
    cos_ang = max(-1, min(1, cos_ang))  # tránh sai số làm cos vượt [-1,1]
    return math.acos(cos_ang)

def latlon_to_xy(lat_ref, lon_ref, lat, lon):
    """
    Chuyển đổi kinh độ, vĩ độ (lat, lon) sang tọa độ phẳng (x, y)
    tính từ điểm gốc (lat_ref, lon_ref), đơn vị: mét.
    """
    # Khoảng cách theo hướng Đông (x)
    x = haversine(lat_ref, lon_ref, lat_ref, lon)
    if lon < lon_ref:
        x = -x
    # Khoảng cách theo hướng Bắc (y)
    y = haversine(lat_ref, lon_ref, lat, lon_ref)
    if lat < lat_ref:
        y = -y
    return (x, y)

def find_path_0(points, start, turn_threshold=math.pi/6):
    unvisited = points.copy()
    path = [start]
    curr = start
    last_dir = None
    while unvisited:
        if last_dir is None:
            candidates = unvisited[:]
        else:
            no_turn = []
            for p in unvisited:
                new_dir = (p[0]-curr[0], p[1]-curr[1])
                angle = angle_between(last_dir, new_dir)
                if angle <= turn_threshold:
                    no_turn.append(p)
            candidates = no_turn if no_turn else unvisited
        best_pt = min(candidates, key=lambda p: distance(p, curr))
        best_dir = (best_pt[0]-curr[0], best_pt[1]-curr[1])
        path.append(best_pt)
        unvisited.remove(best_pt)
        curr = best_pt
        last_dir = best_dir
    return path

def nn_2opt_path(points, start):
    unvisited = points.copy()
    path = []
    curr = start
    while unvisited:
        nearest = min(unvisited, key=lambda p: distance(p, curr))
        path.append(nearest)
        unvisited.remove(nearest)
        curr = nearest
    # simple 2-opt
    improved = True
    while improved:
        improved = False
        n = len(path)
        for i in range(n-1):
            for j in range(i+2, n):
                if j == n-1 and i == 0:
                    continue
                a, b = path[i], path[i+1]
                c, d = path[j], path[(j+1) % n] if (j+1)<n else None
                old = distance(a,b) + (distance(c,d) if d else 0)
                new = distance(a,c) + (distance(b,d) if d else 0)
                if new + 1e-6 < old:
                    path[i+1:j+1] = reversed(path[i+1:j+1])
                    improved = True
    return path

def sa_path(points, start, iterations=1000):
    path = points.copy()
    random.shuffle(path)
    def cost(pth):
        dist = 0
        curr = start
        for p in pth:
            dist += distance(p, curr)
            curr = p
        return dist
    T = 100.0
    alpha = 0.995
    for _ in range(iterations):
        i, j = sorted(random.sample(range(len(path)), 2))
        new_path = path[:i] + path[i:j+1][::-1] + path[j+1:]
        dE = cost(new_path) - cost(path)
        if dE < 0 or math.exp(-dE/T) > random.random():
            path = new_path
        T *= alpha
    return path

def aco_path(points, start, ants=20, iterations=50, alpha=1, beta=3, rho=0.1, Q=100):
    """
    ACO (Ant Colony Optimization) cho bài toán tìm đường UAV qua các điểm grid.
    
    Args:
        points: danh sách các điểm cần thăm [(x,y), ...]
        start: điểm xuất phát (x, y)
        ants: số lượng kiến mô phỏng
        iterations: số vòng lặp cập nhật pheromone
        alpha: trọng số pheromone
        beta: trọng số heuristic (1/khoảng cách)
        rho: hệ số bay hơi pheromone (0–1)
        Q: hằng số điều chỉnh lượng pheromone lắng đọng
        
    Returns:
        best_path: danh sách điểm theo thứ tự tối ưu
    """
    n = len(points)
    all_points = [start] + points
    dist = [[distance(a, b) for b in all_points] for a in all_points]

    # Khởi tạo pheromone
    tau = [[1.0 for _ in range(n+1)] for _ in range(n+1)]

    best_path = []
    best_length = float('inf')

    for it in range(iterations):
        all_ant_paths = []
        all_ant_lengths = []

        for _ in range(ants):
            unvisited = set(range(1, n+1))
            curr = 0  # start index (0 = start)
            path = [curr]
            length = 0

            # Mỗi kiến di chuyển cho đến khi thăm hết các điểm
            while unvisited:
                probs = []
                for j in unvisited:
                    tau_ij = tau[curr][j] ** alpha
                    eta_ij = (1 / dist[curr][j]) ** beta if dist[curr][j] > 0 else 0
                    probs.append((j, tau_ij * eta_ij))
                total = sum(p for _, p in probs)
                if total == 0:
                    next_j = random.choice(list(unvisited))
                else:
                    # chọn điểm tiếp theo theo xác suất
                    r = random.random() * total
                    cum = 0
                    for j, p in probs:
                        cum += p
                        if cum >= r:
                            next_j = j
                            break

                path.append(next_j)
                length += dist[curr][next_j]
                curr = next_j
                unvisited.remove(next_j)

            all_ant_paths.append(path)
            all_ant_lengths.append(length)

            # Cập nhật đường đi tốt nhất
            if length < best_length:
                best_length = length
                best_path = path

        # Bay hơi pheromone
        for i in range(n+1):
            for j in range(n+1):
                tau[i][j] *= (1 - rho)

        # Lắng đọng pheromone
        for k, path in enumerate(all_ant_paths):
            Lk = all_ant_lengths[k]
            for i in range(len(path) - 1):
                a, b = path[i], path[i+1]
                tau[a][b] += Q / Lk
                tau[b][a] += Q / Lk  # đối xứng

        # Có thể in tiến trình để quan sát
        print(f"Iteration {it+1}/{iterations}, best length = {best_length:.2f}")

    # Trả về đường đi tối ưu theo thứ tự điểm (bỏ index 0)
    best_path_points = [all_points[i] for i in best_path[1:]]
    return best_path_points

def ga_path(points, start, pop_size=50, generations=300, mutation_rate=0.1, elite_size=5):
    """
    Thuật toán di truyền (Genetic Algorithm) để tối ưu đường đi UAV.
    - points: danh sách tọa độ điểm
    - start: điểm bắt đầu
    - pop_size: kích thước quần thể
    - generations: số thế hệ lặp
    - mutation_rate: xác suất đột biến
    - elite_size: số cá thể tốt nhất giữ lại mỗi thế hệ
    """

    # --- 1. Hàm tính tổng chiều dài quãng đường ---
    def total_distance(route):
        dist = 0
        curr = start
        for p in route:
            dist += distance(curr, p)
            curr = p
        return dist

    # --- 2. Khởi tạo quần thể ban đầu ---
    population = []
    for _ in range(pop_size):
        individual = points[:]  # copy danh sách điểm
        random.shuffle(individual)
        population.append(individual)

    # --- 3. Hàm chọn lọc: chọn cá thể tốt nhất ---
    def selection(pop):
        ranked = sorted(pop, key=lambda r: total_distance(r))
        return ranked[:elite_size]  # giữ lại elite_size cá thể tốt nhất

    # --- 4. Lai ghép (crossover) giữa 2 cá thể ---
    def crossover(p1, p2):
        a, b = sorted(random.sample(range(len(p1)), 2))
        child = [None]*len(p1)
        child[a:b] = p1[a:b]
        fill = [x for x in p2 if x not in child]
        idx = 0
        for i in range(len(p1)):
            if child[i] is None:
                child[i] = fill[idx]
                idx += 1
        return child

    # --- 5. Đột biến (mutation) ---
    def mutate(route):
        for i in range(len(route)):
            if random.random() < mutation_rate:
                j = random.randint(0, len(route)-1)
                route[i], route[j] = route[j], route[i]
        return route

    # --- 6. Tiến hóa qua nhiều thế hệ ---
    best_route = None
    best_dist = float('inf')

    for _ in range(generations):
        selected = selection(population)
        new_pop = selected[:]  # giữ lại elite
        while len(new_pop) < pop_size:
            p1, p2 = random.sample(selected, 2)
            child = crossover(p1, p2)
            child = mutate(child)
            new_pop.append(child)
        population = new_pop

        # cập nhật cá thể tốt nhất
        curr_best = min(population, key=lambda r: total_distance(r))
        curr_dist = total_distance(curr_best)
        if curr_dist < best_dist:
            best_dist = curr_dist
            best_route = curr_best[:]

    return best_route


def abc_path(points, start, colony_size=30, limit=20, iterations=100):
    """
    ABC (Artificial Bee Colony) cho bài toán tìm đường UAV.
    Mỗi "bee" đại diện cho một route (hoán vị các điểm).
    """
    n = len(points)

    def total_distance(route):
        dist = 0
        curr = start
        for p in route:
            dist += distance(curr, p)
            curr = p
        return dist

    # --- Khởi tạo quần thể ---
    food_sources = [random.sample(points, n) for _ in range(colony_size)]
    fitness = [1 / (1 + total_distance(p)) for p in food_sources]
    trial = [0] * colony_size

    best_route = min(food_sources, key=lambda r: total_distance(r))
    best_dist = total_distance(best_route)

    for it in range(iterations):
        # --- Pha Employed Bee ---
        for i in range(colony_size):
            k = random.choice([x for x in range(colony_size) if x != i])
            new_solution = food_sources[i][:]
            a, b = random.sample(range(n), 2)
            new_solution[a], new_solution[b] = new_solution[b], new_solution[a]
            if total_distance(new_solution) < total_distance(food_sources[i]):
                food_sources[i] = new_solution
                trial[i] = 0
            else:
                trial[i] += 1

        # --- Pha Onlooker Bee ---
        prob = [f / sum(fitness) for f in fitness]
        for i in range(colony_size):
            if random.random() < prob[i]:
                k = random.choice([x for x in range(colony_size) if x != i])
                new_solution = food_sources[i][:]
                a, b = random.sample(range(n), 2)
                new_solution[a], new_solution[b] = new_solution[b], new_solution[a]
                if total_distance(new_solution) < total_distance(food_sources[i]):
                    food_sources[i] = new_solution
                    trial[i] = 0
                else:
                    trial[i] += 1

        # --- Pha Scout Bee ---
        for i in range(colony_size):
            if trial[i] > limit:
                food_sources[i] = random.sample(points, n)
                trial[i] = 0

        # Cập nhật fitness và cá thể tốt nhất
        fitness = [1 / (1 + total_distance(p)) for p in food_sources]
        curr_best = min(food_sources, key=lambda r: total_distance(r))
        curr_dist = total_distance(curr_best)
        if curr_dist < best_dist:
            best_dist = curr_dist
            best_route = curr_best[:]

        print(f"ABC Iter {it+1}/{iterations}: best distance = {best_dist:.3f}")

    return best_route

def ga_path_with_turns(points, start, pop_size=50, generations=200, mutation_rate=0.1, elite_size=5):
    """
    GA tối ưu đường đi: hàm fitness = distance + turn_penalty
    """
    def total_cost(route):
        cost = 0
        curr = start
        prev_vector = None
        for p in route:
            cost += distance(curr, p)
            vector = (p[0]-curr[0], p[1]-curr[1])
            if prev_vector:
                # tính cosine góc giữa 2 vector → tăng penalty nếu quay
                dot = prev_vector[0]*vector[0] + prev_vector[1]*vector[1]
                mag = math.hypot(*prev_vector) * math.hypot(*vector)
                if mag > 0:
                    cos_angle = dot / mag
                    if abs(cos_angle) < 0.99:  # góc lớn → tính như quay
                        cost += 0.1 * distance(curr, p)  # thêm penalty
            prev_vector = vector
            curr = p
        return cost
    
    # --- Khởi tạo quần thể ---
    population = [random.sample(points, len(points)) for _ in range(pop_size)]
    
    best_route = None
    best_cost_val = float('inf')
    
    for _ in range(generations):
        # Chọn lọc
        ranked = sorted(population, key=total_cost)
        new_pop = ranked[:elite_size]  # giữ elite
        while len(new_pop) < pop_size:
            p1, p2 = random.sample(ranked[:elite_size], 2)
            a, b = sorted(random.sample(range(len(p1)), 2))
            # Order crossover
            child = [None]*len(p1)
            child[a:b] = p1[a:b]
            fill = [x for x in p2 if x not in child]
            idx = 0
            for i in range(len(child)):
                if child[i] is None:
                    child[i] = fill[idx]
                    idx += 1
            # Mutation
            if random.random() < mutation_rate:
                i, j = random.sample(range(len(child)), 2)
                child[i], child[j] = child[j], child[i]
            new_pop.append(child)
        population = new_pop
        # Cập nhật best
        curr_best = min(population, key=total_cost)
        curr_cost = total_cost(curr_best)
        if curr_cost < best_cost_val:
            best_cost_val = curr_cost
            best_route = curr_best[:]
    
    return best_route
def astar_path_with_turns(points, start):
    """
    A* giữa grid points, chi phí = distance + turn penalty
    """
    unvisited = set(points)
    path = [start]
    curr = start
    prev_vector = None

    while unvisited:
        best_p = None
        best_cost = float('inf')
        for p in unvisited:
            vector = (p[0]-curr[0], p[1]-curr[1])
            cost = distance(curr, p)
            if prev_vector:
                dot = prev_vector[0]*vector[0] + prev_vector[1]*vector[1]
                mag = math.hypot(*prev_vector) * math.hypot(*vector)
                if mag > 0:
                    cos_angle = dot / mag
                    if abs(cos_angle) < 0.99:
                        cost += 0.1 * distance(curr, p)
            if cost < best_cost:
                best_cost = cost
                best_p = p
        path.append(best_p)
        unvisited.remove(best_p)
        prev_vector = (best_p[0]-curr[0], best_p[1]-curr[1])
        curr = best_p

    return path
# Tinh toan chi phi:
def calculate_cost_for_path(path, turn_angle_threshold_deg=1.0, turn_penalty=150.0):
    """
    path: danh sách các điểm [(lon, lat), (lon, lat), ...]
    """
    if not path or len(path) == 1:
        return 0.0, 0.0, 0  # cost, total_dist, turns

    total_dist = 0.0
    turns = 0
    last_dir = None

    # Lấy điểm gốc làm tham chiếu tọa độ phẳng
    lon_ref, lat_ref = path[0]

    for i in range(1, len(path)):
        lon1, lat1 = path[i - 1]
        lon2, lat2 = path[i]
        d = haversine(lat1, lon1, lat2, lon2)  # ✅ gọi đúng kiểu bạn định nghĩa
        total_dist += d

        # --- 2️⃣ Tính hướng theo hệ tọa độ phẳng ---
        p1 = latlon_to_xy(lat_ref, lon_ref, lat1, lon1)
        p2 = latlon_to_xy(lat_ref, lon_ref, lat2, lon2)
        new_dir = (p2[0] - p1[0], p2[1] - p1[1])
      
        if last_dir is not None:
            ang = math.degrees(angle_between(last_dir, new_dir))
            if ang > turn_angle_threshold_deg:
                turns += 1
        last_dir = new_dir

    cost = total_dist + turns * turn_penalty
    return cost, total_dist, turns

def best_path_sw_uav(points, uav_init_point):
    print("=== 🚀 Running find_zigzag_path to get start point ===")
    zigzag_path, start_point = find_zigzag_path(points.copy(), uav_init_point)
    print(f"✅ Start point for all algorithms: {start_point}")

    # --- Chạy tất cả thuật toán với cùng điểm start ---
    try:
        path_find = find_path_0(points.copy(), start_point)
        if isinstance(path_find, tuple):
            path_find = path_find[0]
    except Exception:
        path_find = []

    try:
        path_nn2opt = nn_2opt_path(points.copy(), start_point)
    except Exception:
        path_nn2opt = []

    try:
        path_sa = sa_path(points.copy(), start_point)
    except Exception:
        path_sa = []

    try:
        path_aco = aco_path(points.copy(), start_point)
    except Exception:
        path_aco = []

    try:
        path_ga = ga_path(points.copy(), start_point)
    except Exception:
        path_ga = []

    try:
        path_abc = abc_path(points.copy(), start_point)
    except Exception:
        path_abc = []

    try:
        path_ga_with_turns = ga_path_with_turns(points.copy(), start_point)
    except Exception:
        path_ga_with_turns = []

    try:
        path_A = astar_path_with_turns(points.copy(), start_point)
    except Exception:
        path_A = []

    algos = {
        "Zigzag": zigzag_path,
        "Find_Path": path_find,
        "NN_2opt": path_nn2opt,
        "SA": path_sa,
        "ACO": path_aco,
        "GA": path_ga,
        "ABC": path_abc,
        "GA_with_turn": path_ga_with_turns,
        "A*_Improved": path_A
    }

    # --- So sánh chi phí ---
    best_name = None
    best_cost_val = float("inf")
    best_path = None

    for name, path in algos.items():
        if not path:
            continue
        cost, total_dist, turns = calculate_cost_for_path(path)
        print(f"Algorithm {name}: cost={cost:.3f}, dist={total_dist:.3f}, turns={turns}")
        if cost < best_cost_val:
            best_cost_val = cost
            best_name = name
            best_path = path

    print(f"==> ✅ Selected algorithm: {best_name} (cost={best_cost_val:.3f})")
    return best_path



# Duc end
def calculate_grid_size_from_hfov_and_vfov(h_fov, v_fov, uav_alt):
    grid_width = 2 * uav_alt * math.tan(math.radians(h_fov / 2)) 
    grid_height = 2 * uav_alt * math.tan(math.radians(v_fov / 2)) 
    return grid_width, grid_height

def calculate_overlapped_grid_size(grid_width, grid_height, h_overlap, v_overlap):
    overlapped_grid_width = grid_width - h_overlap * grid_width
    overlapped_grid_height = grid_height - v_overlap * grid_height
    return overlapped_grid_width, overlapped_grid_height

def find_line_segment_intersection(line, segment_point1, segment_point2):
    x1, y1 = segment_point1 
    x2, y2 = segment_point2
    if y1 == y2:
        return None
    if(y1 <= line <= y2) or (y2 <= line <= y1):
        ratio = (line - y1) / (y2 - y1)
        if 0 <= ratio <= 1:
            x_line = x1 + ratio * (x2 - x1)
            return (x_line, line)
    return None

def find_parallel_polygon_intersection(area_vertices, spacing, number_of_lines):
    longest_edge_length, longest_edge_endpoints = find_longest_edge(area_vertices)
    x1, y1 = longest_edge_endpoints[0]
    x2, y2 = longest_edge_endpoints[1]
    area_min_y = min(v[1] for v in area_vertices)
    area_max_y = max(v[1] for v in area_vertices)

    intersection_points = []
    for i in range(-number_of_lines, number_of_lines + 1):
        if i == 0:
            continue
        line = y1 + i * spacing
        if area_min_y < line < area_max_y:
            for j in range(len(area_vertices)):
                x3, y3 = area_vertices[j]
                x4, y4 = area_vertices[(j+1) % len(area_vertices)]
                intersection = find_line_segment_intersection(line, (x3, y3), (x4, y4))
                if intersection:
                    intersection_points.append(intersection)
    print("Number of intersection points:", len(intersection_points))
    if intersection_points[0][1] > y1:
        is_up = True
        intersection_points.sort(key=lambda x: x[1])
    else: 
        is_up = False
        intersection_points.sort(key=lambda x: x[1], reverse=True) # sort by y coordinate
    print("Intersection points:", intersection_points)

    segment_length = []
    for i in range(1, len(intersection_points), 2):
        point1 = intersection_points[i]
        point2 = intersection_points[i-1]
        length = np.linalg.norm(np.array(point1) - np.array(point2))
        print("Length:", length)
        if length > 0: 
            segment_length.append(length)
    print("Segment lengths:", segment_length)
    return intersection_points, segment_length, is_up