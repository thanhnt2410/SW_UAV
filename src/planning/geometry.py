import math
import numpy as np
from shapely.geometry import LineString, Point as ShapelyPoint, Polygon as ShapelyPolygon
from shapely import BufferCapStyle, BufferJoinStyle
from sympy import Polygon as SympyPolygon
from scipy.spatial import ConvexHull

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
    min_lat = min(positions, key=lambda x: x[0])[0]
    min_lon = min(positions, key=lambda x: x[1])[1]

    cartesian_coords = []

    for lat, lon in positions:
        x = haversine(min_lat, min_lon, min_lat, lon)
        y = haversine(min_lat, min_lon, lat, min_lon)
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


def distance_between_points(p1, p2):
    """Calculate the distance between two points in meters."""
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
    """Find the intersection point of a line segment and a line."""
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

    dot_prod = ba[0] * bc[0] + ba[1] * bc[1]
    mag_ab = math.sqrt(ba[0] ** 2 + ba[1] ** 2)
    mag_bc = math.sqrt(bc[0] ** 2 + bc[1] ** 2)

    if mag_ab == 0 or mag_bc == 0:
        return 0

    cosine_angle = (dot_prod) / (mag_ab * mag_bc)
    angle = math.acos(cosine_angle)
    cross_product = ba[0] * bc[1] - ba[1] * bc[0]
    angle_degrees = math.degrees(angle)
    if cross_product > 0:
        angle_degrees = 360 - angle_degrees
    return angle_degrees


def heron_formula(a, b, c):
    """Apply Heron's formula to calculate the area of a triangle given its side lengths."""
    s = (a + b + c) / 2
    area = math.sqrt(s * (s - a) * (s - b) * (s - c))
    return area


def find_longest_edge2(cartesian_coords):
    """Find the longest edge in a polygon."""
    num_vertices = len(cartesian_coords)
    longest_edge_length = 0
    longest_edge_vertices = (None, None)

    for i in range(num_vertices):
        x1, y1 = cartesian_coords[i]
        x2, y2 = cartesian_coords[(i + 1) % num_vertices]

        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        if distance > longest_edge_length:
            longest_edge_length = distance
            longest_edge_vertices = ((x1, y1), (x2, y2))

    return longest_edge_length, longest_edge_vertices


def find_midpoint(point1, point2):
    x1, y1 = point1
    x2, y2 = point2

    mid_x = (x1 + x2) / 2.0
    mid_y = (y1 + y2) / 2.0

    return (mid_x, mid_y)


def line_equation_from_points(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    if x1 == x2:
        return None, x1

    elif y1 == y2:
        return 0, y1

    else:
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
    mx, my = midpoint

    if slope is not None:
        if -tolerance < slope < tolerance:
            return None, my

    elif slope is None:
        return 0, my

    perp_slope = -1 / slope
    perp_intercept = my - perp_slope * mx
    return perp_slope, perp_intercept


def calculate_new_lat_lon(origin_lat, origin_lon, distance_north, distance_east):
    """Calculate new latitude and longitude from origin given distances north and east."""
    R = 6378000  # Radius of Earth in meters
    delta_lat = distance_north / R  # Change in latitude in radians
    new_lat = origin_lat + math.degrees(delta_lat)  # New latitude in degrees

    r = R * math.cos(math.radians(new_lat))  # Effective radius at new latitude
    delta_lon = distance_east / r  # Change in longitude in radians
    new_lon = origin_lon + math.degrees(delta_lon)  # New longitude in degrees

    return (new_lat, new_lon)


def divide_line_into_segments(x1, y1, x2, y2, n):
    points = []
    for i in range(1, n):
        t = i / n
        xt = (1 - t) * x1 + t * x2
        yt = (1 - t) * y1 + t * y2
        points.append((xt, yt))

    return points

def rotate_and_shift_point(
    x, y, angle, pivot_x, pivot_y, shift_x=0, shift_y=0, units="DEGREES", clockwise=False
):
    if units.upper() == "DEGREES":
        angle = math.radians(angle)

    if clockwise:
        angle = -angle

    x -= pivot_x
    y -= pivot_y

    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)
    x_rotated = (x * cos_theta) - (y * sin_theta)
    y_rotated = (x * sin_theta) + (y * cos_theta)

    x_final = x_rotated + pivot_x + shift_x
    y_final = y_rotated + pivot_y + shift_y

    return x_final, y_final


def revert_rotate_and_shift_point(
    x, y, angle, pivot_x, pivot_y, shift_x=0, shift_y=0, units="DEGREES", clockwise=False
):
    x -= shift_x
    y -= shift_y

    if units.upper() == "DEGREES":
        angle = math.radians(angle)

    if not clockwise:
        angle = -angle

    x -= pivot_x
    y -= pivot_y

    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)
    x_reverted = (x * cos_theta) + (y * sin_theta)
    y_reverted = (-x * sin_theta) + (y * cos_theta)

    x_final = x_reverted + pivot_x
    y_final = y_reverted + pivot_y

    return x_final, y_final


def distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def angle_between(v1, v2):
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    mag1 = math.sqrt(v1[0]**2 + v1[1]**2)
    mag2 = math.sqrt(v2[0]**2 + v2[1]**2)
    if mag1 == 0 or mag2 == 0:
        return 0
    cos_ang = dot / (mag1 * mag2)
    cos_ang = max(-1, min(1, cos_ang))
    return math.acos(cos_ang)


def latlon_to_xy(lat_ref, lon_ref, lat, lon):
    """
    Chuyển đổi kinh độ, vĩ độ (lat, lon) sang tọa độ phẳng (x, y)
    tính từ điểm gốc (lat_ref, lon_ref), đơn vị: mét.
    """
    x = haversine(lat_ref, lon_ref, lat_ref, lon)
    if lon < lon_ref:
        x = -x
    y = haversine(lat_ref, lon_ref, lat, lon_ref)
    if lat < lat_ref:
        y = -y
    return (x, y)


def is_polygon_convex(points):
    """Check if a polygon defined by a list of points is convex."""
    polygon = SympyPolygon(*points)
    return polygon.is_convex()


def sort_polygon_vertices(vertices):
    centroid_x = sum(x for x, y in vertices) / len(vertices)
    centroid_y = sum(y for x, y in vertices) / len(vertices)
    centroid = (centroid_x, centroid_y)

    def angle_from_centroid(vertex):
        return math.atan2(vertex[1] - centroid_y, vertex[0] - centroid_x)

    sorted_vertices = sorted(vertices, key=angle_from_centroid, reverse=True)

    return sorted_vertices


def find_polygon_edges(positions):
    points = np.array(positions)

    print(points)
    hull = ConvexHull(points)

    hull_vertices = points[hull.vertices]

    edge_points = [tuple(point) for point in hull_vertices]

    rest_points = [point for point in positions if point not in edge_points]

    return edge_points, rest_points


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
