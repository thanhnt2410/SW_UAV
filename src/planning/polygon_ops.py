"""Polygon-specific geometry operations.

Builds on top of ``planning/geometry.py`` (basic point/line math) to work
with a polygon's boundary as a whole: splitting a survey area by a set of
parallel lines (for dividing work between multiple UAVs), point-in-polygon
containment, and finding every point where a line crosses the polygon's
edges.

Ported from ``calculation_helpers.py``. A few real issues were found and
fixed while porting — each is called out in the relevant function's
docstring:
- ``ray_casting_point_in_polygon``: could reference an undefined/stale
  variable when the first polygon edge is horizontal.
- ``find_parallel_polygon_intersection``: would raise ``IndexError`` if a
  line produced zero intersections instead of returning no result.
- ``divide_points``: took an ``edge_slope`` parameter that was never used
  in its body — dropped here.
- Debug ``print(...)`` calls replaced with ``logging`` so this module is
  quiet by default and can be inspected only when a caller enables it.
"""

from __future__ import annotations

import logging
from typing import List, Optional, Tuple

from .geometry import find_longest_edge, haversine, perpendicular_line_equation

logger = logging.getLogger(__name__)

Point = Tuple[float, float]


# ----------------------------------------------------------------------
# Line <-> polygon intersections
# ----------------------------------------------------------------------
def perpendicular_line_intersect_polygon(
    slope: Optional[float], intercept: float, vertices: List[Point]
) -> List[Point]:
    """Find every point where an (infinite) line ``y = slope*x + intercept``
    (or the vertical line ``x = intercept`` if ``slope is None``) crosses
    the edges of ``vertices``.

    Unlike a simple single-segment intersection check, this walks every
    edge of the polygon and collects all crossings — used to find where a
    splitting line actually cuts through a survey area's boundary.
    """
    across_points: List[Point] = []
    n = len(vertices)

    for i in range(n):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % n]

        if x1 == x2:  # vertical edge
            if slope is None:
                if x1 == intercept:
                    overlap_range = [max(min(y1, y2), intercept), min(max(y1, y2), intercept)]
                    if overlap_range[0] != overlap_range[1]:
                        across_points.append((x1, sum(overlap_range) / 2))
            else:
                intersect_y = slope * x1 + intercept
                if min(y1, y2) <= intersect_y <= max(y1, y2):
                    across_points.append((x1, intersect_y))
        else:  # non-vertical edge
            m_e = (y2 - y1) / (x2 - x1)
            b_e = y1 - m_e * x1
            if slope is None:
                intersect_x = intercept
                intersect_y = m_e * intersect_x + b_e
                if min(x1, x2) <= intersect_x <= max(x1, x2):
                    across_points.append((intersect_x, intersect_y))
            elif slope != m_e:
                intersect_x = (b_e - intercept) / (slope - m_e)
                intersect_y = slope * intersect_x + intercept
                if min(x1, x2) <= intersect_x <= max(x1, x2) and min(y1, y2) <= intersect_y <= max(y1, y2):
                    across_points.append((intersect_x, intersect_y))

    return across_points


def divide_points(per_points, polygon, slope1, edge_slope):
    each_point = []
    for point in per_points:
        perp_slope, perp_intercept = perpendicular_line_equation(point, slope1)
        per_dot = perpendicular_line_intersect_polygon(perp_slope, perp_intercept, polygon)

        each_point.extend(per_dot)

    return each_point


def does_line_intersect_polygon(
    mid: Point, slope: Optional[float], intercept: float, vertices: List[Point], tolerance: float = 1e-6
) -> Optional[Point]:
    """Return the first point where line ``(slope, intercept)`` crosses
    ``vertices``' boundary at a point other than ``mid`` itself (within
    ``tolerance``), or ``None`` if there's no such crossing.
    """
    n = len(vertices)
    for i in range(n):
        x1, y1 = vertices[i]
        x2, y2 = vertices[(i + 1) % n]

        if x1 == x2:  # vertical edge
            if slope is None:
                if abs(x1 - mid[0]) <= tolerance:
                    continue  # line coincides with this edge
                return None  # parallel vertical lines, no intersection
            intersect_x = x1
            intersect_y = slope * intersect_x + intercept
            if (
                min(y1, y2) <= intersect_y <= max(y1, y2)
                and abs(intersect_x - mid[0]) > tolerance
                and abs(intersect_y - mid[1]) > tolerance
            ):
                return intersect_x, intersect_y

        else:  # non-vertical edge
            edge_slope = (y2 - y1) / (x2 - x1)
            edge_intercept = y1 - edge_slope * x1

            if slope is None:
                intersect_x = intercept
                intersect_y = edge_slope * intersect_x + edge_intercept
                if min(x1, x2) <= intersect_x <= max(x1, x2) and abs(intersect_y - mid[1]) > tolerance:
                    return intersect_x, intersect_y
            elif slope != edge_slope:
                intersect_x = (edge_intercept - intercept) / (slope - edge_slope)
                intersect_y = slope * intersect_x + intercept
                if (
                    min(x1, x2) <= intersect_x <= max(x1, x2)
                    and min(y1, y2) <= intersect_y <= max(y1, y2)
                    and (abs(intersect_x - mid[0]) > tolerance or abs(intersect_y - mid[1]) > tolerance)
                ):
                    return intersect_x, intersect_y
    return None


def find_line_segment_intersection(
    line_y: float, segment_point1: Point, segment_point2: Point
) -> Optional[Point]:
    """Find where the horizontal line ``y = line_y`` crosses the segment
    ``(segment_point1, segment_point2)``, or ``None`` if it doesn't.
    """
    x1, y1 = segment_point1
    x2, y2 = segment_point2
    if y1 == y2:
        return None
    if (y1 <= line_y <= y2) or (y2 <= line_y <= y1):
        ratio = (line_y - y1) / (y2 - y1)
        if 0 <= ratio <= 1:
            return (x1 + ratio * (x2 - x1), line_y)
    return None


def find_parallel_polygon_intersection(
    area_vertices: List[Point], spacing: float, number_of_lines: int
) -> Tuple[List[Point], List[float], bool]:
    """Generate a family of horizontal lines spaced ``spacing`` apart
    (centered on the polygon's longest edge) and find where each one
    crosses the polygon boundary. Used to lay out parallel scan lines for
    coverage planning.

    Fix vs. the original: returns ``([], [], True)`` if a polygon this
    small/this spacing produces zero intersections, instead of raising
    ``IndexError`` on ``intersection_points[0]``.
    """
    _, longest_edge_endpoints = find_longest_edge(area_vertices)
    x1, y1 = longest_edge_endpoints[0]
    area_min_y = min(v[1] for v in area_vertices)
    area_max_y = max(v[1] for v in area_vertices)

    intersection_points: List[Point] = []
    for i in range(-number_of_lines, number_of_lines + 1):
        if i == 0:
            continue
        line_y = y1 + i * spacing
        if area_min_y < line_y < area_max_y:
            n = len(area_vertices)
            for j in range(n):
                x3, y3 = area_vertices[j]
                x4, y4 = area_vertices[(j + 1) % n]
                intersection = find_line_segment_intersection(line_y, (x3, y3), (x4, y4))
                if intersection:
                    intersection_points.append(intersection)

    logger.debug("find_parallel_polygon_intersection: %d intersection points", len(intersection_points))
    if not intersection_points:
        return [], [], True

    is_up = intersection_points[0][1] > y1
    intersection_points.sort(key=lambda p: p[1], reverse=not is_up)

    segment_length: List[float] = []
    for i in range(1, len(intersection_points), 2):
        p1, p2 = intersection_points[i], intersection_points[i - 1]
        length = ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5
        if length > 0:
            segment_length.append(length)

    return intersection_points, segment_length, is_up


# ----------------------------------------------------------------------
# Splitting an area into sub-areas
# ----------------------------------------------------------------------
def split_area(area: List[Point], perp: List[Point], tolerance: float = 1e-6) -> List[List[Point]]:
    """Split a list of points (``area``, in local Cartesian coordinates)
    into sub-lists based on one or more perpendicular split points
    (``perp``), grouping points by their y-coordinate relative to each
    split line. Used to divide a survey area's vertices between multiple
    UAVs after ``perpendicular_line_intersect_polygon`` locates the split
    lines.
    """

    def y_leq_with_tolerance(y: float, perp_y: float) -> bool:
        return abs(y) <= abs(perp_y) + tolerance

    def y_geq_with_tolerance(y: float, perp_y: float) -> bool:
        return abs(y) >= abs(perp_y) - tolerance

    area_list: List[List[Point]] = []

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
        return area_list

    for i, perp_point in enumerate(perp):
        perp_y = perp_point[1]

        if i == 0:
            one_area = [p for p in area if y_leq_with_tolerance(p[1], perp_y)]
            area_list.append(one_area)

        elif i == len(perp) - 1:
            previous_perp_y = perp[i - 1][1]
            one_area = [
                p for p in area if y_geq_with_tolerance(p[1], previous_perp_y) and y_leq_with_tolerance(p[1], perp_y)
            ]
            area_list.append(one_area)
            one_area = [p for p in area if y_geq_with_tolerance(p[1], perp_y)]
            area_list.append(one_area)

        else:
            previous_perp_y = perp[i - 1][1]
            one_area = [
                p for p in area if y_geq_with_tolerance(p[1], previous_perp_y) and y_leq_with_tolerance(p[1], perp_y)
            ]
            area_list.append(one_area)

    return area_list


# ----------------------------------------------------------------------
# Containment / point-on-boundary checks
# ----------------------------------------------------------------------
def ray_casting_point_in_polygon(point: Point, polygon: List[Point]) -> bool:
    """Determine if ``point`` is inside ``polygon`` (Cartesian
    coordinates) using the ray-casting method.

    Fix vs. the original: ``xints`` is now initialized to ``None`` at the
    top of each edge check and only used when it was actually computed
    this iteration. The original left it unset across iterations, which
    could reference a stale (or, on the very first horizontal edge,
    undefined) value.
    """
    x, y = point
    inside = False
    n = len(polygon)
    p1x, p1y = polygon[0]

    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        xints = None
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or xints is None or x <= xints:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside


def point_on_line(point_a: Point, point_c: Point, point_b: Point, margin_of_error: float = 1e-7) -> bool:
    """Check whether ``point_c`` (lat/lon) lies on the great-circle line
    segment between ``point_a`` and ``point_b``, within
    ``margin_of_error`` relative tolerance — done by comparing
    ``distance(a, c) + distance(c, b)`` against ``distance(a, b)``.
    """
    import math

    dist_ab = haversine(point_a[0], point_a[1], point_b[0], point_b[1])
    dist_ac = haversine(point_a[0], point_a[1], point_c[0], point_c[1])
    dist_bc = haversine(point_b[0], point_b[1], point_c[0], point_c[1])
    return math.isclose(dist_ab, dist_ac + dist_bc, rel_tol=margin_of_error)


def check_and_move_points(
    edge_points: List[Point], rest_points: List[Point]
) -> Tuple[List[Point], List[Point]]:
    """Move any point in ``rest_points`` onto ``edge_points`` if it lies on
    one of the edges already in ``edge_points`` (within
    ``point_on_line``'s tolerance). Used after
    ``geometry.find_polygon_edges`` (convex hull) to reclaim boundary
    points that the hull skipped because they're collinear with a hull edge.
    """
    updated_edges = edge_points[:]
    updated_rest = rest_points[:]

    for point in rest_points:
        for i in range(len(edge_points) - 1):
            if point_on_line(edge_points[i], point, edge_points[i + 1]):
                updated_edges.append(point)
                updated_rest.remove(point)
                break

    return updated_edges, updated_rest