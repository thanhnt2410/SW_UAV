import math
import random
import statistics
import numpy as np
from scipy.spatial import ConvexHull
from shapely.geometry import LineString, Polygon as ShapelyPolygon
from shapely import BufferCapStyle, BufferJoinStyle

from .geometry import haversine, distance_between_points, distance, angle_between, latlon_to_xy
from .polygon_ops import point_on_line


def _split_zigzag_rows(points):
    if len(points) <= 2:
        return [points.copy()]

    i = 1
    j = 0
    sub_list = [[] for _ in range(len(points))]
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

    return [row for row in sub_list[:j + 1] if row]


def _zigzag_row_sequence(row_count, start_row, prefer_forward):
    if prefer_forward:
        return list(range(start_row, row_count)) + list(range(start_row - 1, -1, -1))
    return list(range(start_row, -1, -1)) + list(range(start_row + 1, row_count))


def _zigzag_candidate_from_rows(rows, row_sequence, first_row_reversed):
    final_path = []
    for i, row_index in enumerate(row_sequence):
        row = rows[row_index].copy()
        should_reverse = first_row_reversed if i % 2 == 0 else not first_row_reversed
        if should_reverse:
            row.reverse()
        final_path.extend(row)
    return final_path


def _path_distance_from_start(path, start):
    total = 0
    curr = start
    for point in path:
        total += distance_between_points(curr, point)
        curr = point
    return total


def find_zigzag_path(points, uav_init_point):
    rows = _split_zigzag_rows(points)
    if not rows:
        return [], None

    best_path = []
    best_distance = float("inf")

    for start_row in range(len(rows)):
        for prefer_forward in (True, False):
            row_sequence = _zigzag_row_sequence(len(rows), start_row, prefer_forward)
            for first_row_reversed in (False, True):
                candidate_path = _zigzag_candidate_from_rows(
                    rows, row_sequence, first_row_reversed
                )
                candidate_distance = _path_distance_from_start(candidate_path, uav_init_point)
                if candidate_distance < best_distance:
                    best_distance = candidate_distance
                    best_path = candidate_path

    final_path = best_path
    start_point = final_path[0]
    return final_path, start_point

def find_path_0(points, start, turn_threshold=math.pi/6):
    unvisited = [
        (p, latlon_to_xy(start[0], start[1], p[0], p[1]))
        for p in points if p != start
    ]
    path = [start]
    curr_xy = (0.0, 0.0)
    last_dir = None

    nearest_spacing = []
    for i, (_, point_xy) in enumerate(unvisited):
        distances = [
            distance(point_xy, other_xy)
            for j, (_, other_xy) in enumerate(unvisited)
            if i != j and distance(point_xy, other_xy) > 0
        ]
        if distances:
            nearest_spacing.append(min(distances))
    turn_weight = 4 * statistics.median(nearest_spacing) if nearest_spacing else 0.0

    while unvisited:
        def travel_cost(candidate):
            _, point_xy = candidate
            new_dir = (point_xy[0] - curr_xy[0], point_xy[1] - curr_xy[1])
            angle = angle_between(last_dir, new_dir) if last_dir is not None else 0.0
            return distance(point_xy, curr_xy) + turn_weight * max(0.0, angle - turn_threshold)

        best_pt, best_xy = min(unvisited, key=travel_cost)
        best_dir = (best_xy[0] - curr_xy[0], best_xy[1] - curr_xy[1])
        path.append(best_pt)
        unvisited.remove((best_pt, best_xy))
        curr_xy = best_xy
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
    n = len(points)
    all_points = [start] + points
    dist = [[distance(a, b) for b in all_points] for a in all_points]

    tau = [[1.0 for _ in range(n+1)] for _ in range(n+1)]

    best_path = []
    best_length = float('inf')

    for it in range(iterations):
        all_ant_paths = []
        all_ant_lengths = []

        for _ in range(ants):
            unvisited = set(range(1, n+1))
            curr = 0
            path = [curr]
            length = 0

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

            if length < best_length:
                best_length = length
                best_path = path

        for i in range(n+1):
            for j in range(n+1):
                tau[i][j] *= (1 - rho)

        for k, path in enumerate(all_ant_paths):
            Lk = all_ant_lengths[k]
            for i in range(len(path) - 1):
                a, b = path[i], path[i+1]
                tau[a][b] += Q / Lk
                tau[b][a] += Q / Lk

        print(f"Iteration {it+1}/{iterations}, best length = {best_length:.2f}")

    best_path_points = [all_points[i] for i in best_path[1:]]
    return best_path_points

def _ga_crossover(parent_1, parent_2):
    """Cross over waypoint indices so equal coordinates remain distinct visits."""
    start_idx, end_idx = sorted(random.sample(range(len(parent_1)), 2))
    child = [None] * len(parent_1)
    child[start_idx:end_idx + 1] = parent_1[start_idx:end_idx + 1]
    used = set(child[start_idx:end_idx + 1])
    remaining = (index for index in parent_2 if index not in used)
    for i in range(len(child)):
        if child[i] is None:
            child[i] = next(remaining)
    return child


def _route_cost(route, xy_points, turn_weight=0.0):
    cost = 0.0
    current = (0.0, 0.0)
    previous_direction = None
    for index in route:
        point = xy_points[index]
        direction = (point[0] - current[0], point[1] - current[1])
        length = math.hypot(*direction)
        cost += length
        if turn_weight and previous_direction is not None and length:
            cost += turn_weight * angle_between(previous_direction, direction)
        if length:
            previous_direction = direction
        current = point
    return cost


def _ga_grid_spacing(xy_points):
    nearest_distances = []
    for i, point in enumerate(xy_points):
        nearest = float('inf')
        for j, other in enumerate(xy_points):
            if i != j:
                gap = distance(point, other)
                if 0 < gap < nearest:
                    nearest = gap
        if nearest < float('inf'):
            nearest_distances.append(nearest)
    return statistics.median(nearest_distances) if nearest_distances else 0.0


def _seed_survey_routes(points, xy_points):
    """Start the search with local and survey-row routes as well as random routes."""
    unvisited = set(range(len(points)))
    current = (0.0, 0.0)
    nearest_route = []
    while unvisited:
        index = min(unvisited, key=lambda i: (distance(current, xy_points[i]), i))
        nearest_route.append(index)
        unvisited.remove(index)
        current = xy_points[index]

    two_opt_route = nearest_route[:]
    improved = True
    while improved:
        improved = False
        for i in range(len(two_opt_route) - 1):
            before = (0.0, 0.0) if i == 0 else xy_points[two_opt_route[i - 1]]
            first = xy_points[two_opt_route[i]]
            for j in range(i + 1, len(two_opt_route)):
                last = xy_points[two_opt_route[j]]
                after = xy_points[two_opt_route[j + 1]] if j + 1 < len(two_opt_route) else None
                old = distance(before, first) + (distance(last, after) if after is not None else 0)
                new = distance(before, last) + (distance(first, after) if after is not None else 0)
                if new + 1e-6 < old:
                    two_opt_route[i:j + 1] = reversed(two_opt_route[i:j + 1])
                    improved = True

    routes = [two_opt_route, nearest_route]
    rows = _split_zigzag_rows(points)
    row_indices = []
    offset = 0
    for row in rows:
        row_indices.append(list(range(offset, offset + len(row))))
        offset += len(row)
    if offset == len(points):
        for start_row in range(len(row_indices)):
            for prefer_forward in (True, False):
                sequence = _zigzag_row_sequence(len(row_indices), start_row, prefer_forward)
                for reverse_first in (False, True):
                    routes.append(_zigzag_candidate_from_rows(row_indices, sequence, reverse_first))
    return routes


def _simplify_collinear_route(route, xy_points):
    """Remove only intermediate points lying on the same flown segment."""
    simplified = []
    for i, index in enumerate(route):
        if i + 1 < len(route):
            before = (0.0, 0.0) if not simplified else xy_points[simplified[-1]]
            point = xy_points[index]
            after = xy_points[route[i + 1]]
            segment = (after[0] - before[0], after[1] - before[1])
            segment_length_sq = segment[0] ** 2 + segment[1] ** 2
            if segment_length_sq > 0:
                position = ((point[0] - before[0]) * segment[0] +
                            (point[1] - before[1]) * segment[1]) / segment_length_sq
                if 0 < position < 1:
                    projected = (before[0] + position * segment[0], before[1] + position * segment[1])
                    if distance(point, projected) <= 0.02:
                        continue
        simplified.append(index)
    return simplified


def ga_path(points, start, pop_size=50, generations=300, mutation_rate=0.1, elite_size=5):
    if len(points) < 2 or pop_size < 2:
        return points.copy()

    xy_points = [latlon_to_xy(start[0], start[1], p[0], p[1]) for p in points]
    indices = list(range(len(points)))
    elite_count = min(max(2, elite_size), pop_size)

    def total_distance(route):
        return _route_cost(route, xy_points)

    population = sorted(_seed_survey_routes(points, xy_points), key=total_distance)[:pop_size]
    while len(population) < pop_size:
        population.append(random.sample(indices, len(indices)))
    best_route = min(population, key=total_distance)[:]
    best_dist = total_distance(best_route)

    for _ in range(generations):
        selected = sorted(population, key=total_distance)[:elite_count]
        new_pop = selected[:]
        while len(new_pop) < pop_size:
            parent_1, parent_2 = random.sample(selected, 2)
            child = _ga_crossover(parent_1, parent_2)
            for i in range(len(child)):
                if random.random() < mutation_rate:
                    j = random.randrange(len(child))
                    child[i], child[j] = child[j], child[i]
            new_pop.append(child)
        population = new_pop

        current_best = min(population, key=total_distance)
        current_dist = total_distance(current_best)
        if current_dist < best_dist:
            best_dist = current_dist
            best_route = current_best[:]

    return [points[index] for index in _simplify_collinear_route(best_route, xy_points)]

def abc_path(points, start, colony_size=30, limit=20, iterations=100):
    n = len(points)
    if n < 2 or colony_size < 1:
        return points.copy()

    xy_points = [latlon_to_xy(start[0], start[1], p[0], p[1]) for p in points]
    indices = list(range(n))
    food_sources = sorted(_seed_survey_routes(points, xy_points),
                          key=lambda route: _route_cost(route, xy_points))[:min(3, colony_size)]
    while len(food_sources) < colony_size:
        food_sources.append(random.sample(indices, n))
    costs = [_route_cost(route, xy_points) for route in food_sources]
    trials = [0] * colony_size
    best_index = min(range(colony_size), key=lambda i: costs[i])
    best_route = food_sources[best_index][:]
    best_cost = costs[best_index]

    def explore(source_index):
        nonlocal best_route, best_cost
        candidate = food_sources[source_index][:]
        first, last = sorted(random.sample(range(n), 2))
        if random.random() < 0.5:
            candidate[first:last + 1] = reversed(candidate[first:last + 1])
        else:
            candidate[first], candidate[last] = candidate[last], candidate[first]
        candidate_cost = _route_cost(candidate, xy_points)
        if candidate_cost + 1e-8 < costs[source_index]:
            food_sources[source_index] = candidate
            costs[source_index] = candidate_cost
            trials[source_index] = 0
            if candidate_cost < best_cost:
                best_route = candidate[:]
                best_cost = candidate_cost
        else:
            trials[source_index] += 1

    for _ in range(iterations):
        for i in range(colony_size):
            explore(i)

        fitness = [1 / (1 + cost) for cost in costs]
        for _ in range(colony_size):
            selected = random.choices(range(colony_size), weights=fitness, k=1)[0]
            explore(selected)

        for i in range(colony_size):
            if trials[i] >= limit:
                food_sources[i] = random.sample(indices, n)
                costs[i] = _route_cost(food_sources[i], xy_points)
                trials[i] = 0

    return [points[index] for index in _simplify_collinear_route(best_route, xy_points)]

def ga_path_with_turns(points, start, pop_size=50, generations=200, mutation_rate=0.1, elite_size=5):
    if len(points) < 2 or pop_size < 2:
        return points.copy()

    xy_points = [latlon_to_xy(start[0], start[1], p[0], p[1]) for p in points]
    # A large turn should cost several grid steps so survey rows stay intact.
    turn_weight = 4 * _ga_grid_spacing(xy_points)
    indices = list(range(len(points)))
    elite_count = min(max(2, elite_size), pop_size)

    def total_cost(route):
        return _route_cost(route, xy_points, turn_weight)

    population = sorted(_seed_survey_routes(points, xy_points), key=total_cost)[:pop_size]
    while len(population) < pop_size:
        population.append(random.sample(indices, len(indices)))
    best_route = min(population, key=total_cost)[:]
    best_cost_val = total_cost(best_route)

    for _ in range(generations):
        ranked = sorted(population, key=total_cost)
        new_pop = ranked[:elite_count]
        while len(new_pop) < pop_size:
            parent_1, parent_2 = random.sample(ranked[:elite_count], 2)
            child = _ga_crossover(parent_1, parent_2)
            if random.random() < mutation_rate:
                i, j = random.sample(range(len(child)), 2)
                child[i], child[j] = child[j], child[i]
            new_pop.append(child)
        population = new_pop
        curr_best = min(population, key=total_cost)
        curr_cost = total_cost(curr_best)
        if curr_cost < best_cost_val:
            best_cost_val = curr_cost
            best_route = curr_best[:]

    return [points[index] for index in _simplify_collinear_route(best_route, xy_points)]

def astar_path_with_turns(points, start):
    unvisited = set(p for p in points if p != start)
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

def _same_point(p1, p2, tol=1e-9):
    return abs(p1[0] - p2[0]) <= tol and abs(p1[1] - p2[1]) <= tol


def _path_from_uav_start(path, uav_init_point):
    """Ensure a candidate path is evaluated from the UAV's real start point."""
    if not path:
        return []

    normalized_path = []
    for point in path:
        if not _same_point(point, uav_init_point):
            normalized_path.append(point)

    return [uav_init_point] + normalized_path


def best_path_sw_uav(points, uav_init_point):
    if not points:
        return [uav_init_point]

    print("=== Running find_zigzag_path to get start point ===")
    zigzag_path, start_point = find_zigzag_path(points.copy(), uav_init_point)
    print(f"Nearest zigzag entry point: {start_point}")
    print(f"UAV start point for all algorithms: {uav_init_point}")

    try:
        path_find = find_path_0(points.copy(), uav_init_point)
        if isinstance(path_find, tuple):
            path_find = path_find[0]
    except Exception:
        path_find = []

    try:
        path_nn2opt = nn_2opt_path(points.copy(), uav_init_point)
    except Exception:
        path_nn2opt = []

    try:
        path_sa = sa_path(points.copy(), uav_init_point)
    except Exception:
        path_sa = []

    try:
        path_aco = aco_path(points.copy(), uav_init_point)
    except Exception:
        path_aco = []

    try:
        path_ga = ga_path(points.copy(), uav_init_point)
    except Exception:
        path_ga = []

    try:
        path_abc = abc_path(points.copy(), uav_init_point)
    except Exception:
        path_abc = []

    try:
        path_ga_with_turns = ga_path_with_turns(points.copy(), uav_init_point)
    except Exception:
        path_ga_with_turns = []

    try:
        path_A = astar_path_with_turns(points.copy(), uav_init_point)
    except Exception:
        path_A = []

    algos = {
        "Zigzag": _path_from_uav_start(zigzag_path, uav_init_point),
        "Find_Path": path_find,
        "NN_2opt": path_nn2opt,
        "SA": path_sa,
        "ACO": path_aco,
        "GA": path_ga,
        "ABC": path_abc,
        "GA_with_turn": path_ga_with_turns,
        "A*_Improved": path_A
    }

    best_name = None
    best_cost_val = float("inf")
    best_path = _path_from_uav_start(points.copy(), uav_init_point)

    ref_lat, ref_lon = uav_init_point
    xy_points = np.array([latlon_to_xy(ref_lat, ref_lon, p_lat, p_lon) for p_lat, p_lon in points])
    try:
        hull = ConvexHull(xy_points)
        xy_polygon = xy_points[hull.vertices]
    except Exception as e:
        print(f"[WARN] Could not create ConvexHull in best_path_sw_uav: {e}. Using bounding box as polygon.")
        min_x, min_y = np.min(xy_points, axis=0)
        max_x, max_y = np.max(xy_points, axis=0)
        xy_polygon = [(min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y)]


    for name, path in algos.items():
        path = _path_from_uav_start(path, uav_init_point)
        if not path or len(path) < 2:
            print(f"Algorithm {name}: returned no valid path. Skipping.")
            continue
        
        xy_path = np.array([latlon_to_xy(ref_lat, ref_lon, p_lat, p_lon) for p_lat, p_lon in path])

        total_dist = np.sum(np.linalg.norm(np.diff(xy_path, axis=0), axis=1))
        headings = np.arctan2(np.diff(xy_path, axis=0)[:, 1], np.diff(xy_path, axis=0)[:, 0])
        turns = np.sum(np.abs((np.diff(headings) + np.pi) % (2 * np.pi) - np.pi))
        cost = total_dist * 0.1 + turns
        print(f"Algorithm {name}: cost={cost:.3f}, dist={total_dist:.3f}, turns={turns}")
        if cost < best_cost_val:
            best_cost_val = cost
            best_name = name
            best_path = path

    print(f"==> Selected algorithm: {best_name} (cost={best_cost_val:.3f})")
    return best_path
