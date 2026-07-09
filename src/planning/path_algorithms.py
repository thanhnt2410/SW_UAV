import math
import random
import numpy as np
from scipy.spatial import ConvexHull
from shapely.geometry import LineString, Polygon as ShapelyPolygon
from shapely import BufferCapStyle, BufferJoinStyle

from .geometry import haversine, distance_between_points, distance, angle_between, latlon_to_xy
from .polygon_ops import point_on_line


def find_zigzag_path(points, uav_init_point):
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

def find_path_0(points, start, turn_threshold=math.pi/6):
    unvisited = [p for p in points if p != start]
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

def ga_path(points, start, pop_size=50, generations=300, mutation_rate=0.1, elite_size=5):
    def total_distance(route):
        dist = 0
        curr = start
        for p in route:
            dist += distance(curr, p)
            curr = p
        return dist

    population = []
    for _ in range(pop_size):
        individual = points[:]
        random.shuffle(individual)
        population.append(individual)

    def selection(pop):
        ranked = sorted(pop, key=lambda r: total_distance(r))
        return ranked[:elite_size]

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

    def mutate(route):
        for i in range(len(route)):
            if random.random() < mutation_rate:
                j = random.randint(0, len(route)-1)
                route[i], route[j] = route[j], route[i]
        return route

    best_route = None
    best_dist = float('inf')

    for _ in range(generations):
        selected = selection(population)
        new_pop = selected[:]
        while len(new_pop) < pop_size:
            p1, p2 = random.sample(selected, 2)
            child = crossover(p1, p2)
            child = mutate(child)
            new_pop.append(child)
        population = new_pop

        curr_best = min(population, key=lambda r: total_distance(r))
        curr_dist = total_distance(curr_best)
        if curr_dist < best_dist:
            best_dist = curr_dist
            best_route = curr_best[:]

    return best_route

def abc_path(points, start, colony_size=30, limit=20, iterations=100):
    n = len(points)

    def total_distance(route):
        dist = 0
        curr = start
        for p in route:
            dist += distance(curr, p)
            curr = p
        return dist

    food_sources = [random.sample(points, n) for _ in range(colony_size)]
    fitness = [1 / (1 + total_distance(p)) for p in food_sources]
    trial = [0] * colony_size

    best_route = min(food_sources, key=lambda r: total_distance(r))
    best_dist = total_distance(best_route)

    for it in range(iterations):
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

        for i in range(colony_size):
            if trial[i] > limit:
                food_sources[i] = random.sample(points, n)
                trial[i] = 0

        fitness = [1 / (1 + total_distance(p)) for p in food_sources]
        curr_best = min(food_sources, key=lambda r: total_distance(r))
        curr_dist = total_distance(curr_best)
        if curr_dist < best_dist:
            best_dist = curr_dist
            best_route = curr_best[:]

        print(f"ABC Iter {it+1}/{iterations}: best distance = {best_dist:.3f}")

    return best_route

def ga_path_with_turns(points, start, pop_size=50, generations=200, mutation_rate=0.1, elite_size=5):
    def total_cost(route):
        cost = 0
        curr = start
        prev_vector = None
        for p in route:
            cost += distance(curr, p)
            vector = (p[0]-curr[0], p[1]-curr[1])
            if prev_vector:
                dot = prev_vector[0]*vector[0] + prev_vector[1]*vector[1]
                mag = math.hypot(*prev_vector) * math.hypot(*vector)
                if mag > 0:
                    cos_angle = dot / mag
                    if abs(cos_angle) < 0.99:
                        cost += 0.1 * distance(curr, p)
            prev_vector = vector
            curr = p
        return cost
    
    population = [random.sample(points, len(points)) for _ in range(pop_size)]
    
    best_route = None
    best_cost_val = float('inf')
    
    for _ in range(generations):
        ranked = sorted(population, key=total_cost)
        new_pop = ranked[:elite_size]
        while len(new_pop) < pop_size:
            p1, p2 = random.sample(ranked[:elite_size], 2)
            a, b = sorted(random.sample(range(len(p1)), 2))
            child = [None]*len(p1)
            child[a:b] = p1[a:b]
            fill = [x for x in p2 if x not in child]
            idx = 0
            for i in range(len(child)):
                if child[i] is None:
                    child[i] = fill[idx]
                    idx += 1
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
    
    return best_route

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

def reduce_path_collinear(path):
    if len(path) < 3:
        return path

    reduced_path = [path[0]]
    for i in range(1, len(path) - 1):
        if not point_on_line(reduced_path[-1], path[i], path[i+1]):
            reduced_path.append(path[i])
    
    reduced_path.append(path[-1])
    
    return reduced_path

def best_path_sw_uav(points, uav_init_point):
    print("=== Running find_zigzag_path to get start point ===")
    zigzag_path, start_point = find_zigzag_path(points.copy(), uav_init_point)
    print(f"Start point for all algorithms: {start_point}")

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

    best_name = None
    best_cost_val = float("inf")
    best_path = None

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
