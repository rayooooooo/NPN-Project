from typing import List, Dict, Tuple
import itertools
import numpy as np
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from app.routing.shortest_path import shortest_path


def compute_time_matrix(points: List[Tuple[float,float]]) -> np.ndarray:
    n = len(points)
    mat = np.zeros((n,n), dtype=float)
    for i, j in itertools.product(range(n), range(n)):
        if i == j:
            continue
        (lat1, lon1), (lat2, lon2) = points[i], points[j]
        _, _, t = shortest_path(lat1, lon1, lat2, lon2)
        mat[i, j] = t
    return mat


def solve_vrp(vehicles: List[Dict], stops: List[Dict]):
    # Build points list: vehicle starts + stops (+ optional ends)
    starts = [(v['start']['lat'], v['start']['lon']) for v in vehicles]
    ends = [((v.get('end') or v['start'])['lat'], (v.get('end') or v['start'])['lon']) for v in vehicles]
    points = starts + [(s['lat'], s['lon']) for s in stops] + ends
    n_starts = len(starts)
    n_stops = len(stops)
    n_ends = len(ends)

    time_matrix = compute_time_matrix(points)

    manager = pywrapcp.RoutingIndexManager(len(points), n_starts, [i for i in range(n_starts)], [len(points)-n_ends+i for i in range(n_ends)])
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        i, j = manager.IndexToNode(from_index), manager.IndexToNode(to_index)
        return int(time_matrix[i, j])

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    routing.AddDimension(transit_callback_index, 0, 24*3600, True, 'Time')

    # Capacity
    demands = [0]*n_starts + [s.get('demand',1) for s in stops] + [0]*n_ends
    capacities = [v.get('capacity',50) for v in vehicles]

    def demand_callback(from_index):
        i = manager.IndexToNode(from_index)
        return int(demands[i])
    demand_cb = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(demand_cb, 0, capacities, True, 'Capacity')

    # Time windows (optional)
    time_dim = routing.GetDimensionOrDie('Time')
    for idx, s in enumerate(stops):
        node = n_starts + idx
        if s.get('ready_ts') and s.get('due_ts'):
            # Here we parse as seconds-from-now in a real system; demo: use [0, 24h]
            time_dim.CumulVar(manager.NodeToIndex(node)).SetRange(0, 24*3600)

    search = pywrapcp.DefaultRoutingSearchParameters()
    search.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search.time_limit.FromSeconds(30)

    solution = routing.SolveWithParameters(search)
    if solution is None:
        return {"routes": {}}  # no solution

    # Decode routes
    out = {}
    for v in range(n_starts):
        idx = routing.Start(v)
        seq = []
        while not routing.IsEnd(idx):
            node = manager.IndexToNode(idx)
            seq.append(node)
            idx = solution.Value(routing.NextVar(idx))
        seq.append(manager.IndexToNode(idx))
        # map node ids to stop ids
        # nodes: [0..n_starts-1] starts, [n_starts..n_starts+n_stops-1] stops, last n_ends are ends
        stops_in_route = [stops[node - n_starts]['id'] for node in seq if n_starts <= node < n_starts+n_stops]
        out[v] = stops_in_route
    return {"routes": out}
