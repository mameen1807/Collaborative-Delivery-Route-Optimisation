# Collaborative Delivery Route Optimisation — Balanced, All-Vehicles-Used
# Save as: collaborative_routing_balanced.py
# Run:     python collaborative_routing_balanced.py
# Requires: pip install ortools numpy (matplotlib optional for plotting)

from math import sqrt

try:
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2
except Exception as e:
    raise SystemExit(
        "OR-Tools not found. Install with:\n    pip install ortools numpy\nThen run this script again."
    )

# ---------------- Scenario (more balanced across depots) ----------------
# Two depots far enough apart + 12 customers clustered near each depot.
locations = {
    "Depot A": (10, 50),
    "Depot B": (60, 50),
    # Near Depot A
    "C1": (12, 45), "C2": (15, 55), "C3": (20, 48),
    "C4": (18, 60), "C5": (22, 52), "C6": (25, 45),
    # Near Depot B
    "C7": (58, 47), "C8": (62, 53), "C9": (65, 48),
    "C10": (68, 55), "C11": (55, 60), "C12": (70, 45),
}

# 2 vehicles per depot (total 4)
vehicles_per_depot = {"Depot A": 2, "Depot B": 2}

# ---------------- Helpers ----------------
node_names = list(locations.keys())
index_of = {name: i for i, name in enumerate(node_names)}
coords = [locations[n] for n in node_names]

def euclid(a, b):
    return int(round(((a[0] - b[0])**2 + (a[1] - b[1])**2) ** 0.5))

# Distance matrix
n = len(coords)
distance_matrix = [[euclid(coords[i], coords[j]) for j in range(n)] for i in range(n)]

# Starts/Ends per vehicle (each vehicle returns to its own depot)
starts, ends = [], []
for depot_name, cnt in vehicles_per_depot.items():
    depot_idx = index_of[depot_name]
    for _ in range(cnt):
        starts.append(depot_idx)
        ends.append(depot_idx)
num_vehicles = len(starts)

# ---------------- OR-Tools model ----------------
manager = pywrapcp.RoutingIndexManager(n, num_vehicles, starts, ends)
routing = pywrapcp.RoutingModel(manager)

# Cost: distance
def dist_cb(from_index, to_index):
    i = manager.IndexToNode(from_index)
    j = manager.IndexToNode(to_index)
    return distance_matrix[i][j]

transit_idx = routing.RegisterTransitCallback(dist_cb)
routing.SetArcCostEvaluatorOfAllVehicles(transit_idx)

# ---- Dimension 1: Capacity (forces load split) --------------------------
# Demand: 1 per customer, 0 for depots
def demand_cb(index):
    node = manager.IndexToNode(index)
    return 0 if node_names[node].startswith("Depot") else 1

demand_idx = routing.RegisterUnaryTransitCallback(demand_cb)
vehicle_capacities = [4] * num_vehicles   # each truck can handle up to 4 customers
routing.AddDimensionWithVehicleCapacity(
    demand_idx, 0, vehicle_capacities, True, "Capacity"
)

# ---- Dimension 2: Distance limit (keeps routes local & balanced) --------
routing.AddDimension(
    transit_idx,  # same distance callback
    0,            # no slack
    120,          # max distance per vehicle (tune if needed)
    True,
    "Distance"
)

# ---- Dimension 3: ArcCount (forces each vehicle to serve >= 1 customer) --
# One unit per arc traversed. If a route is empty (start->end only) = 1 arc.
# Requiring >= 2 arcs at route end => at least one customer visited.
def one_per_arc_cb(from_index, to_index):
    return 1
arc_idx = routing.RegisterTransitCallback(one_per_arc_cb)
routing.AddDimension(arc_idx, 0, 1000, True, "ArcCount")
arc_dim = routing.GetDimensionOrDie("ArcCount")
solver = routing.solver()
for v in range(num_vehicles):
    end_cumul = arc_dim.CumulVar(routing.End(v))
    solver.Add(end_cumul >= 2)  # ensures at least one customer on each route

# Search params
search = pywrapcp.DefaultRoutingSearchParameters()
search.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
search.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
search.time_limit.FromSeconds(15)
search.log_search = False

# Solve
solution = routing.SolveWithParameters(search)

def print_solution():
    total_distance = 0
    for v in range(num_vehicles):
        index = routing.Start(v)
        route_nodes = []
        route_distance = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_nodes.append(node_names[node_index])
            prev = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(prev, index, v)
        route_nodes.append(node_names[manager.IndexToNode(index)])
        total_distance += route_distance
        print(f"Truck {v}: distance = {route_distance:3d}  route = {route_nodes}")
    print(f"\nTotal distance for all vehicles: {total_distance}")

if not solution:
    print("No solution found. Try increasing Distance cap or time limit.")
else:
    print("Balanced solution found!\n")
    print("Nodes (index -> name)".ljust(28), "Coordinates")
    for i, nname in enumerate(node_names):
        print(f"{i:2d} -> {nname:8s}    {locations[nname]}")
    print("\n--- Routes ---")
    print_solution()

# Optional plotting
try:
    import matplotlib.pyplot as plt
    plt.figure(figsize=(8, 6))
    colors = ['tab:blue','tab:orange','tab:green','tab:red','tab:purple','tab:brown']
    for v in range(num_vehicles):
        index = routing.Start(v)
        xs, ys, labels = [], [], []
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            name = node_names[node_index]
            x, y = locations[name]
            xs.append(x); ys.append(y); labels.append(name)
            index = solution.Value(routing.NextVar(index))
        # end node
        name = node_names[manager.IndexToNode(index)]
        x, y = locations[name]
        xs.append(x); ys.append(y); labels.append(name)
        plt.plot(xs, ys, marker='o', linewidth=2, label=f'Truck {v}', color=colors[v % len(colors)])
        for xi, yi, lab in zip(xs, ys, labels):
            plt.text(xi+0.3, yi+0.3, lab, fontsize=9)
    plt.title('Balanced Collaborative Multi-Depot Routing')
    plt.xlabel('X'); plt.ylabel('Y'); plt.legend(); plt.grid(True); plt.tight_layout()
    plt.show()
except Exception:
    pass
