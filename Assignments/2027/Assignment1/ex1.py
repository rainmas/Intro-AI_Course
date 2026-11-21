import ex1_check
import search
import utils
import heapq

id = ["No numbers - I'm special!"]
INF = 10**12

class ComponentState:
    __slots__ = (
        "truck_positions",
        "ferry_positions",
        "package_positions",
        "truck_weights",
        "ferry_truck_counts",
    )
    def __init__(self, truck_positions, ferry_positions,
                 package_positions, truck_weights, ferry_truck_counts):
        def to_tuple(value):
            if isinstance(value, dict):
                return tuple(value.items())
            return tuple(value)

        self.truck_positions = to_tuple(truck_positions)
        self.ferry_positions = to_tuple(ferry_positions)
        self.package_positions = to_tuple(package_positions)
        self.truck_weights = to_tuple(truck_weights)
        self.ferry_truck_counts = to_tuple(ferry_truck_counts)


    def __eq__(self, other):
        if not isinstance(other, ComponentState):
            return NotImplemented

        return (
            self.truck_positions == other.truck_positions and
            self.ferry_positions == other.ferry_positions and
            self.package_positions == other.package_positions
        )

    def __hash__(self):
        return hash((
            self.truck_positions,
            self.ferry_positions,
            self.package_positions,
        ))

class State:
    __slots__ = ("components",)
    def __init__(self, components):
        self.components = tuple(components)
    def __eq__(self, other):
        if not isinstance(other, State):
            return NotImplemented
        return self.components == other.components
    def __hash__(self):
        return hash(self.components)


class ComponentSolver():
    def __init__(self, idx, locs, trucks, ferries, packages,
                 roads, ports):


        # ---------- 1. store sliced domain data ----------
        self.idx = idx
        self.locs = frozenset(locs)
        self.ports = frozenset(ports)
        self.trucks = trucks            # {tid: (loc, cap)}
        self.ferries = ferries          # {fid: (loc, ports, cap)}
        self.packages = packages        # {pid: (start, w, dest)}

        self.truck_capacities = {t: v[1] for t, v in trucks.items()}
        self.ferry_capacities = {f: v[2] for f, v in ferries.items()}
        self.ferry_ports      = {f: frozenset(v[1]) for f, v in ferries.items()}
        self.package_weights      = {p: v[1] for p, v in packages.items()}
        self.package_destinations = {p: v[2] for p, v in packages.items()}

        # ---------- 2. road graph scoped to this component ----------
        # Union-find guarantees both endpoints of a road are in the same
        # component, so filtering by `a in locs` is enough.
        self.location_neighbors = {l: [] for l in self.locs}
        for a, b in roads:
            if a in self.locs:          # equivalently b in self.locs
                self.location_neighbors[a].append(b)
                self.location_neighbors[b].append(a)

        # ---------- 3. heuristic graph (roads + ferry-port cliques) ----------
        graph = self._build_heuristic_graph()

        # ---------- 4. Dijkstra tables (dense over this component) ----------
        self.distances          = {}
        self.onecost_distances  = {}
        self.min_truck_distances = {}
        self.min_ferry_distances = {}
        for l in self.locs:
            self.distances[l]           = self._dijkstra(graph, (l, False),
                                                         road_cost=1, ferry_cost=3)
            self.onecost_distances[l]   = self._dijkstra(graph, (l, False),
                                                         road_cost=1, ferry_cost=1)
            self.min_truck_distances[l] = self._dijkstra(graph, (l, False),
                                                         road_cost=1, ferry_cost=0)
            self.min_ferry_distances[l] = self._dijkstra(graph, (l, False),
                                                         road_cost=0, ferry_cost=3)

        # ---------- 5. per-component transportation constants ----------
        self._compute_transportation_constants()

        # ---------- 6. precomputed capable-truck lists ----------
        self.capable_trucks = {
            pid: [tid for tid, cap in self.truck_capacities.items()
                  if cap >= self.package_weights[pid]]
            for pid in self.packages
        }

        # ---------- 7. initial ComponentState ----------
        # Deterministic order: preserve insertion order from the sliced dicts.
        truck_positions = tuple(
            (tid, ("land", trucks[tid][0])) for tid in trucks
        )
        ferry_positions = tuple(
            (fid, ferries[fid][0]) for fid in ferries
        )
        package_positions = tuple(
            (pid, ("ground", packages[pid][0])) for pid in packages
        )
        truck_weights = tuple(
            (tid, 0) for tid in trucks
        )
        ferry_truck_counts = tuple(
            (fid, 0) for fid in ferries
        )
        self.initial_component_state = ComponentState(
            truck_positions, ferry_positions, package_positions, truck_weights, ferry_truck_counts
        )

        # ---------- 8. frozen initial h ----------
        # MUST be computed after steps 4 and 5, from the *initial* state.
        self.initial_h = self.h(self.initial_component_state)

    def _compute_transportation_constants(self):
        # Calculates max amount of possible packages that are transported in a single action
        sorted_weights = sorted(self.package_weights.values())
        if self.ferry_capacities:
            sorted_truck_capacities = sorted(
                self.truck_capacities.values(), reverse=True
            )
            max_ferry_capacity = max(self.ferry_capacities.values())
            max_total_capacity = 0
            for i, capacity in enumerate(sorted_truck_capacities):
                if i == max_ferry_capacity:
                    break
                max_total_capacity += capacity

            ferry_count = 0
            total_weight = 0
            for weight in sorted_weights:
                if total_weight + weight > max_total_capacity:
                    break
                ferry_count += 1
                total_weight += weight

        else:
            ferry_count = 0

        if self.truck_capacities:
            max_truck_capacity = max(self.truck_capacities.values())
            truck_count = 0
            total_weight = 0
            for weight in sorted_weights:
                if total_weight + weight > max_truck_capacity:
                    break
                truck_count += 1
                total_weight += weight
        else:
            truck_count = 0

        self.max_ferry_packages = ferry_count
        self.max_truck_packages = truck_count
        self.package_transportation_constant = max(3 * truck_count, ferry_count)

    def _build_heuristic_graph(self):
        graph = {}
        for location in self.location_neighbors:
            graph[(location, False)] = [(neighbor, False) for neighbor in self.location_neighbors[location]]

        for ferry in self.ferry_ports:
            served_ports = self.ferry_ports[ferry]
            for port in served_ports:
                graph[(port, True, ferry)] = [(neighbor, True, ferry) for neighbor in served_ports if neighbor != port]
                graph[(port, True, ferry)].append((port, False))
                graph.setdefault((port, False), [])
                graph[(port, False)].append((port, True, ferry))
        return graph

    def _dijkstra(self, graph, start, road_cost=1, ferry_cost=3):
        distances = {start: 0}
        heap = [(0, start)]

        while heap:
            current_distance, current_vertex = heapq.heappop(heap)

            if current_distance > distances[current_vertex]:
                continue

            for neighbor in graph[current_vertex]:
                if current_vertex[1] and neighbor[1]:
                    edge_cost = ferry_cost
                else:
                    edge_cost = road_cost

                new_distance = current_distance + edge_cost

                if new_distance < distances.get(neighbor, float('inf')):
                    distances[neighbor] = new_distance
                    heapq.heappush(heap, (new_distance, neighbor))

        return distances

    def successor(self, state):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        successors = []
        successors.extend(self.drive_successors(state))
        forcedUnload, actions = self.load_unload_successors(state)
        if forcedUnload:
            return actions
        successors.extend(actions)
        successors.extend(self.sail_successors(state))
        successors.extend(self.board_disembark_successors(state))
        return successors


    def drive_successors(self, state):
        """"Generates successor states derived from drive actions"""
        truck_positions = dict(state.truck_positions)
        successors = []
        for id in truck_positions:
            pos = truck_positions[id]
            if pos[0] != "land":
                continue
            location = pos[1]
            for neighbor in self.location_neighbors[location]:
                truck_positions[id] = ("land", neighbor)
                new_state = ComponentState(truck_positions, state.ferry_positions, state.package_positions, state.truck_weights, state.ferry_truck_counts)
                successors.append((
                    f"DRIVE{{{id},{location},{neighbor}}}",
                    new_state
                ))
                truck_positions[id] = pos

        return successors


    def load_unload_successors(self, state):
        package_positions = dict(state.package_positions)
        truck_positions = dict(state.truck_positions)
        truck_weights = dict(state.truck_weights)
        successors = []

        for pid in package_positions:
            ppos = package_positions[pid]
            if ppos[0] == "ground":
                for tid in truck_positions:
                    tpos = truck_positions[tid]
                    if (tpos[0] == "land" and tpos[1] == ppos[1] and
                    self.package_destinations[pid] != tpos[1] and # Prevent loading of a package already at its destination
                    self.truck_capacities[tid] >= self.package_weights[pid] + truck_weights[tid]):
                        package_positions[pid] = ("truck", tid)
                        truck_weights[tid] += self.package_weights[pid]
                        new_state = ComponentState(truck_positions, state.ferry_positions, package_positions, truck_weights, state.ferry_truck_counts)
                        successors.append((
                            f"LOAD{{{pid},{tid}}}",
                            new_state
                        ))
                        package_positions[pid] = ppos
                        truck_weights[tid] -= self.package_weights[pid]
            elif ppos[0] == "truck":
                tid = ppos[1]
                tpos = truck_positions[tid]
                if tpos[0] == "land":
                    package_positions[pid] = ("ground", tpos[1])
                    truck_weights[tid] -= self.package_weights[pid]
                    new_state = ComponentState(truck_positions, state.ferry_positions, package_positions, truck_weights, state.ferry_truck_counts)
                    action = (
                        f"UNLOAD{{{pid},{tid}}}",
                        new_state
                    )
                    if self.package_destinations[pid] == tpos[1]:
                        return True, [action]
                    successors.append(action)
                    package_positions[pid] = ppos
                    truck_weights[tid] += self.package_weights[pid]

        return False, successors


    def sail_successors(self, state):
        ferry_positions = dict(state.ferry_positions)
        ferry_truck_counts = dict(state.ferry_truck_counts)
        truck_positions = state.truck_positions
        successors = []
        ports_with_trucks = set()
        for tid, tpos in truck_positions:
            if tpos[0] == "land" and tpos[1] in self.ports:
                ports_with_trucks.add(tpos[1])
        for id in ferry_positions:
            pos = ferry_positions[id]
            no_trucks = ferry_truck_counts[id] == 0
            for port in self.ferry_ports[id]:
                if port == pos:
                    continue
                if no_trucks and port not in ports_with_trucks:
                    continue
                ferry_positions[id] = port
                new_state = ComponentState(state.truck_positions, ferry_positions, state.package_positions, state.truck_weights, state.ferry_truck_counts)
                successors.append((
                    f"SAIL{{{id},{pos},{port}}}",
                    new_state
                ))
                ferry_positions[id] = pos

        return successors


    def board_disembark_successors(self, state):
        truck_positions = dict(state.truck_positions)
        ferry_positions = dict(state.ferry_positions)
        ferry_truck_counts = dict(state.ferry_truck_counts)
        successors = []

        for fid in ferry_positions:
            fpos = ferry_positions[fid]
            for tid in truck_positions:
                tpos = truck_positions[tid]
                if (tpos[0] == "land" and tpos[1] == fpos and
                ferry_truck_counts[fid] + 1 <= self.ferry_capacities[fid]):
                    ferry_truck_counts[fid] += 1
                    truck_positions[tid] = ("ferry", fid)
                    new_state = ComponentState(truck_positions, ferry_positions, state.package_positions, state.truck_weights, ferry_truck_counts)
                    successors.append((
                        f"BOARD{{{tid},{fid}}}",
                        new_state
                    ))
                    ferry_truck_counts[fid] -= 1
                    truck_positions[tid] = tpos
                elif tpos[0] == "ferry" and tpos[1] == fid:
                    ferry_truck_counts[fid] -= 1
                    truck_positions[tid] = ("land", fpos)
                    new_state = ComponentState(truck_positions, ferry_positions, state.package_positions, state.truck_weights, ferry_truck_counts)
                    successors.append((
                        f"DISEMBARK{{{tid},{fid}}}",
                        new_state
                    ))
                    ferry_truck_counts[fid] += 1
                    truck_positions[tid] = tpos

        return successors


    def goal_test(self, state):
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        package_positions = state.package_positions
        for id, pos in package_positions:
            if pos[0] != "ground" or pos[1] != self.package_destinations[id]:
                return False

        return True

    def h(self, state):

        return self.calculate_handeling_costs(state) + max(self.max_min_travel_distance(state),
                                                           self.average_reduction_heuristic(state),
                                                           self.combined_truck_ferry_distances_heuristic(state))

    def combined_truck_ferry_distances_heuristic(self, state):
        total_truck_distances = 0
        total_ferry_distances = 0
        package_positions = state.package_positions
        truck_positions = dict(state.truck_positions)
        ferry_positions = dict(state.ferry_positions)
        for id, pos in package_positions:
            destination = self.package_destinations[id]
            if pos[0] == "ground":
                total_truck_distances += self.min_truck_distances[destination][(pos[1], False)]
                total_ferry_distances += self.min_ferry_distances[destination][(pos[1], False)]
            elif pos[0] == "truck":
                tid = pos[1]
                tpos = truck_positions[tid]
                if tpos[0] == "land":
                    total_truck_distances += self.min_truck_distances[destination][(tpos[1], False)]
                    total_ferry_distances += self.min_ferry_distances[destination][(tpos[1], False)]
                elif tpos[0] == "ferry":
                    fid = tpos[1]
                    fpos = ferry_positions[fid]
                    total_truck_distances += self.min_truck_distances[destination][(fpos, False)]
                    total_ferry_distances += self.min_ferry_distances[destination][(fpos, False)]

        if self.max_truck_packages == 0:
            min_truck_cost = 0 if total_truck_distances == 0 else INF
        else:
            min_truck_cost = (total_truck_distances + self.max_truck_packages - 1) // self.max_truck_packages

        ferry_edges = total_ferry_distances // 3
        if ferry_edges == 0 or self.max_ferry_packages == 0:
            min_ferry_cost = 0 if ferry_edges == 0 else INF
        else:
            min_ferry_cost = 3 * ((ferry_edges + self.max_ferry_packages - 1)
                                // self.max_ferry_packages)
        return min_truck_cost + min_ferry_cost


    def average_reduction_heuristic(self, state):
        total_distances = 0
        package_positions = state.package_positions
        truck_positions = dict(state.truck_positions)
        ferry_positions = dict(state.ferry_positions)
        for id, pos in package_positions:
            destination = self.package_destinations[id]
            if pos[0] == "ground":
                total_distances += self.onecost_distances[destination][(pos[1], False)]
            elif pos[0] == "truck":
                tid = pos[1]
                tpos = truck_positions[tid]
                if tpos[0] == "land":
                    total_distances += self.onecost_distances[destination][(tpos[1], False)]
                elif tpos[0] == "ferry":
                    fid = tpos[1]
                    fpos = ferry_positions[fid]
                    total_distances += self.onecost_distances[destination][(fpos, True, fid)]
        if self.package_transportation_constant == 0:
            return 0 if total_distances == 0 else INF
        return (3 * total_distances + self.package_transportation_constant - 1) // self.package_transportation_constant

    def calculate_handeling_costs(self, state):
        package_positions = state.package_positions
        handeling_costs = 0
        for id, pos in package_positions:
            if pos[0] != "ground":
                handeling_costs += 1
            elif pos[1] != self.package_destinations[id]:
                handeling_costs += 2
        return handeling_costs

    def max_min_travel_distance(self, state):
        max_travel_distance = 0
        package_positions = state.package_positions
        truck_positions = dict(state.truck_positions)
        ferry_positions = dict(state.ferry_positions)
        for id, pos in package_positions:
            if pos[0] == "ground":
                if pos[1] == self.package_destinations[id]:
                    continue
                location = pos[1]
                dist = self.distances[self.package_destinations[id]][(location, False)]
                min_truck_dist = INF

                for tid in self.capable_trucks[id]:
                    tpos = truck_positions[tid]
                    if tpos[0] == "land":
                        truck_dist = self.distances[location][(tpos[1], False)]
                    elif tpos[0] == "ferry":
                        fid = tpos[1]
                        fpos = ferry_positions[fid]
                        truck_dist = self.distances[location][(fpos, True, fid)]
                    min_truck_dist = min(min_truck_dist, truck_dist)

                dist += min_truck_dist

            elif pos[0] == "truck":
                tid = pos[1]
                tpos = truck_positions[tid]
                if tpos[0] == "land":
                    location = tpos[1]
                    dist = self.distances[self.package_destinations[id]][(location, False)]
                elif tpos[0] == "ferry":
                    fid = tpos[1]
                    fpos = ferry_positions[fid]
                    dist = self.distances[self.package_destinations[id]][(fpos, True, fid)]

            max_travel_distance = max(max_travel_distance, dist)

        return max_travel_distance

class LogisticsProblem(search.Problem):
    """This class implements a logistics problem"""

    def __init__(self, initial):

        self.location_count = initial["Locations"]
        self.roads = initial["Roads"]
        self.ports = initial["Ports"]

        # ---------- 1. union-find over locations ----------
        parent = list(range(self.location_count))

        def find(x):
            while parent[x] != x:
                parent[x] = parent[parent[x]]   # path halving
                x = parent[x]
            return x

        def union(a, b):
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[ra] = rb

        for a, b in self.roads:
            union(a, b)

        for _, (loc, served, _cap) in initial["Ferries"].items():
            served_list = list(served)
            if not served_list:
                continue
            first = served_list[0]
            for p in served_list[1:]:
                union(first, p)

        self.loc_component = {l: find(l) for l in range(self.location_count)}

        # ---------- 2. slice domain data by component ----------
        comp_locs      = {}
        comp_trucks    = {}
        comp_ferries   = {}
        comp_packages  = {}
        comp_ports     = {}

        for l in range(self.location_count):
            comp_locs.setdefault(self.loc_component[l], set()).add(l)

        for tid, (loc, cap) in initial["Trucks"].items():
            comp_trucks.setdefault(self.loc_component[loc], {})[tid] = (loc, cap)

        for fid, (loc, served, cap) in initial["Ferries"].items():
            comp_ferries.setdefault(self.loc_component[loc], {})[fid] = (loc, served, cap)

        for pid, (start, w, dest) in initial["Packages"].items():
            comp_packages.setdefault(self.loc_component[start], {})[pid] = (start, w, dest)

        for p in self.ports:
            comp_ports.setdefault(self.loc_component[p], set()).add(p)

        # ---------- 3. sanity: solvability wrt components ----------
        # If a package's destination is in a different component than its
        # start, no sequence of actions can deliver it.
        for pid, (start, _w, dest) in initial["Packages"].items():
            if self.loc_component[start] != self.loc_component[dest]:
                raise ValueError(
                    f"unsolvable: package {pid} spans components "
                    f"{self.loc_component[start]} and {self.loc_component[dest]}"
                )

        # ---------- 4. keep only components with actual work ----------
        # A component whose every package already starts at its destination
        # (or which has no packages) contributes h=0 forever; skip it.
        active = []
        for c, pkgs in comp_packages.items():
            if any(start != dest for (start, _w, dest) in pkgs.values()):
                active.append(c)
        active.sort()                     # deterministic order

        # ---------- 5. build solvers ----------
        self.solvers = []
        for idx, c in enumerate(active):
            solver = ComponentSolver(
                idx,
                comp_locs[c],
                comp_trucks.get(c, {}),
                comp_ferries.get(c, {}),
                comp_packages.get(c, {}),
                self.roads,
                comp_ports.get(c, set()),
            )
            self.solvers.append(solver)

        # ---------- 6. initial State + h cache ----------
        initial_components = tuple(s.initial_component_state for s in self.solvers)
        self._h_cache = {}
        search.Problem.__init__(self, State(initial_components))


    def path_cost(self, c, state1, action, state2):
        """ Do not change this. Returns the cost of reaching state2 from state1 via action,
        given cost c to reach state1: SAIL costs 3, every other action costs 1"""
        return c + (3 if action.startswith("SAIL") else 1)

    def successor(self, state):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        idx = self._active_index(state)
        if idx is None:
            return []
        solver = self.solvers[idx]
        comp = state.components[idx]
        comps = list(state.components)
        out = []
        for action, new_comp in solver.successor(comp):
            comps[idx] = new_comp
            out.append((action, State(tuple(comps))))
            comps[idx] = comp
        return out


    def _active_index(self, state):
        for i, solver in enumerate(self.solvers):
            if not solver.goal_test(state.components[i]):
                return i
        return None

    def goal_test(self, state):
        return self._active_index(state) is None

    def h_astar(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""

        state = node.state
        cached = self._h_cache.get(state)
        if cached is not None:
            return cached
        idx = self._active_index(state)
        if idx is None:
            return 0
        total = 0
        for i, solver in enumerate(self.solvers):
            if i < idx:
                continue
            elif i == idx:
                total += solver.h(state.components[i])
            else:
                total += solver.initial_h
        self._h_cache[state] = total
        return total


def create_logistics_problem(game):
    print("<<create_logistics_problem")
    """ Create a logistics problem, based on the description.
    game - dictionary as described in pdf file"""
    return LogisticsProblem(game)


if __name__ == '__main__':
    ex1_check.main()
