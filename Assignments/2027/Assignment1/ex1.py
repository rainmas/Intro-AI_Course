import ex1_check
import search
import utils
import heapq

id = ["No numbers - I'm special!"]

class State:
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
        if not isinstance(other, State):
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

class LogisticsProblem(search.Problem):
    """This class implements a logistics problem"""

    def __init__(self, initial):


        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        self.location_count = initial["Locations"]
        self.roads = initial["Roads"]
        self.ports = initial["Ports"]

        self.location_neighbors = {}
        for road in self.roads:
            self.location_neighbors.setdefault(road[0], []).append(road[1])
            self.location_neighbors.setdefault(road[1], []).append(road[0])

        trucks = initial["Trucks"]
        self.truck_capacities = {}
        truck_positions = {}
        truck_weights = {}
        for truck in trucks:
            truck_positions[truck] = ("land", trucks[truck][0])
            self.truck_capacities[truck] = trucks[truck][1]
            truck_weights[truck] = 0

        self.ferry_capacities = {}
        self.ferry_ports = {}
        ferry_positions = {}
        ferry_truck_counts = {}
        for ferry in initial["Ferries"]:
            ferry_positions[ferry] = initial["Ferries"][ferry][0]
            self.ferry_capacities[ferry] = initial["Ferries"][ferry][2]
            self.ferry_ports[ferry] = initial["Ferries"][ferry][1]
            ferry_truck_counts[ferry] = 0

        packages = initial["Packages"]
        self.package_weights = {}
        self.package_destinations = {}
        package_positions = {}
        for package in packages:
            package_positions[package] = ("ground", packages[package][0])
            self.package_weights[package] = packages[package][1]
            self.package_destinations[package] = packages[package][2]

        state = State(truck_positions, ferry_positions, package_positions, truck_weights, ferry_truck_counts)
        self.precompute_destination_distances()
        self.calculate_package_transportation_constant()

        search.Problem.__init__(self, state)

    def calculate_package_transportation_constant(self):
        # Calculates max amount of possible packages that are transported in a single action
        sorted_weights = sorted(self.package_weights.values())
        if len(self.ferry_capacities) > 0:
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

        max_truck_capacity = max(self.truck_capacities.values())
        truck_count = 0
        total_weight = 0
        for weight in sorted_weights:
            if total_weight + weight > max_truck_capacity:
                break
            truck_count += 1
            total_weight += weight

        self.package_transportation_constant = max(3 * truck_count, ferry_count)

    def construct_heuristic_graph(self):
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

    def dijkstra_on_heuristic_graph(self, graph, start):
        distances = {start: 0}
        heap = [(0, start)]

        while heap:
            current_distance, current_vertex = heapq.heappop(heap)

            if current_distance > distances[current_vertex]:
                continue

            for neighbor in graph[current_vertex]:
                if current_vertex[1] and neighbor[1]:
                    edge_cost = 3
                else:
                    edge_cost = 1

                new_distance = current_distance + edge_cost

                if new_distance < distances.get(neighbor, float('inf')):
                    distances[neighbor] = new_distance
                    heapq.heappush(heap, (new_distance, neighbor))

        return distances

    def precompute_destination_distances(self):
        graph = self.construct_heuristic_graph()
        self.distances = {}
        for i in range(self.location_count):
            self.distances[i] = self.dijkstra_on_heuristic_graph(
                graph, (i, False)
            )


    def path_cost(self, c, state1, action, state2):
        """ Do not change this. Returns the cost of reaching state2 from state1 via action,
        given cost c to reach state1: SAIL costs 3, every other action costs 1"""
        return c + (3 if action.startswith("SAIL") else 1)

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
                new_state = State(truck_positions, state.ferry_positions, state.package_positions, state.truck_weights, state.ferry_truck_counts)
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
                        new_state = State(truck_positions, state.ferry_positions, package_positions, truck_weights, state.ferry_truck_counts)
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
                    new_state = State(truck_positions, state.ferry_positions, package_positions, truck_weights, state.ferry_truck_counts)
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
        successors = []
        for id in ferry_positions:
            pos = ferry_positions[id]
            for port in self.ferry_ports[id]:
                if port == pos:
                    continue
                ferry_positions[id] = port
                new_state = State(state.truck_positions, ferry_positions, state.package_positions, state.truck_weights, state.ferry_truck_counts)
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
                    new_state = State(truck_positions, ferry_positions, state.package_positions, state.truck_weights, ferry_truck_counts)
                    successors.append((
                        f"BOARD{{{tid},{fid}}}",
                        new_state
                    ))
                    ferry_truck_counts[fid] -= 1
                    truck_positions[tid] = tpos
                elif tpos[0] == "ferry" and tpos[1] == fid:
                    ferry_truck_counts[fid] -= 1
                    truck_positions[tid] = ("land", fpos)
                    new_state = State(truck_positions, ferry_positions, state.package_positions, state.truck_weights, ferry_truck_counts)
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

    def h_astar(self, node):
        """ This is the heuristic. It gets a node (not a state)
        and returns a goal distance estimate"""

        state = node.state
        return self.calculate_handeling_costs(state) + max(self.max_min_travel_distance(state), self.average_reduction_heuristic(state))

    def average_reduction_heuristic(self, state):
        total_distances = 0
        package_positions = state.package_positions
        truck_positions = dict(state.truck_positions)
        ferry_positions = dict(state.ferry_positions)
        for id, pos in package_positions:
            destination = self.package_destinations[id]
            if pos[0] == "ground":
                total_distances += self.distances[destination][(pos[1], False)]
            elif pos[0] == "truck":
                tid = pos[1]
                tpos = truck_positions[tid]
                if tpos[0] == "land":
                    total_distances += self.distances[destination][(tpos[1], False)]
                elif tpos[0] == "ferry":
                    fid = tpos[1]
                    fpos = ferry_positions[fid]
                    total_distances += self.distances[destination][(fpos, True, fid)]
        return (
            3 * total_distances
            + self.package_transportation_constant - 1
        ) // self.package_transportation_constant
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
                min_truck_dist = float("inf")

                for tid in truck_positions:
                    if self.truck_capacities[tid] >= self.package_weights[id]:
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

def create_logistics_problem(game):
    print("<<create_logistics_problem")
    """ Create a logistics problem, based on the description.
    game - dictionary as described in pdf file"""
    return LogisticsProblem(game)


if __name__ == '__main__':
    ex1_check.main()
