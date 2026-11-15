import ex1_check
import search
import utils

id = ["No numbers - I'm special!"]

class State:
    def __init__(self, truck_positions, ferry_positions,
                 package_positions, truck_weights, ferry_truck_counts):

        def to_tuple(value):
            if isinstance(value, dict):
                return tuple(value.items())
            return tuple(value)
        
        self.truck_positions = self.to_tuple(truck_positions)
        self.ferry_positions = self.to_tuple(ferry_positions)
        self.package_positions = self.to_tuple(package_positions)
        self.truck_weights = self.to_tuple(truck_weights)
        self.ferry_truck_counts = self.to_tuple(ferry_truck_counts)


    def __eq__(self, other):
        if not isinstance(other, State):
            return NotImplemented

        return (
            self.truck_positions == other.truck_positions and
            self.ferry_positions == other.ferry_positions and
            self.package_positions == other.package_positions and
            self.truck_weights == other.truck_weights
        )

    def __hash__(self):
        return hash((
            self.truck_positions,
            self.ferry_positions,
            self.package_positions,
            self.truck_weights
        ))

class LogisticsProblem(search.Problem):
    """This class implements a logistics problem"""

    def __init__(self, initial):


        """ Constructor only needs the initial state.
        Don't forget to set the goal or implement the goal test"""
        self.locations = initial["Locations"]
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
        search.Problem.__init__(self, state)

    def path_cost(self, c, state1, action, state2):
        """ Do not change this. Returns the cost of reaching state2 from state1 via action,
        given cost c to reach state1: SAIL costs 3, every other action costs 1"""
        return c + (3 if action.startswith("SAIL") else 1)

    def successor(self, state):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        successors = []
        successors.extend(self.drive_successors(state))
        successors.extend(self.load_unload_successors(state))
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
                    successors.append((
                        f"UNLOAD{{{pid},{tid}}}",
                        new_state
                    ))
                    package_positions[pid] = ppos
                    truck_weights[tid] += self.package_weights[pid]

        return successors
    

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

        # Placeholder for actual heuristic
        return 0


def create_logistics_problem(game):
    print("<<create_logistics_problem")
    """ Create a logistics problem, based on the description.
    game - dictionary as described in pdf file"""
    return LogisticsProblem(game)


if __name__ == '__main__':
    ex1_check.main()
