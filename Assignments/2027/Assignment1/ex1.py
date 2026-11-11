import ex1_check
import search
import utils

id = ["No numbers - I'm special!"]



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
        for truck in trucks:
            truck_positions[truck] = ("land", trucks[truck][0])
            self.truck_capacities[truck] = trucks[truck][1]

        self.ferry_capacities = {}
        self.ferry_ports = {}
        ferry_positions = {}
        for ferry in initial["Ferries"]:
            ferry_positions[ferry] = initial["Ferries"][ferry][0]
            self.ferry_capacities[ferry] = initial["Ferries"][ferry][2]
            self.ferry_ports[ferry] = initial["Ferries"][ferry][1]

        packages = initial["Packages"]
        self.package_weights = {}
        self.package_destinations = {}
        package_positions = {}
        for package in packages:
            package_positions[package] = ("ground", packages[package][0])
            self.package_weights[package] = packages[package][1]
            self.package_destinations[package] = packages[package][2]

        state = (tuple(truck_positions.items()), tuple(ferry_positions.items()), tuple(package_positions.items(())))
        search.Problem.__init__(self, state)

    def path_cost(self, c, state1, action, state2):
        """ Do not change this. Returns the cost of reaching state2 from state1 via action,
        given cost c to reach state1: SAIL costs 3, every other action costs 1"""
        return c + (3 if action.startswith("SAIL") else 1)

    def successor(self, state):
        """ Generates the successor states returns [(action, achieved_states, ...)]"""
        utils.raiseNotDefined()


    def drive_successors(self, state):
        """"Generates successor states derived from drive actions"""
        truck_positions = state[0]
        successors = []
        for id, pos in truck_positions:
            if pos[0] != "land":
                continue
            location = pos[1]
            for neighbor in self.location_neighbors[location]:
                new_state = dict(truck_positions)
                new_state[id] = ("land", neighbor)
                successors.append((
                    f"DRIVE{{{id},{location},{neighbor}}}",
                    (tuple(new_state), state[1], state[2])
                ))

        return successors


    def goal_test(self, state):
        """ given a state, checks if this is the goal state, compares to the created goal state returns True/False"""
        package_positions = state[2]
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
