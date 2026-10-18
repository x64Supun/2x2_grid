import settings
import road


# all three possible merging movements,
_merge = {
    frozenset("TL"): "merge_TL",
    frozenset("TR"): "merge_TR",
    frozenset("LR"): "merge_LR"
}


_origin = {"N": 0, "E": 1, "S": 2, "W": 3}

def _speed(v: float) -> float:
    return max(float(v), settings.min_v)


"""
there are 9 types of possible vehicle-to-vehicle interactions.

1. no interaction
2. same approach
3. TT - both through (ex: NT, ET)
4. LL - both left (ex: WL, SL)
5. TL_opp - through + left, but arriving from opposite directions (ex: WT, EL)
6. TL_adj - through + left, but arriving from adjacent directions (ex: WT, SL)

Following three movements, merge into the same lane

7. merge_TL - through + left to the same lane (ex: NT, EL)
8. merge_TR - through + right to the same lane (ex: NT, WR)
9. merge_LR - left + right to the same lane (ex: WR, EL)

Following group() function categorizes any pair of the type (WT, SL) to one of above categories in order to calculate the impedance
value for the given pair of vehicles.
r = first vehicle's movement
s = second vehicle's movement
"""
def group(r: int, s: int) -> str:
    origin_r = road.origins[r]
    origin_s = road.origins[s]

    turn_r = road.turns[r]
    turn_s = road.turns[s]

    if origin_r == origin_s:
        return "same_approach"

    pair = frozenset((turn_r, turn_s))

    if road.destinations[r] == road.destinations[s]:
        return _merge[pair]

    opposite = (_origin[origin_r] - _origin[origin_s]) % 4 == 2
    
    if pair == frozenset("T"):
        return "no" if opposite is True else "TT" # case 3 or no conflict TT
    elif pair == frozenset("L"):
        return "no" if opposite is True else "LL" # case 4 or no conflict LL
    elif pair == frozenset("TL"):
        return "TL_opp" if opposite is True else "TL_adj" # cases 5 or 6
    else:
        return "no" # case 1, no conflict



def lane_group(first: tuple[int, int, int], second: tuple[int, int, int]) -> str:
    first_movement, first_in_lane, first_out_lane = first
    second_movement, second_in_lane, second_out_lane = second

    first_origin = road.origins[first_movement]
    second_origin = road.origins[second_movement]

    first_destination = road.destinations[first_movement]
    second_destination = road.destinations[second_movement]

    first_turn = road.turns[first_movement]
    second_turn = road.turns[second_movement]

    # vehicles approaching in the same physical lane.
    if (
        first_origin == second_origin
        and first_in_lane == second_in_lane
    ):
        return "same_approach"

    # same approach but different lanes.
    if first_origin == second_origin:
        turn_order = {"R": 0, "T": 1, "L": 2}

        if first_in_lane < second_in_lane:
            outer_turn = first_turn
            inner_turn = second_turn
        else:
            outer_turn = second_turn
            inner_turn = first_turn

        if turn_order[outer_turn] > turn_order[inner_turn]:
            return "lane_cross"

        return "no"

    # they merge only when they enter the same outgoing lane.
    if (
        first_destination == second_destination
        and first_out_lane == second_out_lane
    ):
        pair = frozenset((first_turn, second_turn))
        return _merge[pair]

    # use the existing interaction rules for different approaches.
    return group(first_movement, second_movement)


"""
each of the above paiwise interactions has  different entering and exiting distances
following function determines the (exit_distance, entering_distance) for the movement pair (r, s)
"""

def distance(r: int, s: int) -> tuple[float, float] | None:
    g = group(r, s)
    if g in ("no", "same_approach"):
        return None
    if g == "merge_TL":
        conflict_box = settings.face_merge_tl
    elif g in ("merge_TR", "merge_LR"):
        conflict_box = settings.face_sb
    else:
        conflict_box = settings.face_64
    _enter_r, exit_r = conflict_box[road.turns[r]]
    enter_s, _exit_s = conflict_box[road.turns[s]]
    return (exit_r, enter_s)


"""
returns (first_exit_distance, second_entry_distance).

Each lane movement has the structure: (movement_id, incoming_lane, outgoing_lane)
"""
def lane_distance(first: tuple[int, int, int], second: tuple[int, int, int]) -> tuple[float, float] | None:
    
    first_movement, first_lane, _first_out_lane = first
    second_movement, second_lane, _second_out_lane = second

    interaction = lane_group(first, second)

    if interaction in ("no", "same_approach"):
        return None
    if interaction == "merge_TL":
        faces = settings.face_merge_tl_by_lane
    elif interaction in ("merge_TR", "merge_LR"):
        faces = settings.face_sb_by_lane
    else:
        faces = settings.face_main_by_lane

    first_turn = road.turns[first_movement]
    second_turn = road.turns[second_movement]

    _first_enter, first_exit = faces[first_lane][first_turn]
    second_enter, _second_exit = faces[second_lane][second_turn]

    return first_exit, second_enter


"""
for the impedance calculation, we need the minimum of
1. downstream speed (v_down)
2. v_mov - movement speed cap
3. free flow speed limit
"""
def crossing_speed(v_down: float | None = None, v_mov: float | None = None, v_f: float | None = None):
    v_f = settings.v_f if v_f is None else float(v_f)
    v_mov = float(v_f if v_mov is None else v_mov)
    if v_down is None or v_down <= 0.0:
        v_down = v_f
    else:
        v_down = float(v_down)
    return min(v_f, v_mov, v_down)



"""
given an intersection and a direction, we need to find the edge, a vehicle will enter after crossing the said intersection
ex: downstream_edge(I1, NT) = E11S
"""
def downstream_edge(intersection_id: str, movement: int) -> str:
    exit_direction = road.destinations[movement]
    return road.outgoing_edge[intersection_id][exit_direction]




"""
to find the required movement speeed cap, we need to identify a movement by its incoming and outgoing edges, w.r.t. a intersection
ex; function(I1, E0E, E8E) = WT
"""
def movement_from_edges(intersection_id: str, incoming_edge: str, outgoing_edge: str) -> int:

    if road.edges[incoming_edge][1] != intersection_id:
        raise ValueError(f"{incoming_edge} does not end at {intersection_id}")
    
    if road.edges[outgoing_edge][0] != intersection_id:
        raise ValueError(f"{outgoing_edge} does not begin at {intersection_id}")

    travel_direction = incoming_edge[-1]
    opposite = {"N": "S", "S": "N", "E": "W", "W": "E"}
    approach = opposite[travel_direction]

    for movement in settings.directions:
        if road.origins[movement] == approach:
            if downstream_edge(intersection_id, movement) == outgoing_edge:
                return movement

    raise ValueError(f"{incoming_edge} and {outgoing_edge} are not connected at {intersection_id}")



"""
will calculate all crossing speeds for all intersections
ex: funciton(I1, E0E, E11S) = min()
will store all speeds as dict{intersection_id, dict{direction, crossing_speed}}
"""
def network_movement_speeds(edge_speeds: dict[str, float | None] | None = None) -> dict[str, dict[int, float]]:
    if edge_speeds is None:
        edge_speeds = {}

    network_speeds = {}

    for intersection_id in road.intersections:
        speeds = {}

        for m in settings.directions:
            downstream_edge_id = downstream_edge(intersection_id, m)
            v_down = edge_speeds.get(downstream_edge_id)
            v_mov = settings.v_mov[intersection_id][m]
            speeds[m] = crossing_speed(v_down, v_mov, settings.v_f)

        network_speeds[intersection_id] = speeds

    return network_speeds



def network_lane_movement_speeds(edge_lane_speeds: dict[tuple[str, int], float | None] | None = None) -> dict[str, dict[tuple[int, int, int], float]]:

    if edge_lane_speeds is None:
        edge_lane_speeds = {}

    network_speeds = {}

    for intersection_id in road.intersections:
        speeds = {}

        for lane_movement in road.lane_movements:
            movement_id, _incoming_lane, outgoing_lane = lane_movement

            edge_id = downstream_edge(intersection_id, movement_id)

            v_down = edge_lane_speeds.get(
                (edge_id, outgoing_lane)
            )

            v_mov = settings.v_mov[intersection_id][movement_id]

            speeds[lane_movement] = crossing_speed(v_down, v_mov, settings.v_f)

        network_speeds[intersection_id] = speeds

    return network_speeds


"""
given a pair of movements, we can calculate the required impedance once we have all the required information like the distances and speeds
there are three possible impedance value calculations
1. no conflict = impedance is zero
2. same approach = impedance is tau + vehicle_length / first_vehicle_speed + optional_safety_buffer
3. conflicting approach = impedance is (exit_distance + first_vehicle_length) / first_vehcicle_speeds - enter_distance / second_vehicle_speed + optional_safety_buffer

this function is independent of intersection IDs.
"""
def pair_seconds(first: int, second: int, v_first: float, v_second: float | None = None) -> float:
    v_r = _speed(v_first)
    v_s = _speed(v_first if v_second is None else v_second)

    g = group(first, second)
    if g == "no":
        return 0.0
    elif g == "same_approach":
        return max(0.0, settings.tau + settings.length_veh / v_r + settings.delta)
    else:
        dist_exit, dist_enter = distance(first, second)
        return max(0.0, (dist_exit + settings.length_veh)/ v_r - dist_enter / v_s + settings.delta)




def lane_pair_seconds(first: tuple[int, int, int], second: tuple[int, int, int], v_first: float, v_second: float | None = None) -> float:
    v_r = _speed(v_first)
    v_s = _speed(v_first if v_second is None else v_second)

    interaction = lane_group(first, second)

    if interaction == "no":
        return 0.0

    if interaction == "same_approach":
        return max(0.0, settings.tau + settings.length_veh / v_r + settings.delta)

    dist_exit, dist_enter = lane_distance(first, second)

    return max(0.0, (dist_exit + settings.length_veh) / v_r - dist_enter / v_s + settings.delta)




"""
as we currently don't have live downstream data to be used as v_down
we will use a temporary function to set the required speeds for test runs
"""
def default_edge_speeds() -> dict[str, float]:
    edge_speeds = {}
    for edge_id, (start_node, end_node) in road.edges.items():
        if start_node in road.intersections:
            edge_speeds[edge_id] = settings.v_f

    test_v_down = {
            # exit links
        "E0W": settings.v_f,
        "E1N": settings.v_f,
        "E2N": settings.v_f,
        "E3E": settings.v_f,
        "E4E": settings.v_f,
        "E5S": settings.v_f,
        "E6S": settings.v_f,
        "E7W": settings.v_f,
    
            #internal links
        "E8W": settings.v_f,
        "E8E": settings.v_f,
        "E9N": settings.v_f,
        "E9S": settings.v_f,
        "E10W": settings.v_f,
        "E10E": settings.v_f,
        "E11N": settings.v_f,
        "E11S": settings.v_f,
        }

    for edge_id, speed in test_v_down.items():
        if edge_id not in edge_speeds:
            raise ValueError(f"Unknown downstream edge: {edge_id}")
        edge_speeds[edge_id] = speed
    return edge_speeds




def default_edge_lane_speeds() -> dict[tuple[str, int], float]:
    edge_lane_speeds = {}

    # give every downstream lane the free-flow speed.
    for edge_id, (start_node, _end_node) in road.edges.items():
        if start_node in road.intersections:
            for lane_id in road.edge_lanes[edge_id]:
                edge_lane_speeds[(edge_id, lane_id)] = settings.v_f

    # temporary lane-specific test values.
    test_v_down = {
        ("E8E", 0): settings.v_f,
        ("E8E", 1): settings.v_f,
    }

    # override the default values with the test values.
    for edge_lane, speed in test_v_down.items():
        if edge_lane not in edge_lane_speeds:
            raise ValueError(f"Unknown downstream edge and lane: {edge_lane}")

        edge_lane_speeds[edge_lane] = speed

    return edge_lane_speeds



"""
once we have the machinery to calculate the impedance value for any movement pair at any intersection, we have assemble the impedance matrices for all available intersections
this calculates all 144 impedances at each intersection and return the four matrices
"""
def update_impedances(edge_speeds: dict[str, float]) -> dict[str, dict[int, dict[int, float]]]:
    network_speeds = network_movement_speeds(edge_speeds)
    impedances = {}

    for intersection_id in road.intersections:
        speeds = network_speeds[intersection_id]
        impedances[intersection_id] = {}

        for first in settings.directions:
            impedances[intersection_id][first] = {}

            for second in settings.directions:
                gap = pair_seconds(
                    first,
                    second,
                    speeds[first],
                    speeds[second],
                )
                if group(first, second) != "no":
                    second_approach = road.origins[second]
                    exit_direction = road.destinations[second]
                    gap += settings.arrival_buffer[intersection_id][second_approach]
                    gap += settings.departure_buffer[intersection_id][exit_direction]

                impedances[intersection_id][first][second] = gap

    return impedances




def update_lane_impedances(edge_lane_speeds: dict[tuple[str, int], float]) -> dict[str, dict[tuple[int, int, int], dict[tuple[int, int, int], float]]]:
    network_speeds = network_lane_movement_speeds(edge_lane_speeds)

    impedances = {}

    for intersection_id in road.intersections:
        speeds = network_speeds[intersection_id]
        impedances[intersection_id] = {}

        for first in road.lane_movements:
            impedances[intersection_id][first] = {}

            for second in road.lane_movements:
                gap = lane_pair_seconds(first, second, speeds[first], speeds[second])

                if lane_group(first, second) != "no":
                    second_movement = second[0]

                    second_approach = road.origins[second_movement]

                    exit_direction = road.destinations[second_movement]

                    gap += settings.arrival_buffer[intersection_id][second_approach]

                    gap += settings.departure_buffer[intersection_id][exit_direction]

                impedances[intersection_id][first][second] = gap

    return impedances




if __name__ == "__main__":

    edge_lane_speeds = default_edge_lane_speeds()
    lane_speeds = network_lane_movement_speeds(edge_lane_speeds)
    lane_impedances = update_lane_impedances(edge_lane_speeds)

    test_cases = [
        ( # (movement_id, incoming_lane, outgoing_lane)
            "I1", 
            (0, 0, 0),  # WT, lane 0
            (3, 0, 0),  # NT, lane 0
        ),
    ]

    for intersection_id, first, second in test_cases:
        print(f"\nIntersection: {intersection_id}")
        print("First lane movement:", first)
        print("Second lane movement:", second)
        print("Group:", lane_group(first, second))
        print("Distances:", lane_distance(first, second))

        for lane_movement in (first, second):
            movement_id, _incoming_lane, outgoing_lane = lane_movement
            edge_id = downstream_edge(intersection_id, movement_id)
            edge_lane = (edge_id, outgoing_lane)

            print(f"\nLane movement: {lane_movement}")
            print("Downstream edge and lane:", edge_lane)
            print("Downstream speed:", edge_lane_speeds[edge_lane])
            print(
                "Movement speed cap:",
                settings.v_mov[intersection_id][movement_id],
            )
            print(
                "Crossing speed:",
                lane_speeds[intersection_id][lane_movement],
            )

        print(
            "\nImpedance:",
            lane_impedances[intersection_id][first][second],
        )

    for intersection_id in ["I1"]:
        print(f"\nImpedance matrix for {intersection_id}")
        print("Rows: first lane movement; columns: second lane movement")

        print(f"{'':>14}", end="")
        for second in road.lane_movements:
            print(f"{str(second):>14}", end="")
        print()

        for first in road.lane_movements:
            print(f"{str(first):>14}", end="")

            for second in road.lane_movements:
                gap = lane_impedances[intersection_id][first][second]
                print(f"{gap:>14.2f}", end="")

            print()
