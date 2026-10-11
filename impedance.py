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
def group(r: str, s: str) -> str:
    if r[0] == s[0]:
        return "same_approach" # case 2
    if road.destinations[r] == road.destinations[s]:
        return _merge[frozenset(r[1] + s[1])] # cases 7, 8, 9
    opposite = (_origin[r[0]] - _origin[s[0]]) % 4 == 2
    pair = frozenset(r[1] + s[1])
    if pair == frozenset("T"):
        return "no" if opposite is True else "TT" # case 3 or no conflict TT
    elif pair == frozenset("L"):
        return "no" if opposite is True else "LL" # case 4 or no conflict LL
    elif pair == frozenset("TL"):
        return "TL_opp" if opposite is True else "TL_adj" # cases 5 or 6
    else:
        return "no" # case 1, no conflict



"""
each of the above paiwise interactions has  different entering and exiting distances
following function determines the (exit_distance, entering_distance) for the movement pair (r, s)
"""

def distance(r: str, s: str) -> str:
    g = group(r, s)
    if g in ("no", "same_approach"):
        return None
    if g == "merge_TL":
        conflict_box = settings.face_merge_tl
    elif g in ("merge_TR", "merge_LR"):
        conflict_box = settings.face_sb
    else:
        conflict_box = settings.face_64
    _enter_r, exit_r = conflict_box[r[1]]
    enter_s, _exit_s = conflict_box[s[1]]
    return (exit_r, enter_s)



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
def downstream_edge(intersection_id: str, movement: str) -> str:
    exit_direction = road.destinations[movement]
    exit_edge_id = road.outgoing_edge[intersection_id][exit_direction]
    return exit_edge_id




"""
to find the required movement speeed cap, we need to identify a movement by its incoming and outgoing edges, w.r.t. a intersection
ex; function(I1, E0E, E8E) = WT
"""
def movement_from_edges(intersection_id: str, incoming_edge: str, outgoing_edge: str) -> str:
    if road.edges[incoming_edge][1] != intersection_id:
        raise ValueError(f"{incoming_edge} does not end at {intersection_id}")
    if road.edges[outgoing_edge][0] != intersection_id:
        raise ValueError(f"{outgoing_edge} does not begin at {intersection_id}")

    travel_direction = incoming_edge[-1]
    opposite = {"N": "S", "S": "N", "E": "W", "W": "E"}
    approach = opposite[travel_direction]

    for movement in settings.directions:
        if movement[0] == approach:
            if downstream_edge(intersection_id, movement) == outgoing_edge:
                return movement

    raise ValueError(f"{incoming_edge} and {outgoing_edge} are not connected at {intersection_id}")



"""
will calculate all crossing speeds for all intersections
ex: funciton(I1, E0E, E11S) = min()
will store all speeds as dict{intersection_id, dict{direction, crossing_speed}}
"""
def network_movement_speeds(edge_speeds: dict[str, float | None] | None = None) -> dict[str, dict[str, float]]:
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


"""
given a pair of movements, we can calculate the required impedance once we have all the required information like the distances and speeds
there are three possible impedance value calculations
1. no conflict = impedance is zero
2. same approach = impedance is tau + vehicle_length / first_vehicle_speed + optional_safety_buffer
3. conflicting approach = impedance is (exit_distance + first_vehicle_length) / first_vehcicle_speeds - enter_distance / second_vehicle_speed + optional_safety_buffer

this function is independent of intersection IDs.
"""
def pair_seconds(first: str, second: str, v_first: float, v_second: float | None = None) -> float:
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



"""
once we have the machinery to calculate the impedance value for any movement pair at any intersection, we have assemble the impedance matrices for all available intersections
this calculates all 144 impedances at each intersection and return the four matrices
"""
def update_impedances(edge_speeds: dict[str, float]) -> dict[str, dict[str, dict[str, float]]]:
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
                    second_approach = second[0]
                    exit_direction = road.destinations[second]
                    gap += settings.arrival_buffer[intersection_id][second_approach]
                    gap += settings.departure_buffer[intersection_id][exit_direction]

                impedances[intersection_id][first][second] = gap

    return impedances




if __name__ == "__main__":

    edge_speeds = default_edge_speeds()
    impedances = update_impedances(edge_speeds)
    speeds = network_movement_speeds(edge_speeds)


    
    test_cases = [
        ("I1", "EL", "NR"),
    
    ]

    for intersection, first, second in test_cases:
        print("\nGroup: ", group(first, second))
        print("\nDistances: ", distance(first, second))
    
        for movement in (first, second):
            edge = downstream_edge(intersection, movement)
            print(f"downstream edge for {movement}: ", edge)
            print(f"v_down for {movement}: ", edge_speeds[edge])
            print(f"v_mov for {movement}: ", settings.v_mov[intersection][movement])
            print(f"crossing speed for {movement}: ", speeds[intersection][movement])
            print(impedances[intersection][first][second])

    """
    for intersection_id in ["I1"]:
        print(f"\n Impedance matrx for {intersection_id}")
        print("Rows: first movement; Columns: second movement")

        print(f"{'':>8}", end = "")
        for second in settings.directions:
            print(f"{second:>8}", end = "")
        print()

        for first in settings.directions:
            print(f"{first:>8}", end = "")

            for second in settings.directions:
                gap = impedances[intersection_id][first][second]
                print(f"{gap:>8.2f}", end = "")

            print()
    
"""