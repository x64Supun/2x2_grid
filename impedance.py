"""Lane-aware impedance calculations for the through-only testbed.

A lane movement is represented as
``(movement_id, incoming_lane, outgoing_lane)``. The earlier movement-only
impedance helpers were removed after the project was converted to two lanes.
"""

import settings
import road


_origin = {"N": 0, "E": 1, "S": 2, "W": 3}

def _speed(v: float) -> float:
    return max(float(v), settings.min_v)


"""
classifies the interaction between two vehicles
currently this only classifies through movements, same lane approach and no conflicts.
expected input type (0,0,0) = (movement code, first vehicle lane, second vehicle lane)
"""
def lane_group(first: tuple[int, int, int], second: tuple[int, int, int]) -> str:

    if first not in road.lane_movements:
        raise ValueError(f"Inactive lane movement: {first}")
    if second not in road.lane_movements:
        raise ValueError(f"Inactive lane movement: {second}")

    first_movement, first_in_lane, _first_out_lane = first
    second_movement, second_in_lane, _second_out_lane = second

    first_origin = road.origins[first_movement]
    second_origin = road.origins[second_movement]

    if first_origin == second_origin:
        if first_in_lane == second_in_lane:
            return "same_lane_approach"
        return "no"

    opposite = ( _origin[first_origin] - _origin[second_origin]) % 4 == 2

    return "no" if opposite else "TT"


"""
returns (first_exit_distance, second_entry_distance).

each lane movement has the structure: (movement code, incoming lane id, outgoing lane id)
"""
def lane_distance(first: tuple[int, int, int], second: tuple[int, int, int]) -> tuple[float, float] | None:

    first_movement, first_lane, _first_out_lane = first
    second_movement, second_lane, _second_out_lane = second

    interaction = lane_group(first, second)

    if interaction in ("no", "same_lane_approach"):
        return None

    first_turn = road.turns[first_movement]
    second_turn = road.turns[second_movement]

    _first_enter, first_exit = settings.face_main_by_lane[first_lane][first_turn]
    second_enter, _second_exit = settings.face_main_by_lane[second_lane][second_turn]

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
calculates the crossing speed for every intersection, every active movement, every lane
returns a nested dictionary 
network_speeds["I1][(0,0,0)] = crossing speed at I1 for WT using lane 0
"""
def network_lane_movement_speeds(edge_lane_speeds: dict[tuple[str, int], float | None] | None = None) -> dict[str, dict[tuple[int, int, int], float]]:

    if edge_lane_speeds is None:
        edge_lane_speeds = {}

    network_speeds = {}

    for intersection_id in road.intersections:
        speeds = {}

        for lane_movement in road.lane_movements:
            movement_id, _incoming_lane, outgoing_lane = lane_movement

            edge_id = downstream_edge(intersection_id, movement_id)

            v_down = edge_lane_speeds.get((edge_id, outgoing_lane))

            v_mov = settings.v_mov[intersection_id][movement_id]

            speeds[lane_movement] = crossing_speed(v_down, v_mov, settings.v_f)

        network_speeds[intersection_id] = speeds

    return network_speeds


"""
calculates the basic time separation needed for a pair of vehicles
"""
def lane_pair_seconds(first: tuple[int, int, int], second: tuple[int, int, int], v_first: float, v_second: float | None = None) -> float:
    v_r = _speed(v_first)
    v_s = _speed(v_first if v_second is None else v_second)

    interaction = lane_group(first, second)

    if interaction == "no":
        return 0.0

    if interaction == "same_lane_approach":
        return max(0.0, settings.tau + settings.length_veh / v_r + settings.delta)

    dist_exit, dist_enter = lane_distance(first, second)

    return max(0.0, (dist_exit + settings.length_veh) / v_r - dist_enter / v_s + settings.delta)



"""
creates a temporary dictionary containing downstream speeds until live data are given
"""
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
builds the complete lane aware impedance matrix for every intersection
"""
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
                    second_movement, second_in_lane, second_out_lane = second

                    second_approach = road.origins[second_movement]
                    exit_direction = road.destinations[second_movement]

                    gap += settings.arrival_buffer[intersection_id][second_approach][second_in_lane]
                    gap += settings.departure_buffer[intersection_id][exit_direction][second_out_lane]

                impedances[intersection_id][first][second] = gap

    return impedances




if __name__ == "__main__":

    edge_lane_speeds = default_edge_lane_speeds()
    lane_speeds = network_lane_movement_speeds(edge_lane_speeds)
    lane_impedances = update_lane_impedances(edge_lane_speeds)

    test_cases = [
        ( # (movement_id, incoming_lane, outgoing_lane)
            "I1", 
            (0, 0, 0),  # WT, lane 0 to lane 0
            (3, 0, 0),  # NT, lane 0 to lane 0
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
