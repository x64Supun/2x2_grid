from gurobipy import GRB, Model, quicksum
from itertools import combinations

import impedance
import settings
import road
import gurobipy as gp

"""
what this does

vehicle data + current time + current edge speeds 
                        |
calculate crossing speeds and impedance matrices
                        |
create arrival time decision variables
                        |
add travel, ordering, separation, and frozen time constraints
                        |
minimize the sum of arrival times
                        |
return the schedule
"""

"""
vehicles: a list describing the vehicles being scheduled
return type = {vehicle_id: {intersection_id: scheduled_arrival_time}}

vehicle format
{
id: vehicle unique id,
path: remaining intersections in travel order,
movements: numeric movement at each intersection,
distance_to_next: distance to the next intersection,
incoming_edge: the edge vehicle is currently in,
frozen: arrival times that must remain fixed
}
ex:
{
    "id": 0,
    "path": ["I1", "I2"],
    "movements": {"I1": 0, "I2": 0},
    "distance_to_next": 120.0,
    "incoming_edge": "E0E",
    "frozen": {}, ex: {"I1": 10.0}
}
"""
def schedule_arrivals(vehicles: list[dict], t_now: float, edge_speeds: dict[str, float]) -> dict[int, dict[str, float]]:
    # when there are no vehicles to schedule
    if not vehicles:
        return {}

    """
    when the function is called, these two lines will calculate and store speeds and impedances for this run
    """
    crossing_speeds = impedance.network_movement_speeds(edge_speeds)
    impedances = impedance.update_impedances(edge_speeds)
    


    # create the model
    model = gp.Model("arrival_scheduler")

    T = {} # arrival times decision variable ( vehicle_id, intersection_id)
    order = {} # binary order variable (vehicle_i_id, vehicle_k_id, interection_id)

    # creates arrival times for each vehicle at each intersection as decision variables
    for vehicle in vehicles:
        vehicle_id = vehicle["id"]

        # add arrival times as variables
        for intersection_id in vehicle["path"]:
            T[vehicle_id, intersection_id] = model.addVar(lb = 0.0, vtype = GRB.CONTINUOUS, name = f"T_{vehicle_id}_{intersection_id}")

    # arrival time for the next intersection on the path
    for vehicle in vehicles:
        vehicle_id = vehicle["id"]
        next_intersection = vehicle["path"][0]
        frozen_times = vehicle.get("frozen", {})

        if next_intersection not in frozen_times:
            # calculates earliest arrival times for the next intersection
            earliest_arrival = (t_now + vehicle["distance_to_next"] / settings.v_f)

            # constraint 1: earliest arrival to the next intersection
            model.addConstr(T[vehicle_id, next_intersection] >= earliest_arrival, name = f"earliest_{vehicle_id}_{next_intersection}")


        """
        when this is implemented as a rolling optimization,

        read vehicle's position
        if vehicle_id in freeze_zone:
            if previous_schedule is True:
                schedule[vehicle_id][intersection_id] = already existing scheduled time
        """
        # if an arrival time is frozen, we have to preserve that
        for intersection_id, fixed_time in frozen_times.items():
            model.addConstr(T[vehicle_id, intersection_id] == fixed_time, name = f"frozen_{vehicle_id}_{intersection_id}")



    # Travel between consecutive intersections - link propagation
    for vehicle in vehicles:
        vehicle_id = vehicle["id"]
        path = vehicle["path"]

        # visit each consecutive pair of intersections along a vehicle's path
        for index in range(len(path) - 1):
            current_intersection = path[index]
            next_intersection = path[index + 1]

            # identify the movement at the current intersection
            # use its turn type to select crossing distances
            movement = vehicle["movements"][current_intersection]
            turn = road.turns[movement]

            junction_distance = road.junction_distances[turn]
            crossing_speed = crossing_speeds[current_intersection][movement]

            # minimum travel time between two stop bars
            travel_time = (junction_distance / crossing_speed + settings.link_m / settings.v_f)
            # constraint 2: link propagation
            model.addConstr(T[vehicle_id, next_intersection] >= T[vehicle_id, current_intersection] + travel_time, name=(f"travel_{vehicle_id}_"f"{current_intersection}_{next_intersection}"))

    # examine every distinct pair of vehicles once
    for vehicle_i, vehicle_k in combinations(vehicles, 2):

        # find intersections that are shared by two vehicles after the firs one
        shared_intersections = (set(vehicle_i["path"]) & set(vehicle_k["path"]))
        #print(f"Shared intersections: {vehicle_i['id']} and {vehicle_k['id']}, {sorted(shared_intersections)}")

        # vehicles on the same incoming edge must preserve thier order
        if vehicle_i["incoming_edge"] == vehicle_k["incoming_edge"]:

            distance_i = vehicle_i["distance_to_next"]
            distance_k = vehicle_k["distance_to_next"]

            # vehicle closer to the next stop bar is the leader
            if distance_i < distance_k:
                leader = vehicle_i
                follower = vehicle_k
            elif distance_i > distance_k:
                leader = vehicle_k
                follower = vehicle_i
            else:
                raise ValueError("cannot determine vehicle order. Two vehicles on the same edge have equal distances.")

            leader_id = leader["id"]
            follower_id = follower["id"]
            intersection_id = leader["path"][0]
            
            # constraint 3: preserve the same-lane arrival time order
            model.addConstr(T[leader_id, intersection_id] <= T[follower_id, intersection_id], name = f"lane_order_{leader_id}_{follower_id}_{intersection_id}")

        #add separation constraints at every intersection shared by this pair
        for intersection_id in sorted(shared_intersections):
            # read each vehicle's movement at this intersection
            movement_i = vehicle_i["movements"][intersection_id]
            movement_k = vehicle_k["movements"][intersection_id]

            #print(f"At {intersection_id}: vehicle {vehicle_i['id']} uses movement {movement_i} and vehicle {vehicle_k['id']} uses movement {movement_k}")


            gap_i_k = impedances[intersection_id][movement_i][movement_k]
            gap_k_i = impedances[intersection_id][movement_k][movement_i]

            #print(f"{intersection_id}: vehicle {vehicle_i['id']} before {vehicle_k['id']} (movements {movement_i} -> {movement_k}): {gap_i_k:.6f} s")
            #print(f"{intersection_id}: vehicle {vehicle_k['id']} before {vehicle_i['id']} (movements {movement_k} -> {movement_i}): {gap_k_i:.6f} s")

            # create one binary order variable for this vehicle pair at this intersection
            order_key = (vehicle_i['id'], vehicle_k['id'], intersection_id)
            order[order_key] = model.addVar(vtype = GRB.BINARY, name = f"order_{vehicle_i['id']}_{vehicle_k['id']}_{intersection_id}")

            i_first = order[order_key]

            # retrieve the two arrival-time decision variables
            time_i = T[vehicle_i["id"],intersection_id]
            time_k = T[vehicle_k["id"], intersection_id]

            model.addConstr((i_first == 1) >> (time_k >= time_i + gap_i_k), name = f"i_first_{vehicle_i['id']}_{vehicle_k['id']}_{intersection_id}")
            model.addConstr((i_first == 0) >> (time_i >= time_k + gap_k_i), name = f"k_first_{vehicle_i['id']}_{vehicle_k['id']}_{intersection_id}")

        # represent each route as directed links between consecutive intersections
        path_i = vehicle_i["path"]
        links_i = set(zip(path_i, path_i[1:]))

        path_k = vehicle_k["path"]
        links_k = set(zip(path_k, path_k[1:]))

        # find directed links both vehicles will travel along
        shared_links = links_i & links_k

        # require the same vehicle order at both ends of the shared link; prevents the scheduled order from reversing along that link
        for start_intersection, end_intersection in sorted(shared_links):
            order_at_start = order[
                vehicle_i["id"],
                vehicle_k["id"],
                start_intersection,
            ]

            order_at_end = order[
                vehicle_i["id"],
                vehicle_k["id"],
                end_intersection,
            ]

            model.addConstr(
                order_at_start == order_at_end,
                name=(
                    f"link_order_{vehicle_i['id']}_{vehicle_k['id']}_"
                    f"{start_intersection}_{end_intersection}"
                ),
            )

        print(f"Vehicles {vehicle_i['id']} and {vehicle_k['id']} share links: {shared_links}")











    
    model.setObjective(gp.quicksum(T.values()), GRB.MINIMIZE)

    model.optimize()

    if model.status != GRB.OPTIMAL:
        raise RuntimeError(f"No optimal solution found. Status: {model.status}")

    schedule = {}

    for (vehicle_id, intersection_id), variable in T.items():
        if vehicle_id not in schedule:
            schedule[vehicle_id] = {}

        schedule[vehicle_id][intersection_id] = variable.X


    #print("I1, WT followed by WT: ", impedances["I1"][0][0])

    #print(schedule[3]["I1"] - schedule[0]["I1"])
    #print(impedances["I1"][0][0])

    return schedule # expected output = schedule[vehicle_id][intersection_id]





    

if __name__ == "__main__":
    edge_speeds = impedance.default_edge_speeds()
    impedances = impedance.update_impedances(edge_speeds)

    vehicles = [
        # I1 from West: leader at 20 m, follower at 40 m.
        {
            "id": 0,
            "path": ["I1", "I2"],
            "movements": {"I1": 0, "I2": 0},
            "distance_to_next": 20.0,
            "incoming_edge": "E0E",
            "frozen": {}
        },
        {
            "id": 1,
            "path": ["I1", "I2"],
            "movements": {"I1": 0, "I2": 0},
            "distance_to_next": 40.0,
            "incoming_edge": "E0E",
            "frozen": {}
        },

        # I1 from North: leader at 20 m, follower at 40 m.
        {
            "id": 2,
            "path": ["I1", "I4"],
            "movements": {"I1": 3, "I4": 3},
            "distance_to_next": 20.0,
            "incoming_edge": "E1S",
            "frozen": {}
        },
        {
            "id": 3,
            "path": ["I1", "I4"],
            "movements": {"I1": 3, "I4": 3},
            "distance_to_next": 40.0,
            "incoming_edge": "E1S",
            "frozen": {}
        },

        # I2 from North: leader at 20 m, follower at 40 m.
        {
            "id": 4,
            "path": ["I2", "I3"],
            "movements": {"I2": 3, "I3": 3},
            "distance_to_next": 20.0,
            "incoming_edge": "E2S",
            "frozen": {}
        },
        {
            "id": 5,
            "path": ["I2", "I3"],
            "movements": {"I2": 3, "I3": 3},
            "distance_to_next": 40.0,
            "incoming_edge": "E2S",
            "frozen": {}
        },

        # I2 from East: leader at 20 m, follower at 40 m.
        {
            "id": 6,
            "path": ["I2", "I1"],
            "movements": {"I2": 6, "I1": 6},
            "distance_to_next": 20.0,
            "incoming_edge": "E3W",
            "frozen": {}
        },
        {
            "id": 7,
            "path": ["I2", "I1"],
            "movements": {"I2": 6, "I1": 6},
            "distance_to_next": 40.0,
            "incoming_edge": "E3W",
            "frozen": {}
        },

        # I3 from East: leader at 20 m, follower at 40 m.
        {
            "id": 8,
            "path": ["I3", "I4"],
            "movements": {"I3": 6, "I4": 6},
            "distance_to_next": 20.0,
            "incoming_edge": "E4W",
            "frozen": {}
        },
        {
            "id": 9,
            "path": ["I3", "I4"],
            "movements": {"I3": 6, "I4": 6},
            "distance_to_next": 40.0,
            "incoming_edge": "E4W",
            "frozen": {}
        },

        # I3 from South: leader at 20 m, follower at 40 m.
        {
            "id": 10,
            "path": ["I3", "I2"],
            "movements": {"I3": 9, "I2": 9},
            "distance_to_next": 20.0,
            "incoming_edge": "E5N",
            "frozen": {}
        },
        {
            "id": 11,
            "path": ["I3", "I2"],
            "movements": {"I3": 9, "I2": 9},
            "distance_to_next": 40.0,
            "incoming_edge": "E5N",
            "frozen": {}
        },

        # I4 from South: leader at 20 m, follower at 40 m.
        {
            "id": 12,
            "path": ["I4", "I1"],
            "movements": {"I4": 9, "I1": 9},
            "distance_to_next": 20.0,
            "incoming_edge": "E6N",
            "frozen": {}
        },
        {
            "id": 13,
            "path": ["I4", "I1"],
            "movements": {"I4": 9, "I1": 9},
            "distance_to_next": 40.0,
            "incoming_edge": "E6N",
            "frozen": {}
        },

        # I4 from West: leader at 20 m, follower at 40 m.
        {
            "id": 14,
            "path": ["I4", "I3"],
            "movements": {"I4": 0, "I3": 0},
            "distance_to_next": 20.0,
            "incoming_edge": "E7E",
            "frozen": {}
        },
        {
            "id": 15,
            "path": ["I4", "I3"],
            "movements": {"I4": 0, "I3": 0},
            "distance_to_next": 40.0,
            "incoming_edge": "E7E",
            "frozen": {}
        },
    ]

    schedule = schedule_arrivals(vehicles = vehicles, t_now = 0.0, edge_speeds = edge_speeds)

    for vehicle_id, arrivals in schedule.items():
        displayed_arrivals = {intersection_id: round(arrival_time, 1) for intersection_id, arrival_time in arrivals.items()}
        print(f"vehicle {vehicle_id}: {displayed_arrivals}")

    print("\nArrivals by intersection")
    for intersection_id in road.intersections:
        arrivals_here = []

        for vehicle_id, arrivals in schedule.items():
            if intersection_id in arrivals:
                arrival_time = arrivals[intersection_id]
                arrivals_here.append((arrival_time, vehicle_id))

        print(f"\n{intersection_id}: ")

        for arrival_time, vehicle_id in sorted(arrivals_here):
            print(f"vehicle {vehicle_id}: {arrival_time:.2f} seconds")

    #print(f"arrivals for vehicle id 0: ", schedule[0])