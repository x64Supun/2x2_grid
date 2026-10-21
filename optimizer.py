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






def vehicle_lane_movement(vehicle: dict, intersection_id: str) -> tuple[int, int, int]:

    movement_id = vehicle["movements"][intersection_id]
    lane_id = vehicle["lanes"][intersection_id]

    lane_movement = (movement_id, lane_id, lane_id)

    if lane_movement not in road.lane_movements:
        raise ValueError(f"Vehicle {vehicle['id']} has invalid lane movement {lane_movement} at {intersection_id}")

    return lane_movement







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
lanes: lanes used at each intersection
}
ex:
{
    "id": 0,
    "path": ["I1", "I2"],
    "movements": {"I1": 0, "I2": 0},
    "lanes": {"I1": 0, "I2": 0},
    "distance_to_next": 120.0,
    "incoming_edge": "E0E",
    "frozen": {}, ex: {"I1": 10.0}
}
"""
def schedule_arrivals(vehicles: list[dict], t_now: float, edge_lane_speeds: dict[tuple[str, int], float]) -> dict[int, dict[str, float]]:
    # when there are no vehicles to schedule
    if not vehicles:
        return {}

    # lane changes are not allowed anywhere along a vehicle's path.
    for vehicle in vehicles:
        path = vehicle["path"]

        for index in range(len(path) - 1):
            current_intersection = path[index]
            next_intersection = path[index + 1]

            current_lane = vehicle["lanes"][current_intersection]
            next_lane = vehicle["lanes"][next_intersection]

            # when entering vehicles, if the lane assignment is not consistent, this will break
            if current_lane != next_lane:
                raise ValueError(f"Vehicle {vehicle['id']} changes from lane {current_lane} to lane {next_lane} between {current_intersection} and {next_intersection}")

    """
    when the function is called, these two lines will calculate and store speeds and impedances for this run
    """
    crossing_speeds = impedance.network_lane_movement_speeds(edge_lane_speeds)
    impedances = impedance.update_lane_impedances(edge_lane_speeds)
    


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

            """
            constraint 1: earliest arrival to the next intersection
            """
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
            """
            constraint 2: if a time is already committed, it is kept as it is
            """
            model.addConstr(T[vehicle_id, intersection_id] == fixed_time, name = f"frozen_{vehicle_id}_{intersection_id}")



    # travel between consecutive intersections - link propagation
    for vehicle in vehicles:
        vehicle_id = vehicle["id"]
        path = vehicle["path"]

        # visit each consecutive pair of intersections along a vehicle's path
        for index in range(len(path) - 1):
            current_intersection = path[index]
            next_intersection = path[index + 1]

            # identify the movement at the current intersection
            # use its turn type to select crossing distances
            lane_movement = vehicle_lane_movement(vehicle, current_intersection)
            movement_id, incoming_lane, _outgoing_lane = (lane_movement)

            junction_distance = road.lane_junction_distance(movement_id, incoming_lane)
            crossing_speed = crossing_speeds[current_intersection][lane_movement]

            # minimum travel time between two stop bars
            travel_time = (junction_distance / crossing_speed + settings.link_m / settings.v_f)
            """
            constraint 3: link propagation
            """
            model.addConstr(T[vehicle_id, next_intersection] >= T[vehicle_id, current_intersection] + travel_time, name=(f"travel_{vehicle_id}_"f"{current_intersection}_{next_intersection}"))

    # examine every distinct pair of vehicles once
    for vehicle_i, vehicle_k in combinations(vehicles, 2):

        # find intersections that are shared by two vehicles after the firs one
        shared_intersections = (set(vehicle_i["path"]) & set(vehicle_k["path"]))
        #print(f"Shared intersections: {vehicle_i['id']} and {vehicle_k['id']}, {sorted(shared_intersections)}")

        # Vehicles on the same incoming edge and physical lane
        # must preserve their order.
        next_i = vehicle_i["path"][0]
        next_k = vehicle_k["path"][0]

        same_incoming_edge = (vehicle_i["incoming_edge"] == vehicle_k["incoming_edge"])
        same_next_intersection = next_i == next_k
        same_incoming_lane = (vehicle_i["lanes"][next_i] == vehicle_k["lanes"][next_k])

        if (same_incoming_edge and same_next_intersection and same_incoming_lane):

            distance_i = vehicle_i["distance_to_next"]
            distance_k = vehicle_k["distance_to_next"]

            if abs(distance_i - distance_k) <= settings.length_veh:
                raise ValueError(f"Vehicles {vehicle_i['id']} and {vehicle_k['id']} are less than one vehicle length apart.")

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
            
            """
            constraint 4: preserve the same-lane arrival time order
            """
            model.addConstr(T[leader_id, intersection_id] <= T[follower_id, intersection_id], name = f"lane_order_{leader_id}_{follower_id}_{intersection_id}")

        # Add separation constraints at every intersection shared by this pair.
        for intersection_id in sorted(shared_intersections):
            lane_movement_i = vehicle_lane_movement(vehicle_i,intersection_id)
            lane_movement_k = vehicle_lane_movement(vehicle_k,intersection_id)

            #print(f"At {intersection_id}: vehicle {vehicle_i['id']} uses movement {movement_i} and vehicle {vehicle_k['id']} uses movement {movement_k}")


            gap_i_k = impedances[intersection_id][lane_movement_i][lane_movement_k]
            gap_k_i = impedances[intersection_id][lane_movement_k][lane_movement_i]

            #print(f"{intersection_id}: vehicle {vehicle_i['id']} before {vehicle_k['id']} (movements {movement_i} -> {movement_k}): {gap_i_k:.6f} s")
            #print(f"{intersection_id}: vehicle {vehicle_k['id']} before {vehicle_i['id']} (movements {movement_k} -> {movement_i}): {gap_k_i:.6f} s")

            # create one binary order variable for this vehicle pair at this intersection
            order_key = (vehicle_i['id'], vehicle_k['id'], intersection_id)
            order[order_key] = model.addVar(vtype = GRB.BINARY, name = f"order_{vehicle_i['id']}_{vehicle_k['id']}_{intersection_id}")

            i_first = order[order_key]

            # retrieve the two arrival-time decision variables
            time_i = T[vehicle_i["id"],intersection_id]
            time_k = T[vehicle_k["id"], intersection_id]

            """
            constraint 5: for a pair of vehicles, constraints for both candidate crossing orderes are added, so that gurobi can choose the best order
            """
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
            lane_i = vehicle_i["lanes"][start_intersection]
            lane_k = vehicle_k["lanes"][start_intersection]

            # Different-lane vehicles may pass without changing lanes.
            if lane_i != lane_k:
                continue

            order_at_start = order[vehicle_i["id"], vehicle_k["id"], start_intersection]

            order_at_end = order[vehicle_i["id"], vehicle_k["id"], end_intersection]

            """
            constraint 6: preserve vehicle order along a shared lane
            """
            model.addConstr(order_at_start == order_at_end, name=(f"link_order_{vehicle_i['id']}_{vehicle_k['id']}_{start_intersection}_{end_intersection}"))

        # print(
        #     f"Vehicles {vehicle_i['id']} and {vehicle_k['id']} "
        #     f"share links: {shared_links}"
        # )











    
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
    edge_lane_speeds = impedance.default_edge_lane_speeds()
    impedances = impedance.update_lane_impedances(edge_lane_speeds)

    vehicles = [
        # I1 from West: leader at 20 m, follower at 40 m.
        {
            "id": 0,
            "path": ["I1", "I2"],
            "movements": {"I1": 0, "I2": 0},
            "lanes": {"I1": 0, "I2": 0},
            "distance_to_next": 20.0,
            "incoming_edge": "E0E",
            "frozen": {}
        },
        {
            "id": 1,
            "path": ["I1", "I2"],
            "movements": {"I1": 0, "I2": 0},
            "lanes": {"I1": 1, "I2": 1},
            "distance_to_next": 40.0,
            "incoming_edge": "E0E",
            "frozen": {}
        },

        # I1 from North: leader at 20 m, follower at 40 m.
        {
            "id": 2,
            "path": ["I1", "I4"],
            "movements": {"I1": 3, "I4": 3},
            "lanes": {"I1": 0, "I4": 0},
            "distance_to_next": 20.0,
            "incoming_edge": "E1S",
            "frozen": {}
        },
        {
            "id": 3,
            "path": ["I1", "I4"],
            "movements": {"I1": 3, "I4": 3},
            "lanes": {"I1": 1, "I4": 1},
            "distance_to_next": 40.0,
            "incoming_edge": "E1S",
            "frozen": {}
        },

        # I2 from North: leader at 20 m, follower at 40 m.
        {
            "id": 4,
            "path": ["I2", "I3"],
            "movements": {"I2": 3, "I3": 3},
            "lanes": {"I2": 0, "I3": 0},
            "distance_to_next": 20.0,
            "incoming_edge": "E2S",
            "frozen": {}
        },
        {
            "id": 5,
            "path": ["I2", "I3"],
            "movements": {"I2": 3, "I3": 3},
            "lanes": {"I2": 1, "I3": 1},
            "distance_to_next": 40.0,
            "incoming_edge": "E2S",
            "frozen": {}
        },

        # I2 from East: leader at 20 m, follower at 40 m.
        {
            "id": 6,
            "path": ["I2", "I1"],
            "movements": {"I2": 6, "I1": 6},
            "lanes": {"I2": 0, "I1": 0},
            "distance_to_next": 20.0,
            "incoming_edge": "E3W",
            "frozen": {}
        },
        {
            "id": 7,
            "path": ["I2", "I1"],
            "movements": {"I2": 6, "I1": 6},
            "lanes": {"I2": 1, "I1": 1},
            "distance_to_next": 40.0,
            "incoming_edge": "E3W",
            "frozen": {}
        },

        # I3 from East: leader at 20 m, follower at 40 m.
        {
            "id": 8,
            "path": ["I3", "I4"],
            "movements": {"I3": 6, "I4": 6},
            "lanes": {"I3": 0, "I4": 0},
            "distance_to_next": 20.0,
            "incoming_edge": "E4W",
            "frozen": {}
        },
        {
            "id": 9,
            "path": ["I3", "I4"],
            "movements": {"I3": 6, "I4": 6},
            "lanes": {"I3": 1, "I4": 1},
            "distance_to_next": 40.0,
            "incoming_edge": "E4W",
            "frozen": {}
        },

        # I3 from South: leader at 20 m, follower at 40 m.
        {
            "id": 10,
            "path": ["I3", "I2"],
            "movements": {"I3": 9, "I2": 9},
            "lanes": {"I3": 0, "I2": 0},
            "distance_to_next": 20.0,
            "incoming_edge": "E5N",
            "frozen": {}
        },
        {
            "id": 11,
            "path": ["I3", "I2"],
            "movements": {"I3": 9, "I2": 9},
            "lanes": {"I3": 1, "I2": 1},
            "distance_to_next": 40.0,
            "incoming_edge": "E5N",
            "frozen": {}
        },

        # I4 from South: leader at 20 m, follower at 40 m.
        {
            "id": 12,
            "path": ["I4", "I1"],
            "movements": {"I4": 9, "I1": 9},
            "lanes": {"I4": 0, "I1": 0},
            "distance_to_next": 20.0,
            "incoming_edge": "E6N",
            "frozen": {}
        },
        {
            "id": 13,
            "path": ["I4", "I1"],
            "movements": {"I4": 9, "I1": 9},
            "lanes": {"I4": 1, "I1": 1},
            "distance_to_next": 40.0,
            "incoming_edge": "E6N",
            "frozen": {}
        },

        # I4 from West: leader at 20 m, follower at 40 m.
        {
            "id": 14,
            "path": ["I4", "I3"],
            "movements": {"I4": 0, "I3": 0},
            "lanes": {"I4": 0, "I3": 0},
            "distance_to_next": 20.0,
            "incoming_edge": "E7E",
            "frozen": {}
        },
        {
            "id": 15,
            "path": ["I4", "I3"],
            "movements": {"I4": 0, "I3": 0},
            "lanes": {"I4": 1, "I3": 1},
            "distance_to_next": 40.0,
            "incoming_edge": "E7E",
            "frozen": {}
        },

        # Second vehicle in each lane: same through routes as vehicles 0-15.
        {
            "id": 16,
            "path": ["I1", "I2"],
            "movements": {"I1": 0, "I2": 0},
            "lanes": {"I1": 0, "I2": 0},
            "distance_to_next": 40.0,
            "incoming_edge": "E0E",
            "frozen": {}
        },
        {
            "id": 17,
            "path": ["I1", "I2"],
            "movements": {"I1": 0, "I2": 0},
            "lanes": {"I1": 1, "I2": 1},
            "distance_to_next": 20.0,
            "incoming_edge": "E0E",
            "frozen": {}
        },
        {
            "id": 18,
            "path": ["I1", "I4"],
            "movements": {"I1": 3, "I4": 3},
            "lanes": {"I1": 0, "I4": 0},
            "distance_to_next": 40.0,
            "incoming_edge": "E1S",
            "frozen": {}
        },
        {
            "id": 19,
            "path": ["I1", "I4"],
            "movements": {"I1": 3, "I4": 3},
            "lanes": {"I1": 1, "I4": 1},
            "distance_to_next": 20.0,
            "incoming_edge": "E1S",
            "frozen": {}
        },
        {
            "id": 20,
            "path": ["I2", "I3"],
            "movements": {"I2": 3, "I3": 3},
            "lanes": {"I2": 0, "I3": 0},
            "distance_to_next": 40.0,
            "incoming_edge": "E2S",
            "frozen": {}
        },
        {
            "id": 21,
            "path": ["I2", "I3"],
            "movements": {"I2": 3, "I3": 3},
            "lanes": {"I2": 1, "I3": 1},
            "distance_to_next": 20.0,
            "incoming_edge": "E2S",
            "frozen": {}
        },
        {
            "id": 22,
            "path": ["I2", "I1"],
            "movements": {"I2": 6, "I1": 6},
            "lanes": {"I2": 0, "I1": 0},
            "distance_to_next": 40.0,
            "incoming_edge": "E3W",
            "frozen": {}
        },
        {
            "id": 23,
            "path": ["I2", "I1"],
            "movements": {"I2": 6, "I1": 6},
            "lanes": {"I2": 1, "I1": 1},
            "distance_to_next": 20.0,
            "incoming_edge": "E3W",
            "frozen": {}
        },
        {
            "id": 24,
            "path": ["I3", "I4"],
            "movements": {"I3": 6, "I4": 6},
            "lanes": {"I3": 0, "I4": 0},
            "distance_to_next": 40.0,
            "incoming_edge": "E4W",
            "frozen": {}
        },
        {
            "id": 25,
            "path": ["I3", "I4"],
            "movements": {"I3": 6, "I4": 6},
            "lanes": {"I3": 1, "I4": 1},
            "distance_to_next": 20.0,
            "incoming_edge": "E4W",
            "frozen": {}
        },
        {
            "id": 26,
            "path": ["I3", "I2"],
            "movements": {"I3": 9, "I2": 9},
            "lanes": {"I3": 0, "I2": 0},
            "distance_to_next": 40.0,
            "incoming_edge": "E5N",
            "frozen": {}
        },
        {
            "id": 27,
            "path": ["I3", "I2"],
            "movements": {"I3": 9, "I2": 9},
            "lanes": {"I3": 1, "I2": 1},
            "distance_to_next": 20.0,
            "incoming_edge": "E5N",
            "frozen": {}
        },
        {
            "id": 28,
            "path": ["I4", "I1"],
            "movements": {"I4": 9, "I1": 9},
            "lanes": {"I4": 0, "I1": 0},
            "distance_to_next": 40.0,
            "incoming_edge": "E6N",
            "frozen": {}
        },
        {
            "id": 29,
            "path": ["I4", "I1"],
            "movements": {"I4": 9, "I1": 9},
            "lanes": {"I4": 1, "I1": 1},
            "distance_to_next": 20.0,
            "incoming_edge": "E6N",
            "frozen": {}
        },
        {
            "id": 30,
            "path": ["I4", "I3"],
            "movements": {"I4": 0, "I3": 0},
            "lanes": {"I4": 0, "I3": 0},
            "distance_to_next": 40.0,
            "incoming_edge": "E7E",
            "frozen": {}
        },
        {
            "id": 31,
            "path": ["I4", "I3"],
            "movements": {"I4": 0, "I3": 0},
            "lanes": {"I4": 1, "I3": 1},
            "distance_to_next": 20.0,
            "incoming_edge": "E7E",
            "frozen": {}
        },
    ]

    schedule = schedule_arrivals(vehicles=vehicles, t_now=0.0, edge_lane_speeds=edge_lane_speeds)

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
