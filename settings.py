import math

v_f = 20.0
v_left = 8.0
v_right = 6.51

tau = 2.0
delta = 0.0
length_veh = 5.0
accel = 2.5
decel = 4.5
min_v = 0.1

big_m = 100.0


link_m = 500.0 # length of a link
number_of_lanes = 2
lane_width = 3.2
box = 2.0 * number_of_lanes * lane_width # 12.8, conflict square in the middle of the intersection
setback = 4.0 # distance from the stop bar to the middle conflict box
junction_m = box + 2 * setback # 20.8, intersection square dimension
spacing = link_m + junction_m # 520.8, distance between the center points of two consecutive intersections
stub = link_m + (junction_m / 2.0) # 510.4, distance from a J0-J7 node to a connecting intersection's center point
r_left = (junction_m + lane_width) / 2.0 # 8.8, left turn radius
r_right = (junction_m - lane_width) / 2.0 # 5.6, right turn radius
junction_t = junction_m # junction through distance
junction_l = math.pi * r_left / 2.0
junction_r = math.pi * r_right / 2.0


r_right_by_lane = {
    0: setback + lane_width * 0.5,
    1: setback + lane_width * 1.5,
}

r_left_by_lane = {
    0: setback + lane_width * 3.5,
    1: setback + lane_width * 2.5,
}

junction_r_by_lane = {
    0: math.pi * r_right_by_lane[0] / 2.0,
    1: math.pi * r_right_by_lane[1] / 2.0,
}

junction_l_by_lane = {
    0: math.pi * r_left_by_lane[0] / 2.0,
    1: math.pi * r_left_by_lane[1] / 2.0,
}








"""
distances second and first vehicles have to travel in order to enter and exit the conflict box
There are two different conflict boxes depending on the possible movement pairs
1. the middle square of 12.8m
2. the rectangle in between the above square and the stop bar, because a movement pair like (NT, WR) do not meet inside the above square.
"""

# (entering, exiting) distances for the 6.4m square conflict box
face_64 = {"T": (setback, setback + box), # remaining relationships except no and same approach
            "L": (r_left * math.asin(setback / r_left),
                  r_left * math.acos(setback / r_left)),
}

 
face_sb = { # merge_TR, merge_LR
    "T": (setback + box, junction_t),
    "L": (r_left * math.acos(setback / r_left), junction_l),
    "R": (r_right * math.asin(setback / r_right), junction_r),
}


face_merge_tl = { #merge_TL
    "T": (face_64["T"][0], junction_t),
    "L": (face_64["L"][0], junction_l)
}

"""
lane-specific entry and exit distances.

Structure:
face_name_by_lane[lane_id][turn_type]
    -> (enter_distance, exit_distance)
"""

face_main_by_lane = {}
face_sb_by_lane = {}
face_merge_tl_by_lane = {}

for lane_id in range(number_of_lanes):
    left_radius = r_left_by_lane[lane_id]
    right_radius = r_right_by_lane[lane_id]

    left_distance = junction_l_by_lane[lane_id]
    right_distance = junction_r_by_lane[lane_id]

    # middle 12.8 m conflict square
    face_main_by_lane[lane_id] = {
        "T": (
            setback,
            setback + box,
        ),
        "L": (
            left_radius * math.asin(setback / left_radius),
            left_radius * math.acos(setback / left_radius),
        ),
    }

    # stop-bar-side conflict region used by merge_TR and merge_LR
    face_sb_by_lane[lane_id] = {
        "T": (
            setback + box,
            junction_t,
        ),
        "L": (
            left_radius * math.acos(setback / left_radius),
            left_distance,
        ),
        "R": (
            right_radius * math.asin(setback / right_radius),
            right_distance,
        ),
    }

    # conflict region used by merge_TL
    face_merge_tl_by_lane[lane_id] = {
        "T": (
            face_main_by_lane[lane_id]["T"][0],
            junction_t,
        ),
        "L": (
            face_main_by_lane[lane_id]["L"][0],
            left_distance,
        ),
    }

#directions = ("NL", "NT", "NR", "EL", "ET", "ER", "SL", "ST", "SR", "WL", "WT", "WR")
directions = tuple(range(12))


#movement speed cap for each possible movement
v_mov = {
    "I1": {
        0: v_f, 1: v_left, 2: v_right,
        3: v_f, 4: v_left, 5: v_right,
        6: v_f, 7: v_left, 8: v_right,
        9: v_f, 10: v_left, 11: v_right,
    },
    "I2": {
        0: v_f, 1: v_left, 2: v_right,
        3: v_f, 4: v_left, 5: v_right,
        6: v_f, 7: v_left, 8: v_right,
        9: v_f, 10: v_left, 11: v_right,
    },
    "I3": {
        0: v_f, 1: v_left, 2: v_right,
        3: v_f, 4: v_left, 5: v_right,
        6: v_f, 7: v_left, 8: v_right,
        9: v_f, 10: v_left, 11: v_right,
    },
    "I4": {
        0: v_f, 1: v_left, 2: v_right,
        3: v_f, 4: v_left, 5: v_right,
        6: v_f, 7: v_left, 8: v_right,
        9: v_f, 10: v_left, 11: v_right,
    },
}


arrival_buffer = {
    "I1": {"N": 0.0, "E": 0.0, "S": 0.0, "W": 0.0},
    "I2": {"N": 0.0, "E": 0.0, "S": 0.0, "W": 0.0},
    "I3": {"N": 0.0, "E": 0.0, "S": 0.0, "W": 0.0},
    "I4": {"N": 0.0, "E": 0.0, "S": 0.0, "W": 0.0},
}

departure_buffer = {
    "I1": {"N": 0.0, "E": 0.0, "S": 0.0, "W": 0.0},
    "I2": {"N": 0.0, "E": 0.0, "S": 0.0, "W": 0.0},
    "I3": {"N": 0.0, "E": 0.0, "S": 0.0, "W": 0.0},
    "I4": {"N": 0.0, "E": 0.0, "S": 0.0, "W": 0.0},
}