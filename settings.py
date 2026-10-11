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

big_m = 100000


link_m = 500.0 # length of a link
lane_width = 3.2
junction_m = 14.4 # intersection square dimension
box = 2.0 * lane_width # 6.4, conflict square in the middle of the intersection
spacing = link_m + junction_m # 514.4, distance between the center points of two consecutive intersections
stub = link_m + (junction_m / 2.0) # 507.2, distance from a J0-J7 node to a connecting intersection's center point
setback =(junction_m - box) / 2.0 # 4.0, distance between stop bar and the middle conflict box
r_left = (junction_m + lane_width) / 2.0 # 8.8, left turn radius
r_right = (junction_m - lane_width) / 2.0 # 5.6, right turn radius
junction_t = junction_m # junction through distance
junction_l = math.pi * r_left / 2.0 # junction - left turn distance
junction_r = math.pi * r_right / 2.0 # junction - right turn distance

"""
distances second and first vehicles have to travel in order to enter and exit the conflict box
There are two different conflict boxes depending on the possible movement pairs
1. the middle square of 6.4m
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

directions = ("NL", "NT", "NR", "EL", "ET", "ER", "SL", "ST", "SR", "WL", "WT", "WR")

# movement speed cap for each possible movement
v_mov = {
    "I1": {
        "NT": v_f, "ET": v_f, "ST": v_f, "WT": v_f,
        "NL": v_left, "EL": v_left, "SL": v_left, "WL": v_left,
        "NR": v_right, "ER": v_right, "SR": v_right, "WR": v_right,
    },
    "I2": {
        "NT": v_f, "ET": v_f, "ST": v_f, "WT": v_f,
        "NL": v_left, "EL": v_left, "SL": v_left, "WL": v_left,
        "NR": v_right, "ER": v_right, "SR": v_right, "WR": v_right,
    },
    "I3": {
        "NT": v_f, "ET": v_f, "ST": v_f, "WT": v_f,
        "NL": v_left, "EL": v_left, "SL": v_left, "WL": v_left,
        "NR": v_right, "ER": v_right, "SR": v_right, "WR": v_right,
    },
    "I4": {
        "NT": v_f, "ET": v_f, "ST": v_f, "WT": v_f,
        "NL": v_left, "EL": v_left, "SL": v_left, "WL": v_left,
        "NR": v_right, "ER": v_right, "SR": v_right, "WR": v_right,
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