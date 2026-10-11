import settings

"""
This will be a 2x2 symmetric road network. There are four intersections, connected in a grid manner; no diagonal connections.
"""


# intersections are placed in a 2x2 grid. I1 at top left, I2 at top right, I3 at bottom right and I4 at bottom left.

intersections = ("I1", "I2", "I3", "I4")

"""
Jx denote the starting points of junctions connected to intesections.
J0 to I1 from West
J1 to I1 from North
J2 to I2 from North
J3 to I2 from East
J4 to I3 from East
J5 to I3 from South
J6 to I4 from South
J7 to I4 from West
"""

ring = ("J0", "J1", "J2", "J3", "J4", "J5", "J6", "J7")

# absolute positions of intersection center points and nodes
nodes = {
    "I1": (0.0, 0.0),
    "I2": (settings.spacing, 0.0),
    "I3": (settings.spacing, -settings.spacing),
    "I4": (0.0, -settings.spacing),
    "J0": (-settings.stub, 0.0),
    "J1": (0.0, settings.stub),
    "J2": (settings.spacing, settings.stub),
    "J3": (settings.spacing + settings.stub, 0.0),
    "J4": (settings.spacing + settings.stub, -settings.spacing),
    "J5": (settings.spacing, -settings.spacing - settings.stub),
    "J6": (0.0, -settings.spacing - settings.stub),
    "J7": (-settings.stub, -settings.spacing)
}


"""
each node is connected with two bi-directional lanes.
dictionary structure is edge_name: (starting_node, ending_node)
"""
edges = {
    "E0E": ("J0", "I1"), "E0W": ("I1", "J0"),
    "E1S": ("J1", "I1"), "E1N": ("I1", "J1"),
    "E2S": ("J2", "I2"), "E2N": ("I2", "J2"),
    "E3W": ("J3", "I2"), "E3E": ("I2", "J3"),
    "E4W": ("J4", "I3"), "E4E": ("I3", "J4"),
    "E5N": ("J5", "I3"), "E5S": ("I3", "J5"),
    "E6N": ("J6", "I4"), "E6S": ("I4", "J6"),
    "E7E": ("J7", "I4"), "E7W": ("I4", "J7"),
    "E8E": ("I1", "I2"), "E8W": ("I2", "I1"),
    "E9N": ("I3", "I2"), "E9S": ("I2", "I3"),
    "E10E": ("I4", "I3"), "E10W": ("I3", "I4"),
    "E11N": ("I4", "I1"), "E11S": ("I1", "I4"),
}

# entry points and the connected edges
entry_edge = {
    "J0": "E0E",
    "J1": "E1S",
    "J2": "E2S",
    "J3": "E3W",
    "J4": "E4W",
    "J5": "E5N",
    "J6": "E6N",
    "J7": "E7E",
}

# exit points and the connected edges
exit_edge = {
    "J0": "E0W",
    "J1": "E1N",
    "J2": "E2N",
    "J3": "E3E",
    "J4": "E4E",
    "J5": "E5S",
    "J6": "E6S",
    "J7": "E7W",
}

# outbound edges at each intersection
outgoing_edge = {
    "I1": {
        "N": "E1N",
        "E": "E8E",
        "S": "E11S",
        "W": "E0W",
    },
    "I2": {
        "N": "E2N",
        "E": "E3E",
        "S": "E9S",
        "W": "E8W",
    },
    "I3": {
        "N": "E9N",
        "E": "E4E",
        "S": "E5S",
        "W": "E10W",
    },
    "I4": {
        "N": "E11N",
        "E": "E10E",
        "S": "E6S",
        "W": "E7W",
    },
}

#vehicles' inbound directions at an intersections and their distination directions

# Movement ID -> departure side
destinations = {
    0: "E",    # WT
    1: "N",    # WL
    2: "W",    # NR
    3: "S",    # NT
    4: "E",    # NL
    5: "N",    # ER
    6: "W",    # ET
    7: "S",    # EL
    8: "E",    # SR
    9: "N",    # ST
    10: "W",   # SL
    11: "S",   # WR
}


# Movement ID -> arrival side
origins = {
    0: "W",    # WT
    1: "W",    # WL
    2: "N",    # NR
    3: "N",    # NT
    4: "N",    # NL
    5: "E",    # ER
    6: "E",    # ET
    7: "E",    # EL
    8: "S",    # SR
    9: "S",    # ST
    10: "S",   # SL
    11: "W",   # WR
}

# Movement ID -> turn type
turns = {
    0: "T",
    1: "L",
    2: "R",
    3: "T",
    4: "L",
    5: "R",
    6: "T",
    7: "L",
    8: "R",
    9: "T",
    10: "L",
    11: "R",
}