from dataclasses import dataclass, field


@dataclass
class Vehicle:
    id: int
    path: list[str]
    movements: dict[str, int]
    lanes: dict[str, int]
    distance_to_next: float
    incoming_edge: str
    frozen: dict[str, float] = field(default_factory = dict)

    # right after the vehicle is created, these two checks will validate cars
    def __post_init__(self):

        # checks for empty paths
        if not self.path:
            raise ValueError("A vehicle must have at least one intersection on its path.")

        # distance to the next intersection must be non negative
        if self.distance_to_next < 0.0:
            raise ValueError("distance_to_next cannot be negative.")

        # every intersection i the path needs a movement and a lane
        for intersection_id in self.path:
            if intersection_id not in self.movements:
                raise ValueError(f"Missing movement for intersection {intersection_id}")

            if intersection_id not in self.lanes:
                raise ValueError(f"Missing lane for intersection {intersection_id}")

        # no lane changes
        for current_intersection, next_intersection in zip(self.path, self.path[1:]):
            current_lane = self.lanes[current_intersection]
            next_lane = self.lanes[next_intersection]

            if current_lane != next_lane:
                raise ValueError(f"Vehicle {self.id} changes from lane {current_lane} to lane {next_lane} between {current_intersection} and {next_intersection}.")

        # commited time must belong to an intersection in the path 
        for intersection_id, fixed_time in self.frozen.items():
            if intersection_id not in self.path:
                raise ValueError(f"frozen time for {intersection_id} is not in vehicle {self.id}'s path.")




@dataclass
class TestScenario:
    vehicles: list[Vehicle] = field(default_factory = list)

    def add_vehicle(self, vehicle: Vehicle) -> None:
        for existing_vehicle in self.vehicles:
            if existing_vehicle.id == vehicle.id: # unique ID requirement
                raise ValueError(f"vehicle ID {vehicle.id} is already in.")

        self.vehicles.append(vehicle)



if __name__ == "__main__":
    test_vehicle = Vehicle(
        id=0,
        path=["I1", "I2"],
        movements={"I1": 0, "I2": 0},
        lanes={"I1": 0, "I2": 0},
        distance_to_next=20.0,
        incoming_edge="E0E",
    )
    
    #print(test_vehicle)
    #print(test_vehicle.id)
    #print(test_vehicle.path)
    #print(test_vehicle.frozen)
    

    scenario = TestScenario()
    print(scenario.vehicles)

    scenario.add_vehicle(test_vehicle)
    print(scenario.vehicles)