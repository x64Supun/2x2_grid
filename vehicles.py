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



if __name__ == "__main__":
    test_vehicle = Vehicle(
        id=0,
        path=["I1", "I2"],
        movements={"I1": 0, "I2": 0},
        lanes={"I1": 0, "I2": 0},
        distance_to_next=20.0,
        incoming_edge="E0E",
    )

    print(test_vehicle)
    print(test_vehicle.id)
    print(test_vehicle.path)
    print(test_vehicle.frozen)