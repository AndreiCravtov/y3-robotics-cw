import math
import random
import time
from enum import Enum
from statistics import mean
from typing import Any, Callable, Iterable, List, TypeVar, Tuple

import brickpi3  # type: ignore

BP = brickpi3.BrickPi3()

# Change to fit wiring configuration on the robot
LEFT_WHEEL = BP.PORT_A
RIGHT_WHEEL = BP.PORT_B
BOTH_WHEELS = [LEFT_WHEEL, RIGHT_WHEEL]

# Other constants
POLLING_INTERVAL = 0.03  # seconds
LEFT_POWER_LIMIT = 70
RIGHT_POWER_LIMIT = 70  # value between 0 to 100

# Units in cm unless stated otherwise
WHEEL_RADIUS = 1.5
WHEEL_SEPARATION = 16.0

# !! Measured data !!
# Please calibrate these before using them using the appropriate calibration functions
MAX_DPS = 150.0  # maximum degrees per second
RADIUS_MODIFIER = (
    0.98  # represents the multiplier between actual radius and measured radius
)
# RADIUS_MODIFIER = 0.95

# MCL Constants
NUMBER_OF_PARTICLES = 100
# Variance of e,f,g for
e = 5.0
f = 1.0
g = 0.7


class Particle:
    def __init__(self, x: float, y: float, theta: float, weight: float):
        self.x = x
        self.y = y
        self.theta = theta
        self.w = weight

    def move_forward(self, distance: float):
        var = random.gauss(0, sigma=e)
        self.x += math.cos(self.theta) * (distance + var)
        self.y += math.sin(self.theta) * (distance + var)
        self.theta += random.gauss(0, sigma=f)

    def turn(self, angle: float):
        var = random.gauss(0, sigma=g)
        self.theta += angle + var


T = TypeVar("T")


def allF(iterable: Iterable[T], condition: Callable[[T], bool]) -> bool:
    return all(map(condition, iterable))


def forEach(iterable: Iterable[T], func: Callable[[T], Any]) -> None:
    for i in iterable:
        func(i)


def normalise(angle: float) -> float:
    """
    Converts BP's angle [-180, 180] to [0, 360]
    """
    return angle + 180.0


def angle(rad: float) -> float:
    return math.degrees(rad)


def rad(angle: float) -> float:
    return math.radians(angle)


def actual_radius() -> float:
    return WHEEL_RADIUS * RADIUS_MODIFIER


class Rotation(Enum):
    Clockwise = 1
    Counterclockwise = 2


class WheelMovement():
    def __init__(self, wheel, distance: float, speed: float = rad(MAX_DPS) * actual_radius()):
        """
        Initializes a movement command to the specified wheel
        :param wheel: The wheel to move
        :param distance: cm
        :param speed: cm/s
        """
        self.wheel = wheel
        self.remaining_degrees = angle(distance / (2 * actual_radius()))
        self.forward = distance > 0
        self.dps = (speed / actual_radius()) * \
            360.0 * (1 if self.forward else -1)
        self.reset_angle()

    def reset_angle(self):
        BP.reset_motor_encoder(self.wheel)

    def update(self):
        delta_angle = BP.get_motor_encoder(self.wheel)
        self.remaining_degrees -= delta_angle
        BP.offset_motor_encoder(self.wheel, BP.get_motor_encoder(self.wheel))
        print(f"    Wheel {self.wheel} angle elapsed {delta_angle}")

        if self.is_complete():
            BP.set_motor_dps(self.wheel, 0)
            self.reset_angle()

    def begin(self):
        self.reset_angle()
        BP.set_motor_dps(self.wheel, self.dps)

    def is_complete(self):
        return self.remaining_degrees < 0 if self.forward else self.remaining_degrees > 0


def calibrate_MAX_DPS():
    """
    Calibrates the max degrees per seconds achieved by the motors when provided the max power. Please run after changing the weight of the vehicle
    """
    global MAX_DPS
    dps = []

    for wheel in [LEFT_WHEEL, RIGHT_WHEEL]:
        BP.set_motor_limits(wheel, RIGHT_POWER_LIMIT)
        BP.set_motor_power(wheel, RIGHT_POWER_LIMIT)

    time.sleep(3)  # To reach max speed

    for wheel in [LEFT_WHEEL, RIGHT_WHEEL]:
        BP.reset_motor_encoder(wheel)

    time.sleep(0.5)

    for wheel in [LEFT_WHEEL, RIGHT_WHEEL]:
        dps.append(normalise(BP.get_motor_encoder(wheel)) * 2)

    for wheel in [LEFT_WHEEL, RIGHT_WHEEL]:
        BP.set_motor_power(wheel, 0)

    MAX_DPS = mean(dps)
    print(
        f"Calibration for max degrees per second complete\nMax dps = {MAX_DPS}")


def calibrate_RADIUS_MODIFIER(meters: float = 1.0):
    """
    Goes forward by meters, travelling at MAX_DPS (require MAX_DPS to be calibrated). Please measure actual distance travelled, and this will be used to
    """
    global RADIUS_MODIFIER
    distance = meters * 100.0
    motorMovementHandler(
        list(map(lambda w: WheelMovement(w, distance=distance), BOTH_WHEELS))
    )
    actual_distance = float(
        input("What is the actual distance traveled (cm)? "))
    RADIUS_MODIFIER = actual_distance / distance
    print(f"Actual wheel radius: {actual_radius()}")


def dps_to_speed(dps: float = MAX_DPS, reduction_factor: float = 0.7) -> float:
    """
    Converts desired dps to speed in cm/s. By default returns 0.7 * max_dps speed
    """
    return rad(dps) * actual_radius() * reduction_factor


def motorMovementHandler(movements: List[WheelMovement]):
    """
    Moves the wheels ahead by distance centimeters. Does not assume the calibration has been completed yet. Assumes
    vehicle is evenly distributed
    """
    start_time = time.time()

    # Start each movement action
    forEach(movements, lambda mvmt: mvmt.begin())

    print("\nBeginning all movement commands\n")

    # Check every POLLING_INTERVAL the degrees moved by the motors, and subtract that from total degrees required to perform the action
    while not allF(movements, lambda mvmt: mvmt.is_complete()):
        time.sleep(POLLING_INTERVAL)
        forEach(movements, lambda mvmt: mvmt.update())

    print("\nMovement complete\n")
    end_time = time.time()
    elapsed_time = end_time - start_time

    print(f"Elapsed time: {elapsed_time}")


def stop():
    BP.set_motor_dps(LEFT_WHEEL, 0)
    BP.set_motor_dps(RIGHT_WHEEL, 0)

    BP.reset_motor_encoder(LEFT_WHEEL)
    BP.reset_motor_encoder(RIGHT_WHEEL)

    time.sleep(1)


class Robot:
    def __init__(self):

        stop()

        BP.set_motor_limits(LEFT_WHEEL, LEFT_POWER_LIMIT, 250)
        BP.set_motor_limits(RIGHT_WHEEL, RIGHT_POWER_LIMIT, 250)

        BP.set_motor_position_kp(LEFT_WHEEL, 55)
        BP.set_motor_position_kp(RIGHT_WHEEL, 55)

        self.particles = [
            Particle(0, 0, 0, 1 / NUMBER_OF_PARTICLES) for _ in range(100)
        ]

        print("Robot initialized successfully")

    def get_current_position(self) -> Tuple[float, float, float]:
        return (
            sum(p.x * p.w for p in self.particles),
            sum(p.y * p.w for p in self.particles),
            sum(p.theta * p.w for p in self.particles),
        )

    def draw_particles(self):
        for particle in self.particles:
            print("drawParticles:" +
                  f"({particle.x}, {particle.y}, {particle.theta})")

    def forward(self, distance: float):
        """
        Commands the robot to move forward by this distance in centimeters
        """
        distance = distance * (40 / 39.3)

        for particle in self.particles:
            particle.move_forward(distance)

        return motorMovementHandler(
            [
                WheelMovement(
                    LEFT_WHEEL,
                    distance=distance,
                    speed=dps_to_speed(reduction_factor=0.53),
                ),
                WheelMovement(
                    RIGHT_WHEEL,
                    distance=distance,
                    speed=dps_to_speed(reduction_factor=0.5),
                ),
            ]
        )

    def turn(self, direction: Rotation, degrees: float):
        """
        Turns in the direction, by alpha degrees.
        """
        forward_wheel, backward_wheel = (
            (LEFT_WHEEL, RIGHT_WHEEL)
            if direction == Rotation.Clockwise
            else (RIGHT_WHEEL, LEFT_WHEEL)
        )
        distance = WHEEL_SEPARATION * rad(degrees) / 2
        anglesForMovement = angle(distance / (2 * actual_radius()))

        for particle in self.particles:
            particle.turn(anglesForMovement)

        BP.set_motor_position(forward_wheel, anglesForMovement)
        BP.set_motor_position(backward_wheel, -anglesForMovement)

        # motorMovementHandler([
        #     WheelMovement(forward_wheel, distance = distance),
        #     WheelMovement(backward_wheel, distance = -distance)
        # ])

        print("Before sleep")

        time.sleep(1.4 * (anglesForMovement / MAX_DPS))

        print("After sleep")

    def navigate_to_waypoint(self, x: float, y: float):
        current_x, current_y, current_theta = self.get_current_position()
        target_angle = math.atan2(y - current_y, x - current_x)
        angle_to_turn = target_angle - current_theta

        self.turn(
            Rotation.Counterclockwise if angle_to_turn > 0 else Rotation.Clockwise,
            degrees=abs(rad(angle_to_turn)),
        )

        distance = math.sqrt((x - current_x) ** 2 + (y - current_y) ** 2)
        self.forward(distance)


def block():
    input("Please reset robot and press enter to start experiment")


def MCL():
    robot = Robot()
    for _ in range(4):
        for _ in range(4):
            robot.forward(10.0)
            robot.draw_particles()
            time.sleep(0.5)
        robot.turn(Rotation.Counterclockwise, degrees=90.0)
        robot.draw_particles()
        time.sleep(0.5)


def waypointTest():
    robot = Robot()
    robot.navigate_to_waypoint(30, 30)


def main():
    print("Hello Pi!")

    # turn(Rotation.Clockwise, degrees = 45.0)
    # Initial calibration
    # calibrate_max_dps()
    # calibrate_radius_modifier()

    # Block
    # block()

    # Square of 40cm
    # for x in range(4):
    #     forward(-40.0)
    #     stop()
    #     turn(Rotation.Clockwise, degrees=90.0)
    #     stop()

    MCL()
    # motorMovementHandler(
    #         [
    #             WheelMovement(
    #                 LEFT_WHEEL,
    #                 distance=10.0,
    #                 speed=dps_to_speed(reduction_factor=0.53),
    #             ),
    #             WheelMovement(
    #                 RIGHT_WHEEL,
    #                 distance=10.0,
    #                 speed=dps_to_speed(reduction_factor=0.5),
    #             ),
    #         ]
    #     )
    # waypointTest()

    # Block
    # block()
