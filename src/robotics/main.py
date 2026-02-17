import math
import random
import time
from enum import Enum
from statistics import mean
from typing import Any, Callable, Iterable, List, TypeVar, Tuple

import brickpi3

BP = brickpi3.BrickPi3()

# Change to fit wiring configuration on the robot
SONAR_PORT = BP.PORT_1
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
    0.85  # represents the multiplier between actual radius and measured radius, previously 0.98
)
# RADIUS_MODIFIER = 0.95

# MCL Constants
NUMBER_OF_PARTICLES = 500
# Variance of e,f,g for
e = 0
f = 0
g = 0

POINTS = {
    "O": (0, 0),
    "A": (0, 168),
    "B": (84, 168),
    "C": (84, 126),
    "D": (84, 210),
    "E": (168, 210),
    "F": (168, 84),
    "G": (210, 84),
    "H": (210, 0),
}


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
        self.theta = normalize_angle(self.theta + random.gauss(0, sigma=f))

    def turn(self, angle: float):
        var = random.gauss(0, sigma=g)
        self.theta = normalize_angle(self.theta + angle + var)

    def __str__(self):
        return f"({self.x}, {self.y}, {self.theta})"

    def __repr__(self):
        return f"({self.x + 100}, {self.y + 100}, {self.theta})"

    def calculate_likelihood(self, sonar_reading: float):
        c = 0.1
        sigma = 1.0
        closest_dist = float("inf")

        pts = list(POINTS.values())

        for (ax, ay), (bx, by) in zip(pts, pts[1:] + pts[:1], strict=True):

            den = ((by - ay) * math.cos(self.theta) - (bx - ax) * math.sin(self.theta))
            
            if abs(den) < 1e-9:
                continue

            num = ((by - ay) * (ax - self.x) - (bx - ax) * (ay - self.y))

            dist = num / den

            if dist <= 0:
                continue

            wall_x = self.x + dist * math.cos(self.theta)
            wall_y = self.y + dist * math.sin(self.theta)

            eps = 1e-6

            if (min(ax, bx) - eps <= wall_x <= max(ax, bx) + eps and
                min(ay, by) - eps <= wall_y <= max(ay, by) + eps):
                closest_dist = min(closest_dist, dist)

        if closest_dist == float("inf"):
            print("Something went wrong with calculating likelihood, no walls detected:" + self.__str__())

        self.w = math.exp(-((sonar_reading - closest_dist) ** 2) / (2 * sigma**2)) + c


T = TypeVar("T")


def allF(iterable: Iterable[T], condition: Callable[[T], bool]) -> bool:
    return all(map(condition, iterable))


def forEach(iterable: Iterable[T], func: Callable[[T], Any]) -> None:
    for i in iterable:
        func(i)


def normalize_angle(rad_angle: float) -> float:
    return (rad_angle + math.pi) % (2 * math.pi) - math.pi


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
        dps.append(normalize_angle(BP.get_motor_encoder(wheel)) * 2)

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
            Particle(0, 0, 0, 1 / NUMBER_OF_PARTICLES) for _ in range(NUMBER_OF_PARTICLES)
        ]

        print("Robot initialized successfully")

    def get_current_position(self) -> Tuple[float, float, float]:
        return (
            sum(p.x * p.w for p in self.particles),
            sum(p.y * p.w for p in self.particles),
            sum(p.theta * p.w for p in self.particles),
        )
    
    def normalize_particle_weights(self):
        total_weight = sum(p.w for p in self.particles)
        if total_weight == 0:
            print("All particles have zero weight???, resetting to uniform distribution")
            for p in self.particles:
                p.w = 1 / NUMBER_OF_PARTICLES
        else:
            for p in self.particles:
                p.w /= total_weight
    
    def resample_particles(self):
        # Normalize weights before resampling!
        cumulative_weights = []
        cumulative_sum = 0
        for p in self.particles:
            cumulative_sum += p.w
            cumulative_weights.append(cumulative_sum)

        new_particles = []
        for _ in range(NUMBER_OF_PARTICLES):
            r = random.random()
            for i, cw in enumerate(cumulative_weights):
                if r < cw:
                    new_particles.append(Particle(self.particles[i].x, self.particles[i].y, self.particles[i].theta, 1 / NUMBER_OF_PARTICLES))
                    break

        self.particles = new_particles

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

        motorMovementHandler(
            [
                WheelMovement(
                    LEFT_WHEEL,
                    distance=distance,
                    speed=dps_to_speed(reduction_factor=0.5),
                ),
                WheelMovement(
                    RIGHT_WHEEL,
                    distance=distance,
                    speed=dps_to_speed(reduction_factor=0.5),
                ),
            ]
        )

        print(f"Moved {distance} forward")

        return

    def turn(self, degrees: float):
        """
        Turns in the direction, by alpha degrees.
        """
        forward_wheel, backward_wheel = (RIGHT_WHEEL, LEFT_WHEEL)
        distance = WHEEL_SEPARATION * rad(degrees) / 2
        anglesForMovement = angle(distance / (2 * actual_radius()))

        for particle in self.particles:
            particle.turn(rad(degrees))

        stop()
        BP.set_motor_position(forward_wheel, anglesForMovement)
        BP.set_motor_position(backward_wheel, -anglesForMovement)

        print("Before sleep")

        time.sleep(1.4 * (abs(anglesForMovement) / MAX_DPS))

        print("After sleep")

    def navigate_to_waypoint(self, x: float, y: float):
        current_x, current_y, current_theta = self.get_current_position()
        target_angle = math.atan2(y - current_y, x - current_x)
        angle_to_turn = normalize_angle(target_angle - current_theta)
        distance = math.sqrt((x - current_x) ** 2 + (y - current_y) ** 2)
        print(
            f"Current position: ({current_x}, {current_y}, {angle(current_theta)})")
        print(f"Angle to turn {angle(angle_to_turn)}, distance: {distance}")
        self.turn(
            degrees=angle(angle_to_turn)
        )

        print("Turn complete. Moving forward...")
        
        time.sleep(0.75)

        self.forward(distance)

        z = self.get_sonar_reading()
        for particle in self.particles:
            particle.calculate_likelihood(z)
        
        self.normalize_particle_weights()
        self.resample_particles()



    def get_sonar_reading(self) -> float:
        return BP.get_sensor(SONAR_PORT)


def block():
    input("Please reset robot and press enter to start experiment")


def MCL():
    robot = Robot()
    for _ in range(4):
        for _ in range(4):
            robot.forward(10.0)
            robot.draw_particles()
            time.sleep(0.5)
        robot.turn(degrees=90.0)
        robot.draw_particles()
        time.sleep(0.5)


def waypointTest():
    robot = Robot()
    robot.navigate_to_waypoint(30, 30)
    robot.navigate_to_waypoint(30, 0)
    robot.navigate_to_waypoint(0, 30)
    robot.navigate_to_waypoint(0, 0)


def real_world_test():
    robot = Robot()
    robot.navigate_to_waypoint(84, 30)
    robot.navigate_to_waypoint(180, 30)
    robot.navigate_to_waypoint(180, 54)
    robot.navigate_to_waypoint(138, 54)
    robot.navigate_to_waypoint(138, 168)
    robot.navigate_to_waypoint(114, 168)
    robot.navigate_to_waypoint(114, 84)
    robot.navigate_to_waypoint(84, 84)
    robot.navigate_to_waypoint(84, 30)

real_world_test()
