import brickpi3
from typing import Any, Callable, Iterable, List, TypeVar, Tuple
from statistics import mean
from enum import Enum
import time
import random
import math
from .draw import Canvas, Cm, Map, Particles, Point, PointExt, Rad
from dataclasses import dataclass

BP = brickpi3.BrickPi3()

# Change to fit wiring configuration on the robot
FORWARD_SONAR_PORT = BP.PORT_1
RIGHT_SONAR_PORT = BP.PORT_2
LEFT_WHEEL = BP.PORT_A
RIGHT_WHEEL = BP.PORT_B
BOTH_WHEELS = [LEFT_WHEEL, RIGHT_WHEEL]

STARTING_COORDINATE = (84, 30)

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
    0.821  # represents the multiplier between actual radius and measured radius, previously 0.98
)
# RADIUS_MODIFIER = 0.95

# MCL Constants
NUMBER_OF_PARTICLES = 50
# Variance of e,f,g for
# e = 0.55 # forward sonar helps with reducing uncertainty with repeated measurements throughout the execution of a movement
# f = 0.08 # sideward facing sonar helps with reducing uncertainty with repeated measurements throughout the execution of a movement
# g = 0.02

e = 0.1
f = 0.04
g = 0.05

# e = 0
# f = 0
# g = 0

# Arena
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
        return f"({self.x}, {self.y}, {angle(self.theta)})"

    def __repr__(self):
        return f"({self.x + 100}, {self.y + 100}, {self.theta}, {self.w})"

    def calculate_likelihood(self, sonar_reading: float, sonar_direction: str = "forward"):
        assert sonar_direction in ["forward", "left"]
        sonar_bearing = self.theta if sonar_direction == "forward" else self.theta + \
            rad(90.0)
        c = 0.001
        sigma = 0.5  # was 1.0, but suggested to be 2-3cm on spec
        closest_dist = float("inf")

        pts = list(POINTS.values())

        for (ax, ay), (bx, by) in zip(pts, pts[1:] + pts[:1]):

            den = ((by - ay) * math.cos(sonar_bearing) -
                   (bx - ax) * math.sin(sonar_bearing))

            if abs(den) < 1e-9:
                continue

            num = ((by - ay) * (ax - self.x) - (bx - ax) * (ay - self.y))

            dist = num / den

            if dist <= 0:
                continue

            wall_x = self.x + dist * math.cos(sonar_bearing)
            wall_y = self.y + dist * math.sin(sonar_bearing)

            eps = 1e-6

            if (min(ax, bx) - eps <= wall_x <= max(ax, bx) + eps and
                    min(ay, by) - eps <= wall_y <= max(ay, by) + eps):
                closest_dist = min(closest_dist, dist)

        if closest_dist == float("inf"):
            # print("Something went wrong with calculating likelihood, no walls detected:" + self.__str__())
            pass

        self.w = math.exp(-((sonar_reading - closest_dist)
                          ** 2) / (2 * sigma**2)) + c


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
    def __init__(self, starting_coordinates: Tuple[int, int] = STARTING_COORDINATE, sensor_mode: str = "forward_only"):
        assert sensor_mode in ["forward_only", "all_sensors"]
        self.sensor_mode = sensor_mode

        stop()

        BP.set_sensor_type(FORWARD_SONAR_PORT, BP.SENSOR_TYPE.NXT_ULTRASONIC)
        BP.set_sensor_type(RIGHT_SONAR_PORT, BP.SENSOR_TYPE.NXT_ULTRASONIC)
        # time.sleep(3)

        BP.set_motor_limits(LEFT_WHEEL, LEFT_POWER_LIMIT, 250)
        BP.set_motor_limits(RIGHT_WHEEL, RIGHT_POWER_LIMIT, 250)

        BP.set_motor_position_kp(LEFT_WHEEL, 55)
        BP.set_motor_position_kp(RIGHT_WHEEL, 55)

        self.particles = [
            Particle(starting_coordinates[0], starting_coordinates[1], 0, 1 / NUMBER_OF_PARTICLES) for _ in range(NUMBER_OF_PARTICLES)
        ]

        print("Robot initialized successfully")

    def get_current_position(self) -> Tuple[float, float, float]:
        return (
            sum(p.x * p.w for p in self.particles),
            sum(p.y * p.w for p in self.particles),
            # sum(normalize_angle(p.theta) * p.w for p in self.particles),
            math.atan2(sum(math.sin(p.theta) * p.w for p in self.particles),
                       sum(math.cos(p.theta) * p.w for p in self.particles))
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
                    new_particles.append(Particle(
                        self.particles[i].x, self.particles[i].y, self.particles[i].theta, 1 / NUMBER_OF_PARTICLES))
                    break

        self.particles = new_particles
        assert (len(self.particles) == NUMBER_OF_PARTICLES)

    def draw_walls(self):
        pts = list(POINTS.values())
        for (ax, ay), (bx, by) in zip(pts, pts[1:] + pts[:1]):
            print(f"drawLine:({ax + 100}, {ay + 100}, {bx + 100}, {by + 100})")

    def draw_particles(self):
        self.draw_walls()
        print("drawParticles:" + str(self.particles))

    def forward(self, distance: float, update_particles: bool = True):
        """
        Commands the robot to move forward by this distance in centimeters
        """
        # distance = distance * (40 / 41)

        if update_particles:
            for particle in self.particles:
                particle.move_forward(distance)

        motorMovementHandler(
            [
                WheelMovement(
                    LEFT_WHEEL,
                    distance=distance,
                    speed=dps_to_speed(reduction_factor=0.75),
                ),
                WheelMovement(
                    RIGHT_WHEEL,
                    distance=distance,
                    speed=dps_to_speed(reduction_factor=0.7),
                ),
            ]
        )

        print(f"Moved {distance} forward")

        return

    def turn(self, degrees: float, update_particles: bool = True):
        """
        Turns in the direction, by alpha degrees.
        """
        forward_wheel, backward_wheel = (RIGHT_WHEEL, LEFT_WHEEL)
        distance = WHEEL_SEPARATION * rad(degrees) / 2
        anglesForMovement = angle(distance / (2 * actual_radius()))

        if update_particles:
            for particle in self.particles:
                particle.turn(rad(degrees))

        stop()
        BP.set_motor_position(forward_wheel, anglesForMovement)
        BP.set_motor_position(backward_wheel, -anglesForMovement)

        # print("Before sleep")

        time.sleep(1.2 * (abs(anglesForMovement) / MAX_DPS))

        # print("After sleep")

    def navigate_to_waypoint(self, x: float, y: float, step_size: float = 15.0, verbose: bool = True):
        """"
        Navigates to the waypoint in step_size sprints, with MCL at each sprint's end
        """
        # Compute directions to the waypoint from the current MCL cloud
        current_x, current_y, current_theta = self.get_current_position()
        target_angle = math.atan2(y - current_y, x - current_x)
        angle_to_turn = normalize_angle(target_angle - current_theta)
        distance = math.sqrt((x - current_x) ** 2 + (y - current_y) ** 2)

        # Describe the waypoint action
        if verbose:
            print(
                f"Current position: ({current_x}, {current_y}, {angle(current_theta)})")
            print(
                f"Angle to turn {angle(angle_to_turn)}, distance: {distance}")

        # Turn towards the waypoint, from the precomputed directional values
        self.turn(degrees=angle(angle_to_turn))
        if verbose:
            print("Turn complete. Moving forward...")
        time.sleep(0.3)

        # Determine if another step has to be performed after this, or is this the final sprint
        if distance < step_size:
            end_this_turn = True
        else:
            end_this_turn = False
        distance = min(step_size, distance)

        # Go forward by this distance (complete the sprint)
        self.forward(distance)

        # MCL localisation using sonar measurements, and resamples the cloud of particles
        self.resample(self.sensor_mode)

        # Recursively call the function without verbose if we have yet to reach our waypoint, which will resample
        # our location from the newly repopulated cloud
        if not end_this_turn:
            return self.navigate_to_waypoint(x=x, y=y, step_size=step_size, verbose=False)
        else:
            return None

    def get_forward_sonar_reading(self) -> float:
        while True:
            try:
                reading = BP.get_sensor(FORWARD_SONAR_PORT)
                if reading is not None and reading > 0:
                    return reading
            except brickpi3.SensorError as e:
                pass
                # print(f"Sensor error: {e}")
            time.sleep(0.1)

    def get_right_sensor_reading(self) -> float:
        while True:
            try:
                reading = BP.get_sensor(FORWARD_SONAR_PORT)
                if reading is not None and reading > 0:
                    return reading
            except brickpi3.SensorError as e:
                pass
                # print(f"Sensor error: {e}")
            time.sleep(0.1)

    def resample(self, mode: str = "forward_only"):
        """
        Performs a resampling of the points by taking measurements from the sonar.
        """
        assert mode in ["forward_only", "all_sensors"]
        z = self.get_forward_sonar_reading()
        for particle in self.particles:
            particle.calculate_likelihood(z, sonar_direction="forward")
        self.normalize_particle_weights()
        self.resample_particles()

        # side sonar data fusion
        if mode == "double":
            d = self.get_right_sensor_reading()
            for particle in self.particles:
                particle.calculate_likelihood(d, sonar_direction="right")
            self.normalize_particle_weights()
            self.resample_particles()

        self.draw_particles()

    def snap_to_wall(self, width_angle: int = 20, sonar: str = "forward"):
        """
        Rotate the robot slowly between -20 to 20 from the current position, and "snap" to the lowest sonar reading
        to get the robot to 90 degrees from the wall
        """

        assert sonar in ["forward", "right"]

        print("Snapping to wall...")

        # Save prevoius angle, to update all particles collectively at the end
        previous_angle = self.get_current_position()[2]

        # Turn to begin the scan
        self.turn(degrees=-width_angle, update_particles=False)

        # Depending on which sonar we are "snapping to the wall", we use a different sensor reading source
        get_sonar_reading = (lambda: self.get_forward_sonar_reading(
        )) if sonar == "forward" else (lambda: self.get_right_sensor_reading())

        # Perform the scan, which takes 10 measurements spread over the cone described by the width angle
        snapto_angle_relative = 0
        min_angle = -width_angle
        min_sonar_reading = get_sonar_reading()
        angular_step = int(width_angle / 5)
        for angle in range(int(width_angle * 2 / angular_step)):
            sonar_reading = get_sonar_reading()
            if sonar_reading < min_sonar_reading:
                print("New sonar reading lower than previous readings")
                # New sonar reading is lower than our previous minimum sonar reading
                min_sonar_reading = sonar_reading
                snapto_angle_relative = 0
                min_angle = angle * angular_step
            else:
                # Otherwise, we have gone over the optimal angle, so we keep track of how many degrees we need to
                # rotate back to make the robot face the optimal angle
                print("Incrementing snapto_angle_relative")
                snapto_angle_relative += angular_step

            # Turn by some angular step to take the next measurement
            self.turn(degrees=angular_step, update_particles=False)

        # Reset the robot to face the optimal angle
        self.turn(degrees=-snapto_angle_relative *
                  (0.75), update_particles=False)

        # Update all the particles with the new "fixed" angle, removing the previous angle noise
        noise = 0.1
        new_angle = previous_angle - width_angle + min_angle
        for particle in self.particles:
            particle.theta = new_angle + random.gauss(1, noise)


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
    robot = Robot(sensor_mode="forward_only")
    # robot.navigate_to_waypoint(84, 30)
    robot.navigate_to_waypoint(180, 30)
    robot.navigate_to_waypoint(180, 54)
    robot.navigate_to_waypoint(138, 54)
    robot.navigate_to_waypoint(138, 168)
    robot.navigate_to_waypoint(114, 168)
    robot.navigate_to_waypoint(114, 84)
    robot.navigate_to_waypoint(84, 84)
    robot.navigate_to_waypoint(84, 30)


def read_world_test_odometry():
    robot = Robot()
    robot.forward(96)  # -> 180, 30
    robot.turn(90)
    robot.forward(24)  # -> 180, 54
    robot.turn(90)
    robot.forward(42)  # -> 138, 54
    robot.turn(-90)
    robot.forward(114)  # -> 138, 168
    robot.turn(90)
    robot.forward(24)  # -> 114, 168
    robot.turn(90)
    robot.forward(84)  # -> 114, 84
    robot.turn(-90)
    robot.forward(30)  # -> 84, 84
    robot.turn(90)
    robot.forward(54)  # to origin


def mock_test():
    # Test wall points, 1 meter away
    global POINTS

    POINTS = {
        "O": (100, -1000),
        "A": (100, 1000),
        "B": (-50, 1000),
        "C": (-50, -1000),
    }

    robot = Robot(starting_coordinates=(0, 0))
    robot.navigate_to_waypoint(50, 0)
    robot.navigate_to_waypoint(0, 0)
    # robot.navigate_to_waypoint(30, 0)
    # robot.navigate_to_waypoint(-10, 0)
    # robot.navigate_to_waypoint(50, 0)
    # robot.navigate_to_waypoint(0, 0)


def look_ahead():
    robot = Robot(starting_coordinates=(0, 0))
    while True:
        print(robot.get_forward_sonar_reading())
        time.sleep(0.3)


def main():
    # ---- Set up drawing objects ----
    canvas = Canvas()
    map = Map(canvas)
    map.draw()
    particles = Particles(canvas)

    for _ in range(10):
        time.sleep(0.5)
        particles.update_and_draw(
            [PointExt(Point(Cm(random.gauss(80, 3) + 70*(math.sin(0))), Cm(random.gauss(70, 3) + 60*(math.sin(0)))), Rad(math.radians(random.randint(0, 360))), random.random())])
    # --------------------------------

    # real_world_test()
    # mock_test()
    # read_world_test_odometry()
    # robot = Robot()
    # for x in range(4 * 4):
    #     robot.forward(50.0)
    #     robot.turn(degrees=90)
