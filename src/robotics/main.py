import math
import time
from statistics import mean
from typing import List
import brickpi3

from robotics.scripts.square import square
from robotics.utils import forEach, WheelMovement, Rotation, allF

BP = brickpi3.BrickPi3()

# Change to fit wiring configuration on the robot
left_wheel = BP.PORT_A
right_wheel = BP.PORT_B
both_wheels = [left_wheel, right_wheel]

# Other constants
polling_interval = 0.2 # seconds
power_limit = 70 # value between 0 to 100

# Units in cm unless stated otherwise
wheel_radius = 3.0
wheel_separation = 16.0

# !! Measured data !!
# Please calibrate these before using them using the appropriate calibration functions
max_dps = 150.0 # maximum degrees per second
radius_modifier = 1.0 # represents the multiplier between motor rotation and distance moved

def calibrate_max_dps():
    """
    Calibrates the max degrees per seconds achieved by the motors when provided the max power. Please run after changing the weight of the vehicle
    """
    global max_dps
    dps = []

    for wheel in [left_wheel, right_wheel]:
        BP.set_motor_limits(wheel, power_limit)
        BP.set_motor_power(wheel, power_limit)

    time.sleep(3) # To reach max speed

    for wheel in [left_wheel, right_wheel]:
        BP.reset_motor_encoder(wheel)

    time.sleep(0.5)

    for wheel in [left_wheel, right_wheel]:
        dps.append(abs(BP.get_motor_encoder(wheel)) * 2)

    max_dps = mean(dps)
    print(f"Calibration for max degrees per second complete\nMax dps = {max_dps}")

def calibrate_radius_modifier(meters: float = 1.0):
    """
    Goes forward by meters, travelling at max_dps (require max_dps to be calibrated). Please measure actual distance travelled, and this will be used to
    """
    global radius_modifier
    distance = meters * 100.0
    motorMovementHandler(list(map(lambda w : WheelMovement(w, distance = distance, speed =max_dps * actual_radius()), both_wheels)))
    actual_distance = float(input("What is the actual distance traveled (cm)? "))
    radius_modifier = actual_distance / distance
    print(f"Actual wheel radius: {actual_radius()}")

def abs(angle: float) -> float:
    """
    Converts BP's angle [-180, 180] to [0, 360]
    """
    return angle + 180.0

def angle(rad: float) -> float:
    return rad * math.pi / 180.0

def rad(angle: float) -> float:
    return math.radians(angle)

def dps_to_speed(dps: float = max_dps, reduction_factor: float = 0.7) -> float:
    """
    Converts desired dps to speed in cm/s. By default returns 0.7 * max_dps speed
    """
    return dps * actual_radius() * reduction_factor

def actual_radius() -> float:
    return wheel_radius * radius_modifier

def motorMovementHandler(movements: List[WheelMovement]):
    """
    Moves the wheels ahead by distance centimeters. Does not assume the calibration has been completed yet. Assumes
    vehicle is evenly distributed
    """
    start_time = time.time()

    # Start each movement action
    forEach(movements, lambda mvmt: mvmt.begin())

    # Check every POLLING_INTERVAL the degrees moved by the motors, and subtract that from total degrees required to perform the action
    while not allF(movements, lambda mvmt : mvmt.is_complete()):
        forEach(movements, lambda mvmt : mvmt.reset_angle())
        time.sleep(polling_interval)
        forEach(movements, lambda mvmt : mvmt.update())

    end_time = time.time()
    elapsed_time = end_time - start_time

    print(f"Elapsed time: {elapsed_time}")

def forward(distance: float):
    """
    Commands the robot to move forward by this distance
    """
    return motorMovementHandler([
        WheelMovement(left_wheel, distance = distance, speed = dps_to_speed()),
        WheelMovement(right_wheel, distance = distance, speed = dps_to_speed()),
    ])

def turn(direction: Rotation, degrees: float, rotational_speed_modifier: float = 0.5):
    """
    Turns in the direction, by alpha degrees. The speed of the wheels during the rotation is rotational_speed_modifier * max_dps
    """
    forward_wheel, backward_wheel = (left_wheel, right_wheel) if direction == Rotation.Clockwise else (right_wheel, left_wheel)
    distance = wheel_separation * rad(degrees)
    motorMovementHandler([
        WheelMovement(forward_wheel, distance = distance, speed = dps_to_speed(reduction_factor=0.5)),
        WheelMovement(backward_wheel, distance = -distance, speed = dps_to_speed(reduction_factor=0.5))
    ])

def block():
    input("Please reset robot and press enter to start experiment")

def main():
    print("Hello Pi!")

    # Initial calibration
    calibrate_max_dps()
    # calibrate_radius_modifier()
    #
    # # Block
    # block()
    #
    # # Square 40
    # square(40.0)
    #
    # # Block
    # block()


if __name__ == "__name__":
    main()
