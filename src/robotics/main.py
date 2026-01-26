import math
import time
from statistics import mean
from typing import List
import brickpi3

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

def main():
    print("Hello Pi!")

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

        BP.reset_motor_encoder(wheel)
        time.sleep(0.5)
        dps.append(abs(BP.get_motor_encoder(wheel)) * 2)

    max_dps = mean(dps)
    print(f"Calibration for max degrees per second complete\nMax dps = {max_dps}")

def calibrate_radius_modifier(meters: float = 1.0):
    """
    Goes forward by meters, travelling at max_dps (require max_dps to be calibrated). Please measure actual distance travelled, and this will be used to
    """
    global radius_modifier
    distance = meters * 100.0
    ahead(list(map(lambda w : WheelMovement(w, distance = distance, speed = max_dps * actual_radius()), both_wheels)))
    actual_distance = float(input("What is the actual distance traveled? "))
    radius_modifier = distance / actual_distance



def abs(angle: float) -> float:
    """
    Converts BP's angle [-180, 180] to [0, 360]
    """
    return angle + 180.0

def angle(rad: float) -> float:
    return rad * math.pi / 180.0

def rad(angle: float) -> float:
    return math.radians(angle)

def actual_radius() -> float:
    return wheel_radius * radius_modifier

def ahead(movements: List[WheelMovement]):
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

def turn(direction: Rotation, alpha: float, rotational_speed_modifier: float = 0.5):
    """
    Turns in the direction, by alpha degrees. The speed of the wheels during the rotation is rotational_speed_modifier * max_dps
    """
    forward_wheel, backward_wheel = (left_wheel, right_wheel) if direction == Rotation.Clockwise else (right_wheel, left_wheel)
    distance = wheel_separation * rad(alpha)
    ahead([
        WheelMovement(forward_wheel, distance = distance, speed = max_dps * actual_radius() * 0.5),
        WheelMovement(backward_wheel, distance = -distance, speed = max_dps * actual_radius() * 0.5)
    ])


if __name__ == "__name__":
    main()
