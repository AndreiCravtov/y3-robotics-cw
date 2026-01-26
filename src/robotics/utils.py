from enum import Enum
from typing import Iterable, Callable, Any, TypeVar

from robotics.main import *

T = TypeVar('T')
def allF(iterable: Iterable[T], condition: Callable[[T], bool]) -> bool:
    return all(map(condition, iterable))

def forEach(iterable: Iterable[T], func: Callable[[T], Any]) -> None:
    for i in iterable:
        func(i)

class Rotation(Enum):
    Clockwise = 1
    Counterclockwise = 2

class WheelMovement():
    def __init__(self, wheel, distance: float, speed: float):
        """
        Initializes a movement command to the specified wheel
        :param wheel: The wheel to move
        :param distance: cm
        :param speed: cm/s
        """
        self.wheel = wheel
        self.remaining_degrees = angle(distance / actual_radius())
        self.dps = (speed / actual_radius()) * 360.0 * (1 if distance > 0 else -1)
        self.reset_angle()

    def reset_angle(self):
        BP.reset_motor_encoder(self.wheel)

    def update(self):
        delta_angle = BP.get_motor_encoder(self.wheel)
        self.remaining_degrees -= delta_angle

        if self.is_complete():
            BP.set_motor_dps(self.wheel, 0)

    def begin(self):
        BP.set_motor_dps(self.wheel, self.dps)

    def is_complete(self):
        return self.remaining_degrees > 0