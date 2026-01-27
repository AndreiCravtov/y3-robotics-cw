from robotics.main import forward, turn
from robotics.utils import Rotation


def square(edge_length: float = 40.0):
    # Square of 40cm
    for x in range(4):
        forward(edge_length)
        turn(Rotation.Counterclockwise, degrees= 90.0)