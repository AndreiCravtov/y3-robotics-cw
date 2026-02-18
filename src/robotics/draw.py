import time
import random
import math
from typing import NamedTuple
import weakref

# Functions to generate some dummy particles data:
t = 0


def calcX():
    return Cm(random.gauss(80, 3) + 70*(math.sin(t)))


def calcY():
    return Cm(random.gauss(70, 3) + 60*(math.sin(2*t)))


def calcTheta():
    return Rad(math.radians(random.randint(0, 360)))


def calcW():
    return random.random()


# A Canvas class for drawing a map and particles:
# 	- it takes care of a proper scaling and coordinate transformation between
# the map frame of reference (in cm) and the display (in pixels)


class Cm(float):
    pass


class Px(float):
    pass


class Rad(float):
    pass


class Point(NamedTuple):
    x: Cm
    y: Cm


class PointExt(NamedTuple):
    p: Point
    theta: Rad
    weight: float
    """
    In the range [0,1]
    """


class Line(NamedTuple):
    p1: Point
    p2: Point


class Canvas:
    """
    A Canvas class for drawing lines and particles in physical space.

    It uses physical units (in cm) and a cartesian origin (bottom right);
    conversions to screen units (in px) handled automatically.
    """

    def __init__(self, physical_size: Cm = Cm(210.0), canvas_size: Px = Px(768.0), margin_coeff: float = 0.05):
        self.physical_size: Cm = physical_size
        self.canvas_size: Px = canvas_size
        self.margin: Cm = Cm(margin_coeff * physical_size)
        self.scale_cm_px: float = canvas_size / \
            (physical_size + 2 * self.margin)

    def drawLine(self, line: Line):
        x1 = self._x_cm_px(line.p1.x)
        y1 = self._y_cm_px(line.p1.y)
        x2 = self._x_cm_px(line.p2.x)
        y2 = self._y_cm_px(line.p2.y)
        print("drawLine:" + str((x1, y1, x2, y2)))

    def drawParticles[T: Point | PointExt](self, particles: list[T]):
        display = []
        for p in particles:
            match p:
                case PointExt():
                    if p.weight < 0 or p.weight > 1:
                        raise ValueError(
                            f"The point weight must be in interval [0,1], found {p.weight}")
                    display.append(
                        (self._x_cm_px(p.p.x), self._y_cm_px(p.p.y), math.degrees(p.theta), p.weight))
                case Point():
                    display.append(
                        (self._x_cm_px(p.x), self._y_cm_px(p.y)))
        print("drawParticles:" + str(display))

    def _x_cm_px(self, x: Cm) -> Px:
        return Px((x + self.margin) * self.scale_cm_px)

    def _y_cm_px(self, y: Cm) -> Px:
        return Px((self.margin + self.physical_size - y) * self.scale_cm_px)


class Map:
    """A Map class containing walls"""

    def __init__(self):
        self.walls: list[Line] = []

    def add_wall(self, wall: Line):
        self.walls.append(wall)

    def clear(self):
        self.walls = []

    def draw(self):
        for wall in self.walls:
            canvas.drawLine(wall)


class Particles:
    """Simple Particles set"""

    def __init__(self, n: int = 10):
        self.n = n
        self.particles = []

    def update(self):
        self.particles = [PointExt(Point(calcX(), calcY()), calcTheta(), calcW())
                          for _ in range(self.n)]

    def draw(self):
        canvas.drawParticles(self.particles)


canvas = Canvas()  # global canvas we are going to draw on

mymap = Map()
# Definitions of walls
# a: O to A
# b: A to B
# c: C to D
# d: D to E
# e: E to F
# f: F to G
# g: G to H
# h: H to O
mymap.add_wall(Line(Point(Cm(0), Cm(0)), Point(Cm(0), Cm(168))))        # a
mymap.add_wall(Line(Point(Cm(0), Cm(168)), Point(Cm(84), Cm(168))))     # b
mymap.add_wall(Line(Point(Cm(84), Cm(126)), Point(Cm(84), Cm(210))))    # c
mymap.add_wall(Line(Point(Cm(84), Cm(210)), Point(Cm(168), Cm(210))))   # d
mymap.add_wall(Line(Point(Cm(168), Cm(210)), Point(Cm(168), Cm(84))))   # e
mymap.add_wall(Line(Point(Cm(168), Cm(84)), Point(Cm(210), Cm(84))))    # f
mymap.add_wall(Line(Point(Cm(210), Cm(84)), Point(Cm(210), Cm(0))))     # g
mymap.add_wall(Line(Point(Cm(210), Cm(0)), Point(Cm(0), Cm(0))))        # h
mymap.draw()

particles = Particles()


def main():
    t = 0
    while True:
        particles.update()
        particles.draw()
        t += 0.05
        time.sleep(0.05)
