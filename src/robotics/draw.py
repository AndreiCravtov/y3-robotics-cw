import math
from typing import NamedTuple, Sequence


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

    def drawParticles(self, particles: Sequence[Point | PointExt]):
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


DEFAULT_WALLS = [
    Line(Point(Cm(0), Cm(0)), Point(Cm(0), Cm(168))),        # a
    Line(Point(Cm(0), Cm(168)), Point(Cm(84), Cm(168))),     # b
    Line(Point(Cm(84), Cm(126)), Point(Cm(84), Cm(210))),    # c
    Line(Point(Cm(84), Cm(210)), Point(Cm(168), Cm(210))),   # d
    Line(Point(Cm(168), Cm(210)), Point(Cm(168), Cm(84))),   # e
    Line(Point(Cm(168), Cm(84)), Point(Cm(210), Cm(84))),    # f
    Line(Point(Cm(210), Cm(84)), Point(Cm(210), Cm(0))),     # g
    Line(Point(Cm(210), Cm(0)), Point(Cm(0), Cm(0))),        # h
]
"""
Definitions of walls:
  a: O to A
  b: A to B
  c: C to D
  d: D to E
  e: E to F
  f: F to G
  g: G to H
  h: H to O
"""


class Map:
    """A Map class containing walls"""

    def __init__(self, canvas: Canvas, walls: list[Line] = DEFAULT_WALLS):
        self.canvas: Canvas = canvas
        self.walls: list[Line] = walls

    def add_wall(self, wall: Line):
        self.walls.append(wall)

    def clear(self):
        self.walls = []

    def draw(self):
        for wall in self.walls:
            self.canvas.drawLine(wall)


class Particles:
    """Simple Particles set"""

    def __init__(self, canvas: Canvas, particles: Sequence[Point | PointExt] = []):
        self.canvas: Canvas = canvas
        self.particles: Sequence[Point | PointExt] = particles

    def update_only(self, particles: Sequence[Point | PointExt]):
        self.particles = particles

    def draw_only(self):
        self.canvas.drawParticles(self.particles)

    def update_and_draw(self, particles: Sequence[Point | PointExt]):
        self.update_only(particles)
        self.draw_only()
