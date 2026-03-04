"""
This module contains logic for a drawing abstraction in general,
as well as specific abstractions for drawing maps and Monte-Carlo Localization particle-clouds.
"""
from __future__ import annotations

import math
from typing import Sequence
from .datatypes import Cm, Line, Particle, Point, Pr, Px, Rad


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

    def draw_line(self, line: Line):
        x1 = self._x_cm_px(line.p1.x)
        y1 = self._y_cm_px(line.p1.y)
        x2 = self._x_cm_px(line.p2.x)
        y2 = self._y_cm_px(line.p2.y)
        print("drawLine:" + str((x1, y1, x2, y2)))

    def draw_particles(self, particles: Sequence[Point | Particle]):
        display = []
        # for p in particles:
        #     match p:
        #         case Particle():
        #             if p.weight < 0 or p.weight > 1:
        #                 raise ValueError(
        #                     f"The point weight must be in interval [0,1], found {p.weight}")
        #             display.append(
        #                 (self._x_cm_px(p.p.x), self._y_cm_px(p.p.y), math.degrees(p.theta), p.weight))
        #         case Point():
        #             display.append(
        #                 (self._x_cm_px(p.x), self._y_cm_px(p.y)))
        print("drawParticles:" + str(display))

    def _x_cm_px(self, x: Cm) -> Px:
        return Px((x + self.margin) * self.scale_cm_px)

    def _y_cm_px(self, y: Cm) -> Px:
        return Px((self.margin + self.physical_size - y) * self.scale_cm_px)


class Map:
    """A Map class containing walls"""

    def __init__(self, canvas: Canvas, walls: list[Line]):
        self.canvas: Canvas = canvas
        self.walls: list[Line] = walls

    def add_wall(self, wall: Line):
        self.walls.append(wall)

    def clear(self):
        self.walls = []

    def draw(self):
        for wall in self.walls:
            self.canvas.draw_line(wall)


class Particles:
    """Set of particles complete with Monte-Carlo Localization mathematics"""

    def __init__(self, canvas: Canvas, points: list[Point], starting_theta: Rad = Rad(0)):
        self.canvas: Canvas = canvas
        self._validate_and_set(points, starting_theta)

    def _validate_and_set(self, points: list[Point], starting_theta: Rad):
        # Ensure at least 1 point, then init
        n = len(points)
        if n == 0:
            raise ValueError(f"Expected at least 1 particle, got {n!r}")
        starting_weight = Pr(1 / len(points))
        self.particles: list[Particle] = [
            Particle(p, starting_theta, starting_weight) for p in points]

    def reset(self,  points: list[Point], starting_theta: Rad = Rad(0)):
        self._validate_and_set(points, starting_theta)

    def draw(self):
        self.canvas.draw_particles(self.particles)
