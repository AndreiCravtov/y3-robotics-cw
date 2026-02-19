"""
This module contains general, non-specific data types/structures.

A little bit like a "utils" for types.
"""

import math
from typing import NamedTuple, Self


class Cm(float):
    """Centimeter newtype"""

    def to_px(self, px_per_cm: float) -> "Px":
        return Px(self * px_per_cm)


class Px(float):
    """Pixel newtype"""

    def to_cm(self, cm_per_px: float) -> Cm:
        return Cm(self * cm_per_px)


class Rad(float):
    """Radiant newtype"""

    def to_deg(self) -> "Deg":
        return Deg(math.degrees(self))


class Deg(float):
    """Degree newtype"""

    def to_rad(self) -> Rad:
        return Rad(math.radians(self))


class Pr(float):
    """
    Probability newtype.

    Values must be in the range [0,1].
    """

    __slots__ = ()

    def __new__(cls, value: float) -> Self:
        v = float(value)
        if not (0.0 <= v <= 1.0):
            raise ValueError(f"Pr must be in [0, 1], got {v!r}")
        return super().__new__(cls, v)  # calls float.__new__ via MRO


class Point(NamedTuple):
    """2D point in Cartesian space"""

    x: Cm
    y: Cm


class Line(NamedTuple):
    """2D line in Cartesian space"""

    p1: Point
    p2: Point


class Particle(NamedTuple):
    """
    Monte-Carlo localization particle 
    TODO: add better docs here??
    """

    p: Point
    theta: Rad
    weight: Pr

    @classmethod
    def of(cls, x: Cm, y: Cm, theta: Rad, weight: Pr) -> Self:
        return cls.__new__(cls, Point(x, y), theta, weight)
