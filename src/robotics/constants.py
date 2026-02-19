# MCL Constants
from .datatypes import Cm, Line, Point


NUMBER_OF_PARTICLES = 50
# Variance of e,f,g for
e = 0.1
"""
Forward sonar helps with reducing uncertainty with repeated measurements throughout the execution of a movement
"""
f = 0.04
"""
Sideward facing sonar helps with reducing uncertainty with repeated measurements throughout the execution of a movement
"""
g = 0.05


MAP_WALLS = [Line(Point(Cm(x1), Cm(y1)), Point(Cm(x2), Cm(y2))) for (x1, y1, x2, y2) in [
    (0, 0, 0, 168),        # a
    (0, 168, 84, 168),     # b
    (84, 126, 84, 210),    # c
    (84, 210, 168, 210),   # d
    (168, 210, 168, 84),   # e
    (168, 84, 210, 84),    # f
    (210, 84, 210, 0),     # g
    (210, 0, 0, 0),        # h
]]
"""
Definitions of map walls to draw:
  a: O to A
  b: A to B
  c: C to D
  d: D to E
  e: E to F
  f: F to G
  g: G to H
  h: H to O
"""
