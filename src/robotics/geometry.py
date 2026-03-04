import math
from typing import Tuple

class Pose2D:
    """Represents a 2D pose with orientation (x, y, theta)."""

    def __init__(self, x: float, y: float, theta: float = 0.0):
        """
        Initialise a Pose2D.

        Args:
            x: x-coordinate
            y: y-coordinate
            theta: orientation angle in radians
        """
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)

    @classmethod
    def from_tuple2(cls, components: Tuple[float, float]) -> 'Pose2D':
        """Alternative constructor from a (x, y, theta) tuple."""
        return cls(components[0], components[1])

    @classmethod
    def from_tuple3(cls, components: Tuple[float, float, float]) -> 'Pose2D':
        """Alternative constructor from a (x, y, theta) tuple."""
        return cls(components[0], components[1], components[2])

    def to_tuple(self) -> Tuple[float, float, float]:
        """Return the pose as a tuple (x, y, theta)."""
        return (self.x, self.y, self.theta)

    def __repr__(self) -> str:
        return f"Pose2D(x={self.x}, y={self.y}, theta={self.theta})"

    def __add__(self, other: 'Vector') -> 'Pose2D':
        return Pose2D(self.x + other.x, self.y + other.y, self.angle_to(other)).normalise_theta()

    def to(self, other: 'Pose2D') -> 'Vector':
        """
        The vector between two positions.
        self -> other
        """
        if not isinstance(other, Pose2D):
            return NotImplemented
        return Vector(other.x - self.x, other.y - self.y)

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Pose2D):
            return False
        # Use math.isclose for floating-point comparison
        return (math.isclose(self.x, other.x) and
                math.isclose(self.y, other.y) and
                math.isclose(self.theta, other.theta))

    def normalise_theta(self) -> 'Pose2D':
        """
        Return a new Pose2D with theta normalised to the range [-π, π].
        The original pose is unchanged.
        """
        norm_theta = math.fmod(self.theta, 2 * math.pi)
        if norm_theta > math.pi:
            norm_theta -= 2 * math.pi
        elif norm_theta < -math.pi:
            norm_theta += 2 * math.pi
        return Pose2D(self.x, self.y, norm_theta)

    def angle_to(self, other: 'Vector') -> float:
        """
        Returns the signed angle (in degrees) to rotate from the current orientation
        to face the position of the other pose.

        A positive angle means a counter‑clockwise rotation; negative means clockwise.
        The result is normalised to the range (-180, 180] degrees.
        """
        dx = other.x - self.x
        dy = other.y - self.y
        target_angle = math.atan2(dy, dx)  # angle of the vector to the other pose
        diff = target_angle - self.theta

        # Normalise to (-π, π]
        diff = math.fmod(diff, 2 * math.pi)
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff <= -math.pi:
            diff += 2 * math.pi

        return math.degrees(diff)

    def relative_to_world(self, relative_pos : 'Pose2D') -> 'Pose2D':
        """
        Convert a point from the robot's local coordinate frame to the world frame.

        Returns:
            A tuple (world_x, world_y) representing the point in the global coordinate system.
        """
        rel_x = relative_pos.x
        rel_y = relative_pos.y

        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)
        world_x = self.x + rel_x * cos_theta - rel_y * sin_theta
        world_y = self.y + rel_x * sin_theta + rel_y * cos_theta
        return Pose2D(world_x, world_y)

    def world_to_relative(self, world_pos : 'Pose2D') -> 'Pose2D':
        """
        Convert a point from the world frame to the robot's local coordinate frame.

        Args:
            world_x: x-coordinate in the global world frame
            world_y: y-coordinate in the global world frame

        Returns:
            A tuple (rel_x, rel_y) representing the point in the robot's local coordinate system,
            where the robot's x‑axis points in the direction of its orientation (theta).
        """
        world_x = world_pos.x
        world_y = world_pos.y

        dx = world_x - self.x
        dy = world_y - self.y
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)

        # Rotate the translated point by -theta
        rel_x = dx * cos_theta + dy * sin_theta
        rel_y = -dx * sin_theta + dy * cos_theta

        return Pose2D(rel_x, rel_y)

    def __str__(self):
        return f"({int(self.x)}, {int(self.y)}, {math.degrees(self.theta).__round__(3)})"

class Vector:
    def __init__(self, x : float, y : float):
        self.x = x
        self.y = y

    def __add__(self, other: 'Vector') -> 'Vector':
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other: 'Vector') -> 'Vector':
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, other: 'Vector') -> float:
        return self.x * other.x + self.y * other.y

    def __neg__(self) -> 'Vector':
        return Vector(-self.x, -self.y)

    def __eq__(self, other: 'Vector') -> bool:
        return self.x == other.x and self.y == other.y

    def magnitude(self) -> float:
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def normalise(self) -> 'Vector':
        return Vector(self.x / self.magnitude(), self.y / self.magnitude())

    def scale_to(self, length: float) -> 'Vector':
        norm = self.normalise()
        return Vector(norm.x * length, norm.y * length)

    def __str__(self):
        return f"[{int(self.x)}, {int(self.y)}]"