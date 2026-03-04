from src.robotics.geometry import Pose2D

class RelativeFrameObstacle:
    def __init__(self, center : Pose2D, left_corner : Pose2D, right_corner : Pose2D):
        """
        An obstacle, which supplies information to the navigation algorithm to help the robot plan a route that avoids
        them. This is still in relative frame relative to the robot's orientation, so it must be transformed into a WorldFrameObstacle before data can be used.
        """

        self.center = center
        self.left_corner = left_corner
        self.right_corner = right_corner

    def to_world_frame(self, robot_pos : Pose2D) -> 'WorldFrameObstacle':
        """
        Converts the obstacle into world frame data, if the obstacle has not yet been converted yet
        """
        center = robot_pos.relative_to_world(self.center)
        left_corner = robot_pos.relative_to_world(self.left_corner)
        right_corner= robot_pos.relative_to_world(self.right_corner)
        actual_width = left_corner.to(right_corner).magnitude()
        return WorldFrameObstacle(center, left_corner, right_corner, actual_width)

class WorldFrameObstacle:
    def __init__(self, center : Pose2D, left_corner : Pose2D, right_corner : Pose2D, actual_width : float):
        """
        An obstacle, which supplies information to the navigation algorithm to help the robot plan a route that avoids
        them. This is still in relative frame relative to the robot's orientation, so it must be transformed into a WorldFrameObstacle before data can be used.
        """

        self.center = center
        self.left_corner = left_corner
        self.right_corner = right_corner
        self.width = actual_width

    def to_relative_frame(self, robot_pos : Pose2D) -> 'RelativeFrameObstacle':
        """
        Converts the world frame positioning system for this obstacle, into a relative frame coordinate, which can then be visualised easily.
        """
        center = robot_pos.world_to_relative(self.center)
        left_corner = robot_pos.world_to_relative(self.left_corner)
        right_corner = robot_pos.world_to_relative(self.right_corner)
        return RelativeFrameObstacle(center, left_corner, right_corner)
