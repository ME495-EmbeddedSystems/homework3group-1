from rclpy.node import Node
from rclpy.action import ActionClient


class MoveItApi():
    """
    Wraps the moveit ROS API for easy of use
    """

    def __init__(self, node: Node):
        self.node = node
