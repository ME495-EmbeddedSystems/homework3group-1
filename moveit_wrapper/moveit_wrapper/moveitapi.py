from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup


class MoveItApi():
    """
    Wraps the moveit ROS API for easy of use
    """

    def __init__(self, node: Node):
        self.node = node
        self.move_group_action_client = ActionClient(
            self.node,
            MoveGroup,
            'move_action'
        )
