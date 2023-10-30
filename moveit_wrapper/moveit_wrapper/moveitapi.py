from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    RobotState,
    MoveItErrorCodes,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    RobotTrajectory
)
from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import JointState
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from typing import Tuple, Optional


class MoveItApi():
    """
    Wraps the moveit ROS API for easy of use
    """

    def __init__(self, node: Node, base_frame: str, end_effector_frame: str):
        self.node = node
        self.move_group_action_client = ActionClient(
            self.node,
            MoveGroup,
            'move_action'
        )

        # Creating tf Listener
        self.joint_state = JointState()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.tf_parent_frame = base_frame
        self.tf_child_frame = end_effector_frame

    async def plan_path(self,
                        point: Point = None,
                        orientation: Quaternion = None,
                        start_pose: Pose = None,
                        execute: bool = False) -> RobotTrajectory:
        """
        Plans a path to a point and orientation

        Implements 1,2,3
        """
        pass

    # TODO: Stephen
    def execute_trajectory(trajectory: RobotTrajectory):
        """
        Executes the trajectory
        """
        pass

    # TODO: Kyle
    def get_joint_states(self, pose: Pose) -> JointState:
        """
        Calculates joint states for a given pose

        Arguments:
            - pose (geometry_msgs/Pose) - The pose of the end effector

        Returns
        -------
            The states for each joint to reach that pose
        """
        pass

    # TODO: jihai
    def joint_state_callback(self, joint_states: JointState):
        """
        Joint State subscriber callback
        """
        pass

    # TODO: jihai
    def current_state_to_robot_state(self) -> RobotState:
        """
        Constructs a robot state object from the internal joint states
        """
        pass

    # TODO: Anuj
    async def perform_IK_request(self, pose: Pose
                                 ) -> Tuple[RobotState, MoveItErrorCodes]:
        """
        Constructs an IK request and calls the service
        """
        pass

    # TODO: Stephen
    def create_goal_constraint(point: Point, orientation: Quaternion) -> Constraints:
        pass

    # TODO: Carter
    def create_position_constraint(point: Point) -> PositionConstraint:
        pass

    # TODO: Lyle
    def create_orientation_constraint(orientation: Quaternion) -> OrientationConstraint:
        pass
