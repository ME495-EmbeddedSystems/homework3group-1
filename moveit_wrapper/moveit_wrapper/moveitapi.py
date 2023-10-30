from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    RobotState,
    MoveItErrorCodes,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    RobotTrajectory,
    PositionIKRequest
)
from moveit_srvs.srv import GetPositionIK
from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from typing import Tuple, Optional


class MoveItApi():
    """
    Wraps the moveit ROS API for easy of use
    """

    def __init__(self, node: Node, base_frame: str, end_effector_frame: str, group_name: str, ik_link: str, frame_id: str):
        self.node = node
        self.move_group_action_client = ActionClient(
            self.node,
            MoveGroup,
            'move_action'
        )

        #Creating IK client
        self.node.cbgroup = MutuallyExclusiveCallbackGroup()
        self.ik_client = self.create_client(GetPositionIK, "compute_ik",self.node.cbgroup)
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Timeout waiting for "compute_ik" service to become available')
        
        self.groupname = group_name
        self.ik_link_name = ik_link
        self.frame_id = frame_id
        
        # Creating tf Listener
        self.joint_state = JointState()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.tf_parent_frame = base_frame
        self.end_effector_frame = end_effector_frame


        self.subscription_joint = self.create_subscription(
            JointState, "joint_states", self.joint_states_callback, 10
        )
        
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
        self.joint_state = joint_states

    # TODO: jihai
    def current_state_to_robot_state(self) -> RobotState:
        """
        Constructs a robot state object from the internal joint states
        """
        robot_state = RobotState()
        robot_state.joint_state = self.joint_state
        return robot_state

    # TODO: Anuj
    async def perform_IK_request(self, pose: Pose
                                 ) -> Tuple[RobotState, MoveItErrorCodes]:
        """
        Constructs an IK request and calls the service

        Arguments:
            - pose (geometry_msgs/Pose) - The pose of the end effector

        Returns
        -------
            RobotState of the start position and error_code of the GetPositionIK service
        """
        request = PositionIKRequest()
        request.group_name = self.groupname
        request.robot_state = self.current_state_to_robot_state()
        request.ik_link_name = self.ik_link_name
        request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.pose_stamped.header.frame_id = self.frame_id
        request.pose_stamped.pose = pose
        request_IK = GetPositionIK.request(ik_request=request)
        result = await self.ik_client.call_async(request_IK)
        return (result.solution,result.error_code)
    
    # TODO: Update constraint weights as a team

    # TODO: Stephen
    def create_goal_constraint(point: Point, orientation: Quaternion) -> Constraints:
        pass

    # TODO: Carter
    def create_position_constraint(point: Point) -> PositionConstraint:
        pass


    def create_orientation_constraint(self, orientation: Quaternion) -> OrientationConstraint:
        """
        Construct a moveit_msgs/OrientationConstraint for the end effector using a given quaternion

        Arguments:
            orientation (geometry_msgs/Quaternion) -- orienntation constraint of the end effector

        Returns:
            OrientationConstraint message type
        """
        header = Header(frame_id = self.end_effector_frame,
                        stamp = self.get_clock().now().to_msg())
        link_name = self.ik_link_name
        
        return OrientationConstraint(header = header,
                                     link_name = link_name,
                                     orientation = orientation,
                                     weight = 1.0)
