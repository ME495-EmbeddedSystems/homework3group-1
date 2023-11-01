from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    RobotState,
    MoveItErrorCodes,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    RobotTrajectory,
    PositionIKRequest,
    BoundingVolume,
    MotionPlanRequest,
    PlanningOptions,
    WorkspaceParameters,
    PositionConstraint
)


from moveit_msgs.srv import GetPositionIK
from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from shape_msgs.msg import SolidPrimitive
from typing import Tuple, Optional
from enum import Enum


class ErrorCodes(Enum):
    NO_ERROR = 0,
    IK_FAIL = 1,
    GOAL_NOT_SPECIFIED = 2,


class PlanResult:
    def __init__(self,
                 error_code,
                 trajectory,
                 moveiterror=None
                 ):
        self.error_code = error_code
        self.trajectory = trajectory
        self.moveiterror = moveiterror


class MoveItApi():
    """
    Wraps the moveit ROS API for easy of use
    """

    def __init__(self, node: Node, base_frame: str, end_effector_frame: str, group_name: str, ik_link: str):
        """
        Arguments:
            + node (rclpy.node.Node) - the running node used to interface with ROS
            + base_frame (str) - the fixed body frame of the robot
            + end_effector_frame (str) - the frame of the end effector
            + group_name (str) - the name of the planning group to use
            + ik_link (str) - the name of the link
        """
        self.node = node

        # Create MoveGroup.action client
        self.move_group_action_client = ActionClient(
            self.node,
            MoveGroup,
            'move_action'
        )

        # Creating IK client
        self.cbgroup = MutuallyExclusiveCallbackGroup()
        self.ik_client = self.node.create_client(
            GetPositionIK, "compute_ik", callback_group=self.cbgroup)

        # Create ExecuteTrajectory.action client
        self.execute_trajectory_action_client = ActionClient(
            self.node,
            ExecuteTrajectory,
            'execute_trajectory'
        )

        self.node.get_logger().warn("trajectory client")

        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                'Timeout waiting for "compute_ik" service to become available')

        self.groupname = group_name
        self.ik_link_name = ik_link
        self.base_frame = base_frame

        # Creating tf Listener
        self.joint_state = JointState()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.end_effector_frame = end_effector_frame

        self.subscription_joint = self.node.create_subscription(
            JointState, "px100/joint_states", self.joint_state_callback, 10
        )

    async def plan_path(self,
                        max_velocity_scaling_factor=0.1,
                        max_acceleration_scaling_factor=0.1,
                        point: Point = None,
                        orientation: Quaternion = None,
                        start_pose: Pose = None,
                        execute: bool = False) -> PlanResult:
        """
        Plans a path to a point and orientation

        Implements 1,2,3
        """

        # define goal constraints
        goal_constraint = self.create_goal_constraint(point, orientation)

        motion_plan_request = MotionPlanRequest(
            workspace_parameters=WorkspaceParameters(
                header=Header(
                    stamp=self.node.get_clock().now().to_msg(),
                    frame_id=self.base_frame
                ),
                min_corner=Vector3(x=-1.0, y=-1.0, z=-1.0),
                max_corner=Vector3(x=1.0, y=1.0, z=1.0)
            ),
            goal_constraints=[goal_constraint],
            group_name=self.groupname,
            allowed_planning_time=10.0,
            num_planning_attempts=10,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor,
        )

        if start_pose is not None:
            # if a pose other than the current is defined, set robot state
            robot_state, error_code = await self.perform_IK_request(start_pose)

            if error_code != 1:
                return PlanResult(ErrorCodes.IK_FAIL, None, error_code)

            motion_plan_request.start_state = robot_state

        if point is None and orientation is None:
            # you have to specify at least one throw error
            return PlanResult(ErrorCodes.GOAL_NOT_SPECIFIED, None)

        goal = MoveGroup.Goal(
            request=motion_plan_request,
            planning_options=PlanningOptions(
                plan_only=(not execute)
            ),
        )

        result: MoveGroup.Result = await self.move_group_action_client.send_goal_async(goal)

        if execute:
            return PlanResult(ErrorCodes.NO_ERROR, result.executed_trajectory)
        else:
            return PlanResult(ErrorCodes.NO_ERROR, result.planned_trajectory)

    # TODO: Stephen
    def execute_trajectory(self, trajectory: RobotTrajectory):
        """
        Executes the trajectory
        """
        self.execute_trajectory_action_client.wait_for_server()

        return self.execute_trajectory_action_client.send_goal_async(
            ExecuteTrajectory.Goal(trajectory=trajectory))

    async def get_joint_states(self, pose: Pose) -> Optional[JointState]:
        """
        Calculates joint states for a given pose

        Arguments:
            - pose (geometry_msgs/Pose) - The pose of the end effector

        Returns
        -------
            The states for each joint to reach that pose
        """
        # Perform IK request on Pose to get the jointStates
        results = await self.perform_IK_request(pose)

        # Return of -31 means no IK solution
        if results[1].val == -31:
            self.node.get_logger.error(
                "No solution found for given start pose")
            return None
        else:
            return results[0].joint_state

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
        self.node.get_logger().warn(f"{robot_state.joint_state}")
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
        request.pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        request.pose_stamped.header.frame_id = self.base_frame
        request.pose_stamped.pose = pose
        request_IK = GetPositionIK.Request(ik_request=request)
        result = await self.ik_client.call_async(request_IK)
        return (result.solution, result.error_code)

    # TODO: Update constraint weights as a team

    def create_joint_constraint(self, point: Point, orientation: Quaternion) -> Constraints:

        pass

    # TODO: Stephen
    def create_goal_constraint(self, point: Point, orientation: Quaternion) -> Constraints:
        """
        Construct a moveit_msgs/Constraint for the end effector using a given quaternion and point

        Arguments:
            point (geometry_msgs/Point) -- position goal constraint of the end effector
            orientation (geometry_msgs/Quaternion) -- orienntation goal constraint of the end effector

        Returns:
            Constraints message type
        """

        position_constraints = []
        if point is not None:
            position_constraints = [self.create_position_constraint(point)]

        orConstraints = []
        if orientation is not None:
            orConstraints = [self.create_orientation_constraint(orientation)]

        return Constraints(position_constraints=position_constraints,
                           orientation_constraints=orConstraints,
                           )

    # TODO: Carter
    def create_position_constraint(self, point: Point) -> PositionConstraint:
        """
        Creates a position constraint for the end effector.

        Arguments:
            + point (geometry_msgs/Point) - The point to constrain the end effector to.

        Returns
        -------
            A PositionConstraint for the end effector.
        """

        # create sphere primitive representing goal pose region
        dimensions = [0]
        dimensions[SolidPrimitive.SPHERE_RADIUS] = 0.01
        primitive = SolidPrimitive(
            type=SolidPrimitive.SPHERE,
            dimensions=dimensions
        )

        # create bounding volume
        volume = BoundingVolume(
            primitives=[primitive],
            primitive_poses=[
                Pose(
                    position=point,
                    orientation=Quaternion(
                        x=0.0,
                        y=0.0,
                        z=0.0,
                        w=1.0
                    )
                ),
            ]
        )

        self.node.get_logger().warn(f"position link name {self.ik_link_name}")

        # create position constraint
        position_constraint = PositionConstraint(
            header=Header(
                frame_id="world",
                stamp=self.node.get_clock().now().to_msg()
            ),
            link_name=self.ik_link_name,
            constrait_region=volume,
            target_point_offset=Vector3(
                x=0.0,
                y=0.0,
                z=0.0
            ),
            weight=1.0
        )
        return position_constraint

    def create_orientation_constraint(self, orientation: Quaternion) -> OrientationConstraint:
        """
        Construct a moveit_msgs/OrientationConstraint for the end effector using a given quaternion

        Arguments:
            orientation (geometry_msgs/Quaternion) -- orienntation constraint of the end effector

        Returns:
            OrientationConstraint message type
        """
        header = Header(frame_id=self.base_frame,
                        stamp=self.node.get_clock().now().to_msg())
        link_name = self.ik_link_name

        return OrientationConstraint(header=header,
                                     link_name=link_name,
                                     orientation=orientation,
                                     weight=1.0,
                                     parameterization=OrientationConstraint.XYZ_EULER_ANGLES,
                                     absolute_x_axis_tolerance=1.0,
                                     absolute_y_axis_tolerance=1.0,
                                     absolute_z_axis_tolerance=1.0)
