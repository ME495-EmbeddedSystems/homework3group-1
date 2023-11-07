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
    PositionConstraint,
    JointConstraint,
    PlanningScene,
    CollisionObject,
)

from moveit_msgs.srv import GetPositionIK
from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Point,
    Quaternion,
    Vector3,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from shape_msgs.msg import SolidPrimitive
from typing import Tuple, Optional, List
from enum import Enum
from rclpy.task import Future


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

    def __init__(self,
                 node: Node,
                 base_frame: str,
                 end_effector_frame: str,
                 group_name: str,
                 joint_state_topic: str):
        """
        Arguments:
            + node (rclpy.node.Node) - the running node used to interface with ROS
            + base_frame (str) - the fixed body frame of the robot
            + end_effector_frame (str) - the frame of the end effector
            + group_name (str) - the name of the planning group to use
            + joint_state_topic (str) - the topic to subscribe to for joint states
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

        if not self.ik_client.wait_for_service(timeout_sec=20.0):
            raise RuntimeError(
                'Timeout waiting for "compute_ik" service to become available')

        self.groupname = group_name
        self.base_frame = base_frame

        # Creating tf Listener
        self.joint_state = JointState()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.end_effector_frame = end_effector_frame

        self.subscription_joint = self.node.create_subscription(
            JointState, joint_state_topic, self.joint_state_callback, 10
        )

        # planning scene interface
        self.planning_scene_publisher = self.node.create_publisher(
            PlanningScene, "planning_scene", 10)

    def plan(self,
             max_velocity_scaling_factor=0.1,
             max_acceleration_scaling_factor=0.1,
             point: Point = None,
             orientation: Quaternion = None,
             start_pose: Pose = None,
             execute: bool = False,
             use_jc: bool = True):

        self.fut = Future()

        rclpy.get_global_executor().create_task(self.plan_async(
            max_velocity_scaling_factor,
            max_acceleration_scaling_factor,
            point,
            orientation,
            start_pose,
            execute,
            use_jc
        )).add_done_callback(self.done)

        return self.fut
    
    def plan_joint(self,
             joint_name: List[str],
             joint_values: List[float],
             max_velocity_scaling_factor=0.1,
             max_acceleration_scaling_factor=0.1,
             execute: bool = False):

        self.fut = Future()

        rclpy.get_global_executor().create_task(self.plan_joint_async(
            joint_name,
            joint_values,
            max_velocity_scaling_factor,
            max_acceleration_scaling_factor,
            execute
        )).add_done_callback(self.done)

        return self.fut

    def done(self, plan_result):
        self.fut.set_result(plan_result)

    async def plan_async(self,
                         max_velocity_scaling_factor=0.1,
                         max_acceleration_scaling_factor=0.1,
                         point: Point = None,
                         orientation: Quaternion = None,
                         start_pose: Pose = None,
                         execute: bool = False,
                         use_jc: bool = True) -> PlanResult:
        """
        Plans a path to a point and orientation

        Implements 1,2,3
        """

        # define goal constraints
        goal_constraint = await self.create_goal_constraint(
            point, orientation, use_jc=use_jc)

        motion_plan_request = MotionPlanRequest(
            workspace_parameters=WorkspaceParameters(
                header=Header(
                    stamp=self.node.get_clock().now().to_msg(),
                    frame_id=self.base_frame
                ),
                min_corner=Vector3(x=-1.0, y=-1.0, z=0.0),
                max_corner=Vector3(x=1.0, y=1.0, z=1.0)
            ),
            planner_id="move_group",
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

        goal_handle = await self.move_group_action_client.send_goal_async(goal)
        result_response = await goal_handle.get_result_async()
        result = result_response.result
        if execute:
            return PlanResult(ErrorCodes.NO_ERROR, result.executed_trajectory)
        else:
            return PlanResult(ErrorCodes.NO_ERROR, result.planned_trajectory)
        
    async def plan_joint_async(self,
                         joint_name: List[str],
                         joint_values: List[float],
                         max_velocity_scaling_factor=0.1,
                         max_acceleration_scaling_factor=0.1,
                         execute: bool = False) -> PlanResult:
        """
        Plans a path to a point and orientation

        Implements 1,2,3
        """
        joint_constraints = []
        for i in range(0, len(joint_name)):
            joint_constraint = JointConstraint(
                joint_name=joint_name[i],
                position=joint_values[i],
                tolerance_above=0.01,
                tolerance_below=0.01
            )
            joint_constraints.append(joint_constraint)
        # define goal constraints
        goal_constraint = Constraints(
            joint_constraint=joint_constraints
        )
        self.node.get_logger().warn(
            f"Creating Joint Constraint {joint_constraints}")
        motion_plan_request = MotionPlanRequest(
            workspace_parameters=WorkspaceParameters(
                header=Header(
                    stamp=self.node.get_clock().now().to_msg(),
                    frame_id=self.base_frame
                ),
                min_corner=Vector3(x=-1.0, y=-1.0, z=0.0),
                max_corner=Vector3(x=1.0, y=1.0, z=1.0)
            ),
            planner_id="move_group",
            goal_constraints=[goal_constraint],
            group_name=self.groupname,
            allowed_planning_time=10.0,
            num_planning_attempts=10,
            max_velocity_scaling_factor=max_velocity_scaling_factor,
            max_acceleration_scaling_factor=max_acceleration_scaling_factor,
        )

        goal = MoveGroup.Goal(
            request=motion_plan_request,
            planning_options=PlanningOptions(
                plan_only=(not execute)
            ),
        )

        goal_handle = await self.move_group_action_client.send_goal_async(goal)
        result_response = await goal_handle.get_result_async()
        result = result_response.result
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
            self.node.get_logger().error(
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
        self.node.get_logger().warn(
            f"Getting Current Robot Joints {robot_state.joint_state}")
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
        request.ik_link_name = self.end_effector_frame
        request.pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        request.pose_stamped.header.frame_id = self.base_frame
        request.pose_stamped.pose = pose
        request_IK = GetPositionIK.Request(ik_request=request)
        result = await self.ik_client.call_async(request_IK)
        return (result.solution, result.error_code)

    async def create_joint_constraint(self, point: Point, orientation: Quaternion) -> Constraints:

        # IK for goal position to get robot state
        pose = Pose()
        current_pose = await self.get_end_effector_pose()

        if point is None:
            pose.position = current_pose.pose.position
        else:
            pose.position = point

        if orientation is None:
            pose.orientation = current_pose.pose.orientation
        else:
            pose.orientation = orientation

        joint_state = await self.get_joint_states(pose)

        # convert robot state to joint constraints
        joint_constraints = []
        for i in range(0, len(joint_state.name)):

            joint_constraint = JointConstraint(
                joint_name=joint_state.name[i],
                position=joint_state.position[i],
                tolerance_above=0.01,
                tolerance_below=0.01
            )
            joint_constraints.append(joint_constraint)

        self.node.get_logger().warn(
            f"Creating Joint Constraint {joint_constraints}")
        return joint_constraints

    # TODO: Stephen
    async def create_goal_constraint(self, point: Point, orientation: Quaternion, use_jc: bool = True) -> Constraints:
        """
        Construct a moveit_msgs/Constraint for the end effector using a given quaternion and point

        Arguments:
            point (geometry_msgs/Point) -- position goal constraint of the end effector
            orientation (geometry_msgs/Quaternion) -- orienntation goal constraint of the end effector
            use_jc (bool) -- use joint constraints instead of position and orientation constraints
        Returns:
            Constraints message type
        """

        if use_jc:
            return Constraints(
                joint_constraints=await self.create_joint_constraint(
                    point, orientation)
            )
        else:
            position_constraints = []
            if point is not None:
                position_constraints = [self.create_position_constraint(point)]

            orConstraints = []
            if orientation is not None:
                orConstraints = [
                    self.create_orientation_constraint(orientation)]

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
        dimensions = [0.01]
        primitive = SolidPrimitive(
            type=SolidPrimitive.SPHERE,
            dimensions=dimensions
        )

        pose = Pose(position=point)

        # create bounding volume
        volume = BoundingVolume(
            primitives=[primitive],
            primitive_poses=[pose]
        )

        self.node.get_logger().warn(
            f"position link name {self.end_effector_frame}")

        # create position constraint
        position_constraint = PositionConstraint(
            header=Header(
                frame_id=self.base_frame,
                stamp=self.node.get_clock().now().to_msg()
            ),
            target_point_offset=Vector3(),
            link_name=self.end_effector_frame,
            constrait_region=volume,
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
        link_name = self.end_effector_frame

        return OrientationConstraint(header=header,
                                     link_name=link_name,
                                     orientation=orientation,
                                     weight=1.0,
                                     absolute_x_axis_tolerance=0.01,
                                     absolute_y_axis_tolerance=0.01,
                                     absolute_z_axis_tolerance=0.01)

    async def get_end_effector_pose(self) -> PoseStamped:
        """ Gets the current end effector pose

        Returns:
        -------
        A Pose describing the end effector
        """
        ee_tf = await self.tf_buffer.lookup_transform_async(
            self.base_frame, self.end_effector_frame, Time(seconds=0))

        return PoseStamped(
            header=Header(
                frame_id=ee_tf.header.frame_id,
                stamp=ee_tf.header.stamp
            ),
            pose=Pose(
                position=Point(
                    x=ee_tf.transform.translation.x,
                    y=ee_tf.transform.translation.y,
                    z=ee_tf.transform.translation.z,
                ),
                orientation=ee_tf.transform.rotation
            )
        )

    def spawn_box(self, pose: Pose, size: Vector3, name: str):
        """Spawns a box in the planning scene

        Arguments:
            + pose (geometry_ns/Pose) - The pose of the box
            + size (geometry_ns/Vector3) - The size of the box
            + name (str) - The name of the box, used as the id of the collision
                object in the scene

        Returns:
        -------
            None
        """

        # create primitive box
        primitive = SolidPrimitive(
            type=SolidPrimitive.BOX,
            dimensions=[size.x, size.y, size.z]
        )

        # create collision object
        collision_object = CollisionObject(
            header=Header(
                frame_id=self.base_frame,
                stamp=self.node.get_clock().now().to_msg()
            ),
            id=name,
            primitive_poses=[pose],
            primitives=[primitive],
            operation=CollisionObject.ADD
        )

        # add the collision object to the planning scene
        # make it a difference to just add the box and not change the rest of the scene
        planning_scene = PlanningScene(
            is_diff=True,
        )
        planning_scene.world.collision_objects.append(collision_object)

        self.planning_scene_publisher.publish(planning_scene)
