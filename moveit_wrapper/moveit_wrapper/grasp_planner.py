from geometry_msgs.msg import Pose, Point
from control_msgs.action import GripperCommand
import control_msgs.msg as control_msg
from moveit_wrapper.moveitapi import MoveItApi, ErrorCodes
from franka_msgs.action import Grasp
from rclpy.action import ActionClient
from typing import List
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import TransformListener, Buffer, TransformBroadcaster


class GraspPlan:
    def __init__(self,
                 approach_pose: Pose,
                 grasp_pose: Pose,
                 grasp_command: Grasp.Goal,
                 retreat_pose: Pose):
        self.approach_pose = approach_pose
        self.grasp_pose = grasp_pose
        self.grasp_command = grasp_command
        self.retreat_pose = retreat_pose


class GraspPlanner:
    """
    Plans and executes a grasp action
    """

    def __init__(self,
                 moveit_api: MoveItApi,
                 gripper_action_name: str):
        self.moveit_api = moveit_api
        self.node = self.moveit_api.node

        self.grasp_client = ActionClient(
            self.node,
            Grasp,
            gripper_action_name,
            callback_group=ReentrantCallbackGroup(),
        )

        # Create tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_parent_frame = "panda_link0"


    async def execute_grasp_plan(self, grasp_plan: GraspPlan):

        self.node.get_logger().warn("going to approach point!")
        curr_poseStamped = await self.moveit_api.get_end_effector_pose()
        curr_pose = curr_poseStamped.pose

        waypoints = linearly_interpolate_position(curr_pose, grasp_plan.approach_pose)
        self.node.get_logger().warn(
            f"grasp pose: {grasp_plan.approach_pose.orientation}")
        await self.moveit_api.create_cartesian_path(waypoints)
        # plan_result = await self.moveit_api.plan_async(
        #     point=grasp_plan.approach_pose.position,
        #     orientation=grasp_plan.approach_pose.orientation,
        #     execute=True
        # )

        self.node.get_logger().warn(f"succeeded in going to approach point")

        self.node.get_logger().warn("going to grasp point!")
        self.node.get_logger().warn(
            f"grasp pose: {grasp_plan.grasp_pose.orientation}")
        plan_result = await self.moveit_api.plan_async(
            point=grasp_plan.grasp_pose.position,
            orientation=grasp_plan.grasp_pose.orientation,
            execute=True
        )

        self.node.get_logger().warn("grasping...")
        goal_handle = await self.grasp_client.send_goal_async(grasp_plan.grasp_command)
        grasp_command_result = await goal_handle.get_result_async()
        self.node.get_logger().warn("finished grasp")

        plan_result = await self.moveit_api.plan_async(
            point=grasp_plan.retreat_pose.position,
            orientation=grasp_plan.retreat_pose.orientation,
            execute=True
        )

        actual_grasp_position = self.buffer.lookup_transform(
            "panda_link0", "panda_hand_tcp", Time())

        return actual_grasp_position


def linearly_interpolate_position(pose1: Pose, pose2: Pose, n: int) -> List[Pose]:
    x_space = np.linspace(pose1.position.x, pose2.position.x, n)
    y_space = np.linspace(pose1.position.y, pose2.position.y, n)
    z_space = np.linspace(pose1.position.z, pose2.position.z, n)

    poses = [pose1]
    for x, y, z in zip(x_space, y_space, z_space):
        poses.append(Pose(
            position=Point(
                x=x,
                y=y,
                z=z
            ),
            orientation=pose1.orientation
        ))
    poses.append(pose2)

    # overshoot for pose 2
    xdiff = (pose1.position.x - pose2.position.x)/10
    ydiff = (pose1.position.y - pose2.position.y)/10
    zdiff = (pose1.position.z - pose2.position.z)/10

    overshoot = Pose(
        position=Point(
            x=pose2.position.x - xdiff,
            y=pose2.position.y - ydiff,
            z=pose2.position.z - zdiff
        ),
        orientation=pose2.orientation
    )

    return poses
