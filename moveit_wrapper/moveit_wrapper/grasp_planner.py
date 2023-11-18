from geometry_msgs.msg import Pose, Point
from control_msgs.action import GripperCommand
import control_msgs.msg as control_msg
from moveitapi import MoveItApi, ErrorCodes
from rclpy.action import ActionClient
from typing import List
import numpy as np


class GraspPlan:
    def __init__(self,
                 approach_pose: Pose,
                 grasp_pose: Pose,
                 gripper_command: control_msg.GripperCommand,
                 retreat_pose: Pose):
        self.approach_pose = approach_pose
        self.grasp_pose = grasp_pose
        self.gripper_command = gripper_command
        self.retreat_pose: retreat_pose


class GraspPlanner:
    """
    Plans and executes a grasp action
    """

    def __init__(self,
                 moveit_api: MoveItApi,
                 gripper_action_name: str):
        self.moveit_api = moveit_api
        self.node = self.moveit_api.node

        self.gripper_command_client = ActionClient(
            self.node,
            GripperCommand,
            gripper_action_name,
        )

    async def execute_grasp_plan(self, grasp: GraspPlan):

        plan_result = await self.moveit_api.plan_async(
            point=grasp.approach_pose.position,
            orientation=grasp.approach_pose.orientation
        )

        if plan_result.error_code != ErrorCodes.NO_ERROR:
            self.node.get_logger().warn(
                f"Failed grasp approach point {plan_result.error_code}")
            return

        interpolated_poses = linearly_interpolate_position(
            grasp.approach_pose, grasp.grasp_pose, 100)

        to_grasp_result = await self.moveit_api.create_cartesian_path(interpolated_poses)

        if to_grasp_result != ErrorCodes.NO_ERROR:
            self.node.get_logger().warn(
                f"Failed grasp approach point {plan_result.error_code}")
            return

        goal_handle = await self.gripper_command_client.send_goal_async(
            GripperCommand.Goal(command=grasp.gripper_command))
        grasp_command_result = await goal_handle.get_result_async()

        interpolated_poses = linearly_interpolate_position(
            grasp.grasp_pose, grasp.approach_pose, 100
        )

        to_retreat_result = await self.moveit_api.create_cartesian_path(interpolated_poses)


def linearly_interpolate_position(pose1: Pose, pose2: Pose, n: int) -> List[Pose]:
    x_space = np.linspace(pose1.position.x, pose2.position.x, n)
    y_space = np.linspace(pose1.position.y, pose2.position.y, n)
    z_space = np.linspace(pose1.position.z, pose2.position.z, n)

    poses = []
    for x, y, z in zip(x_space, y_space, z_space):
        poses.append(Pose(
            position=Point(
                x=x,
                y=y,
                z=z
            ),
            orientation=pose1.orientation
        ))

    return poses
