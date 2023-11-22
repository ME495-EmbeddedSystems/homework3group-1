from geometry_msgs.msg import Pose, Point
import control_msgs.msg as control_msg
from moveit_wrapper.moveitapi import MoveItApi, ErrorCodes
from franka_msgs.action import Grasp
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from typing import List
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from typing import Awaitable, Optional


class GraspPlan:
    def __init__(self,
                 approach_pose: Pose,
                 grasp_pose: Pose,
                 grasp_goal: Grasp.Goal,
                 retreat_pose: Pose):
        self.approach_pose = approach_pose
        self.grasp_pose = grasp_pose
        self.grasp_goal = grasp_goal
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

    async def execute_grasp_plan(self, grasp_plan: GraspPlan) -> Awaitable[bool]:

        self.node.get_logger().warn("going to approach point!")
        self.node.get_logger().warn(
            f"grasp pose: {grasp_plan.approach_pose.orientation}")
        plan_result = await self.moveit_api.plan_async(
            point=grasp_plan.approach_pose.position,
            orientation=grasp_plan.approach_pose.orientation,
            execute=True
        )

        self.node.get_logger().warn(f"succeeded in going to approach point")

        self.node.get_logger().warn("going to grasp point!")
        self.node.get_logger().warn(
            f"grasp pose: {grasp_plan.grasp_pose.orientation}")
        plan_result = await self.moveit_api.plan_async(
            point=grasp_plan.grasp_pose.position,
            orientation=grasp_plan.grasp_pose.orientation,
            execute=True
        )

        # 3) grasp
        grasp_result = self.grasp(grasp_plan.grasp_goal)
        if grasp_result is None:
            self.node.get_logger().error("Grasp Action Goal rejected!")
            return False

        # 4) retreat
        plan_result = await self.moveit_api.plan_async(
            point=grasp_plan.retreat_pose.position,
            orientation=grasp_plan.retreat_pose.orientation,
            execute=True
        )

        return True

    async def grasp(self, grasp_action_goal: Grasp.Goal) -> Optional[Grasp.Result]:
        """
        Sends a GraspAction to the gripper.

        Arguments:
            + grasp_action_goal (franka_msgs/action/GraspAction.Goal): the grasp goal to send to the gripper

        Returns:
        -------
            Optional[Grasp.Result] if the goal was accepted and completed, None otherwise.
        """
        goal_handle: ClientGoalHandle = await self.grasp_client.send_goal_async(grasp_action_goal)
        if not goal_handle.accepted:
            return None
        result: Grasp.Result = await goal_handle.get_result_async()
        return result


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
