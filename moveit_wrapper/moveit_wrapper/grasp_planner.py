from geometry_msgs.msg import Pose
from moveit_wrapper.moveitapi import MoveItApi, ErrorCodes
from franka_msgs.action import Grasp
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from typing import Awaitable, Optional


class GraspPlan:
    """
    Holds all the information needed to plan and execute a grasp. Acts
    primarily as a struct of data, implements no logic.
    """

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
    A class that plans and executes grasps.

    A simple model of a grasp is a sequence of four actions. 1) Going to
    an approach pose that lines up the robot for going to the grasp pose.
    2) Going to the grasp pose, which is the pose that puts the end effector
    in a position to grasp. 3) Grasping the object. 4) Retreating from the object
    to a potentially different pose than the approach pose.
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
        """
        Plans and executes a grasp plan.

        Arguments:
            + grasp_plan (GraspPlan): the grasp plan to execute.

        Returns:
        -------
            A bool indicating whether the grasp plan was executed successfully.
        """

        # 1) Goto approach point
        plan_result = await self.moveit_api.plan_async(
            point=grasp_plan.approach_pose.position,
            orientation=grasp_plan.approach_pose.orientation,
            execute=True
        )
        if plan_result.error_code != ErrorCodes.SUCCESS:
            self.node.get_logger().error("Failed to plan to approach point!")
            return False

        # 2) Goto grasp point
        plan_result = await self.moveit_api.plan_async(
            point=grasp_plan.grasp_pose.position,
            orientation=grasp_plan.grasp_pose.orientation,
            execute=True
        )
        if plan_result.error_code != ErrorCodes.SUCCESS:
            self.node.get_logger().error("Failed to plan to grasp point!")
            return False

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
        if plan_result.error_code != ErrorCodes.SUCCESS:
            self.node.get_logger().error("Failed to plan retreat!")
            return False

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
