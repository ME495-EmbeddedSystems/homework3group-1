from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
from sensor_msgs.msg import JointState
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion

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

    def get_curent_states(self):
        transform = self.tf_buffer.lookup_transform(self.tf_parent_frame,
                                                    self.tf_child_frame,
                                                    rclpy.time.Time())
        
        return Pose(position = Point(x=transform.translation.x,
                                     y=transform.translation.y,
                                     z=transform.translation.z),
                    orientation = Quaternion(x=transform.rotation.x,
                                             y=transform.rotation.y,
                                             z=transform.rotation.z,
                                             w=transform.rotation.w))
        