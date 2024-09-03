from rclpy.action import ActionClient
from rclpy.node import Node

from isaac_moveit_msgs.action import MoveToPose
from geometry_msgs.msg import PoseStamped


class ROSInterface(Node):

    def __init__(self):
        super().__init__("isaac_ros_interface")
        self._action_client = ActionClient(self, MoveToPose, "/execute_trajectory_new")

    def send_goal(self, goal_position):
        self._action_client.wait_for_server()

        goal_msg = MoveToPose.Goal()
        goal_msg.goal = PoseStamped()
        goal_msg.goal.header.frame_id = "panda_link0"
        goal_msg.goal.pose.position.x = float(goal_position[0])
        goal_msg.goal.pose.position.y = float(goal_position[1])
        goal_msg.goal.pose.position.z = float(goal_position[2]) + (0.15 / 2.0)
        goal_msg.goal.pose.orientation.x = 0.707
        goal_msg.goal.pose.orientation.y = 0.0
        goal_msg.goal.pose.orientation.z = 0.707
        goal_msg.goal.pose.orientation.w = 0.0

        return self._action_client.send_goal_async(goal_msg)
