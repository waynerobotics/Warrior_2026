

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped



class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.get_logger().info('Pure Pursuit Controller Node started')

        self.goal_pose = None

        self.goal_subscription = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.get_logger().info(f'Received new goal pose: ({msg.pose.position.x}, {msg.pose.position.y})')