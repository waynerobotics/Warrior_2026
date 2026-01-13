

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
import math



class DirectPathController(Node):
    def __init__(self):
        super().__init__('direct_path_controller')
        self.get_logger().info('Direct Path Controller Node started')

        self.lookahead_point = None
        self.current_pose = None

        self.path_subscription = self.create_subscription(PoseStamped, 'goal_pose', self.lookahead_callback, 10)
        self.pose_subscription = self.create_subscription(PoseStamped, 'robot_map_pose', self.pose_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

    def lookahead_callback(self, msg: PoseStamped):
        self.lookahead_point = msg
    
    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def control_loop(self):
        if self.lookahead_point is None or self.current_pose is None:
            return
        
        dx = self.lookahead_point.pose.position.x - self.current_pose.pose.position.x
        dy = self.lookahead_point.pose.position.y - self.current_pose.pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        angle_to_lookahead = math.atan2(dy, dx)
        yaw = self.get_yaw_from_quaternion(self.current_pose.pose.orientation)
        angle_diff = self.normalize_angle(angle_to_lookahead - yaw)

        k_linear = 0.5
        k_angular = 1.0

        linear_velocity = k_linear * distance
        angular_velocity = k_angular * angle_diff

        cmd_msg = TwistStamped()
        cmd_msg.twist.linear.x = linear_velocity
        cmd_msg.twist.angular.z = angular_velocity

        self.get_logger().info(f'Publishing cmd_vel: linear={linear_velocity:.2f}, angular={angular_velocity:.2f}')
        self.cmd_vel_publisher.publish(cmd_msg)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
def main():
    rclpy.init()
    node = DirectPathController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()