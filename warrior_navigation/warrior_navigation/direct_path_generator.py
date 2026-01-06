import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros

class DirectPathGenerator(Node):
    def __init__(self):
        super().__init__('direct_path_generator')
        self.get_logger().info('Direct Path Generator Node started')

        self.robot_pose = None
        self.goal_pose = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.path_publisher = self.create_publisher(Path, 'dir_path', 10)
        self.goal_subscription = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        self.robot_pose_subscription = self.create_subscription(PoseStamped, 'robot_map_pose', self.robot_pose_callback, 10)

        self.timer = self.create_timer(0.1, self.publish_direct_path)

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg

    def robot_pose_callback(self, msg: PoseStamped):
        self.robot_pose = msg

    def publish_direct_path(self):
        if self.goal_pose is None:
            return
        try:
            path = Path()
            path.header.frame_id = 'map'
            path.header.stamp = self.get_clock().now().to_msg()
            path.poses = [self.robot_pose, self.goal_pose]

            self.get_logger().info('Publishing direct path from current position to goal' +
                                   f'({self.robot_pose.pose.position.x}, {self.robot_pose.pose.position.y}) to ' +
                                   f'({self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y})')
            self.path_publisher.publish(path)
        except Exception as e:
            self.get_logger().warn(f'Transform lookup failed: {e}')


def main():
    rclpy.init()
    node = DirectPathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()