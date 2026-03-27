import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros

class DirectPathGenerator(Node):
    def __init__(self):
        super().__init__('direct_path_generator')
        self.get_logger().info('Direct Path Generator Node started')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value

        self.goal_pose = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_publisher = self.create_publisher(Path, 'dir_path', 10)
        self.goal_subscription = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, 10
        )

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.publish_direct_path()

    def get_robot_pose_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame=self.robot_base_frame,
                time=rclpy.time.Time()
            )

            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = tf.transform.translation.x
            ps.pose.position.y = tf.transform.translation.y
            ps.pose.position.z = tf.transform.translation.z
            ps.pose.orientation = tf.transform.rotation

            return ps

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}')
            return None

    def publish_direct_path(self):
        if self.goal_pose is None:
            return

        robot_pose = self.get_robot_pose_map()
        if robot_pose is None:
            return

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [robot_pose, self.goal_pose]

        self.path_publisher.publish(path)
        self.get_logger().info('Published direct path using TF pose')

def main():
    rclpy.init()
    node = DirectPathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
