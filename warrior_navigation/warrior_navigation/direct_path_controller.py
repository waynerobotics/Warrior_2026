import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import tf2_ros
from tf2_ros import TransformException
import math

class DirectPathController(Node):
    def __init__(self):
        super().__init__('direct_path_controller')
        self.get_logger().info('Direct Path Controller Node started')

        self.lookahead_point = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_subscription = self.create_subscription(
            PoseStamped, 'goal_pose', self.lookahead_callback, 10
        )

        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped, 'cmd_vel', 10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

    def lookahead_callback(self, msg: PoseStamped):
        self.lookahead_point = msg

    def get_robot_pose_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='base_link',
                time=rclpy.time.Time()
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = self.get_yaw_from_quaternion(tf.transform.rotation)

            return x, y, yaw

        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}')
            return None

    def control_loop(self):
        if self.lookahead_point is None:
            return

        state = self.get_robot_pose_map()
        if state is None:
            return

        robot_x, robot_y, yaw = state
        target_x = self.lookahead_point.pose.position.x
        target_y = self.lookahead_point.pose.position.y

        dx = target_x - robot_x
        dy = target_y - robot_y
        distance = math.sqrt(dx**2 + dy**2)

        angle_to_target = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_target - yaw)

        k_linear = 0.5
        k_angular = 1.2

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = k_linear * distance
        cmd.twist.angular.z = k_angular * angle_error

        self.cmd_vel_publisher.publish(cmd)

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
