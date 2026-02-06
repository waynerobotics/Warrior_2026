import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException

import math



# Implement a pure pursuit to take a path and output velocity to cmd_vel to follow it

class ComplexPathController(Node):
    def __init__(self):
        super().__init__('complex_path_controller')
        self.get_logger().info('Complex Path Controller node started')

        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, 'a_star_path', self.path_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.path = None
        self.robot_pose = None

        self.lookahead_distance = 0.1
        self.max_linear_vel = 0.2
        self.max_angular_vel = 2.0
        self.goal_tolerance = 0.1

        self.timer = self.create_timer(0.05, self.control_loop)

    def get_robot_pose_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame='odom',
                source_frame='base_link',
                time=rclpy.time.Time()
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            return x, y

        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}')
            return None

    
    def path_callback(self, msg):
        self.path = msg
    
    def yaw_from_quaternion(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )

    def get_robot_pose_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame='odom',
                source_frame='base_link',
                time=rclpy.time.Time()
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = self.yaw_from_quaternion(tf.transform.rotation)

            return x, y, yaw

        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}')
            return None

    def control_loop(self):
        robot_pose = self.get_robot_pose_map()
        if self.path is None or robot_pose is None or len(self.path.poses) == 0:
            return
        # --- Current robot pose 
        rx, ry, yaw = robot_pose
        # --- Goal check ---
        # self.get_logger().info(f"Current Path: ({len(self.path.poses)} poses)")
        goal_pose = self.path.poses[-1].pose.position
        goal_dist = math.hypot(goal_pose.x - rx, goal_pose.y - ry)

        if goal_dist < self.goal_tolerance:
            self.publish_stop()
            return

        # --- Lookahead point ---
        target = self.find_lookahead_point(rx, ry)
        if target is None:
            self.publish_stop()
            return

        tx, ty = target

        # --- Transform target to robot frame ---
        dx = tx - rx
        dy = ty - ry

        x_r =  math.cos(-yaw) * dx - math.sin(-yaw) * dy
        y_r =  math.sin(-yaw) * dx + math.cos(-yaw) * dy

        # --- Pure pursuit curvature ---
        L = self.lookahead_distance
        if L < 1e-3:
            return
        curvature = (2.0 * y_r) / (L * L)

        # --- Velocity commands ---
        linear_vel = self.max_linear_vel
        angular_vel = curvature * linear_vel

        # Clamp angular velocity
        angular_vel = max(
            -self.max_angular_vel,
            min(self.max_angular_vel, angular_vel)
        )

        # --- Publish cmd_vel ---
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"
        twist.twist.linear.x = linear_vel
        twist.twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)


    def find_lookahead_point(self, robot_x, robot_y):
        """
        Returns (x, y) of the lookahead point in MAP frame.
        Implements proper pure pursuit:
        1. Find closest point on the path
        2. Walk forward until lookahead distance is met
        """

        if self.path is None or len(self.path.poses) == 0:
            return None

        # --- Step 1: find closest path index ---
        closest_idx = None
        min_dist = float('inf')

        for i, pose in enumerate(self.path.poses):
            px = pose.pose.position.x
            py = pose.pose.position.y
            d = math.hypot(px - robot_x, py - robot_y)

            if d < min_dist:
                min_dist = d
                closest_idx = i

        if closest_idx is None:
            return None

        # --- Step 2: search forward for lookahead ---
        for i in range(closest_idx, len(self.path.poses)):
            px = self.path.poses[i].pose.position.x
            py = self.path.poses[i].pose.position.y

            d = math.hypot(px - robot_x, py - robot_y)

            if d >= self.lookahead_distance:
                return (px, py)

        # --- Step 3: fallback to goal ---
        last_pose = self.path.poses[-1].pose.position
        return (last_pose.x, last_pose.y)

    def publish_stop(self):
        twist = TwistStamped()
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        # self.get_logger().info('Published stop command to cmd_vel.')

def main(args=None):
    rclpy.init(args=args)
    node = ComplexPathController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()