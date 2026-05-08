import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from tf2_ros import TransformListener, Buffer
from tf2_ros import TransformException

import math



# Implement a pure pursuit to take a path and output velocity to cmd_vel to follow it

class FollowPath(Node):
    def __init__(self):
        super().__init__('follow_path')
        self.get_logger().info('Follow Path node started')

        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value

        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '~/tracking_status', 10)
        self.path_sub = self.create_subscription(Path, 'a_star_path', self.path_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.path = None
        self.robot_pose = None

        self.lookahead_distance = 0.1
        self.max_linear_vel = 0.2
        self.max_angular_vel = 2.0
        self.goal_tolerance = 0.1
        self.tf_timeout = Duration(seconds=0.1)
        self._last_status = None

        self.timer = self.create_timer(0.05, self.control_loop)

    
    def path_callback(self, msg):
        self.path = msg
        if len(msg.poses) == 0:
            self.publish_stop()
            self.publish_status('idle')
        else:
            self.publish_status('following')
    
    def yaw_from_quaternion(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )

    def get_robot_pose_in_frame(self, frame_id):
        try:
            if not self.tf_buffer.can_transform(
                target_frame=frame_id,
                source_frame=self.robot_base_frame,
                time=Time(),
                timeout=self.tf_timeout
            ):
                self.get_logger().debug(
                    f'TF not ready yet between {frame_id} and {self.robot_base_frame}'
                )
                return None

            tf = self.tf_buffer.lookup_transform(
                target_frame=frame_id,
                source_frame=self.robot_base_frame,
                time=Time()
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = self.yaw_from_quaternion(tf.transform.rotation)

            return x, y, yaw

        except TransformException as ex:
            self.get_logger().warn(f'TF lookup failed: {ex}')
            return None

    def control_loop(self):
        if self.path is None or len(self.path.poses) == 0:
            return

        path_frame = self.path.header.frame_id if self.path.header.frame_id else 'map'
        robot_pose = self.get_robot_pose_in_frame(path_frame)
        if robot_pose is None:
            return

        # --- Current robot pose 
        rx, ry, yaw = robot_pose
        # --- Goal check ---
        # self.get_logger().info(f"Current Path: ({len(self.path.poses)} poses)")
        goal_pose = self.path.poses[-1].pose.position
        goal_dist = math.hypot(goal_pose.x - rx, goal_pose.y - ry)

        if goal_dist < self.goal_tolerance:
            self.publish_stop()
            self.publish_status('goal_reached')
            return

        # --- Lookahead point ---
        target = self.find_lookahead_point(rx, ry)
        if target is None:
            self.publish_stop()
            self.publish_status('idle')
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
        twist.header.frame_id = self.robot_base_frame
        twist.twist.linear.x = linear_vel
        twist.twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)
        self.publish_status('following')


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

    def publish_status(self, status: str):
        if status == self._last_status:
            return

        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self._last_status = status

def main(args=None):
    rclpy.init(args=args)
    node = FollowPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
