"""Convert joystick axes to stamped velocity commands for the swerve base."""

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoySwerve(Node):
    """Publish forward and yaw commands from a five-axis joystick."""

    def __init__(self):
        super().__init__('joy_swerve')

        self.declare_parameter('frame_id', 'base_footprint')
        self.frame_id = self.get_parameter('frame_id').value

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_vel_stamped', 10)
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info('joy_swerve node has started.')

    def joy_callback(self, msg: Joy):
        """Map left-Y and axis 4 to forward speed and yaw rate."""
        if len(msg.axes) <= 4:
            self.get_logger().warning(
                'Ignoring Joy message with fewer than five axes')
            return

        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = self.frame_id
        cmd_msg.twist.linear.x = msg.axes[1]
        cmd_msg.twist.angular.z = msg.axes[4]
        self.cmd_pub.publish(cmd_msg)


def main(args=None):
    """Run the joystick converter."""
    rclpy.init(args=args)
    joy_swerve = JoySwerve()
    rclpy.spin(joy_swerve)
    joy_swerve.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
