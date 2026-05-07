"""
warrior_serial.twist_to_motor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ROS 2 node: twist_to_motor

Subscribes to /cmd_vel (geometry_msgs/Twist) and publishes one
warrior_msgs/MotorCommand for the currently-selected swerve target.

RB (right bumper, buttons[5]) cycles the active target:
  02_swerve → 03_swerve → 04_swerve → 02_swerve → …

Mapping
-------
  Twist.linear.x   → spark   (-100..100)
  Twist.angular.z  → flipsky (-100..100)

ROS 2 parameters
----------------
  targets          (string list, default ["02_swerve","03_swerve","04_swerve"])
  scale_spark      (double, default 100.0)  multiplier for linear.x  → spark
  scale_flipsky    (double, default 100.0)  multiplier for angular.z → flipsky
  rb_button_index  (int,    default 5)      Joy button index for RB
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from warrior_msgs.msg import MotorCommand


class TwistToMotorNode(Node):

    def __init__(self):
        super().__init__('twist_to_motor')

        self._targets = self.declare_parameter(
            'targets', ['02_swerve', '03_swerve', '04_swerve']).value
        self._scale_spark = self.declare_parameter(
            'scale_spark', 100.0).value
        self._scale_flipsky = self.declare_parameter(
            'scale_flipsky', 100.0).value
        self._rb_index = self.declare_parameter(
            'rb_button_index', 5).value

        # Which target is currently active (index into self._targets)
        self._active_idx: int = 0
        self._rb_prev: int = 0  # previous RB state for rising-edge detection

        self._pub = self.create_publisher(MotorCommand, '/motor_cmd', 10)
        self._sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._joy_sub = self.create_subscription(
            Joy, '/joy', self._joy_cb, 10)

        self.get_logger().info(
            f'twist_to_motor started; targets={self._targets}  '
            f'active="{self._targets[self._active_idx]}"  '
            f'RB=buttons[{self._rb_index}]')

    # ------------------------------------------------------------------

    def _joy_cb(self, msg: Joy) -> None:
        """Detect RB rising edge and advance to the next target."""
        if self._rb_index >= len(msg.buttons):
            return
        rb_now = msg.buttons[self._rb_index]
        if rb_now == 1 and self._rb_prev == 0:
            self._active_idx = (self._active_idx + 1) % len(self._targets)
            self.get_logger().info(
                f'[twist_to_motor] RB pressed → active target: '
                f'"{self._targets[self._active_idx]}"')
        self._rb_prev = rb_now

    def _cmd_vel_cb(self, msg: Twist) -> None:
        spark   = int(msg.linear.x  * self._scale_spark)
        flipsky = int(msg.angular.z * self._scale_flipsky)

        spark   = max(-100, min(100, spark))
        flipsky = max(-100, min(100, flipsky))

        target = self._targets[self._active_idx]
        cmd = MotorCommand()
        cmd.target  = target
        cmd.spark   = spark
        cmd.flipsky = flipsky
        self._pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToMotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

