"""
warrior_serial.twist_to_motor
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ROS 2 node: twist_to_motor

Subscribes to /cmd_vel (geometry_msgs/Twist) and publishes one
warrior_msgs/MotorCommand per swerve target on /motor_cmd.

Motor selection (RB / LB cycling) is handled by motor_manager, which
filters which device actually receives each command.

Mapping
-------
  Twist.linear.x   → spark   (-100..100)
  Twist.angular.z  → flipsky (-100..100)

ROS 2 parameters
----------------
  targets          (string list, default ["02_swerve","03_swerve","04_swerve"])
  scale_spark      (double, default 100.0)  multiplier for linear.x  → spark
  scale_flipsky    (double, default 100.0)  multiplier for angular.z → flipsky
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from warrior_msgs.msg import SwerveCmd


class TwistToMotorNode(Node):

    def __init__(self):
        super().__init__('twist_to_motor')

        self._targets = self.declare_parameter(
            'targets', ['02_swerve', '03_swerve', '04_swerve']).value
        self._scale_spark = self.declare_parameter(
            'scale_spark', 100.0).value
        self._scale_flipsky = self.declare_parameter(
            'scale_flipsky', 100.0).value

        self._pub = self.create_publisher(SwerveCmd, '/swerve_cmd', 10)
        self._sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        self.get_logger().info(f'twist_to_motor started; targets={self._targets}')

    def _cmd_vel_cb(self, msg: Twist) -> None:
        spark   = int(msg.linear.x  * self._scale_spark)
        flipsky = int(msg.angular.z * self._scale_flipsky)

        spark   = max(-100, min(100, spark))
        flipsky = max(-100, min(100, flipsky))

        for target in self._targets:
            cmd = SwerveCmd()
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

