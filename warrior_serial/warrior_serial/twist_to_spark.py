"""
warrior_serial.twist_to_spark
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ROS 2 node: twist_to_spark

Rate-to-position steering integrator for SPARK MAX closed-loop position control.

The joystick angular.z (-1..1) is treated as a steering RATE.  A timer
integrates that rate into a running position accumulator which is published as
the SPARK MAX closed-loop position setpoint (rotations).

  Joystick centered  → rate = 0   → position holds where it is
  Joystick full left → position moves left at `rate_scale` rotations/sec
  Joystick full right→ position moves right at `rate_scale` rotations/sec

The SPARK MAX always receives an absolute position target — it stays in
closed-loop position mode.  The "rate" behaviour is implemented entirely in
this node.

ROS 2 parameters
----------------
  targets         (string list, default ["02_spark","03_spark","04_spark"])
  rate_scale      (double, default 2.0)   rotations/sec at full joystick deflection
  max_position    (double, default 5.0)   soft position clamp (±rotations)
  update_rate_hz  (double, default 20.0)  integration + publish frequency
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from warrior_msgs.msg import SparkCommand


class TwistToSparkNode(Node):

    def __init__(self):
        super().__init__('twist_to_spark')

        self._targets      = self.declare_parameter(
            'targets', ['02_spark', '03_spark', '04_spark']).value
        self._rate_scale   = self.declare_parameter(
            'rate_scale', 2.0).value        # rotations/sec at full deflection
        self._max_position = self.declare_parameter(
            'max_position', 5.0).value      # soft limit ±rotations
        update_hz          = self.declare_parameter(
            'update_rate_hz', 20.0).value

        self._dt       = 1.0 / update_hz
        self._position = 0.0  # accumulated position setpoint (rotations)
        self._rate     = 0.0  # latest joystick rate demand (rot/s, scaled)

        self._pub = self.create_publisher(SparkCommand, '/spark_cmd', 10)
        self._sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'twist_to_spark started — targets={self._targets}  '
            f'rate_scale={self._rate_scale} rot/s  '
            f'max_position=±{self._max_position} rot  '
            f'update_rate={update_hz} Hz')

    def _cmd_vel_cb(self, msg: Twist) -> None:
        # Convert joystick (-1..1) to a rate demand in rotations/sec
        self._rate = msg.angular.z * self._rate_scale

    def _tick(self) -> None:
        # Integrate rate → position
        self._position += self._rate * self._dt
        # Clamp to soft limits
        self._position = max(-self._max_position,
                             min(self._max_position, self._position))
        # Publish the same position target to all SPARK MAX targets
        for target in self._targets:
            cmd = SparkCommand()
            cmd.target   = target
            cmd.setpoint = float(self._position)
            self._pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToSparkNode()
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
