"""
warrior_serial.base_driver
~~~~~~~~~~~~~~~~~~~~~~~~~~
ROS 2 node: warrior_base_driver

Connects to the ``00_base`` Arduino (LCD Keypad + RadioLink SBUS).
Reads ``<MOT,swerve_id,spark,flipsky>`` frames from the device and publishes
them as :class:`warrior_msgs.msg.SwerveCmd` on ``/swerve_cmd``.

State machine:
    DISCONNECTED -> DISCOVERING -> CONNECTED -> DISCONNECTED (on error)
"""

import glob

import rclpy
from rclpy.node import Node

import serial

from warrior_msgs.msg import SwerveCmd
from warrior_serial.serial_protocol import (
    WarriorSerial,
    parse_message,
    query_device_name,
    BAUD_RATE_DEFAULT,
)

EXPECTED_DEVICE = '00_base'


def _candidate_ports() -> list:
    """Return a sorted list of candidate serial port paths."""
    return sorted(
        glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    )


class BaseDriverNode(Node):

    # State constants
    _DISCONNECTED = 'DISCONNECTED'
    _CONNECTED = 'CONNECTED'

    def __init__(self):
        super().__init__('warrior_base_driver')

        # Parameters
        self._device_name = self.declare_parameter('device_name', EXPECTED_DEVICE).value
        self._baud_rate = self.declare_parameter('baud_rate', BAUD_RATE_DEFAULT).value
        self._discovery_retry_s = self.declare_parameter('discovery_retry_period_s', 2.0).value
        self._read_timeout_s = self.declare_parameter('read_timeout_s', 0.1).value

        self._pub = self.create_publisher(SwerveCmd, '/swerve_cmd', 10)

        self._ws: WarriorSerial | None = None
        self._state = self._DISCONNECTED
        self._next_discovery_time: float = 0.0  # monotonic seconds

        # Single timer drives the state machine
        self._timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f'warrior_base_driver started; looking for "{self._device_name}"')

    # ------------------------------------------------------------------
    # State machine tick (~20 Hz)
    # ------------------------------------------------------------------

    def _tick(self):
        import time as _time
        if self._state == self._DISCONNECTED:
            if _time.monotonic() >= self._next_discovery_time:
                self._do_discover()

        elif self._state == self._CONNECTED:
            self._do_read()

    # ------------------------------------------------------------------
    # Discovery
    # ------------------------------------------------------------------

    def _do_discover(self):
        self.get_logger().info('Scanning serial ports for "%s"…' %
                               self._device_name)
        for port in _candidate_ports():
            name = query_device_name(port, self._baud_rate)
            if name == self._device_name:
                self.get_logger().info(f'Found "{self._device_name}" on {port}')
                self._ws = WarriorSerial(
                    port, self._baud_rate, self._read_timeout_s)
                try:
                    self._ws.open()
                    self._state = self._CONNECTED
                    return
                except serial.SerialException as exc:
                    self.get_logger().error(f'Failed to open {port}: {exc}')
                    self._ws = None

        import time as _time
        self._next_discovery_time = _time.monotonic() + self._discovery_retry_s
        self.get_logger().warn(
            f'"{self._device_name}" not found. '
            f'Retrying in {self._discovery_retry_s:.1f} s…')

    # ------------------------------------------------------------------
    # Connected read loop (called every timer tick)
    # ------------------------------------------------------------------

    def _do_read(self):
        try:
            line = self._ws.read_line()
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial read error: {exc}; reconnecting…')
            self._disconnect()
            return

        if line is None:
            return

        fields = parse_message(line)
        if fields is None or len(fields) < 4 or fields[0] != 'MOT':
            return  # skip non-MOT frames (ACK, ERR, etc.)

        target, spark_s, flipsky_s = fields[1], fields[2], fields[3]
        try:
            spark = int(spark_s)
            flipsky = int(flipsky_s)
        except ValueError:
            self.get_logger().warn(f'Bad MOT values: {line}')
            return

        msg = SwerveCmd()
        msg.swerve_id = target
        msg.spark = spark
        msg.flipsky = flipsky
        self._pub.publish(msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _disconnect(self):
        if self._ws:
            self._ws.close()
            self._ws = None
        self._state = self._DISCONNECTED

    def destroy_node(self):
        self._disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriverNode()
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
