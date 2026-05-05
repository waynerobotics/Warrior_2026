"""
warrior_serial.swerve_driver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ROS 2 node: warrior_swerve_driver

One instance per physical swerve module (02_swerve / 03_swerve / 04_swerve).
Subscribes to ``/motor_cmd``, filters by ``target == device_name``, and
forwards matching commands as ``<MOT,target,spark,flipsky>`` over USB serial.

The swerve firmware has a 500 ms safety watchdog — if no matching MOT frame
arrives within 500 ms it returns motors to 1500 µs neutral.  This node
deliberately lets that watchdog fire when no commands are published (i.e. a
target is "disabled" upstream).  The keep-alive mechanism is NOT implemented
here; the upstream publisher (base_driver) is responsible for the ≥ 2 Hz rate.

State machine:
    DISCONNECTED -> DISCOVERING -> CONNECTED -> DISCONNECTED (on error)
"""

import glob

import rclpy
from rclpy.node import Node

import serial

from warrior_msgs.msg import MotorCommand
from warrior_serial.serial_protocol import (
    WarriorSerial,
    query_device_name,
    BAUD_RATE_DEFAULT,
)

VALID_SWERVES = {'02_swerve', '03_swerve', '04_swerve'}


def _candidate_ports() -> list:
    return sorted(
        glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    )


class SwerveDriverNode(Node):

    _DISCONNECTED = 'DISCONNECTED'
    _CONNECTED = 'CONNECTED'

    def __init__(self):
        super().__init__('warrior_swerve_driver')

        # Parameters
        self._device_name = self.declare_parameter(
            'device_name', '').value
        self._baud_rate = self.declare_parameter(
            'baud_rate', BAUD_RATE_DEFAULT).value
        self._discovery_retry_s = self.declare_parameter(
            'discovery_retry_period_s', 2.0).value
        self._read_timeout_s = self.declare_parameter(
            'read_timeout_s', 0.1).value

        if not self._device_name:
            raise ValueError(
                'warrior_swerve_driver requires the "device_name" parameter '
                '(e.g. "02_swerve")')

        if self._device_name not in VALID_SWERVES:
            self.get_logger().warn(
                f'device_name "{self._device_name}" is not a known swerve '
                f'({VALID_SWERVES}); continuing anyway.')

        self._ws: WarriorSerial | None = None
        self._state = self._DISCONNECTED
        self._next_discovery_time: float = 0.0  # monotonic seconds

        # Subscribe to motor commands — filter by target in callback
        self._sub = self.create_subscription(
            MotorCommand, '/motor_cmd', self._motor_cmd_cb, 10)

        # Timer drives discovery; also used to drain any device replies
        self._timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f'warrior_swerve_driver started; device_name="{self._device_name}"')

    # ------------------------------------------------------------------
    # State machine tick (~20 Hz)
    # ------------------------------------------------------------------

    def _tick(self):
        import time as _time
        if self._state == self._DISCONNECTED:
            if _time.monotonic() >= self._next_discovery_time:
                self._do_discover()

        elif self._state == self._CONNECTED:
            # Drain any unsolicited bytes (ACK, ERR) from the swerve to
            # prevent the RX buffer from filling up.
            self._drain_rx()

    # ------------------------------------------------------------------
    # Discovery
    # ------------------------------------------------------------------

    def _do_discover(self):
        self.get_logger().info(
            f'Scanning serial ports for "{self._device_name}"…')
        for port in _candidate_ports():
            name = query_device_name(port, self._baud_rate)
            if name == self._device_name:
                self.get_logger().info(
                    f'Found "{self._device_name}" on {port}')
                self._ws = WarriorSerial(
                    port, self._baud_rate, self._read_timeout_s)
                try:
                    self._ws.open()
                    self._state = self._CONNECTED
                    return
                except serial.SerialException as exc:
                    self.get_logger().error(
                        f'Failed to open {port}: {exc}')
                    self._ws = None

        import time as _time
        self._next_discovery_time = _time.monotonic() + self._discovery_retry_s
        self.get_logger().warn(
            f'"{self._device_name}" not found. '
            f'Retrying in ~{self._discovery_retry_s:.1f} s…')

    # ------------------------------------------------------------------
    # RX drain (keep TX buffer clear, log errors)
    # ------------------------------------------------------------------

    def _drain_rx(self):
        if self._ws is None:
            return
        try:
            # Non-blocking read; just discard
            self._ws.read_line()
        except serial.SerialException as exc:
            self.get_logger().error(
                f'Serial read error on "{self._device_name}": {exc}; '
                'reconnecting…')
            self._disconnect()

    # ------------------------------------------------------------------
    # Motor command subscriber callback
    # ------------------------------------------------------------------

    def _motor_cmd_cb(self, msg: MotorCommand):
        if msg.target != self._device_name:
            return  # not for us

        if self._state != self._CONNECTED:
            # Drop — the swerve watchdog will hold neutral until we reconnect
            return

        try:
            self._ws.write_message('MOT', msg.target, msg.spark, msg.flipsky)
        except serial.SerialException as exc:
            self.get_logger().error(
                f'Serial write error on "{self._device_name}": {exc}; '
                'reconnecting…')
            self._disconnect()

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
    node = SwerveDriverNode()
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
