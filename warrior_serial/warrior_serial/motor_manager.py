"""
warrior_serial.motor_manager
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ROS 2 node: motor_manager

Single node that owns all serial connections to the swerve Arduinos.
Replaces the three separate warrior_swerve_driver instances.

Responsibilities
----------------
* Discovery — periodically scans available USB ports in a background thread,
  probes each one with ``<WHO>`` (exclusive open), and keeps the connection
  open if the device name matches a wanted target.
* Routing — subscribes to ``/motor_cmd`` (warrior_msgs/MotorCommand) and
  writes ``<MOT,target,spark,flipsky>`` only to the device whose name matches
  ``msg.target``.
* Reads — polls every connected device on a 50 Hz timer and logs
  ``<ACK,…>`` / ``<ERR,…>`` replies.
* Reconnection — if a write or read raises SerialException the device is
  dropped; the discovery thread will pick it back up on the next scan.

ROS 2 parameters
----------------
  targets                  (string list, default ["02_swerve","03_swerve","04_swerve"])
  baud_rate                (int,    default 115200)
  discovery_retry_period_s (double, default 2.0)  — seconds between full scans
  read_timeout_s           (double, default 0.1)  — serial read timeout per device
"""

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports
from sensor_msgs.msg import Joy

from warrior_msgs.msg import MotorCommand
from .serial_protocol import (
    WarriorSerial, parse_message, BAUD_RATE_DEFAULT, OPEN_RESET_DELAY_S,
)

# How often to print a connection-status summary even when nothing changes.
_STATUS_INTERVAL_S = 5.0


class MotorManagerNode(Node):

    def __init__(self):
        super().__init__('motor_manager')

        self._targets: list = self.declare_parameter(
            'targets', ['02_swerve', '03_swerve', '04_swerve']).value
        self._baud_rate: int = self.declare_parameter(
            'baud_rate', BAUD_RATE_DEFAULT).value
        self._discovery_retry_s: float = self.declare_parameter(
            'discovery_retry_period_s', 2.0).value
        self._read_timeout_s: float = self.declare_parameter(
            'read_timeout_s', 0.1).value

        # Active-target cycling via RB (all) / LB (cycle one at a time)
        # buttons[5] = RB,  buttons[4] = LB  on Xbox controller
        # _active_idx = None  → broadcast to all targets
        # _active_idx = int   → only that index in self._targets receives commands
        self._rb_index: int = self.declare_parameter('rb_button_index', 5).value
        self._lb_index: int = self.declare_parameter('lb_button_index', 4).value
        self._active_idx = None  # start in "all" mode
        self._rb_prev: int = 0
        self._lb_prev: int = 0

        # device_name -> WarriorSerial (open, exclusive lock held)
        self._connections: dict = {}
        self._lock = threading.Lock()

        self._sub = self.create_subscription(
            MotorCommand, '/motor_cmd', self._motor_cmd_cb, 10)
        self._joy_sub = self.create_subscription(
            Joy, '/joy', self._joy_cb, 10)

        self._last_status_time: float = 0.0

        # 50 Hz read timer (non-blocking reads — only uses read_timeout_s)
        self._read_timer = self.create_timer(0.02, self._read_tick)

        # Discovery in a background thread so the 2 s Arduino reset wait
        # and the 3 s NAME-reply timeout never block the ROS executor.
        self._stop_event = threading.Event()
        self._discovery_thread = threading.Thread(
            target=self._discovery_loop, daemon=True, name='motor_manager_discovery')
        self._discovery_thread.start()

        self.get_logger().info(
            f'motor_manager started; managing targets={self._targets}  '
            f'active=ALL  '
            f'RB=buttons[{self._rb_index}] (all)  LB=buttons[{self._lb_index}] (cycle)')

    # ------------------------------------------------------------------
    # Joystick — active target cycling
    # ------------------------------------------------------------------

    def _active_label(self) -> str:
        return 'ALL' if self._active_idx is None else self._targets[self._active_idx]

    def _joy_cb(self, msg: Joy) -> None:
        """RB → select ALL targets.  LB (rising edge) → cycle one at a time."""
        if self._rb_index < len(msg.buttons):
            rb_now = msg.buttons[self._rb_index]
            if rb_now == 1 and self._rb_prev == 0:
                self._active_idx = None
                self.get_logger().info('[motor_manager] RB → active target: ALL')
            self._rb_prev = rb_now

        if self._lb_index < len(msg.buttons):
            lb_now = msg.buttons[self._lb_index]
            if lb_now == 1 and self._lb_prev == 0:
                if self._active_idx is None:
                    self._active_idx = 0
                else:
                    self._active_idx = (self._active_idx + 1) % len(self._targets)
                self.get_logger().info(
                    f'[motor_manager] LB → active target: "{self._targets[self._active_idx]}"')
            self._lb_prev = lb_now

    # ------------------------------------------------------------------
    # Discovery — background thread
    # ------------------------------------------------------------------

    def _discovery_loop(self) -> None:
        while not self._stop_event.is_set():
            missing = self._missing_targets()
            if missing:
                self.get_logger().info(
                    f'[discovery] Scanning for missing targets: {missing}')
                self._scan_ports(missing)
            with self._lock:
                connected = list(self._connections.keys())
            self.get_logger().info(
                f'[discovery] Connected: {connected}  |  Missing: {self._missing_targets()}')
            self._stop_event.wait(self._discovery_retry_s)

    def _missing_targets(self) -> list:
        with self._lock:
            return [t for t in self._targets if t not in self._connections]

    def _scan_ports(self, missing: list) -> None:
        """Probe each available (non-ttyS) port that is not already claimed."""
        all_ports = [
            p.device for p in serial.tools.list_ports.comports()
            if 'ttyS' not in p.device
        ]

        with self._lock:
            claimed_ports = {ws._port for ws in self._connections.values()}

        remaining = list(missing)  # local copy to track within this scan
        for port in all_ports:
            if not remaining:
                break
            if port in claimed_ports:
                continue
            if self._stop_event.is_set():
                return

            result = self._probe_port(port)
            if result is None:
                continue

            name, ws = result
            if name in remaining:
                with self._lock:
                    self._connections[name] = ws
                remaining.remove(name)
                claimed_ports.add(port)
                self.get_logger().info(
                    f'[motor_manager] Connected to {name} on {port}')
            else:
                self.get_logger().debug(
                    f'[motor_manager] Found {name!r} on {port} — not a managed target, skipping')
                ws.close()

    def _probe_port(self, port: str) -> Optional[tuple]:
        """
        Open *port* exclusively, send ``<WHO>``, wait for ``<NAME,…>``.

        Returns ``(name, open_WarriorSerial)`` on success, or ``None`` on
        timeout / busy port / serial error.  The caller owns the returned
        WarriorSerial and must close it if the name is not wanted.
        """
        # Use a short read timeout during probing so we can poll stop_event
        ws = WarriorSerial(port, self._baud_rate, read_timeout_s=0.2)
        try:
            ws.open()  # exclusive=True — raises SerialException if busy
        except serial.SerialException as exc:
            self.get_logger().debug(
                f'[motor_manager] {port} open failed: {exc}')
            return None

        try:
            ws.write_message('WHO')
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                if self._stop_event.is_set():
                    ws.close()
                    return None
                line = ws.read_line()
                if line is None:
                    continue
                fields = parse_message(line)
                if fields and fields[0] == 'NAME' and len(fields) >= 2:
                    # Switch to the configured read timeout for normal operation
                    ws._ser.timeout = self._read_timeout_s
                    return fields[1], ws
            # Timeout — no NAME received
            ws.close()
            return None
        except serial.SerialException as exc:
            self.get_logger().debug(
                f'[motor_manager] {port} probe error: {exc}')
            ws.close()
            return None

    # ------------------------------------------------------------------
    # Read tick — ROS timer (50 Hz)
    # ------------------------------------------------------------------

    def _read_tick(self) -> None:
        """Non-blocking read from every connected device.

        Checks ``in_waiting`` before calling ``readline()`` so this callback
        never blocks the ROS executor — if there is nothing in the OS RX
        buffer we return immediately without sleeping.
        """
        to_remove: list = []

        with self._lock:
            items = list(self._connections.items())

        for name, ws in items:
            try:
                if ws._ser is None or ws._ser.in_waiting == 0:
                    continue  # nothing in buffer — skip entirely, no blocking
                line = ws.read_line()
                if line is None:
                    continue
                fields = parse_message(line)
                if fields:
                    self._handle_incoming(name, fields)
            except serial.SerialException as exc:
                self.get_logger().warn(
                    f'[motor_manager] {name} read error: {exc} — will reconnect')
                to_remove.append(name)

        if to_remove:
            with self._lock:
                for name in to_remove:
                    ws = self._connections.pop(name, None)
                    if ws:
                        ws.close()

    def _handle_incoming(self, name: str, fields: list) -> None:
        msg_type = fields[0] if fields else ''
        if msg_type == 'ACK':
            self.get_logger().info(f'[rx] {name} → ACK {fields[1:]}')
        elif msg_type == 'ERR':
            self.get_logger().warn(f'[rx] {name} → ERR {fields[1:]}')
        else:
            self.get_logger().debug(f'[rx] {name} → {fields}')

    # ------------------------------------------------------------------
    # Motor command callback
    # ------------------------------------------------------------------

    def _motor_cmd_cb(self, msg: MotorCommand) -> None:
        # Determine which targets should receive this command
        if self._active_idx is None:
            active_targets = self._targets  # ALL mode
        else:
            active_targets = [self._targets[self._active_idx]]

        if msg.target not in active_targets:
            return

        with self._lock:
            ws = self._connections.get(msg.target)
        if ws is None:
            now = time.monotonic()
            if now - self._last_status_time >= _STATUS_INTERVAL_S:
                self._last_status_time = now
                with self._lock:
                    connected = list(self._connections.keys())
                self.get_logger().warn(
                    f'[tx] DROP cmd for "{msg.target}" — not connected yet. '
                    f'Connected: {connected}')
            return
        frame = f'<MOT,{msg.target},{msg.spark},{msg.flipsky}>'
        self.get_logger().info(f'[tx] → {frame}')
        try:
            ws.write_message('MOT', msg.target, str(msg.spark), str(msg.flipsky))
        except serial.SerialException as exc:
            self.get_logger().warn(
                f'[tx] {msg.target} write error: {exc} — will reconnect')
            with self._lock:
                dropped = self._connections.pop(msg.target, None)
            if dropped:
                dropped.close()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self._stop_event.set()
        self._discovery_thread.join(timeout=6.0)
        with self._lock:
            for ws in self._connections.values():
                ws.close()
            self._connections.clear()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorManagerNode()
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
