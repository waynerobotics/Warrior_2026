"""
warrior_serial.test_swerve_module
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Self-contained Phase 0 test: drives a single SPARK MAX from the right stick
Y axis of a gamepad. No motor manager, no Arduino, no helper imports.

Joy axes[3] (right stick Y) → rate input in [-1, +1]
    integrated at 50 Hz →  position in [0, 2π] rad  (hard clamped, no wrap)
    converted              →  motor rotations = (pos_rad / 2π) * 42
    sent every 20 ms over SLCAN as a Position-mode setpoint frame.

The SLCAN protocol details (frame ids, control-mode + enable broadcast
frames, status frame layout) were reverse-engineered with sniff_usb.py and
match what REV Hardware Client writes.

Run with:
    ros2 launch warrior_serial test_swerve_module.launch.py
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import serial.tools.list_ports


# SPARK MAX framing (lower 6 bits = device_id, set in REV Hardware Client).
# 0x02050100 = dev_type=0x02 | mfr=0x05 | api_class=0x00 | api_index=0x04
_SETPOINT_ID_BASE = 0x02050100

# Constant broadcast frames — "all SPARK MAXes use Position mode" + "robot enabled".
# Without ENABLE the controller leaves outputs at 0% no matter the setpoint.
_SET_MODE_FRAME = "T02052C80802" + "00" * 7 + "\r"
_ENABLE_FRAME = "T000502C0101\r"


def _find_sparkmax_port() -> str:
    """Pick the first ACM/REV port. Same heuristic as talk_can.py."""
    for p in serial.tools.list_ports.comports():
        if "ACM" in p.device or "REV" in (p.description or ""):
            return p.device
    return ""


class TestSwerveModuleNode(Node):

    def __init__(self):
        super().__init__('test_swerve_module')

        self._device_id: int = self.declare_parameter('device_id', 1).value
        self._rate_scale: float = self.declare_parameter(
            'rate_scale_rad_per_sec', 2.0).value
        self._encoder_rot_per_2pi: float = self.declare_parameter(
            'encoder_rot_per_2pi', 42.0).value
        self._joy_axis: int = self.declare_parameter('joy_axis', 3).value
        self._baud: int = self.declare_parameter('baud_rate', 115200).value
        # For the test, cap position at [0, 2π] (no wrap).
        self._max_pos: float = self.declare_parameter(
            'max_position_rad', 2.0 * math.pi).value
        self._min_pos: float = self.declare_parameter(
            'min_position_rad', 0.0).value

        self._setpoint_id = _SETPOINT_ID_BASE | (self._device_id & 0x3F)

        # State
        self._rate_input: float = 0.0          # [-1, +1] from joystick
        self._position_rad: float = 0.0        # [0, 2π]
        self._enabled: bool = False            # gates the heartbeat output
        self._state_lock = threading.Lock()

        # Live telemetry from reader_loop (Status 0/2 frames)
        self._reported_pos_rot: float = 0.0
        self._reported_out_pct: float = 0.0
        self._reported_faults: int = 0

        # Serial — open eagerly so we fail fast if no SPARK MAX is plugged in.
        port = _find_sparkmax_port()
        if not port:
            self.get_logger().error(
                'No SPARK MAX USB device found (looking for ACM*/REV*). '
                'Plug one in and relaunch.')
            raise RuntimeError('no SPARK MAX device')
        self._port = port
        self._ser = serial.Serial(port, self._baud, timeout=0.1)
        self.get_logger().info(
            f'Opened {port} for device_id={self._device_id} '
            f'(setpoint CAN ID = 0x{self._setpoint_id:08X})')

        # ROS plumbing
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self._integrate_timer = self.create_timer(0.02, self._integrate_tick)
        self._log_timer = self.create_timer(1.0, self._log_status)

        # Background heartbeat + reader
        self._running = True
        self._tx_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name='spark_tx')
        self._rx_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name='spark_rx')
        self._tx_thread.start()
        self._rx_thread.start()

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _joy_cb(self, msg: Joy) -> None:
        if self._joy_axis < len(msg.axes):
            val = float(msg.axes[self._joy_axis])
            # Clamp to [-1, +1] in case the joystick driver returns slightly
            # over-saturated values at extremes.
            val = max(-1.0, min(1.0, val))
            with self._state_lock:
                self._rate_input = val
                # Any joystick input enables the motor; the heartbeat will
                # keep ENABLE going as long as the node is alive.
                self._enabled = True

    def _integrate_tick(self) -> None:
        """50 Hz — integrate rate into position, hard clamp at the bounds."""
        dt = 0.02
        with self._state_lock:
            self._position_rad += self._rate_input * self._rate_scale * dt
            if self._position_rad > self._max_pos:
                self._position_rad = self._max_pos
            elif self._position_rad < self._min_pos:
                self._position_rad = self._min_pos

    def _log_status(self) -> None:
        with self._state_lock:
            rate_in = self._rate_input
            pos_rad = self._position_rad
        target_rot = (pos_rad / (2.0 * math.pi)) * self._encoder_rot_per_2pi
        self.get_logger().info(
            f'rate={rate_in:+.2f}  pos={pos_rad:5.2f} rad  '
            f'target={target_rot:6.2f} rot  '
            f'reported={self._reported_pos_rot:6.2f} rot  '
            f'out={self._reported_out_pct:+5.1f}%  '
            f'faults=0x{self._reported_faults:04X}')

    # ------------------------------------------------------------------
    # SLCAN heartbeat (background thread, 50 Hz)
    # ------------------------------------------------------------------

    def _heartbeat_loop(self) -> None:
        while self._running:
            with self._state_lock:
                enabled = self._enabled
                pos_rad = self._position_rad
            if enabled:
                motor_rot = (pos_rad / (2.0 * math.pi)) * self._encoder_rot_per_2pi
                try:
                    payload = struct.pack('<ff', float(motor_rot), 0.0)
                    setpoint_frame = (
                        f'T{self._setpoint_id:08X}8{payload.hex()}\r')
                    self._ser.write(
                        (setpoint_frame + _SET_MODE_FRAME + _ENABLE_FRAME).encode())
                except (serial.SerialException, OSError) as exc:
                    self.get_logger().warn(f'tx error: {exc}')
            time.sleep(0.02)

    # ------------------------------------------------------------------
    # SLCAN reader (background thread)
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        buf = bytearray()
        while self._running:
            try:
                chunk = self._ser.read(256)
            except (serial.SerialException, OSError):
                time.sleep(0.05)
                continue
            for b in chunk:
                if b in (0x0D, 0x0A):
                    if buf:
                        self._consume_slcan(buf.decode(errors='ignore'))
                        buf.clear()
                else:
                    buf.append(b)

    def _consume_slcan(self, line: str) -> None:
        """Parse Status 0 (applied %, faults) and Status 2 (position)."""
        if not line or line[0] not in ('t', 'T'):
            return
        id_len = 8 if line[0] == 'T' else 3
        try:
            can_id = int(line[1:1 + id_len], 16)
            dlc = int(line[1 + id_len:2 + id_len], 16)
            data = bytes.fromhex(line[2 + id_len:2 + id_len + 2 * dlc])
        except ValueError:
            return
        api_cls = (can_id >> 10) & 0x3F
        api_idx = (can_id >> 6) & 0x0F
        if api_cls != 0x2E:
            return
        if api_idx == 0 and len(data) >= 4:
            applied_raw, faults = struct.unpack_from('<hH', data)
            self._reported_out_pct = applied_raw / 32768.0 * 100.0
            self._reported_faults = faults
        elif api_idx == 2 and len(data) >= 8:
            self._reported_pos_rot, = struct.unpack_from('<f', data, 4)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        self._running = False
        if hasattr(self, '_tx_thread'):
            self._tx_thread.join(timeout=1.0)
        if hasattr(self, '_rx_thread'):
            self._rx_thread.join(timeout=1.0)
        if getattr(self, '_ser', None):
            try:
                self._ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TestSwerveModuleNode()
    except RuntimeError:
        rclpy.shutdown()
        return
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
