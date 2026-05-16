"""
warrior_serial.test_swerve_module
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Self-contained Phase 0 test: drives a single SPARK MAX from the right stick
Y axis of a gamepad. No motor manager, no Arduino, no helper imports.

Behavior
--------
* We do not transmit anything until the SPARK MAX has reported a position
  back to us in a Status 2 frame. That first reading becomes our initial
  commanded position (no startup jump).
* Joystick axes[3] (right stick Y) in [-1, +1] is treated as a rate of
  change of the commanded position, in motor rotations per second
  (`rate_scale_rot_per_sec` parameter, default 10).
* The commanded position is hard-capped to a window of 43 motor rotations
  starting at the captured encoder reading — i.e. one full swerve wheel
  revolution from boot.
* The Spark's encoder is incremental; "0" is wherever it was when the
  controller booted. Anchoring on the first reported position is what keeps
  the initial setpoint = current shaft position = no jump.

The SLCAN protocol details (frame ids, control-mode + enable broadcast
frames, status frame layout) were reverse-engineered with sniff_usb.py and
match what REV Hardware Client writes.

Run with:
    ros2 launch warrior_serial test_swerve_module.launch.py
"""

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

# One full wheel revolution = 43 motor rotations (gear ratio).
_ROT_PER_WHEEL_REV = 43.0


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
        self._rate_scale_rot_per_sec: float = self.declare_parameter(
            'rate_scale_rot_per_sec', 10.0).value
        self._joy_axis: int = self.declare_parameter('joy_axis', 3).value
        self._baud: int = self.declare_parameter('baud_rate', 115200).value
        self._window_rot: float = self.declare_parameter(
            'window_rot', _ROT_PER_WHEEL_REV).value

        self._setpoint_id = _SETPOINT_ID_BASE | (self._device_id & 0x3F)

        # State
        self._rate_input: float = 0.0          # [-1, +1] from joystick
        self._state_lock = threading.Lock()
        self._joy_seen: bool = False
        self._tx_error_logged: bool = False

        # Position state, all in absolute motor rotations.
        # `_target_motor_rot` is None until the first Status 2 frame arrives,
        # at which point we initialize it to the reported encoder reading.
        # Heartbeat sends nothing while it is None.
        self._target_motor_rot = None       # type: float | None
        self._target_min: float = 0.0
        self._target_max: float = 0.0
        self._latest_encoder_rot: float = 0.0
        self._encoder_seen: bool = False

        # Open the SPARK MAX port. Fail fast if nothing is plugged in.
        port = _find_sparkmax_port()
        if not port:
            self.get_logger().error(
                'No SPARK MAX USB device found. Plug one in and relaunch.')
            raise RuntimeError('no SPARK MAX device')
        self._port = port
        self._ser = serial.Serial(port, self._baud, timeout=0.1)
        self.get_logger().info(f'SPARK MAX port: {port} (device_id={self._device_id})')

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
        if not self._joy_seen:
            self._joy_seen = True
            self.get_logger().info(
                f'Joystick connected ({len(msg.axes)} axes, {len(msg.buttons)} buttons)')
        if self._joy_axis < len(msg.axes):
            val = float(msg.axes[self._joy_axis])
            val = max(-1.0, min(1.0, val))
            with self._state_lock:
                self._rate_input = val

    def _integrate_tick(self) -> None:
        """50 Hz — adjust the commanded target by the joystick rate, hard
        clamped to the window captured at startup."""
        dt = 0.02
        with self._state_lock:
            if self._target_motor_rot is None:
                return
            new_target = (
                self._target_motor_rot
                + self._rate_input * self._rate_scale_rot_per_sec * dt)
            if new_target > self._target_max:
                new_target = self._target_max
            elif new_target < self._target_min:
                new_target = self._target_min
            self._target_motor_rot = new_target

    def _log_status(self) -> None:
        if not self._joy_seen:
            return
        with self._state_lock:
            target = self._target_motor_rot
            rate_in = self._rate_input
            encoder = self._latest_encoder_rot
        if target is None:
            self.get_logger().info(
                f'rate={rate_in:+.2f}  waiting for first encoder reading from Spark…')
            return
        self.get_logger().info(
            f'rate={rate_in:+.2f}  cmd={target:7.2f} rot  '
            f'(encoder={encoder:7.2f} rot)')

    # ------------------------------------------------------------------
    # SLCAN heartbeat (background thread, 50 Hz)
    # ------------------------------------------------------------------

    def _heartbeat_loop(self) -> None:
        while self._running:
            with self._state_lock:
                target = self._target_motor_rot
            if target is None:
                # No encoder reading yet — do nothing. The Spark streams Status
                # frames on its own, so we just wait for one to arrive.
                time.sleep(0.02)
                continue
            try:
                payload = struct.pack('<ff', float(target), 0.0)
                setpoint_frame = (
                    f'T{self._setpoint_id:08X}8{payload.hex()}\r')
                self._ser.write(
                    (setpoint_frame + _SET_MODE_FRAME + _ENABLE_FRAME).encode())
                if self._tx_error_logged:
                    self.get_logger().info('tx recovered')
                    self._tx_error_logged = False
            except (serial.SerialException, OSError) as exc:
                if not self._tx_error_logged:
                    self.get_logger().warn(
                        f'tx error: {exc} (suppressing further until recovery)')
                    self._tx_error_logged = True
                time.sleep(0.5)
                continue
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
        """Parse Status 2 (position) frames. Status 0 (applied %, faults) is
        ignored on the happy path; we don't surface it."""
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
        if api_cls != 0x2E or api_idx != 2 or len(data) < 8:
            return
        encoder_rot, = struct.unpack_from('<f', data, 4)
        with self._state_lock:
            self._latest_encoder_rot = encoder_rot
            if not self._encoder_seen:
                self._encoder_seen = True
                self._target_motor_rot = encoder_rot
                self._target_min = encoder_rot
                self._target_max = encoder_rot + self._window_rot
                first = True
            else:
                first = False
        if first:
            self.get_logger().info(
                f'Encoder reports {encoder_rot:.2f} rot — using as starting '
                f'commanded position. Window: '
                f'[{self._target_min:.2f}, {self._target_max:.2f}]')

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
