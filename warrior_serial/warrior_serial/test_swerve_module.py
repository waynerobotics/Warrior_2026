"""
warrior_serial.test_swerve_module
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Swerve test-rig coordinator. A single ROS 2 node owns all of the SPARK MAX
steering controllers on the bus and drives them from one shared commanded
position.

Model
-----
* One global ``cmd`` (motor rotations) starts at 0.
* Each wheel carries its own ``offset[i]``. The setpoint actually sent to
  wheel *i* is ``cmd + offset[i]``.
* Offsets are armed lazily: the first time wheel *i* reports a Status 2
  encoder reading, ``offset[i] = encoder_i - cmd`` so the very first
  setpoint equals the live encoder reading (no startup jump).

Modes (Xbox face buttons, exclusive; no toggle-off)
---------------------------------------------------
* **A** → ``ALL``: pushing the right-stick Y axis integrates ``cmd``, so
  every wheel moves in lockstep.
* **X** → ``ONLY:2``: pushing the stick integrates ``offset[2]`` only.
  The shared ``cmd`` is unchanged, so wheel 2's setpoint changes relative
  to the others. This is the calibration path. Pressing A reactivates the
  shared command with the new offset persisting.
* **B** → ``ONLY:3``, **Y** → ``ONLY:4`` (same idea per wheel).
* Initial mode is ``ALL``.

Coordinated stop (ALL mode only)
--------------------------------
In ALL mode, advancing ``cmd`` is gated by the per-wheel lag
``(cmd + offset[i]) - encoder[i]``. If pushing forward would drive
``max(lag) > +LAG_CAP_ROT`` the rate is zeroed; same for the reverse
direction at ``-LAG_CAP_ROT``. ONLY modes have no software cap — the
SPARK MAX's own soft/hard limits are the backstop during calibration.

The SLCAN protocol details (frame ids, mode-bitmask + enable broadcast
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

# Enable broadcast — without this the controller leaves outputs at 0% no
# matter the setpoint.
_ENABLE_FRAME = "T000502C0101\r"

# Xbox button index -> selection mode. Standard ROS `joy` Xbox mapping
# (A=0, B=1, X=2, Y=3). A is the "all wheels" mode; the others are
# exclusive single-wheel selectors. Pressing the same button twice is a
# no-op (there is no toggle-off).
_BUTTON_MODE_MAP = {
    0: 'ALL',        # A
    2: 'ONLY:2',     # X
    1: 'ONLY:3',     # B
    3: 'ONLY:4',     # Y
}

# In ALL mode, cap |cmd + offset[i] - encoder[i]| at this magnitude.
_LAG_CAP_ROT = 10.0

# How fresh a Status 2 frame must be (seconds) for that wheel to count
# toward the lag cap. Past this age we treat the encoder reading as stale
# and skip it.
_STATUS2_STALE_S = 0.5

# REV SPARK MAX USB CDC identification — confirmed via sniff_usb.py.
_SPARK_VID = 0x0483
_SPARK_PID = 0xA30E


def _make_mode_frame(device_id: int) -> str:
    """Build the SLCAN "follow setpoints" broadcast for `device_id`.

    Byte 0 is a bitmask of which device_ids should follow setpoints
    (bit N = device_id N), discovered 2026-05-17 by sniffing REV
    Hardware Client. The hard-coded `0x02` we used to ship only worked
    by accident when all controllers were at CAN ID 1.
    """
    return f"T02052C808{(1 << device_id):02X}" + "00" * 7 + "\r"


def _list_spark_ports():
    """All USB ports that look like a SPARK MAX. Filter by VID:PID first;
    fall back to the description string for older udev setups."""
    out = []
    for p in serial.tools.list_ports.comports():
        if p.vid == _SPARK_VID and p.pid == _SPARK_PID:
            out.append(p.device)
        elif 'SPARK MAX' in (p.description or ''):
            out.append(p.device)
    return out


def _scan_device_id(port: str, scan_seconds: float = 1.0):
    """Open *port*, listen passively, return the first CAN device_id we see
    streaming out of it. Returns None on timeout / open error.

    `exclusive=True` so a second opener fails cleanly (returns None)
    instead of racing the first reader."""
    try:
        ser = serial.Serial(port, 115200, timeout=0.05, exclusive=True)
    except Exception:
        return None
    line_buf = bytearray()
    end = time.monotonic() + scan_seconds
    try:
        while time.monotonic() < end:
            chunk = ser.read(256)
            for b in chunk:
                if b in (0x0D, 0x0A):
                    if line_buf:
                        line = line_buf.decode(errors='ignore')
                        line_buf.clear()
                        if line and line[0] in ('t', 'T'):
                            id_len = 8 if line[0] == 'T' else 3
                            try:
                                can_id = int(line[1:1 + id_len], 16)
                            except ValueError:
                                continue
                            return can_id & 0x3F
                else:
                    line_buf.append(b)
    finally:
        ser.close()
    return None


class SparkSession:
    """One SPARK MAX serial port + the tx/rx threads driving it.

    Pure transport — does no integration and knows nothing about ROS. The
    coordinator writes `setpoint_rot` each integrate tick; the tx thread
    reads it and emits the 50 Hz heartbeat. The rx thread parses Status 2
    frames into `encoder_rot` + `last_s2_monotonic`.
    """

    def __init__(self, port: str, device_id: int, baud: int = 115200):
        self.port = port
        self.device_id = device_id
        self._ser = serial.Serial(port, baud, timeout=0.1, exclusive=True)
        self._mode_frame = _make_mode_frame(device_id)
        self._setpoint_id = _SETPOINT_ID_BASE | (device_id & 0x3F)

        self._lock = threading.Lock()
        self._encoder_rot = None       # type: float | None
        self._setpoint_rot = None      # type: float | None  (None = idle, tx skips)
        self._last_s2_monotonic = 0.0
        self.tx_count = 0
        self.tx_error_logged = False

        self._running = True
        self._tx_thread = threading.Thread(
            target=self._tx_loop, daemon=True, name=f'spark{device_id}_tx')
        self._rx_thread = threading.Thread(
            target=self._rx_loop, daemon=True, name=f'spark{device_id}_rx')
        self._tx_thread.start()
        self._rx_thread.start()

    # ---- Accessors (used by coordinator) -----------------------------

    @property
    def encoder_rot(self):
        with self._lock:
            return self._encoder_rot

    @property
    def last_s2_monotonic(self) -> float:
        with self._lock:
            return self._last_s2_monotonic

    def set_setpoint(self, rot: float) -> None:
        with self._lock:
            self._setpoint_rot = float(rot)

    # ---- Background threads ------------------------------------------

    def _tx_loop(self):
        while self._running:
            with self._lock:
                sp = self._setpoint_rot
            if sp is None:
                time.sleep(0.02)
                continue
            try:
                payload = struct.pack('<ff', sp, 0.0)
                setpoint_frame = f'T{self._setpoint_id:08X}8{payload.hex()}\r'
                self._ser.write(
                    (setpoint_frame + self._mode_frame + _ENABLE_FRAME).encode())
                self.tx_count += 1
                self.tx_error_logged = False
            except (serial.SerialException, OSError) as exc:
                if not self.tx_error_logged:
                    self.tx_error_logged = True
                    # Print once per error burst; the coordinator's 1 Hz log
                    # will surface ongoing issues via tx_count freezing.
                    print(f'[spark{self.device_id}] tx error: {exc}')
                time.sleep(0.5)
                continue
            time.sleep(0.02)

    def _rx_loop(self):
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
                        self._consume(buf.decode(errors='ignore'))
                        buf.clear()
                else:
                    buf.append(b)

    def _consume(self, line: str) -> None:
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
        enc, = struct.unpack_from('<f', data, 4)
        with self._lock:
            self._encoder_rot = enc
            self._last_s2_monotonic = time.monotonic()

    def close(self):
        self._running = False
        self._tx_thread.join(timeout=1.0)
        self._rx_thread.join(timeout=1.0)
        try:
            self._ser.close()
        except Exception:
            pass


class SwerveCoordinatorNode(Node):

    def __init__(self):
        super().__init__('swerve_coordinator')

        self._target_ids = list(self.declare_parameter('wheels', [2, 3, 4]).value)
        self._rate_scale = float(self.declare_parameter(
            'rate_scale_rot_per_sec', 10.0).value)
        self._joy_axis = int(self.declare_parameter('joy_axis', 3).value)

        # Coordinator state
        self._state_lock = threading.Lock()
        self._cmd: float = 0.0
        self._offset: dict = {}              # device_id -> float
        self._mode: str = 'ALL'
        self._rate_input: float = 0.0
        self._prev_btn_state: dict = {}
        self._joy_seen: bool = False
        self._wheels_armed: set = set()
        self._unarmed_warned: bool = False
        self._init_monotonic = time.monotonic()

        # Discover ports and open one session per target wheel.
        self._sessions = self._discover_sessions()

        # ROS plumbing
        self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self.create_timer(0.02, self._integrate_tick)
        self.create_timer(1.0, self._log_status)

    # ------------------------------------------------------------------
    # Discovery
    # ------------------------------------------------------------------

    def _discover_sessions(self) -> dict:
        ports = _list_spark_ports()
        self.get_logger().info(
            f'Discovering SPARK MAX(es) for device_ids {self._target_ids} '
            f'on USB ports {ports}…')
        scanned = {}
        sessions: dict = {}
        for port in ports:
            dev = _scan_device_id(port)
            scanned[port] = dev
            if dev in self._target_ids and dev not in sessions:
                sessions[dev] = SparkSession(port, device_id=dev)
                self.get_logger().info(f'  opened device_id={dev} on {port}')
        missing = sorted(set(self._target_ids) - set(sessions))
        if missing:
            for s in sessions.values():
                s.close()
            self.get_logger().error(
                f'Missing SPARK MAX device_ids {missing}. Scanned: {scanned}')
            raise RuntimeError(f'missing sparks: {missing}')
        return sessions

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _joy_cb(self, msg: Joy) -> None:
        if not self._joy_seen:
            self._joy_seen = True
            self.get_logger().info(
                f'Joystick connected ({len(msg.axes)} axes, '
                f'{len(msg.buttons)} buttons). A=ALL, X=ONLY:2, B=ONLY:3, '
                f'Y=ONLY:4. Initial mode: {self._mode}.')

        # Rising-edge mode selection. Pressing the same button while
        # already in that mode is a no-op (no toggle-off).
        for btn_idx, mode_str in _BUTTON_MODE_MAP.items():
            pressed = 0 <= btn_idx < len(msg.buttons) and bool(msg.buttons[btn_idx])
            was_pressed = self._prev_btn_state.get(btn_idx, False)
            if pressed and not was_pressed and self._mode != mode_str:
                with self._state_lock:
                    self._mode = mode_str
                self.get_logger().info(f'btn{btn_idx} -> mode {mode_str}')
            self._prev_btn_state[btn_idx] = pressed

        if 0 <= self._joy_axis < len(msg.axes):
            val = float(msg.axes[self._joy_axis])
            val = max(-1.0, min(1.0, val))
        else:
            val = 0.0
        with self._state_lock:
            self._rate_input = val

    def _integrate_tick(self) -> None:
        dt = 0.02
        with self._state_lock:
            rate = self._rate_input * self._rate_scale
            mode = self._mode
            cmd = self._cmd

        # Arm wheels whose first Status 2 frame just arrived. cmd is
        # whatever value it is right now, so initial setpoint = encoder
        # regardless of when this happens (still "no jump").
        for dev, sess in self._sessions.items():
            enc = sess.encoder_rot
            if enc is None or dev in self._wheels_armed:
                continue
            self._offset[dev] = enc - cmd
            self._wheels_armed.add(dev)
            self.get_logger().info(
                f'Wheel {dev} armed at encoder {enc:+.2f} rot, '
                f'offset set to {self._offset[dev]:+.2f}.')

        # One-shot warning about wheels that never came up.
        if (not self._unarmed_warned
                and time.monotonic() - self._init_monotonic > 5.0):
            still_missing = sorted(set(self._target_ids) - self._wheels_armed)
            if still_missing:
                self.get_logger().warn(
                    f'After 5 s, wheels {still_missing} have not reported '
                    f'Status 2. Check Status 2 Period in REV Hardware Client.')
            self._unarmed_warned = True

        if not self._wheels_armed:
            return

        if mode == 'ALL':
            # Compute lags over wheels with fresh encoder data.
            now = time.monotonic()
            lags = []
            for dev in self._wheels_armed:
                sess = self._sessions[dev]
                if now - sess.last_s2_monotonic > _STATUS2_STALE_S:
                    continue
                enc = sess.encoder_rot
                if enc is None:
                    continue
                lags.append((cmd + self._offset[dev]) - enc)
            if lags:
                if rate > 0.0 and max(lags) >= _LAG_CAP_ROT:
                    rate = 0.0
                elif rate < 0.0 and min(lags) <= -_LAG_CAP_ROT:
                    rate = 0.0
            cmd = cmd + rate * dt
        elif mode.startswith('ONLY:'):
            try:
                dev = int(mode.split(':', 1)[1])
            except ValueError:
                dev = -1
            if dev in self._wheels_armed:
                self._offset[dev] = self._offset[dev] + rate * dt

        with self._state_lock:
            self._cmd = cmd

        # Push current commanded position to every armed wheel every tick.
        for dev in self._wheels_armed:
            self._sessions[dev].set_setpoint(cmd + self._offset[dev])

    def _log_status(self) -> None:
        if not self._joy_seen and not self._wheels_armed:
            return
        with self._state_lock:
            mode = self._mode
            cmd = self._cmd
            rate_in = self._rate_input
        now = time.monotonic()
        parts = [f'mode={mode}', f'cmd={cmd:+7.2f}', f'rate_in={rate_in:+.2f}']
        for dev in sorted(self._sessions):
            sess = self._sessions[dev]
            enc = sess.encoder_rot
            if dev not in self._wheels_armed or enc is None:
                parts.append(f'dev{dev}[unarmed]')
                continue
            offset = self._offset.get(dev, 0.0)
            age = now - sess.last_s2_monotonic
            lag = (cmd + offset) - enc
            parts.append(
                f'dev{dev}[enc={enc:+7.2f} off={offset:+7.2f} '
                f'lag={lag:+5.2f} age={age:.2f}s]')
        self.get_logger().info('  '.join(parts))

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self) -> None:
        for sess in getattr(self, '_sessions', {}).values():
            try:
                sess.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SwerveCoordinatorNode()
    except RuntimeError as exc:
        print(f'init failed: {exc}')
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
