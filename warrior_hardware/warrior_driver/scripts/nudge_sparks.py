"""
warrior_serial.nudge_sparks
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Standalone PoC: nudge every SPARK MAX on the USB bus by a small fixed
delta (in motor rotations) and exit. Now with diagnostic output so we can
see what the controller is actually doing while we command it.

Protocol details lifted from test_swerve_module.py / talk_can.py.

Run:  python3 -m warrior_serial.nudge_sparks
"""

import struct
import threading
import time

import serial
import serial.tools.list_ports


_SPARK_VID = 0x0483
_SPARK_PID = 0xA30E

_SETPOINT_ID_BASE = 0x02050100
_ENABLE_FRAME = "T000502C0101\r"

# SLCAN channel-open sequence. A cold-booted SPARK MAX brings its USB-SLCAN
# bridge up with the CAN channel CLOSED — every T... frame we write is
# silently dropped and the controller never streams Status 0/2 to anyone.
# REV Hardware Client opens it on connect; captured 2026-05-29 with
# sniff_usb.py, REV sends exactly "S8\r" (bitrate 1 Mbit/s) then "O\r"
# (open) before any traffic, and status starts ~20 ms later. SLCAN keeps the
# channel open until a "C\r" or power-cycle, which is why a single REV connect
# used to unstick the whole session. Send it ourselves at every session open.
_OPEN_FRAME = "S8\rO\r"

# Telemetry-enable. Opening the channel gets Status 0 streaming, but a cold
# controller still won't broadcast Status 2-6 (encoder position lives in
# Status 2) until something enables them — which is why position-discovery
# used to need a REV "connect". Sniffing REV (2026-05-29) showed Status 2-6
# all turn on ~20 ms after a single frame: CAN ID (0x02050400 | device_id),
# dlc 4, payload 7c 00 ff ff. The device_id is in the low 6 bits, so it must
# be built per controller (verified on dev 2 -> ...402 and dev 3 -> ...403).
# Confirmed REV-free with probe_status2.py.
_TELEMETRY_ID_BASE = 0x02050400


def _make_enable_telemetry_frame(device_id: int) -> str:
    """SLCAN frame that makes one SPARK MAX start broadcasting Status 2-6."""
    can_id = _TELEMETRY_ID_BASE | (device_id & 0x3F)
    return f"T{can_id:08X}47C00FFFF\r"


def _make_mode_frame(device_ids) -> str:
    """Build the broadcast-mode SLCAN frame for the given device_ids.

    Byte 0 of the broadcast (CAN ID 0x02052C80) is a *bitmask* of which
    device_ids should be enabled — bit N enables device_id N — not an
    enumerated control-mode value. This was discovered 2026-05-17 by
    sniffing REV Hardware Client: it sends 0x04 when controlling dev 2,
    0x08 for dev 3, 0x10 for dev 4. Our old constant 0x02 (bit 1) only
    worked when controllers were at CAN ID 1.
    """
    bitmask = 0
    for d in device_ids:
        bitmask |= 1 << d
    return f"T02052C808{bitmask:02X}" + "00" * 7 + "\r"

NUDGE_ROT = 5.0
HOLD_SECONDS = 5.0


def list_spark_ports():
    out = []
    for p in serial.tools.list_ports.comports():
        if p.vid == _SPARK_VID and p.pid == _SPARK_PID:
            out.append(p.device)
        elif 'SPARK MAX' in (p.description or ''):
            out.append(p.device)
    return out


class SparkSession:
    """Open one SPARK MAX port and run a tx/rx pair of threads for it,
    mirroring talk_can.py's working structure. Exposes the latest decoded
    status so we can poll it from the main thread."""

    def __init__(self, port: str):
        self.port = port
        self.ser = serial.Serial(port, 115200, timeout=0.05, exclusive=True)
        # Open the SLCAN channel before anything else, or a cold controller
        # silently drops every frame we send (see _OPEN_FRAME).
        self.ser.write(_OPEN_FRAME.encode())
        self.device_id: int | None = None
        self.cur_pos: float | None = None
        self.cur_out_pct: float = 0.0
        self.cur_faults: int = 0
        self.target_pos: float | None = None
        self.is_enabled = False
        self.running = True
        self.tx_count = 0
        self.status_0_count = 0
        self.status_2_count = 0
        self.other_frame_count = 0
        # One-shot SLCAN frames to splice into the tx burst from another
        # thread (used by probe_status2.py to replay captured REV frames
        # without interleaving bytes mid-frame). list.pop(0) is GIL-atomic.
        self._inject: list[str] = []
        self.tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.tx_thread.start()
        self.rx_thread.start()

    def _tx_loop(self):
        # SPARK MAXes time out (~50 ms) and go silent if they don't see an
        # enable heartbeat. We can't wait for device_id discovery first or
        # the controller never streams Status 0/2 for us to read. So from
        # session open we blast enable + a broadcast mode-bitmask (all 8
        # device_ids enabled — bits get ignored on devices that aren't
        # present); once we have a setpoint we add it to the burst.
        broadcast_mode = _make_mode_frame(range(8))
        while self.running:
            try:
                if (self.is_enabled and self.target_pos is not None
                        and self.device_id is not None):
                    setpoint_id = _SETPOINT_ID_BASE | (self.device_id & 0x3F)
                    payload = struct.pack('<ff', float(self.target_pos), 0.0)
                    setpoint_frame = f'T{setpoint_id:08X}8{payload.hex()}\r'
                    mode_frame = _make_mode_frame([self.device_id])
                    burst = setpoint_frame + mode_frame + _ENABLE_FRAME
                else:
                    # Heartbeat only — keeps the controller streaming status.
                    burst = broadcast_mode + _ENABLE_FRAME
                # Once we know the device_id, keep asking it to broadcast
                # Status 2-6 until position actually shows up; stop after, so
                # we don't keep poking the register on a working controller.
                if self.device_id is not None and self.status_2_count == 0:
                    burst += _make_enable_telemetry_frame(self.device_id)
                self.ser.write(burst.encode())
                # Splice in any one-shot frames queued from another thread,
                # whole-frame at a time so we never corrupt a \r-terminated
                # SLCAN line.
                while self._inject:
                    self.ser.write(self._inject.pop(0).encode())
                self.tx_count += 1
            except Exception as exc:
                print(f'[{self.port}] tx error: {exc}')
            time.sleep(0.02)

    def _rx_loop(self):
        buf = bytearray()
        while self.running:
            try:
                chunk = self.ser.read(256)
            except Exception:
                time.sleep(0.05)
                continue
            for b in chunk:
                if b in (0x0D, 0x0A):
                    if buf:
                        self._consume(buf.decode(errors='ignore'))
                        buf.clear()
                else:
                    buf.append(b)

    def _consume(self, line: str):
        if not line or line[0] not in ('t', 'T'):
            return
        id_len = 8 if line[0] == 'T' else 3
        try:
            can_id = int(line[1:1 + id_len], 16)
            dlc = int(line[1 + id_len:2 + id_len], 16)
            data = bytes.fromhex(line[2 + id_len:2 + id_len + 2 * dlc])
        except ValueError:
            return
        if self.device_id is None:
            self.device_id = can_id & 0x3F
        api_cls = (can_id >> 10) & 0x3F
        api_idx = (can_id >> 6) & 0x0F
        if api_cls != 0x2E:
            self.other_frame_count += 1
            return
        if api_idx == 0 and len(data) >= 4:
            self.status_0_count += 1
            applied_raw, faults = struct.unpack_from('<hH', data)
            self.cur_out_pct = applied_raw / 32768.0 * 100.0
            self.cur_faults = faults
        elif api_idx == 2 and len(data) >= 8:
            self.status_2_count += 1
            self.cur_pos, = struct.unpack_from('<f', data, 4)
        else:
            self.other_frame_count += 1

    def inject(self, frame: str):
        """Queue a raw SLCAN frame (e.g. 'T0205040247c00ffff\\r') to be sent
        once, spliced cleanly between heartbeat bursts by the tx thread."""
        self._inject.append(frame)

    def close(self):
        self.running = False
        self.tx_thread.join(timeout=1.0)
        self.rx_thread.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass


def nudge_one(port: str) -> bool:
    print(f'[{port}] opening…')
    try:
        sess = SparkSession(port)
    except Exception as exc:
        print(f'[{port}] open failed: {exc}')
        return False
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0:
            if sess.device_id is not None and sess.cur_pos is not None:
                break
            time.sleep(0.05)
        if sess.device_id is None or sess.cur_pos is None:
            print(f'[{port}] discovery timeout — device_id={sess.device_id}, '
                  f'pos={sess.cur_pos}, S0_frames={sess.status_0_count}, '
                  f'S2_frames={sess.status_2_count}, '
                  f'other={sess.other_frame_count}')
            return False

        start_pos = sess.cur_pos
        target = start_pos + NUDGE_ROT
        sess.target_pos = target
        sess.is_enabled = True
        print(f'[{port}] device_id={sess.device_id}  start_pos={start_pos:+.3f}  '
              f'-> target={target:+.3f} rot (delta={NUDGE_ROT:+.3f})')

        deadline = time.monotonic() + HOLD_SECONDS
        next_log = time.monotonic()
        while time.monotonic() < deadline:
            if time.monotonic() >= next_log:
                print(f'[{port}]   t={time.monotonic()-t0:4.1f}s  '
                      f'pos={sess.cur_pos:+7.3f}  out={sess.cur_out_pct:+6.1f}%  '
                      f'faults=0x{sess.cur_faults:04X}  tx={sess.tx_count}')
                next_log += 1.0
            time.sleep(0.05)

        sess.is_enabled = False
        time.sleep(0.2)
        final = sess.cur_pos
        print(f'[{port}] final pos={final:+.3f}  moved={final-start_pos:+.3f}  '
              f'last_out={sess.cur_out_pct:+.1f}%  last_faults=0x{sess.cur_faults:04X}')
        return True
    finally:
        sess.close()


DISCOVERY_SECONDS = 10.0


def nudge_all(ports) -> int:
    """Open every port at once, wait DISCOVERY_SECONDS for all to publish
    device_id + Status 2, then command each one to nudge in parallel."""
    sessions = []
    for port in ports:
        try:
            sessions.append(SparkSession(port))
            print(f'[{port}] opened.')
        except Exception as exc:
            print(f'[{port}] open failed: {exc}')

    if not sessions:
        return 0

    print(f'Waiting up to {DISCOVERY_SECONDS:.0f}s for all controllers '
          f'to publish device_id + position…')
    t0 = time.monotonic()
    while time.monotonic() - t0 < DISCOVERY_SECONDS:
        if all(s.device_id is not None and s.cur_pos is not None for s in sessions):
            break
        time.sleep(0.1)

    ready, missing = [], []
    for s in sessions:
        if s.device_id is not None and s.cur_pos is not None:
            ready.append(s)
        else:
            missing.append(s)
            print(f'[{s.port}] not ready — device_id={s.device_id}, '
                  f'pos={s.cur_pos}, S0={s.status_0_count}, '
                  f'S2={s.status_2_count}, other={s.other_frame_count}')

    if not ready:
        print('No controllers became ready. Closing.')
        for s in sessions:
            s.close()
        return 0

    print(f'Ready: {[(s.port, s.device_id) for s in ready]}')

    starts = {}
    for s in ready:
        starts[s.port] = s.cur_pos
        s.target_pos = s.cur_pos + NUDGE_ROT
        s.is_enabled = True
        print(f'[{s.port}] device_id={s.device_id}  start={s.cur_pos:+.3f}  '
              f'-> target={s.target_pos:+.3f} (delta={NUDGE_ROT:+.3f})')

    deadline = time.monotonic() + HOLD_SECONDS
    next_log = time.monotonic()
    while time.monotonic() < deadline:
        if time.monotonic() >= next_log:
            t_rel = time.monotonic() - t0
            for s in ready:
                print(f'[{s.port}] t={t_rel:4.1f}s  dev={s.device_id}  '
                      f'pos={s.cur_pos:+7.3f}  out={s.cur_out_pct:+6.1f}%  '
                      f'faults=0x{s.cur_faults:04X}  tx={s.tx_count}')
            next_log += 1.0
        time.sleep(0.05)

    for s in ready:
        s.is_enabled = False
    time.sleep(0.3)

    moved_count = 0
    for s in ready:
        delta = s.cur_pos - starts[s.port]
        print(f'[{s.port}] dev={s.device_id}  final={s.cur_pos:+.3f}  '
              f'moved={delta:+.3f}  last_out={s.cur_out_pct:+.1f}%  '
              f'last_faults=0x{s.cur_faults:04X}')
        if abs(delta) > 0.1:
            moved_count += 1

    for s in sessions:
        s.close()

    return moved_count


def main() -> int:
    ports = list_spark_ports()
    if not ports:
        print('No SPARK MAX USB devices found.')
        return 1
    print(f'Found {len(ports)} SPARK MAX port(s): {ports}')
    moved = nudge_all(ports)
    print(f'Done. {moved}/{len(ports)} moved.')
    return 0 if moved == len(ports) else 2


if __name__ == '__main__':
    raise SystemExit(main())
