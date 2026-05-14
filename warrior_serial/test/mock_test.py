"""
mock_test.py

Soak-test driver for the 02_test mock swerves. Acts as a drop-in replacement
for serial_bridge.py during reconnection testing:

  1. Auto-discovers the base (00_base) and any mock swerves (02/03/04_swerve)
     by sending <WHO> and matching DEVICE_NAME.
  2. Forwards <MOT,target,spark,flipsky> from base to the addressed swerve
     port (same as serial_bridge).
  3. Every ~1 s, sends <STATS> to each alive swerve and reads back its
     counters. Detects reboots when reported uptime goes backwards and keeps
     a cumulative tally across reboots.
  4. Redraws a status table so you can yank cables, replug into different
     hub ports, power-cycle Arduinos, etc., and watch communication recover.

Designed for: Uno=00_base (with shield), Mega + Nano ESP32 each flashed
with sketches/tests/02_test/ as a mock swerve (set DEVICE_NAME at top).

Usage:
    python mock_test.py                  # auto-discover everything
    python mock_test.py --base COM5      # pin the base
    python mock_test.py --no-base        # synthesize MOT @ 20 Hz, no base needed
    python mock_test.py --reset          # send <RESET> to all mocks at startup

The script never exits on disconnect. Press Ctrl+C to stop.
"""

import argparse
import time
from dataclasses import dataclass, field
from typing import Optional

import serial
from serial.tools import list_ports

from serial_protocol import query_device_name


BAUDRATE = 115200
QUERY_TIMEOUT_S = 3.0
READ_TIMEOUT_S = 0.05
RECONNECT_INTERVAL_S = 2.0
STATS_INTERVAL_S = 1.0
TABLE_INTERVAL_S = 0.5
SYNTHETIC_MOT_HZ = 20

SWERVE_NAMES = ("02_swerve", "03_swerve", "04_swerve")
BASE_NAME = "00_base"

CSI_HOME_CLEAR = "\x1b[H\x1b[2J"


# ====================================================================
# Frame extraction (non-blocking)
# ====================================================================
class FrameReader:
    """Pull complete <...> frames out of an arbitrary byte stream."""

    def __init__(self):
        self._buf = ""
        self._inside = False

    def feed(self, data: str) -> list[str]:
        out: list[str] = []
        for ch in data:
            if ch == "<":
                self._buf = ""
                self._inside = True
            elif ch == ">" and self._inside:
                out.append(self._buf)
                self._buf = ""
                self._inside = False
            elif self._inside:
                self._buf += ch
        return out

    def reset(self) -> None:
        self._buf = ""
        self._inside = False


def open_serial(port: str) -> serial.Serial:
    s = serial.Serial(
        port=port,
        baudrate=BAUDRATE,
        timeout=READ_TIMEOUT_S,
        write_timeout=1.0,
    )
    # Arduino auto-reset on open (DTR toggle on AVR boards). Wait for the
    # bootloader window to expire before sending anything. 2.5 s is enough
    # for both AVR optiboot (~1 s) and Nano ESP32 native USB (~1.5 s).
    time.sleep(2.5)
    s.reset_input_buffer()
    return s


VERBOSE = False


def vlog(*args, **kwargs) -> None:
    if VERBOSE:
        print(*args, **kwargs, flush=True)


def drain_available(ser: serial.Serial, reader: FrameReader,
                    label: str = "") -> list[str]:
    n = ser.in_waiting
    if not n:
        return []
    data = ser.read(n).decode("utf-8", errors="ignore")
    if VERBOSE and label:
        vlog(f"[{label} <-] {data!r}")
    return reader.feed(data)


# ====================================================================
# Per-swerve tracker
# ====================================================================
@dataclass
class SwerveTracker:
    name: str
    port: str
    ser: serial.Serial
    reader: FrameReader = field(default_factory=FrameReader)

    # Most-recent values from a STATS reply
    uptime_ms: int = 0
    recv: int = 0
    wrong: int = 0
    parse_err: int = 0
    unknown: int = 0
    ms_since_last: int = 0
    rate_hz: int = 0
    last_spark: int = 0
    last_flipsky: int = 0

    # Cumulative across reboots
    cum_recv: int = 0
    cum_wrong: int = 0
    cum_parse_err: int = 0
    cum_unknown: int = 0
    reboots: int = 0
    last_seen_uptime: int = 0

    # Health
    last_stats_reply_at: float = 0.0
    last_stats_request_at: float = 0.0
    alive: bool = True
    error: str = ""

    def close(self) -> None:
        try:
            if self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def request_stats(self) -> None:
        try:
            self.ser.write(b"<STATS>\n")
            self.ser.flush()
            vlog(f"[{self.name} ->] <STATS>")
            self.last_stats_request_at = time.monotonic()
        except Exception as exc:
            self._fail(f"write STATS: {exc}")

    def send_reset(self) -> None:
        try:
            self.ser.write(b"<RESET>\n")
            self.ser.flush()
        except Exception as exc:
            self._fail(f"write RESET: {exc}")

    def forward_mot(self, target: str, spark: str, flipsky: str) -> None:
        try:
            self.ser.write(f"<MOT,{target},{spark},{flipsky}>\n".encode("utf-8"))
        except Exception as exc:
            self._fail(f"write MOT: {exc}")

    def drain(self) -> None:
        if not self.alive:
            return
        try:
            frames = drain_available(self.ser, self.reader, label=self.name)
        except Exception as exc:
            self._fail(f"read: {exc}")
            return
        for f in frames:
            self._handle_frame(f)

    def _handle_frame(self, frame: str) -> None:
        if frame.startswith("STATS,"):
            self._parse_stats(frame)
        # Other frames (ACKs, NAMEs) are ignored — we don't need them here.

    def _parse_stats(self, frame: str) -> None:
        # STATS,name,uptimeMs,motForMe,motForOther,parseErr,unknown,
        # msSinceLast,rateHz,lastSpark,lastFlipsky
        parts = frame.split(",")
        if len(parts) != 11:
            return
        try:
            new_uptime = int(parts[2])
            new_recv = int(parts[3])
            new_wrong = int(parts[4])
            new_parse = int(parts[5])
            new_unknown = int(parts[6])
            ms_since = int(parts[7])
            rate = int(parts[8])
            ls = int(parts[9])
            lf = int(parts[10])
        except ValueError:
            return

        # Reboot detection: uptime went backwards. Roll the previous
        # in-memory totals into the cumulative tally before overwriting.
        if new_uptime < self.last_seen_uptime:
            self.cum_recv += self.recv
            self.cum_wrong += self.wrong
            self.cum_parse_err += self.parse_err
            self.cum_unknown += self.unknown
            self.reboots += 1

        self.uptime_ms = new_uptime
        self.last_seen_uptime = new_uptime
        self.recv = new_recv
        self.wrong = new_wrong
        self.parse_err = new_parse
        self.unknown = new_unknown
        self.ms_since_last = ms_since
        self.rate_hz = rate
        self.last_spark = ls
        self.last_flipsky = lf
        self.last_stats_reply_at = time.monotonic()
        self.error = ""

    def _fail(self, reason: str) -> None:
        self.alive = False
        self.error = reason
        try:
            self.ser.close()
        except Exception:
            pass


# ====================================================================
# Base tracker — only reads MOT messages, never queries
# ====================================================================
@dataclass
class BaseTracker:
    port: str
    ser: serial.Serial
    reader: FrameReader = field(default_factory=FrameReader)
    alive: bool = True
    error: str = ""

    def close(self) -> None:
        try:
            if self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def drain(self) -> list[str]:
        if not self.alive:
            return []
        try:
            return drain_available(self.ser, self.reader)
        except Exception as exc:
            self.alive = False
            self.error = f"read: {exc}"
            self.close()
            return []


# ====================================================================
# Discovery
# ====================================================================
def find_named(targets: set[str], skip_ports: set[str]) -> dict[str, str]:
    """Return {device_name: port} for ports whose WHO reply matches targets."""
    found: dict[str, str] = {}
    for p in list_ports.comports():
        if p.device in skip_ports:
            continue
        try:
            name = query_device_name(p.device, BAUDRATE, timeout=QUERY_TIMEOUT_S)
        except Exception:
            continue
        if name and name.strip() in targets:
            found[name.strip()] = p.device
    return found


def try_open_base(base_arg: Optional[str],
                  skip_ports: Optional[set[str]] = None) -> Optional[BaseTracker]:
    if base_arg:
        try:
            ser = open_serial(base_arg)
            return BaseTracker(port=base_arg, ser=ser)
        except Exception:
            return None
    found = find_named({BASE_NAME}, skip_ports=skip_ports or set())
    port = found.get(BASE_NAME)
    if not port:
        return None
    try:
        ser = open_serial(port)
    except Exception:
        return None
    return BaseTracker(port=port, ser=ser)


def try_open_swerves(skip_ports: set[str],
                     existing: dict[str, SwerveTracker]) -> dict[str, SwerveTracker]:
    """Find swerves we don't already have an alive connection to and open them."""
    needed = {n for n in SWERVE_NAMES
              if n not in existing or not existing[n].alive}
    if not needed:
        return {}
    found = find_named(needed, skip_ports=skip_ports)
    new: dict[str, SwerveTracker] = {}
    for name, port in found.items():
        try:
            ser = open_serial(port)
        except Exception:
            continue
        new[name] = SwerveTracker(name=name, port=port, ser=ser)
    return new


# ====================================================================
# Status table
# ====================================================================
def render_table(base: Optional[BaseTracker],
                 swerves: dict[str, SwerveTracker],
                 base_arg: Optional[str],
                 no_base: bool,
                 base_msgs: int,
                 forwarded: int) -> str:
    lines = []
    lines.append("mock_test — soak driver  (Ctrl+C to quit)")
    if no_base:
        base_state = f"synthetic @ {SYNTHETIC_MOT_HZ} Hz"
    elif base is None or not base.alive:
        err = f" [{base.error}]" if base else ""
        base_state = f"<lost>{err}  searching..."
    else:
        base_state = f"{BASE_NAME} @ {base.port}  msgs={base_msgs}  forwarded={forwarded}"
    lines.append(f"  base:    {base_state}")
    lines.append("")

    header = f"  {'name':<10} {'port':<14} {'uptime':>7} {'recv':>7} {'wrong':>5} {'parse':>5} {'unk':>4} {'late_ms':>7} {'rate':>4} {'last_s/f':>10} {'reboots':>7}  {'state'}"
    lines.append(header)
    lines.append("  " + "-" * (len(header) - 2))

    for name in SWERVE_NAMES:
        sw = swerves.get(name)
        if sw is None:
            lines.append(f"  {name:<10} {'-':<14} {'-':>7} {'-':>7} {'-':>5} {'-':>5} {'-':>4} {'-':>7} {'-':>4} {'-':>10} {'-':>7}  <not found>")
            continue

        if not sw.alive:
            lines.append(f"  {name:<10} {sw.port:<14} {'-':>7} {'-':>7} {'-':>5} {'-':>5} {'-':>4} {'-':>7} {'-':>4} {'-':>10} {sw.reboots:>7}  <lost: {sw.error}>")
            continue

        late_warn = ""
        if sw.last_stats_reply_at == 0.0 or sw.ms_since_last == 0xFFFFFFFF:
            # Either we've never gotten a STATS reply, or the swerve has
            # never received a MOT (sentinel = 0xFFFFFFFF from the sketch).
            late_str = "-"
        else:
            late_str = str(sw.ms_since_last)
            if sw.ms_since_last > 200 and not no_base and base and base.alive:
                late_warn = "!"
        # Stats freshness from our perspective
        local_age = time.monotonic() - sw.last_stats_reply_at if sw.last_stats_reply_at else None
        if local_age is None:
            state = "<no stats yet>"
        elif local_age < 3.0:
            state = "ok"
        else:
            state = f"<no STATS for {local_age:.1f}s>"

        recv_total = sw.cum_recv + sw.recv
        wrong_total = sw.cum_wrong + sw.wrong
        parse_total = sw.cum_parse_err + sw.parse_err
        unk_total = sw.cum_unknown + sw.unknown

        uptime_s = f"{sw.uptime_ms / 1000:.1f}s"
        sf = f"{sw.last_spark}/{sw.last_flipsky}"
        lines.append(
            f"  {name:<10} {sw.port:<14} {uptime_s:>7} {recv_total:>7} "
            f"{wrong_total:>4}{'!' if wrong_total else ' '} "
            f"{parse_total:>4}{'!' if parse_total else ' '} "
            f"{unk_total:>4} {late_str+late_warn:>7} {sw.rate_hz:>4} {sf:>10} "
            f"{sw.reboots:>7}  {state}"
        )

    lines.append("")
    lines.append("  '!' marks a counter that should be zero or a stale value")
    return "\n".join(lines) + "\n"


# ====================================================================
# Main loop
# ====================================================================
def main() -> None:
    global VERBOSE
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", help="COM port for 00_base (skip auto-discovery)")
    parser.add_argument("--no-base", action="store_true",
                        help="Don't use a base; synthesize MOT @ 20 Hz instead")
    parser.add_argument("--reset", action="store_true",
                        help="Send <RESET> to each swerve once on first connection")
    parser.add_argument("--verbose", action="store_true",
                        help="Log raw send/recv bytes instead of redrawing the table")
    args = parser.parse_args()
    VERBOSE = args.verbose

    base: Optional[BaseTracker] = None
    swerves: dict[str, SwerveTracker] = {}
    reset_sent: set[str] = set()

    last_reconnect_attempt = 0.0
    last_stats_request_round = 0.0
    last_table_draw = 0.0
    last_synthetic_mot = 0.0
    synthetic_idx = 0
    forwarded_count = 0
    base_msgs = 0

    print("Discovering devices on startup...")

    try:
        while True:
            now = time.monotonic()

            # --- Reconnect anything lost ---------------------------------
            if now - last_reconnect_attempt >= RECONNECT_INTERVAL_S:
                last_reconnect_attempt = now

                # Compute ports we already own *before* scanning so the scan
                # never opens a second fd against a port held by an alive
                # tracker — that fd would steal bytes from the main loop.
                used_ports = set()
                if base and base.alive:
                    used_ports.add(base.port)
                for sw in swerves.values():
                    if sw.alive:
                        used_ports.add(sw.port)

                if not args.no_base and (base is None or not base.alive):
                    if base is not None:
                        base.close()
                    base = try_open_base(args.base, skip_ports=used_ports)
                    if base and base.alive:
                        used_ports.add(base.port)

                new_swerves = try_open_swerves(skip_ports=used_ports, existing=swerves)
                for name, tracker in new_swerves.items():
                    if name in swerves:
                        swerves[name].close()
                    swerves[name] = tracker

            # --- Optional one-time RESET --------------------------------
            if args.reset:
                for name, sw in swerves.items():
                    if sw.alive and name not in reset_sent:
                        sw.send_reset()
                        reset_sent.add(name)

            # --- Forward MOT from base to addressed swerve --------------
            if not args.no_base and base and base.alive:
                frames = base.drain()
                for msg in frames:
                    base_msgs += 1
                    parts = msg.split(",")
                    if parts[0] != "MOT" or len(parts) != 4:
                        continue
                    target, spark, flipsky = parts[1], parts[2], parts[3]
                    sw = swerves.get(target)
                    if sw and sw.alive:
                        sw.forward_mot(target, spark, flipsky)
                        forwarded_count += 1

            # --- Or synthesize MOT --------------------------------------
            if args.no_base:
                interval = 1.0 / SYNTHETIC_MOT_HZ
                if now - last_synthetic_mot >= interval:
                    last_synthetic_mot = now
                    targets = list(swerves.keys()) or list(SWERVE_NAMES)
                    target = targets[synthetic_idx % len(targets)]
                    synthetic_idx += 1
                    spark = (synthetic_idx % 201) - 100
                    flipsky = -spark
                    sw = swerves.get(target)
                    if sw and sw.alive:
                        sw.forward_mot(target, str(spark), str(flipsky))
                        forwarded_count += 1

            # --- Drain swerve replies (STATS responses) -----------------
            for sw in swerves.values():
                sw.drain()

            # --- Periodic STATS round -----------------------------------
            if now - last_stats_request_round >= STATS_INTERVAL_S:
                last_stats_request_round = now
                for sw in swerves.values():
                    if sw.alive:
                        sw.request_stats()

            # --- Redraw table -------------------------------------------
            # Skip the screen-clearing redraw in verbose mode so the raw
            # send/recv log lines stay readable.
            if not VERBOSE and now - last_table_draw >= TABLE_INTERVAL_S:
                last_table_draw = now
                print(CSI_HOME_CLEAR + render_table(
                    base, swerves, args.base, args.no_base,
                    base_msgs, forwarded_count
                ), end="", flush=True)

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nStopping mock_test.")
    finally:
        if base is not None:
            base.close()
        for sw in swerves.values():
            sw.close()


if __name__ == "__main__":
    main()
