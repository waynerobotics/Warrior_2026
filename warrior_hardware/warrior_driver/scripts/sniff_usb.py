#!/usr/bin/env python3
"""
sniff_usb.py — Live kernel-level USB sniffer for the SPARK MAX.

Decodes the SPARK MAX's Bulk-OUT (host -> device) and Bulk-IN (device -> host)
endpoints as SLCAN ASCII, so you can see the exact CAN frames REV Hardware
Client - or any other tool - writes to the controller while the motor runs.

Captures *every* SPARK MAX on the bus at once (not just the first), tags each
line with the source device, and prepends the usbmon timestamp so TX/RX can be
correlated in time. This is what lets you answer "what frame does REV send at
connect that our heartbeat doesn't, and which controllers does it write to?" —
see CLAUDE.md, the 2026-05-27 cold-boot OPEN ISSUE.

Why this instead of strace:
  REV Hardware Client is Electron + JCEF helpers; the process that actually
  owns the /dev/ttyACM* file descriptor isn't the one you can easily attach
  to.  usbmon captures at the kernel layer, regardless of process.

One-time setup:
    sudo modprobe usbmon

Run (logs to stdout; tee it for later analysis):
    sudo python3 scripts/sniff_usb.py | tee /tmp/rev_cold_connect.log
"""
import os
import re
import sys

SPARK_VID = 0x0483
SPARK_PID = 0xa30e


def find_sparks():
    """Return {(bus, dev): label} for every SPARK MAX on USB.

    label is "<bus>.<dev>" so each physical controller is distinguishable in
    the output even before its CAN device_id is known.
    """
    base = "/sys/bus/usb/devices"
    found = {}
    for name in os.listdir(base):
        d = os.path.join(base, name)
        try:
            vid = int(open(os.path.join(d, "idVendor")).read(), 16)
            pid = int(open(os.path.join(d, "idProduct")).read(), 16)
        except (OSError, ValueError):
            continue
        if vid == SPARK_VID and pid == SPARK_PID:
            try:
                bus = int(open(os.path.join(d, "busnum")).read())
                dev = int(open(os.path.join(d, "devnum")).read())
            except (OSError, ValueError):
                continue
            found[(bus, dev)] = f"{bus}.{dev}"
    return found


# Bo:1:003:2  ->  type=B dir=o bus=1 dev=003 endpoint=2
ADDR_RE = re.compile(r"^([BICZ])([io]):(\d+):(\d+):\d+$")


def parse(raw: str):
    """Return (timestamp_us, event, dir, bus, dev, data) or None.

    usbmon 's' line layout:  <urb-tag> <timestamp> <event> <address> ...
    """
    parts = raw.split()
    if len(parts) < 4:
        return None
    try:
        ts = int(parts[1])
    except ValueError:
        ts = 0
    event = parts[2]
    m = ADDR_RE.match(parts[3])
    if not m:
        return None
    urb_type, urb_dir = m.group(1), m.group(2)
    bus, dev = int(m.group(3)), int(m.group(4))
    if urb_type != "B":
        return None
    if "=" not in parts:
        return ts, event, urb_dir, bus, dev, b""
    try:
        data = bytes.fromhex("".join(parts[parts.index("=") + 1:]))
    except ValueError:
        return ts, event, urb_dir, bus, dev, b""
    return ts, event, urb_dir, bus, dev, data


def flush_lines(buf: bytearray, ts: int, tag: str):
    while True:
        nl = -1
        for i, b in enumerate(buf):
            if b in (0x0D, 0x0A):
                nl = i
                break
        if nl < 0:
            return
        text = bytes(buf[:nl]).decode("ascii", errors="replace")
        del buf[:nl + 1]
        if text:
            # ts is microseconds; print as seconds.milliseconds for readability.
            print(f"{ts / 1e6:14.6f}  {tag}  {text}", flush=True)


def main():
    sparks = find_sparks()
    if not sparks:
        sys.exit(f"No SPARK MAX (VID 0x{SPARK_VID:04X} / PID 0x{SPARK_PID:04X}) on USB.")

    # The global "0u" monitor captures every bus, so a single tail handles
    # controllers that enumerate on different buses.
    mon = "/sys/kernel/debug/usb/usbmon/0u"
    if not os.access(mon, os.R_OK):
        sys.exit(
            f"Cannot read {mon}\n"
            f"  sudo modprobe usbmon\n"
            f"  sudo python3 {sys.argv[0]}"
        )

    print(f"# Sniffing {len(sparks)} SPARK MAX device(s): "
          f"{sorted(sparks.values())}")
    print(f"# tailing {mon}")
    print("# columns: <timestamp_s>  <DIR label>  <SLCAN line>\n", flush=True)

    # One TX/RX line buffer per device so interleaved bulk transfers from
    # different controllers don't corrupt each other's partial lines.
    buffers = {}  # (bus, dev, dir) -> bytearray
    with open(mon, "r", buffering=1) as fp:
        for raw in fp:
            p = parse(raw.rstrip())
            if not p:
                continue
            ts, event, urb_dir, bus, dev, data = p
            label = sparks.get((bus, dev))
            if label is None or not data:
                continue
            if urb_dir == "o" and event == "S":
                buf = buffers.setdefault((bus, dev, "o"), bytearray())
                buf.extend(data)
                flush_lines(buf, ts, f"[{label}] TX ->")
            elif urb_dir == "i" and event == "C":
                buf = buffers.setdefault((bus, dev, "i"), bytearray())
                buf.extend(data)
                flush_lines(buf, ts, f"[{label}] RX <-")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
