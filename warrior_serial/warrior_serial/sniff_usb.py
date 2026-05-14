#!/usr/bin/env python3
"""
sniff_usb.py — Live kernel-level USB sniffer for the SPARK MAX.

Decodes the SPARK MAX's Bulk-OUT (host → device) and Bulk-IN (device → host)
endpoints as SLCAN ASCII, so you can see the exact CAN frames REV Hardware
Client — or any other tool — writes to the controller while the motor runs.

Why this instead of strace:
  REV Hardware Client is Electron + JCEF helpers; the process that actually
  owns the /dev/ttyACM* file descriptor isn't the one you can easily attach
  to.  usbmon captures at the kernel layer, regardless of process.

One-time setup:
    sudo modprobe usbmon

Run:
    sudo python3 scripts/sniff_usb.py

Workflow:
    1. Plug in the SPARK MAX.
    2. Start this sniffer.
    3. Open REV Hardware Client and operate the motor (drag the position
       slider, hit Apply, whatever makes it move).
    4. Each '\\r'-terminated ASCII line that prints is one SLCAN frame:
         "TX →"  — bytes REV Client wrote to the controller
         "RX ←"  — bytes the controller wrote back
    5. Copy the relevant "TX →" frame format into talk_can.py.

Ctrl-C to stop.
"""
import os
import re
import sys

SPARK_VID = 0x0483
SPARK_PID = 0xa30e


def find_spark():
    base = "/sys/bus/usb/devices"
    for name in os.listdir(base):
        d = os.path.join(base, name)
        try:
            vid = int(open(os.path.join(d, "idVendor")).read(), 16)
            pid = int(open(os.path.join(d, "idProduct")).read(), 16)
        except (OSError, ValueError):
            continue
        if vid == SPARK_VID and pid == SPARK_PID:
            bus = int(open(os.path.join(d, "busnum")).read())
            dev = int(open(os.path.join(d, "devnum")).read())
            return bus, dev
    return None


ADDR_RE = re.compile(r"^([BICZ])([io]):\d+:(\d+):\d+$")


def parse(raw: str):
    """Return (event, dir, dev_num, data_bytes) for bulk transfers, else None."""
    parts = raw.split()
    if len(parts) < 4:
        return None
    event = parts[2]                  # 'S' submit, 'C' complete, 'E' error
    m = ADDR_RE.match(parts[3])
    if not m:
        return None
    urb_type, urb_dir, dev = m.group(1), m.group(2), int(m.group(3))
    if urb_type != "B":               # only care about bulk
        return None
    if "=" not in parts:
        return event, urb_dir, dev, b""
    try:
        data = bytes.fromhex("".join(parts[parts.index("=") + 1:]))
    except ValueError:
        return event, urb_dir, dev, b""
    return event, urb_dir, dev, data


def flush_lines(buf: bytearray, tag: str):
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
            print(f"{tag}  {text}", flush=True)


def main():
    found = find_spark()
    if not found:
        sys.exit(f"SPARK MAX (VID 0x{SPARK_VID:04X} / PID 0x{SPARK_PID:04X}) not on USB.")
    bus, dev = found
    mon = f"/sys/kernel/debug/usb/usbmon/{bus}u"
    if not os.access(mon, os.R_OK):
        sys.exit(
            f"Cannot read {mon}\n"
            f"  sudo modprobe usbmon\n"
            f"  sudo python3 {sys.argv[0]}"
        )

    print(f"# SPARK MAX = bus {bus} device {dev}")
    print(f"# tailing {mon}\n")

    tx_buf, rx_buf = bytearray(), bytearray()
    with open(mon, "r", buffering=1) as fp:
        for raw in fp:
            p = parse(raw.rstrip())
            if not p:
                continue
            event, urb_dir, dnum, data = p
            if dnum != dev or not data:
                continue
            # OUT bytes ride along with the SUBMIT; IN bytes arrive at COMPLETE.
            if urb_dir == "o" and event == "S":
                tx_buf.extend(data)
                flush_lines(tx_buf, "TX →")
            elif urb_dir == "i" and event == "C":
                rx_buf.extend(data)
                flush_lines(rx_buf, "RX ←")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
