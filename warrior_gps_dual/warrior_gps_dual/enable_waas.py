#!/usr/bin/env python3
"""
Enable SBAS/WAAS on a u-blox GPS receiver via the UBX binary protocol.
Polls current config, sends CFG-SBAS enable, saves to BBR, then verifies.
Run ONCE per receiver — the setting persists in battery-backed RAM.

Usage:
    ros2 run warrior_gps_dual enable_waas [PORT] [BAUD]
    ros2 run warrior_gps_dual enable_waas               # auto-detect by USB VID
    ros2 run warrior_gps_dual enable_waas /dev/serial/by-id/usb-u-blox_...-if00
    python3 enable_waas.py /dev/serial/by-id/usb-u-blox_...-if00 9600

Enabling SBAS changes fix quality from GPS (GPGGA field 6 = 1) to DGPS (= 2)
and improves horizontal accuracy from ~2-5 m to ~1-3 m. Allow 30-60 s after
enabling for WAAS satellite acquisition (PRN 135/138 over the US).

Requires: pyserial (rosdep key python3-serial).
"""

import sys
import struct
import time

import serial
import serial.tools.list_ports


# ── UBX framing ───────────────────────────────────────────────────────────────

def ubx_build(cls, msg_id, payload=b''):
    """Build a complete UBX message with sync chars and checksum."""
    body = bytes([cls, msg_id]) + struct.pack('<H', len(payload)) + payload
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return b'\xb5\x62' + body + bytes([ck_a, ck_b])


def ubx_wait_ack(ser, target_cls, target_id, timeout=3.0):
    """Wait for UBX-ACK-ACK/NAK for the given msg. True=ACK, False=NAK, None=timeout."""
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        buf.extend(chunk)
        for i in range(len(buf) - 9):
            if (buf[i] == 0xB5 and buf[i + 1] == 0x62 and
                    buf[i + 2] == 0x05 and
                    buf[i + 4] == 0x02 and buf[i + 5] == 0x00):
                ack_type = buf[i + 3]    # 0x01=ACK, 0x00=NAK
                resp_cls = buf[i + 6]
                resp_id = buf[i + 7]
                if resp_cls == target_cls and resp_id == target_id:
                    return ack_type == 0x01
        if len(buf) > 512:
            buf = buf[-256:]
    return None


def ubx_poll_response(ser, target_cls, target_id, timeout=3.0):
    """Send a poll (empty payload) and return the response payload bytes, or None."""
    ser.write(ubx_build(target_cls, target_id))
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        buf.extend(chunk)
        for i in range(len(buf) - 5):
            if (buf[i] == 0xB5 and buf[i + 1] == 0x62 and
                    buf[i + 2] == target_cls and buf[i + 3] == target_id):
                plen = struct.unpack_from('<H', buf, i + 4)[0]
                end = i + 6 + plen + 2
                if len(buf) >= end:
                    return bytes(buf[i + 6: i + 6 + plen])
        if len(buf) > 512:
            buf = buf[-256:]
    return None


# ── CFG-SBAS helpers ──────────────────────────────────────────────────────────

CFG_SBAS = (0x06, 0x16)
CFG_CFG = (0x06, 0x09)


def parse_sbas(payload):
    """Parse CFG-SBAS response payload (8 bytes)."""
    if len(payload) < 8:
        return None
    mode, usage, max_sbas, scanmode2 = struct.unpack_from('BBBB', payload, 0)
    scanmode1 = struct.unpack_from('<I', payload, 4)[0]
    return {
        "enabled": bool(mode & 0x01),
        "test_mode": bool(mode & 0x02),
        "usage_range": bool(usage & 0x01),
        "usage_diffcorr": bool(usage & 0x02),
        "usage_integrity": bool(usage & 0x04),
        "max_sbas": max_sbas,
        "scanmode1": scanmode1,    # 0 = auto-scan all PRNs
        "scanmode2": scanmode2,
    }


def print_sbas(cfg, label=""):
    print(f"\n  {label}CFG-SBAS:")
    print(f"    Enabled        : {cfg['enabled']}")
    print(f"    Test mode      : {cfg['test_mode']}")
    print(f"    Use corrections: {cfg['usage_diffcorr']}")
    print(f"    Use ranging    : {cfg['usage_range']}")
    print(f"    Use integrity  : {cfg['usage_integrity']}")
    print(f"    Max SBAS sats  : {cfg['max_sbas']}")
    prn_note = "auto-scan all" if cfg['scanmode1'] == 0 else hex(cfg['scanmode1'])
    print(f"    PRN scan mask  : {prn_note}")


# ── Main ──────────────────────────────────────────────────────────────────────

UBLOX_VID = 0x1546   # u-blox AG


def find_port():
    """Find a single u-blox GPS port by USB VID (keyword fallback). Returns the
    device path, or None — and refuses to guess if several match, since the
    robot has many /dev/ttyACM* devices that are NOT the GPS."""
    keywords = ["gps", "gnss", "u-blox", "ublox"]
    matches = []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if getattr(p, "vid", None) == UBLOX_VID or any(k in desc for k in keywords):
            matches.append(p.device)
    matches = sorted(set(matches))
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        print("Multiple u-blox ports found — specify one explicitly:")
        for d in matches:
            print(f"  {d}")
    else:
        print("No u-blox GPS port auto-detected. Pass the port explicitly.")
    return None


def main():
    port = sys.argv[1] if len(sys.argv) >= 2 else find_port()
    baud = int(sys.argv[2]) if len(sys.argv) >= 3 else 9600

    if not port:
        print("No serial port found.")
        sys.exit(1)

    print(f"Opening {port} at {baud} baud…")

    with serial.Serial(port, baud, timeout=1) as ser:
        time.sleep(0.5)
        ser.reset_input_buffer()

        # 1. Poll current config
        print("\n── Current configuration ─────────────────────────────────")
        payload = ubx_poll_response(ser, *CFG_SBAS)
        if payload:
            cfg = parse_sbas(payload)
            print_sbas(cfg, "Before: ") if cfg else print("  (could not parse response)")
        else:
            print("  (no response to poll — receiver may not support UBX CFG-SBAS)")

        # 2. Send CFG-SBAS: enable, corrections+ranging+integrity, auto-scan
        print("\n── Sending CFG-SBAS enable ───────────────────────────────")
        sbas_payload = struct.pack('<BBBBi',
                                   0x01,   # mode: bit0=enable
                                   0x07,   # usage: range | diffCorr | integrity
                                   0x03,   # maxSBAS: up to 3 SBAS sats
                                   0x00,   # scanmode2: 0=auto
                                   0)      # scanmode1: 0=auto-scan all PRNs
        ser.write(ubx_build(*CFG_SBAS, sbas_payload))

        result = ubx_wait_ack(ser, *CFG_SBAS)
        if result is True:
            print("  ✓ ACK received — SBAS enabled")
        elif result is False:
            print("  ✗ NAK received — receiver rejected the config (check baud rate)")
            sys.exit(1)
        else:
            print("  ? No ACK/NAK within 3 s — continuing anyway")

        # 3. Save to battery-backed RAM (survives power cycle)
        print("\n── Saving config to BBR ──────────────────────────────────")
        cfg_cfg_payload = struct.pack('<IIIB',
                                      0x00000000,   # clearMask: nothing
                                      0x0000FFFF,   # saveMask: all config sections
                                      0x00000000,   # loadMask: nothing
                                      0x01)         # deviceMask: BBR only
        ser.write(ubx_build(*CFG_CFG, cfg_cfg_payload))
        result = ubx_wait_ack(ser, *CFG_CFG)
        if result is True:
            print("  ✓ ACK — config saved to battery-backed RAM")
        elif result is False:
            print("  ✗ NAK — save failed (config still active this session)")
        else:
            print("  ? No ACK/NAK (config may still be saved)")

        # 4. Verify
        print("\n── Verified configuration ────────────────────────────────")
        time.sleep(0.5)
        payload = ubx_poll_response(ser, *CFG_SBAS)
        if payload:
            cfg = parse_sbas(payload)
            if cfg:
                print_sbas(cfg, "After:  ")
                if cfg["enabled"]:
                    print("\n  SBAS/WAAS is ON. The receiver will scan for WAAS sats")
                    print("  (PRN 135/138 over the US). Fix quality should change from")
                    print("  'GPS' → 'DGPS' within 30-60 s once one is acquired.")
                else:
                    print("\n  Warning: SBAS still shows disabled after sending enable.")
        else:
            print("  (no poll response)")


if __name__ == "__main__":
    main()
