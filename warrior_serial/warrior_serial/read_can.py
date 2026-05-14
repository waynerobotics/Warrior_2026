#!/usr/bin/env python3
"""
read_can.py — Full Sniffer for REV SPARK MAX over USB.
Includes Raw hex mode and full status decoding.
"""

import argparse
import struct
import sys
import time
import serial

# ── CAN ID field extraction ──────────────────────────────────────────────────

def decode_can_id(raw: int) -> dict:
    # FRC CAN layout: dev_type[5] | mfr[8] | api_class[6] | api_index[4] | dev_num[6]
    return {
        "dev_type":  (raw >> 24) & 0x1F,
        "mfr":       (raw >> 16) & 0xFF,
        "api_class": (raw >> 10) & 0x3F,
        "api_index": (raw >>  6) & 0x0F,
        "device_id":  raw        & 0x3F,
    }

# ── Status frame parsers (API Class 0x6) ─────────────────────────────────────

def parse_status0(data: bytes) -> str:
    applied_raw, faults, sticky, misc1, misc2 = struct.unpack_from("<hHHBB", data)
    applied_pct = applied_raw / 32768.0 * 100.0
    return (f"STATUS_0 | Output: {applied_pct:+.1f}% | Faults: 0x{faults:04X}")

def parse_status1(data: bytes) -> str:
    velocity, = struct.unpack_from("<f", data, 0)
    temp = data[4]
    v_raw = ((data[6] & 0x0F) << 8) | data[5]
    i_raw = (data[7] << 4) | ((data[6] >> 4) & 0x0F)
    voltage = v_raw / 128.0
    current = i_raw / 32.0
    return (f"STATUS_1 | Vel: {velocity:7.1f} RPM | {temp}°C | {voltage:5.2f}V | {current:5.2f}A")

def parse_status2(data: bytes) -> str:
    # Bytes 4..7 = primary encoder position (rotations). Confirmed against REV Hardware Client.
    position, = struct.unpack_from("<f", data, 4)
    return f"STATUS_2 | Pos: {position:.4f} rot"

# ── Config / Heartbeat parsers (API Class 0x0B) ──────────────────────────────

def parse_config_heartbeat(data: bytes, api_idx: int) -> str:
    if api_idx == 8:
        try:
            val, = struct.unpack_from("<f", data, 4)
            return f"CONFIG_HB | Info Frame | Tail_Float: {val:.3f}"
        except:
            return f"CONFIG_HB | Raw: {data.hex(' ')}"
    elif api_idx == 9:
        return f"CONFIG_HB | Sync Heartbeat (Idx 9)"
    elif api_idx == 12:
        return f"CONFIG_HB | Checksum/Sync: {data.hex(' ')}"
    return f"CONFIG_HB | Unknown Index {api_idx}"

# ── Mapping ──────────────────────────────────────────────────────────────────

STATUS_PARSERS = {0: parse_status0, 1: parse_status1, 2: parse_status2}

API_CLASS_NAMES = {
    0x2E: "Status",
    0x2F: "StatusExt",
}

# ── SLCAN line parser ────────────────────────────────────────────────────────

def parse_slcan_line(line: str):
    if not line or line[0] not in ("t", "T", "r", "R"):
        return None
    ext = line[0] in ("T", "R")
    rem = line[0] in ("r", "R")
    id_len = 8 if ext else 3
    try:
        can_id = int(line[1:1+id_len], 16)
        dlc = int(line[1+id_len:1+id_len+1], 16)
        data = bytes.fromhex(line[1+id_len+1:1+id_len+1+(2*dlc)]) if not rem else b""
        return can_id, data, ext, rem
    except ValueError:
        return None

# ── Main logic ───────────────────────────────────────────────────────────────

SNAPSHOT_INTERVAL = 3.0  # seconds


def handle_frame(can_id, data, extended, remote, show_raw, state):
    f = decode_can_id(can_id)
    dev_id = f["device_id"]
    api_cls = f["api_class"]
    api_idx = f["api_index"]

    if show_raw:
        print(f"RAW | ID: 0x{can_id:08X} | Dev: {dev_id:2d} | Data: [{data.hex(' ')}]")
        return

    if api_cls == 0x2E:
        parser = STATUS_PARSERS.get(api_idx)
        output_str = parser(data) if parser and len(data) >= 4 else f"Status{api_idx} | Data: [{data.hex(' ')}]"
    else:
        cls_name = API_CLASS_NAMES.get(api_cls, f"CLS_0x{api_cls:02X}")
        output_str = f"{cls_name} | Idx: {api_idx} | Data: [{data.hex(' ')}]"

    state["frames"][(dev_id, api_cls, api_idx)] = output_str

    now = time.monotonic()
    if now - state["last_dump"] >= SNAPSHOT_INTERVAL:
        state["last_dump"] = now
        print(f"--- {time.strftime('%H:%M:%S')} ---")
        for (d, _, _), line in sorted(state["frames"].items()):
            print(f"DEV {d:2d} >> {line}")


def run(port: str, show_raw: bool, baud: int):
    print(f"Monitoring {port} (Raw={show_raw})...")
    with serial.Serial(port, baud, timeout=0.1) as ser:
        line_buf = bytearray()
        state = {"frames": {}, "last_dump": time.monotonic()}
        while True:
            chunk = ser.read(128)
            for b in chunk:
                if b in (0x0D, 0x0A):
                    if line_buf:
                        parsed = parse_slcan_line(line_buf.decode(errors='ignore'))
                        if parsed:
                            handle_frame(*parsed, show_raw, state)
                        line_buf.clear()
                else:
                    line_buf.append(b)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM1")
    parser.add_argument("--raw", action="store_true", help="Show all frames in hex")
    args = parser.parse_args()
    try:
        run(args.port, args.raw, 115200)
    except KeyboardInterrupt:
        print("\nStopped.")