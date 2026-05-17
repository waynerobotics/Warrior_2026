#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import struct
import time
import threading
import sys

print("--- Initializing SparkMax Controller ---")

class SparkMaxController:
    def __init__(self, device_id=1):
        self.device_id = device_id
        self.port = None
        self.ser = None
        self.running = False
        self.target_position = 0.0
        self.is_enabled = False
        # Live state populated by reader_loop
        self.cur_pos = 0.0
        self.cur_out = 0.0
        self.cur_faults = 0

        # Frame IDs confirmed by USB-sniffing REV Hardware Client (sniff_usb.py).
        # Setpoint: dev_type=0x02, mfr=0x05, api_class=0x00, api_index=0x04, dev=N
        #   → 0x02050100 | dev
        self.setpoint_id = 0x02050100 | (self.device_id & 0x3F)

    def find_port(self):
        print("Searching for USB CDC devices...")
        ports = serial.tools.list_ports.comports()
        for p in ports:
            print(f"  Found: {p.device} - {p.description}")
            if "ACM" in p.device or "REV" in p.description:
                self.port = p.device
                return True
        return False

    def connect(self):
        if not self.find_port():
            print("!! Error: No SPARK MAX found on USB. Is it plugged in?")
            return False
        
        try:
            # 115200 is the standard SLCAN baud
            self.ser = serial.Serial(self.port, 115200, timeout=0.1)
            print(f"Successfully opened {self.port}")
            return True
        except serial.SerialException as e:
            print(f"!! Port Error: {e}")
            print("   Is another script (like the sniffer) still running?")
            return False

    def heartbeat_loop(self):
        """Send the three-frame triplet REV Hardware Client sends every cycle."""
        print("Heartbeat thread active.")

        # Broadcast frames (constant, don't depend on target):
        # "Follow your setpoint" broadcast — byte 0 is a bitmask of device_ids
        # whose bit is set (bit N = device_id N). Discovered 2026-05-17 by
        # sniffing REV Hardware Client; the old hard-coded `0x02` was bit 1 and
        # only worked when controllers were at CAN ID 1.
        SET_MODE = f"T02052C808{(1 << self.device_id):02X}" + "00" * 7 + "\r"
        # "Robot is enabled" broadcast — dev_type=0, single 0x01 data byte. Without
        # this the SPARK MAX leaves outputs at 0% no matter what setpoints arrive.
        ENABLE = "T000502C0101\r"

        while self.running:
            if self.is_enabled and self.ser:
                try:
                    # [setpoint(float32 LE) | arbFF(float32 LE)]
                    payload = struct.pack("<ff", float(self.target_position), 0.0)
                    setpoint_frame = f"T{self.setpoint_id:08X}8{payload.hex()}\r"
                    self.ser.write((setpoint_frame + SET_MODE + ENABLE).encode())
                except Exception as e:
                    print(f"Send error: {e}")
            time.sleep(0.02)  # 50 Hz

    def reader_loop(self):
        """Parse Status 0/2 frames coming back and print live state once per second."""
        line_buf = bytearray()
        last_print = 0.0
        while self.running:
            if not self.ser:
                time.sleep(0.05)
                continue
            try:
                chunk = self.ser.read(256)
            except Exception:
                continue
            for b in chunk:
                if b in (0x0D, 0x0A):
                    if line_buf:
                        self._consume_slcan(line_buf.decode(errors="ignore"))
                        line_buf.clear()
                else:
                    line_buf.append(b)
            now = time.monotonic()
            if now - last_print >= 1.0:
                last_print = now
                tag = "ON " if self.is_enabled else "off"
                # \r puts the status on the same line as the input prompt
                sys.stdout.write(
                    f"\r[{tag}] tgt={self.target_position:6.2f}  pos={self.cur_pos:7.2f}  "
                    f"out={self.cur_out:+6.1f}%  faults=0x{self.cur_faults:04X}     \n"
                )
                sys.stdout.flush()

    def _consume_slcan(self, line: str) -> None:
        if not line or line[0] not in ("t", "T"):
            return
        id_len = 8 if line[0] == "T" else 3
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
            applied_raw, faults = struct.unpack_from("<hH", data)
            self.cur_out = applied_raw / 32768.0 * 100.0
            self.cur_faults = faults
        elif api_idx == 2 and len(data) >= 8:
            self.cur_pos, = struct.unpack_from("<f", data, 4)

    def run(self):
        if not self.connect():
            return

        self.running = True
        self.tx_thread = threading.Thread(target=self.heartbeat_loop, daemon=True)
        self.rx_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.tx_thread.start()
        self.rx_thread.start()

        print(f"\n--- CONTROLLING DEVICE {self.device_id} ---")
        print("Enter 0.0 to 50.0 to move.")
        print("Enter 's' to disable, 'q' to quit.")

        try:
            while True:
                user_input = input(f"Target (currently {self.target_position}) >> ").strip().lower()
                if user_input == 'q':
                    break
                elif user_input == 's':
                    self.is_enabled = False
                    print("Safety: Motor Disabled.")
                else:
                    try:
                        val = float(user_input)
                        self.target_position = max(0, min(50, val))
                        self.is_enabled = True
                    except ValueError:
                        print("Invalid input. Enter a number.")
        finally:
            self.running = False
            if self.ser:
                self.ser.close()
            print("Exited cleanly.")

if __name__ == "__main__":
    app = SparkMaxController(device_id=2)
    app.run()