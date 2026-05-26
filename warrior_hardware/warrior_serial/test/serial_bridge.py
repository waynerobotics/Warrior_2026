"""
serial_bridge.py

Reads <MOT,target,spark,flipsky> messages from the 00_base Arduino and routes
them to whichever of 02_swerve, 03_swerve, 04_swerve is currently connected.
Each swerve also filters by its DEVICE_NAME, so a missing or
mis-routed message is harmless.

Runs forever — if any Arduino is unplugged or rebooted, the bridge
re-discovers and reconnects. Press Ctrl+C to stop.

Usage:
    python serial_bridge.py                  # auto-discover everything
    python serial_bridge.py --base COM5      # pin the base, auto-discover swerves
    python serial_bridge.py --scan           # list every port and its DEVICE_NAME
"""

import argparse
import time
from serial.tools import list_ports
from Warrior_2026.warrior_hardware.warrior_serial.test.serial_protocol import WarriorSerial, query_device_name


BAUDRATE = 115200
QUERY_TIMEOUT = 3.0
READ_TIMEOUT  = 0.1
RETRY_DELAY_S = 2.0

SWERVE_NAMES = ("02_swerve", "03_swerve", "04_swerve")


def scan_ports() -> None:
    """Print every COM port and what device name it reports."""
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    print(f"Found {len(ports)} port(s):")
    for p in ports:
        print(f"  {p.device}  ({p.description})", end="  →  ")
        try:
            name = query_device_name(p.device, BAUDRATE, timeout=QUERY_TIMEOUT)
            print(name if name else "<no response>")
        except Exception as exc:
            print(f"<error: {exc}>")


def find_named_devices(targets: set[str], verbose: bool) -> dict[str, str]:
    """Scan USB serial ports only, return {device_name: port} for any port whose
    DEVICE_NAME is in `targets`."""
    found: dict[str, str] = {}
    for p in list_ports.comports():
        # Skip hardware UARTs (/dev/ttyS*) — only USB-attached Arduinos matter
        if p.device.startswith('/dev/ttyS'):
            continue
        if verbose:
            print(f"  Querying {p.device} ({p.description})...", end=" ", flush=True)
        try:
            name = query_device_name(p.device, BAUDRATE, timeout=QUERY_TIMEOUT)
            if verbose:
                print(name if name else "<no response>")
            if name and name.strip() in targets:
                found[name.strip()] = p.device
        except Exception as exc:
            if verbose:
                print(f"<error: {exc}>")
    return found


def find_base_blocking(base_port_arg: str | None) -> str:
    """Return the base port. Auto-discover unless overridden via --base."""
    if base_port_arg:
        print(f"Using manual base port: {base_port_arg}")
        return base_port_arg

    print("Looking for DEVICE_NAME=\"00_base\"...")
    verbose = True
    while True:
        found = find_named_devices({"00_base"}, verbose=verbose)
        if "00_base" in found:
            print(f"  FOUND DEVICE_NAME=\"00_base\" on {found['00_base']}")
            return found["00_base"]
        if verbose:
            print(f"  00_base not found; will keep retrying every {RETRY_DELAY_S:.0f}s "
                  "(close any open Serial Monitors)")
            verbose = False
        time.sleep(RETRY_DELAY_S)


def find_swerves_blocking() -> dict[str, str]:
    """Scan repeatedly until at least one swerve (02/03/04) responds."""
    print(f"Looking for swerves: {', '.join(SWERVE_NAMES)}...")
    verbose = True
    while True:
        found = find_named_devices(set(SWERVE_NAMES), verbose=verbose)
        if found:
            for name in sorted(found):
                print(f"  FOUND DEVICE_NAME=\"{name}\" on {found[name]}")
            missing = [n for n in SWERVE_NAMES if n not in found]
            if missing:
                print(f"  Missing (continuing without): {', '.join(missing)}")
            return found
        if verbose:
            print(f"  No swerves found; will keep retrying every {RETRY_DELAY_S:.0f}s")
            verbose = False
        time.sleep(RETRY_DELAY_S)


def open_blocking(conn: WarriorSerial, label: str, port: str) -> None:
    """Open a serial port, retrying forever on permission/access errors."""
    first = True
    while True:
        try:
            conn.open()
            print(f"  Connected: {label} on {port}")
            return
        except Exception as exc:
            if first:
                print(f"  Waiting for {label} on {port}: {exc}")
                first = False
            time.sleep(RETRY_DELAY_S)


def discover_and_open(base_port_arg: str | None
                      ) -> tuple[WarriorSerial, str, dict[str, WarriorSerial]]:
    """Find and open the base + every available swerve. Blocks until ready."""
    base_port = find_base_blocking(base_port_arg)
    base = WarriorSerial(base_port, BAUDRATE, timeout=READ_TIMEOUT)
    open_blocking(base, "00_base", base_port)

    swerve_ports = find_swerves_blocking()
    swerves: dict[str, WarriorSerial] = {}
    for name, port in swerve_ports.items():
        conn = WarriorSerial(port, BAUDRATE, timeout=READ_TIMEOUT)
        open_blocking(conn, name, port)
        swerves[name] = conn

    return base, base_port, swerves


def safe_close(conn: WarriorSerial) -> None:
    try:
        conn.close()
    except Exception:
        pass


def run_bridge(base: WarriorSerial, swerves: dict[str, WarriorSerial]) -> None:
    """Forward <MOT,target,spark,flipsky> from base to the addressed swerve.
    Raises on disconnect."""
    forwarded = 0
    skipped_no_target = 0
    last_log_time = 0.0
    LOG_INTERVAL_S = 2.0

    while True:
        msg = base.read_message(timeout=READ_TIMEOUT)
        if msg is None:
            continue

        parts = msg.split(",")
        if parts[0] != "MOT" or len(parts) != 4:
            print(f"  [skip] non-MOT or malformed message from 00_base: <{msg}>")
            continue

        target, spark, flipsky = parts[1], parts[2], parts[3]
        sw = swerves.get(target)
        if sw is None:
            skipped_no_target += 1
            continue

        sw.send_message(parts[0], target, spark, flipsky)
        forwarded += 1

        now = time.time()
        if now - last_log_time >= LOG_INTERVAL_S:
            last_log_time = now
            connected = ",".join(sorted(swerves.keys()))
            print(f"[{forwarded:6d}] last: <MOT,{target},{spark:>4},{flipsky:>4}>  "
                  f"connected: {connected}  skipped: {skipped_no_target}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", help="COM port for 00_base (e.g. COM5)")
    parser.add_argument("--scan", action="store_true",
                        help="Scan all ports and exit")
    args = parser.parse_args()

    if args.scan:
        scan_ports()
        return

    print("Press Ctrl+C to stop.\n")

    while True:
        base, base_port, swerves = discover_and_open(args.base)
        connected = ", ".join(f"{n}@{c.port}" for n, c in swerves.items())
        print(f"\nBridge running: 00_base@{base_port}  →  {connected}\n")
        try:
            run_bridge(base, swerves)
        except Exception as exc:
            print(f"\n[DISCONNECT] {exc}")
            safe_close(base)
            for sw in swerves.values():
                safe_close(sw)
            print("Re-discovering...\n")
            time.sleep(RETRY_DELAY_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBridge stopped.")
