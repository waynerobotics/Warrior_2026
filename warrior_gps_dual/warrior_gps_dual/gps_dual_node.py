#!/usr/bin/env python3
"""
warrior_gps_dual — publish two independent u-blox NMEA GPS receivers as
separate sensor_msgs/NavSatFix topics for downstream EKF fusion.

    receiver 1 (port1) -> /gps1/fix  (+ /gps1/nmea)
    receiver 2 (port2) -> /gps2/fix  (+ /gps2/nmea)

The receivers are NOT combined here. The navigation EKF
(navsat_transform / robot_localization, in warrior_localization) fuses them.

One reader thread per serial port keeps the latest GPGGA fix under a lock; a
ROS timer reads both and publishes at publish_rate. Hardware: vfan USB GPS,
u-blox UBX-G70xx, NMEA-0183 @ 9600 baud, 1 Hz. Enable SBAS/WAAS once per
receiver with `ros2 run warrior_gps_dual enable_waas <port>`.
"""
import threading
import time

import rclpy
from rclpy.node import Node

import serial
import serial.tools.list_ports

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String


# u-blox AG USB vendor ID — the vfan / UBX-G70xx receivers enumerate under this.
UBLOX_VID = 0x1546


def discover_gps_ports():
    """Return likely u-blox GPS serial ports, sorted for deterministic
    gps1/gps2 assignment.

    The robot has many /dev/ttyACM* devices (swerve Arduinos + SPARK MAXes), so
    pinning a fixed number is unreliable. Match the receivers by USB VID first,
    with a description-keyword fallback for adapters that don't expose the VID.
    """
    keywords = ("u-blox", "ublox", "gnss", "gps")
    found = []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if getattr(p, "vid", None) == UBLOX_VID or any(k in desc for k in keywords):
            found.append(p.device)
    return sorted(found)


# ── NMEA parsing ──────────────────────────────────────────────────────────────

def parse_ddmm(value, hemisphere):
    """NMEA ddmm.mmmm + hemisphere -> signed decimal degrees (or None)."""
    if not value:
        return None
    dot = value.index(".")
    deg = float(value[:dot - 2])
    minutes = float(value[dot - 2:])
    dec = deg + minutes / 60.0
    return -dec if hemisphere in ("S", "W") else dec


def nmea_checksum_ok(line):
    """Validate the trailing *XX XOR checksum of an NMEA sentence."""
    if "*" not in line or not line.startswith("$"):
        return False
    body, _, cks = line[1:].partition("*")
    try:
        want = int(cks[:2], 16)
    except ValueError:
        return False
    got = 0
    for ch in body:
        got ^= ord(ch)
    return got == want


def parse_gpgga(fields):
    """Parse a GPGGA/GNGGA field list to a fix dict, or None if no valid fix."""
    try:
        lat = parse_ddmm(fields[2], fields[3])
        lon = parse_ddmm(fields[4], fields[5])
        fix = int(fields[6])              # 0=none 1=GPS 2=DGPS/SBAS 4=RTK 5=float
        sats = int(fields[7]) if fields[7] else 0
        hdop = float(fields[8]) if fields[8] else 99.0
        alt_msl = float(fields[9]) if fields[9] else 0.0      # orthometric height
        geoid = float(fields[11]) if len(fields) > 11 and fields[11] else 0.0
        if lat is None or lon is None or fix == 0:
            return None
        return {
            "lat": lat, "lon": lon, "fix": fix, "sats": sats, "hdop": hdop,
            # NavSatFix wants height above the WGS-84 ellipsoid:
            #   ellipsoid = orthometric (MSL, field 9) + geoid separation (field 11)
            "alt": alt_msl + geoid,
            "ts": time.time(),
        }
    except (ValueError, IndexError):
        return None


# ── Serial reader thread ──────────────────────────────────────────────────────

class GPSReader(threading.Thread):
    """One serial receiver. Reads GGA sentences, keeps the latest fix + raw line
    under a lock, and reconnects on serial error (port unplug / late enumerate)."""

    def __init__(self, node, name, port, baud):
        super().__init__(daemon=True)
        self._node = node
        self.name_ = name
        self.port = port
        self.baud = baud
        self._latest = None
        self._raw = None
        self._lock = threading.Lock()
        self.running = True

    def snapshot(self):
        with self._lock:
            return self._latest, self._raw

    def run(self):
        log = self._node.get_logger()
        while self.running and rclpy.ok():
            try:
                with serial.Serial(self.port, self.baud, timeout=2) as ser:
                    log.info(f"[{self.name_}] open {self.port} @ {self.baud} baud")
                    while self.running and rclpy.ok():
                        raw = ser.readline()
                        if not raw:
                            continue
                        line = raw.decode("ascii", errors="replace").strip()
                        # accept GPGGA or GNGGA (talker GP/GN), reject corrupt lines
                        if len(line) < 6 or line[3:6] != "GGA":
                            continue
                        if not nmea_checksum_ok(line):
                            continue
                        pos = parse_gpgga(line.split(","))
                        with self._lock:
                            self._raw = line
                            if pos:
                                self._latest = pos
            except serial.SerialException as e:
                log.warning(f"[{self.name_}] {self.port}: {e} — retrying in 2 s")
                time.sleep(2.0)
            except Exception as e:  # noqa: BLE001 — keep the thread alive
                log.error(f"[{self.name_}] unexpected: {e} — retrying in 2 s")
                time.sleep(2.0)


# ── Node ──────────────────────────────────────────────────────────────────────

# GPGGA fix quality (field 6) -> NavSatStatus
FIX_STATUS = {
    0: NavSatStatus.STATUS_NO_FIX,
    1: NavSatStatus.STATUS_FIX,
    2: NavSatStatus.STATUS_SBAS_FIX,   # DGPS / WAAS
    4: NavSatStatus.STATUS_GBAS_FIX,   # RTK fixed
    5: NavSatStatus.STATUS_GBAS_FIX,   # RTK float
}

BASE_ACCURACY_M = 2.5   # ~DGPS 1-sigma per HDOP unit (covariance approximation)


class GpsDualNode(Node):
    def __init__(self):
        super().__init__("gps_dual_node")

        # Ports default to empty -> auto-discover by USB VID. Set a param only to
        # pin a specific device (e.g. /dev/serial/by-id/...).
        self.declare_parameter("port1", "")
        self.declare_parameter("port2", "")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("frame_id1", "gps1")
        self.declare_parameter("frame_id2", "gps2")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("publish_nmea", True)

        port1 = self.get_parameter("port1").value
        port2 = self.get_parameter("port2").value
        self.baud = int(self.get_parameter("baud").value)
        self.frame1 = self.get_parameter("frame_id1").value
        self.frame2 = self.get_parameter("frame_id2").value
        rate = float(self.get_parameter("publish_rate").value)
        self.publish_nmea = bool(self.get_parameter("publish_nmea").value)

        # Fill any unset port from auto-discovery (skipping ones already pinned).
        if not port1 or not port2:
            discovered = discover_gps_ports()
            self.get_logger().info(f"discovered u-blox GPS ports: {discovered or 'none'}")
            if not port1:
                port1 = next((d for d in discovered if d != port2), "")
            if not port2:
                port2 = next((d for d in discovered if d != port1), "")

        # Default (reliable) QoS so robot_localization / navsat_transform match.
        self.pub_fix1 = self.create_publisher(NavSatFix, "/gps1/fix", 10)
        self.pub_fix2 = self.create_publisher(NavSatFix, "/gps2/fix", 10)
        self.pub_nmea1 = self.pub_nmea2 = None
        if self.publish_nmea:
            self.pub_nmea1 = self.create_publisher(String, "/gps1/nmea", 10)
            self.pub_nmea2 = self.create_publisher(String, "/gps2/nmea", 10)

        self._last_ts = {"gps1": 0.0, "gps2": 0.0}
        self.r1 = self._start_reader("gps1", port1)
        self.r2 = self._start_reader("gps2", port2)

        self.timer = self.create_timer(1.0 / max(rate, 0.1), self._on_timer)
        self.get_logger().info(
            f"gps_dual_node: {port1 or '(none)'} -> /gps1/fix, "
            f"{port2 or '(none)'} -> /gps2/fix @ {rate} Hz")

    def _start_reader(self, name, port):
        if not port:
            self.get_logger().warn(
                f"[{name}] no port resolved — plug in a u-blox GPS or set the "
                f"'{'port1' if name == 'gps1' else 'port2'}' param")
            return None
        reader = GPSReader(self, name, port, self.baud)
        reader.start()
        return reader

    def _on_timer(self):
        if self.r1:
            self._publish(self.r1, self.frame1, self.pub_fix1, self.pub_nmea1)
        if self.r2:
            self._publish(self.r2, self.frame2, self.pub_fix2, self.pub_nmea2)

    def _publish(self, reader, frame_id, fix_pub, nmea_pub):
        pos, raw = reader.snapshot()
        if pos is None:
            return
        # Only publish a fix once — skip if nothing new arrived since last tick.
        if pos["ts"] <= self._last_ts[reader.name_]:
            return
        self._last_ts[reader.name_] = pos["ts"]

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id

        msg.status.status = FIX_STATUS.get(pos["fix"], NavSatStatus.STATUS_FIX)
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = pos["lat"]
        msg.longitude = pos["lon"]
        msg.altitude = pos["alt"]

        sigma = pos["hdop"] * BASE_ACCURACY_M
        cov = sigma * sigma
        msg.position_covariance = [cov, 0.0, 0.0,
                                   0.0, cov, 0.0,
                                   0.0, 0.0, cov * 2.25]   # VDOP ~1.5x HDOP
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        fix_pub.publish(msg)

        if nmea_pub is not None and raw is not None:
            nmea_pub.publish(String(data=raw))

    def destroy_node(self):
        for reader in (self.r1, self.r2):
            if reader:
                reader.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsDualNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
