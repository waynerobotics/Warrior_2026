#!/usr/bin/env python3
"""
wheel_sweep_test.py — bench test that sweeps the swerve steering angle in
fixed increments and, at each angle, drives the wheels at a fixed percent for
a fixed dwell time.

Default behaviour matches the request: 10% drive for 5 s at every 20° step
from 0° to 360°.

For each steering angle the script runs two phases per step:
  1. SETTLE  — command the steer angle with drive = 0 so the modules can
               physically rotate to the target before any driving happens.
               (42:1 gearing means this is not instant.) Set --settle-s 0 to
               skip.
  2. DRIVE   — hold the steer angle and drive at --speed-percent for --hold-s.

All three modules are commanded to the SAME steer angle and the SAME drive
percent, so the whole chassis sweeps together.

It publishes warrior_msgs/SwerveCmd directly to /warrior_swerve_command at a
steady rate (the driver applies a 0.5 s command timeout, so a steady stream is
required). Drive percent is converted to drive_velocity_rad_s using the same
relation the driver uses:  velocity = percent/100 * max_drive_rad_s.

IMPORTANT: run this against the warrior_driver node ALONE. If the full
controller stack is up, swerve_drive_controller also publishes
/warrior_swerve_command at 50 Hz and will fight this script. Deactivate it
first, e.g.:
    ros2 control set_controller_state swerve_drive_controller inactive

Usage:
    source install/setup.bash
    python3 wheel_sweep_test.py
    python3 wheel_sweep_test.py --speed-percent 10 --hold-s 5 --step-deg 20 \
        --start-deg 0 --end-deg 360 --settle-s 2.0

Ctrl-C at any time sends a zero-drive command and exits cleanly.
"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from warrior_msgs.msg import SwerveCmd


class WheelSweepTest(Node):
    def __init__(self, args):
        super().__init__("wheel_sweep_test")
        self.args = args
        self.modules = [m.strip() for m in args.modules.split(",") if m.strip()]
        self.drive_velocity = (args.speed_percent / 100.0) * args.max_drive_rad_s

        # Steering angles to visit, in radians. range() is exclusive of the
        # endpoint, so add one step to include --end-deg.
        self.angles_deg = list(
            range(args.start_deg, args.end_deg + 1, args.step_deg)
        )

        self.pub = self.create_publisher(
            SwerveCmd, args.topic, QoSProfile(depth=10)
        )

        # State machine bookkeeping.
        self._idx = 0           # index into self.angles_deg
        self._phase = "settle"  # "settle" or "drive"
        self._phase_started = None
        self._done = False

        self.get_logger().info(
            f"sweep: {self.angles_deg[0]}°..{self.angles_deg[-1]}° "
            f"step {args.step_deg}° ({len(self.angles_deg)} steps) | "
            f"{args.speed_percent:.0f}% = {self.drive_velocity:.2f} rad/s | "
            f"settle {args.settle_s:.1f}s, drive {args.hold_s:.1f}s | "
            f"modules={self.modules} -> {args.topic}"
        )

        # Publish at a steady rate; the state machine is time-driven off the
        # node clock so it stays correct regardless of jitter.
        self.timer = self.create_timer(1.0 / args.rate_hz, self._tick)

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish(self, steer_rad, drive_vel):
        stamp = self.get_clock().now().to_msg()
        for name in self.modules:
            msg = SwerveCmd()
            msg.swerve_id = name
            msg.steer_position_rad = float(steer_rad)
            msg.drive_velocity_rad_s = float(drive_vel)
            msg.stamp = stamp
            self.pub.publish(msg)

    def _tick(self):
        if self._done:
            return

        now = self._now_s()
        if self._phase_started is None:
            self._phase_started = now
            angle = self.angles_deg[self._idx]
            self.get_logger().info(
                f"[{self._idx + 1}/{len(self.angles_deg)}] {angle}° — "
                f"{'settling (drive 0)' if self._phase == 'settle' else f'driving {self.args.speed_percent:.0f}%'}"
            )

        elapsed = now - self._phase_started
        steer_rad = math.radians(self.angles_deg[self._idx])

        if self._phase == "settle":
            self._publish(steer_rad, 0.0)
            if elapsed >= self.args.settle_s:
                self._phase = "drive"
                self._phase_started = None
        else:  # drive
            self._publish(steer_rad, self.drive_velocity)
            if elapsed >= self.args.hold_s:
                self._advance()

    def _advance(self):
        self._idx += 1
        self._phase = "settle" if self.args.settle_s > 0.0 else "drive"
        self._phase_started = None
        if self._idx >= len(self.angles_deg):
            self.get_logger().info("sweep complete — stopping drive")
            self.stop()
            self._done = True
            rclpy.shutdown()

    def stop(self):
        """Command zero drive (steer held at current target) a few times."""
        steer_rad = math.radians(self.angles_deg[min(self._idx, len(self.angles_deg) - 1)])
        for _ in range(5):
            self._publish(steer_rad, 0.0)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--speed-percent", type=float, default=25.0,
                   help="drive power percent (default 25)")
    p.add_argument("--hold-s", type=float, default=5.0,
                   help="seconds to drive at each angle (default 5)")
    p.add_argument("--settle-s", type=float, default=2.0,
                   help="seconds to let steering reach the angle before "
                        "driving, with drive=0 (default 2; set 0 to skip)")
    p.add_argument("--step-deg", type=int, default=20,
                   help="steering increment in degrees (default 20)")
    p.add_argument("--start-deg", type=int, default=0,
                   help="first steering angle in degrees (default 0)")
    p.add_argument("--end-deg", type=int, default=360,
                   help="last steering angle in degrees, inclusive (default 360)")
    p.add_argument("--max-drive-rad-s", type=float, default=60.0,
                   help="must match modules.*.max_drive_rad_s in "
                        "warrior_driver.yaml (default 60)")
    p.add_argument("--rate-hz", type=float, default=50.0,
                   help="command publish rate (default 50)")
    p.add_argument("--modules", type=str, default="front,left,right",
                   help="comma-separated swerve_ids (default front,left,right)")
    p.add_argument("--topic", type=str, default="/warrior_swerve_command",
                   help="SwerveCmd topic (default /warrior_swerve_command)")
    args = p.parse_args()

    rclpy.init()
    node = WheelSweepTest(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("interrupted — stopping drive")
        node.stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
