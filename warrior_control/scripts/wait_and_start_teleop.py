#!/usr/bin/env python3
"""Wait for a controller to become ACTIVE, then start teleop_twist_keyboard.

This helper avoids launch-time race conditions by polling the controller
manager until `swerve_drive_controller` reports the `active` state, then
attempts to open teleop in a new terminal. If `gnome-terminal` is not
available it falls back to running `ros2 run` directly.
"""
import time
import subprocess
import sys
import os

CONTROLLER_NAME = os.environ.get("SWERVE_CONTROLLER_NAME", "swerve_drive_controller")
POLL_INTERVAL = float(os.environ.get("SWERVE_POLL_INTERVAL", "0.5"))


def controller_is_active(name: str) -> bool:
    try:
        proc = subprocess.run(["ros2", "control", "list_controllers"], capture_output=True, text=True, timeout=5)
        out = proc.stdout + proc.stderr
        # The CLI prints controller info including state. We look for the
        # controller name and the word 'active' on the same output.
        for line in out.splitlines():
            if name in line and "active" in line:
                return True
    except Exception as e:
        print("error calling ros2 control list_controllers:", e, file=sys.stderr)
    return False


def start_teleop():
    # Try to open teleop in a new gnome-terminal (so keyboard input works),
    # otherwise fallback to running it in the background.
    try:
        cmd = ["gnome-terminal", "--", "ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"]
        print("Starting teleop with:", " ".join(cmd))
        subprocess.Popen(cmd)
    except Exception:
        try:
            cmd = ["ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"]
            print("gnome-terminal not available, running:", " ".join(cmd))
            subprocess.Popen(cmd)
        except Exception as e:
            print("Failed to start teleop:", e, file=sys.stderr)


def main():
    print(f"wait_and_start_teleop: waiting for controller '{CONTROLLER_NAME}' to become active")
    while True:
        if controller_is_active(CONTROLLER_NAME):
            print(f"Controller {CONTROLLER_NAME} is active — starting teleop")
            start_teleop()
            return
        time.sleep(POLL_INTERVAL)


if __name__ == "__main__":
    main()
