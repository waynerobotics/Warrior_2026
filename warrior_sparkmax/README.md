# warrior_sparkmax

ROS 2 Jazzy C++ package — USB CDC driver for REV SPARK MAX motor controllers.

Controls **steering position** on the Warrior 2026 swerve modules using
closed-loop position control over USB. Drive speed (Flipsky) remains on the
Arduino over ASCII serial via `motor_manager`. This package handles the SPARK
MAX side only.

### Control approach — rate-to-position integrator

The joystick is treated as a **steering rate**, not a direct position target.
The `twist_to_spark` node (in `warrior_serial`) integrates that rate into a
running position accumulator and sends the accumulated value as the SPARK MAX
closed-loop position setpoint on every timer tick:

```
Joystick axes[4]  (-1 .. 1)
  → joy_swerve   → Twist.angular.z
  → twist_to_spark (rate integrator)
      position += angular.z × rate_scale × dt
      position clamped to ±max_position
  → SparkCommand.setpoint (absolute rotations)
  → spark_max_driver
  → 12-byte USB position packet
  → SPARK MAX closed-loop PID → motor
```

| Joystick action | Result |
|---|---|
| Held left / right | Wheel moves continuously at up to ±`rate_scale` rot/s |
| Released to center | Rate = 0 → wheel **holds** current angle |
| Hits soft limit | Clamps at `±max_position`, won't wind further |

---

## Background — why a custom driver?

REVLib (the official SPARK MAX library) requires the WPILib HAL and only runs in
an FRC robot environment. On a Linux PC it cannot be used.

The SPARK MAX USB interface is a CDC-ACM virtual serial port (`/dev/ttyACM*`) that
speaks a **fixed 12-byte binary protocol** mirroring the CAN extended-frame format.
This package implements that protocol directly, giving full position control without
any dependency on WPILib or Java.

---

## Package contents

```
warrior_sparkmax/
├── include/warrior_sparkmax/
│   └── spark_protocol.hpp   — 12-byte packet constants + builder (header-only)
├── src/
│   └── spark_max_driver.cpp — ROS 2 node
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## How it works

### Wire protocol

Every message is exactly **12 bytes**, little-endian:

| Bytes | Field | Description |
|-------|-------|-------------|
| 0–3 | Command ID | 32-bit word encoding a CAN 29-bit extended ID |
| 4–7 | Data word 0 | Payload (e.g. setpoint as `float32`) |
| 8–11 | Data word 1 | Secondary payload or zero |

The Command ID packs these fields:

```
Bits [28:24]  Device Type  = 0x02  (Motor Controller)
Bits [23:16]  Manufacturer = 0x05  (REV Robotics)
Bits [15:12]  API Class              — command group
Bits [11:8]   API Index              — specific command
Bits [7:2]    Device ID (CAN ID)    — 0 on USB
Bits [1:0]    Reserved = 0
```

Relevant API classes/indices implemented in `spark_protocol.hpp`:

| Command | API Class | API Index |
|---------|-----------|-----------|
| Heartbeat | 0x06 | 0x02 |
| Position setpoint (rotations) | 0x02 | 0x02 |

### Heartbeat requirement

The SPARK MAX disables motor output if no heartbeat packet arrives within **~100 ms**.
The node sends a heartbeat on a dedicated background thread every `heartbeat_ms`
milliseconds (default 50 ms), independent of the ROS executor.

### Port discovery

On startup the driver scans `/dev/ttyACM*`, reads USB VID/PID from sysfs
(`/sys/class/tty/ttyACMx/device/idVendor` and `idProduct`), and connects to the
first port matching SPARK MAX (VID `0x0483`, PID `0x5740`). The port is opened
with `O_EXCL` so no other process can share it. If the SPARK MAX is unplugged,
the driver closes the port and retries discovery automatically.

---

## ROS 2 interface

### Subscribed topics

| Topic | Type | Description |
|-------|------|-------------|
| `/spark_cmd` | `warrior_msgs/SparkCommand` | Position setpoint command |

`SparkCommand` fields:
- `string target` — must match the node's `device_name` parameter
- `float32 setpoint` — target position in **rotations**

### Published topics

| Topic | Type | Description |
|-------|------|-------------|
| `/spark_feedback` | `warrior_msgs/SparkFeedback` | Motor telemetry |

> **Note:** Telemetry is currently stubbed (zeros). Parsing periodic status frames
> from the SPARK MAX USB stream is not yet implemented.

### Parameters — `spark_max_driver` (this package)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device_name` | string | `"spark_max"` | Logical name, matched against `SparkCommand.target` |
| `device_id` | int | `1` | CAN Device ID set in REV Hardware Client (1–62) |
| `heartbeat_ms` | int | `50` | Heartbeat interval in milliseconds (keep < 80) |
| `discovery_retry_period_s` | double | `2.0` | Seconds between port scan retries |

### Parameters — `twist_to_spark` (warrior_serial)

These control how the joystick maps to steering motion. Tune these on the robot.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rate_scale` | double | `2.0` | Steering speed: rotations/sec at full joystick deflection |
| `max_position` | double | `5.0` | Soft clamp: maximum position in either direction (±rotations) |
| `update_rate_hz` | double | `20.0` | Integration + publish frequency in Hz |
| `targets` | string list | `["02_spark", "03_spark", "04_spark"]` | SPARK MAX device names to command |

**Tuning guide:**
- Increase `rate_scale` if the steering feels too slow (stick barely moves the wheel).
- Decrease `rate_scale` if steering is too twitchy or overshoots.
- Set `max_position` to match the physical steering range of your swerve module — if the wheel can physically turn ±3 rotations before hitting a stop, set this to `3.0`.
- Lower `update_rate_hz` only if USB bandwidth is a concern; 20 Hz is fine for steering.

---

## Building

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select warrior_msgs warrior_sparkmax --symlink-install
source install/setup.bash
```

---

## Running

### As part of the full swerve teleop launch

```bash
ros2 launch warrior_bringup warrior_swerve_teleop.launch.py
```

This starts one `spark_max_driver` node per swerve module (device IDs 2, 3, 4)
and the `twist_to_spark` rate integrator node.

Override steering tuning parameters at the command line:

```bash
ros2 launch warrior_bringup warrior_swerve_teleop.launch.py \
    rate_scale:=2.0    \
    max_position:=5.0
```

### Standalone (single SPARK MAX)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run warrior_sparkmax spark_max_driver \
    --ros-args \
    -p device_name:=02_spark \
    -p device_id:=2 \
    -p heartbeat_ms:=50
```

### Sending a manual position command

```bash
# Move to 3.0 rotations
ros2 topic pub --once /spark_cmd warrior_msgs/msg/SparkCommand \
    '{target: "02_spark", setpoint: 3.0}'
```

### Checking connectivity

```bash
# Should show spark_max_driver subscribed
ros2 topic info /spark_cmd

# Confirm SPARK MAX USB port is visible
lsusb | grep 0483

# Check which ttyACM device it claimed
ls -la /dev/ttyACM*
```

---

## SPARK MAX setup (REV Hardware Client)

Before using this driver the SPARK MAX must be configured with **REV Hardware Client**
(Windows or Linux via Wine):

1. Set the **CAN ID** (1–62) — must match the `device_id` parameter.
2. Set the **control mode** to **Position** (closed-loop).
3. Configure the **PID gains** (kP, kI, kD) for the steering mechanism.
4. Configure the **encoder** (built-in hall sensor or external).
5. Set **soft limits** appropriate for your steering range (e.g. ±3 rotations).

The driver does not write configuration — it only sends setpoint packets and
reads periodic status frames.

---

## Verifying the wire protocol

The API constants in `spark_protocol.hpp` were derived from the SPARK MAX CAN
protocol documentation and community research. Verify them before trusting on
real hardware:

```bash
# Load the USB monitor kernel module
sudo modprobe usbmon

# Open Wireshark, select the usbmonX interface matching the SPARK MAX bus,
# and apply filter:  usb.src == "X.Y.0"  (replace X.Y with your bus.device)
# Then send a command from REV Hardware Client and compare the 12 raw bytes.
wireshark
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Node starts but motor doesn't move | Missing or wrong CAN ID in REV Hardware Client | Match `device_id` param to configured CAN ID |
| Port not found, retrying | SPARK MAX not plugged in, or VID/PID mismatch | `lsusb \| grep 0483`; re-flash firmware if needed |
| Motor disables after ~100 ms | Heartbeat not reaching controller | Check `heartbeat_ms` < 80; verify port stays open |
| Another process owns the port | Conflict with REV Hardware Client or `brltty` | Kill conflicting process; `sudo systemctl stop brltty` |
| `O_EXCL` open fails | Port already held exclusively | Same as above |

### Common udev conflict — `brltty`

On Ubuntu, `brltty` (Braille TTY daemon) claims CDC-ACM devices automatically.
Disable it if it steals the SPARK MAX port:

```bash
sudo systemctl stop brltty
sudo systemctl disable brltty
```
