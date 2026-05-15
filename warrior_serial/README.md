# warrior_serial

ROS 2 bridge between the joystick and the Warrior swerve hardware. Drive
velocity goes to Arduinos over USB-serial; steering position goes to SPARK
MAX motor controllers over USB-SLCAN.

## Architecture

```
 ┌──────────┐  /joy    ┌────────────┐  /cmd_vel   ┌────────────────┐
 │ joy_node │─────────▶│ joy_swerve │────────────▶│ motor_manager  │
 └──────────┘          └────────────┘             └──┬──────────┬──┘
                                                    │          │
                                       <MOT,…> over │          │ /spark_cmd
                                       USB-serial   │          ▼
                                                    │     ┌────────────┐
                                                    │     │spark_driver│
                                                    │     └─────┬──────┘
                                                    │           │ SLCAN
                                                    ▼           ▼
                                          02/03/04_swerve  02/03/04_spark
                                          (Arduino, PWM)   (SPARK MAX, position)
```

`00_base` is also discovered over USB-serial and republishes any `<MOT,…>`
frames it streams onto `/motor_cmd` (legacy path; still wired).

## Inputs / outputs / mappings

**Joystick → /cmd_vel (Twist):**

| Joy axis | Twist field | Meaning |
| --- | --- | --- |
| `axes[0]` left X | `linear.x` | velocity X |
| `axes[1]` left Y | `linear.y` | velocity Y |
| `axes[3]` right Y | `angular.z` | steering rate input, −1..+1 |

**Joystick buttons → active swerve target** (held in `motor_manager`):

| Button | Active target(s) |
| --- | --- |
| A `buttons[0]` | ALL (default at startup) |
| X `buttons[2]` | 02 only |
| B `buttons[1]` | 03 only |
| Y `buttons[3]` | 04 only |

**Topics:** `/joy`, `/cmd_vel`, `/motor_cmd`, `/spark_cmd`, `/spark_feedback`

**Devices:**

| Logical name | Transport | Discovered by |
| --- | --- | --- |
| `00_base`, `02_swerve`, `03_swerve`, `04_swerve` | USB-serial ASCII | `<WHO>` → `<NAME,…>` handshake |
| `02_spark`, `03_spark`, `04_spark` | USB SLCAN | CAN device_id (lower 6 bits of any incoming frame) |

**Steering math:** right stick Y in [−1, +1] is a rate; integrated to a
position in [0, 2π] rad (capped, no wrap in test mode); sent to the SPARK as
`(pos_rad / 2π) × 42` motor rotations.

## Build & run

```bash
colcon build --packages-select warrior_msgs warrior_serial warrior_joy
source install/setup.bash

# Phase 0 test (joystick → ONE SPARK MAX, no Arduinos):
ros2 launch warrior_serial test_swerve_module.launch.py

# Full stack (drive + steering):
ros2 launch warrior_serial warrior_drivers.launch.py
ros2 launch warrior_joy joy_swerve.launch.py
```

USB auto-connect troubleshooting lives in [CLAUDE.md](../CLAUDE.md).
