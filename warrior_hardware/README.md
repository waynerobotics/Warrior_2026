# warrior_hardware

The real-robot hardware stack: ros2_control SystemInterface plugin +
single C++ node that owns every USB connection. Steering position goes
to REV SPARK MAX motor controllers over USB-SLCAN; drive velocity goes
to swerve Arduinos as ASCII frames over USB-serial.

See [TEST_PLAN.md](TEST_PLAN.md) for the bring-up walkthrough and
[../CLAUDE.md](../CLAUDE.md) for USB auto-connect rules & known issues.

## Architecture

```
                                                        ┌────────────────────┐
 /cmd_vel ─▶ swerve_drive_controller ─▶ ros2_control ─▶ │ SwerveTopicBridge  │
                                                        │ (warrior_system    │
                                                        │  SystemInterface)  │
                                                        └─────────┬──────────┘
                                                                  │
                                            /warrior_swerve_command (3× SwerveCmd / tick)
                                                                  │
                                                                  ▼
                                                  ┌──────────────────────────────┐
                                                  │ warrior_hardware_manager     │
                                                  │ owns every USB connection    │
                                                  └──┬──────────────────────┬────┘
                                                     │                      │
                                       <DRV,name,pct>│                      │SPARK MAX SLCAN setpoints
                                       USB-serial    │                      │USB-CDC, one port per controller
                                                     ▼                      ▼
                                          02/03/04_swerve              02/03/04_spark
                                          (Arduino Nano ESP32,         (REV SPARK MAX,
                                           PWM to Flipsky ESC)          position closed-loop)
                                                     │                      │
                                                     │                      │ Status 0 / 2 frames
                                                     ▼                      ▼
                                                  ┌──────────────────────────────┐
                                                  │ warrior_hardware_manager     │
                                                  │ publishes /warrior_swerve_state │
                                                  └──────────────────────────────┘
```

## Packages

| Package | Language | Role |
|---|---|---|
| [warrior_hardware_manager](warrior_hardware_manager/) | C++ | One node, one process. Owns USB discovery + reconnect, fans `SwerveCmd` to drive Arduinos and SPARK MAXes, drains feedback. |
| [warrior_system](warrior_system/) | C++ | ros2_control `SystemInterface` plugin (`warrior::system::SwerveTopicBridge`). Translates joint-level command/state interfaces to/from `/warrior_swerve_command` & `/warrior_swerve_state`. |
| [warrior_serial](warrior_serial/) | Python | Legacy + reference. The drive bridge is dead, but `nudge_sparks.py` and `sniff_usb.py` are the authoritative SPARK MAX SLCAN reference until the C++ port lands. |

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/warrior_swerve_command` | `warrior_msgs/SwerveCmd` | bridge → manager | One message per module per controller tick. `swerve_id ∈ {front,left,right}`, `steer_position_rad`, `drive_velocity_rad_s`. |
| `/warrior_swerve_state` | `warrior_msgs/SwerveState` | manager → bridge | One per module at controller rate. Drive velocity is the **commanded** value echoed back (open-loop, no encoder); steer position/velocity come from SPARK MAX Status 2 / Status 1. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | manager → ✶ | Per-module status + per-transport (Arduino, SLCAN) up/down. |

## Devices

| Logical name | Transport | Discovered by |
|---|---|---|
| `02_swerve`, `03_swerve`, `04_swerve` | USB-serial ASCII | `<WHO>` → `<NAME,…>` handshake |
| `02_spark`, `03_spark`, `04_spark` | USB-SLCAN, **one port per controller** | VID:PID `0x0483:0xa30e` filter, then passive scan of the low 6 bits of any incoming CAN ID |

Each SPARK MAX is its own SLCAN-over-USB endpoint — there is no shared
CAN bus / external adapter. Logical-name ↔ `device_id` mapping lives in
[warrior_hardware_manager/config/hardware_manager.yaml](warrior_hardware_manager/config/hardware_manager.yaml).

## Wire protocol

### Drive Arduinos

ASCII, `\n`-terminated, `<TYPE,...>` framing:

| Direction | Message | Meaning |
|---|---|---|
| host → arduino | `<WHO>` | Identify yourself. Reply: `<NAME,XX_swerve>`. |
| host → arduino | `<DRV,XX_swerve,pct>` | Drive command, `pct` ∈ −100..100. Per-device filter by `name`. Ack: `<ACK,DRV,XX_swerve>`. |
| arduino → host | `<ERR,reason>` | Bad / unsupported frame. |

Watchdog: if a swerve receives no matching `<DRV,…>` for 500 ms it
returns to 0 % (neutral PWM). This means any node producing drive
commands must send to each active target at ≥ 2 Hz.

### SPARK MAX (SLCAN over USB-CDC)

Per-controller USB port speaking standard CANable-style SLCAN ASCII
(`T<8-hex-id><1-hex-dlc><N*2-hex-bytes>\r`). The CRITICAL details
discovered the hard way (see [../CLAUDE.md](../CLAUDE.md) for the full
history):

- **Setpoint frame** — `T<setpoint_id:08X>8<float32_le_rot><float32_le_aux>\r`,
  where `setpoint_id = 0x02050100 | device_id`. (api_class = 0,
  api_index = 4 for set-position.)
- **Mode-bitmask frame** — `T02052C80 80 <(1 << device_id):02X> 00 00 00 00 00 00 00 \r`
  enables that controller to follow setpoints. Byte 0 is a **bitmask**,
  not a control-mode enum.
- **Enable frame** — `T000502C0101\r`.
- **All three must be sent on every tx tick** (~50 Hz). Drop the enable
  + mode and the controller silently stays at 0 % output even with
  setpoints arriving.
- **Status 0** (api_cls `0x2E`, api_idx `0`): applied output (int16
  signed scaled by 32768) + faults (uint16) at payload offsets 0..3.
- **Status 2** (api_cls `0x2E`, api_idx `2`): position float32 at
  payload offset **4** (not 0).

The byte-exact authoritative implementation is
[warrior_serial/warrior_serial/nudge_sparks.py](warrior_serial/warrior_serial/nudge_sparks.py).
When porting, follow that file literally; the values came from
`sniff_usb.py` recordings of REV Hardware Client and any deviation
silently misbehaves.

## Build & run

```bash
cd ~/ros2_ws
colcon build --packages-up-to warrior_hardware_manager warrior_system --symlink-install
source install/setup.bash

# Manager alone (no controller):
ros2 launch warrior_hardware_manager hardware_manager.launch.py

# Full real-robot stack (controller_manager + bridge + manager):
ros2 launch warrior_control swerve_drive.real.launch.py
# (in another terminal)
ros2 launch warrior_hardware_manager hardware_manager.launch.py
```

For step-by-step bring-up + pass/fail criteria per phase, see
[TEST_PLAN.md](TEST_PLAN.md).

## SPARK MAX troubleshooting

Quick poke-tools in `warrior_serial/warrior_serial/`:

| Tool | What it does |
|---|---|
| `nudge_sparks.py` | Open every detected SPARK MAX, nudge each by +5 motor rotations, print live position / applied output / faults. Confirms wiring + REV config end-to-end. |
| `sniff_usb.py` | Kernel-level usbmon decoder. Shows every SLCAN byte the host sends and the controller emits. Use to compare your traffic against REV Hardware Client when something doesn't move. |

If a SPARK MAX sends **zero** bytes back even with enable frames being
blasted at it, the most common causes are (in order): 12 V not powered;
Status 0 + Status 2 both set to 0 ms in REV (controller is alive but
silent); USB cable iffy. See [../CLAUDE.md](../CLAUDE.md) §5 for the
Status-2-specific failure mode.
