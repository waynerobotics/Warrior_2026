# Warrior_2026

ROS 2 workspace for the Warrior swerve robot. Drive velocity is handled by
three swerve Arduinos addressed by name over USB-serial; steering position
is handled by three REV SPARK MAX motor controllers over USB SLCAN. A
joystick → `/cmd_vel` (Twist) → `motor_manager` pipeline fans commands to
both transports. See [warrior_serial/README.md](warrior_serial/README.md)
for the topic-level diagram.

## USB auto-connect — known issues and rules

> When you find a new USB auto-connect issue, append it here with the date
> (YYYY-MM-DD) and the fix. The history is more valuable than tidy prose.

### Rules (always apply)

- **Never hardcode a port path.** `/dev/ttyACM*` numbering changes across
  reboots and replugs. Discover devices, don't pin them.
- **Arduinos identify by handshake.** Send `<WHO>`, wait for `<NAME,…>`. The
  helper is `query_device_name()` in
  [warrior_serial/warrior_serial/serial_protocol.py](warrior_serial/warrior_serial/serial_protocol.py).
- **SPARK MAXes have no software handshake.** They stream status frames
  unprompted. Identify by the lower 6 bits of any incoming CAN ID
  (`can_id & 0x3F` == device_id). Set the device_id once in REV Hardware
  Client and treat it as the only source of truth (logical name → device_id
  table lives in `spark_driver` parameters).
- **Open serial ports exclusively.** pyserial `exclusive=True` + Linux
  `TIOCEXCL`. Without this, two nodes will fight over the same port and
  both will see corrupted bytes. See
  [serial_protocol.py:72-86](warrior_serial/warrior_serial/serial_protocol.py#L72-L86).
- **Wait after opening an Arduino.** DTR toggles on open and triggers a
  reset; the sketch is unresponsive for ~2 s. `OPEN_RESET_DELAY_S = 2.0`.
- **Skip `/dev/ttyS*` during scans.** Those are hardware UARTs, not our
  devices, and opens are slow.
- **Filter probe replies strictly.** Devices stream `<MOT,…>` or SLCAN
  status frames continuously. A discovery probe must match the *specific*
  expected reply (e.g. `<NAME,…>`) and discard everything else — otherwise
  you get false matches on background traffic.
- **On any read/write error, drop the handle and let discovery reclaim it.**
  Do not try to recover the existing connection in-place. The discovery
  thread runs every `discovery_retry_period_s` (default 2.0 s) and will
  pick the port back up after the kernel releases it.

### History

- 2026-05-15 — Initial set of rules above, captured during the
  position-driven swerve teleop rearchitecture
  (branch `position-driven-swerve-tele-op`). SPARK MAX support added via
  USB SLCAN; identification scheme switched from `<NAME>` handshake (used
  for Arduinos) to passive CAN device_id parsing (used for Sparks).

## Repository layout

- [warrior_serial/](warrior_serial/) — serial + SLCAN ROS 2 nodes
  (`warrior_base_driver`, `motor_manager`, `spark_driver`,
  `test_swerve_module`).
- [warrior_joy/](warrior_joy/) — joystick → Twist nodes (`joy_swerve`,
  `joy_2stick`).
- [warrior_msgs/](warrior_msgs/) — `MotorCommand`, `SparkCommand`,
  `SparkFeedback`.
- Other packages (`warrior_control`, `warrior_drive`, `warrior_navigation`,
  …) are not part of the position-driven teleop pipeline and are
  documented in their own READMEs.
