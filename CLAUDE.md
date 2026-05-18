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
- **SPARK MAX mode-broadcast byte 0 is a per-device-id bitmask.** Frame
  `T02052C80…`'s first data byte = `(1 << device_id)` enables that
  controller to follow setpoints; OR multiple bits together to drive
  multiple controllers in one frame. *Not* an enumerated control-mode
  value. The control mode itself (Position / Velocity / Duty / …) is
  persistent per-device config in REV Hardware Client. Hard-coding the
  byte (e.g. `0x02`) silently fails as soon as device_ids change. Helper:
  `_make_mode_frame()` in
  [warrior_serial/warrior_serial/nudge_sparks.py](warrior_serial/warrior_serial/nudge_sparks.py).
- **If position-discovery times out on a SPARK MAX, suspect Status 2 is
  off.** Status 2 (encoder position) is a per-device periodic frame
  configured in REV Hardware Client (Periodic Frame Periods → Status 2
  Period). If it's set to 0 or huge, the controller still streams Status 0
  (faults / applied output) but you'll never see position. Diagnostic
  counters (`status_0_count`, `status_2_count`) in `nudge_sparks.py` let
  you tell that apart from a dead controller. Fix: in REV, set Status 2 ≤
  ~50 ms, **Burn Flash**, **power-cycle** that controller.
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
- 2026-05-15 — **Never identify a SPARK MAX by `"ACM" in device.name` —
  the swerve Arduinos (Nano ESP32 with TinyUSB CDC) also enumerate as
  `/dev/ttyACM*` and you will pick one up.** On this rig the 7 ACM ports
  break down as: 3 × SPARK MAX (VID `0x0483`, PID `0xa30e`,
  description `'SPARK MAX Motor Controller'`), 3 × Nano ESP32 (VID
  `0x2341`, PID `0x0070`), 1 × Arduino UNO (`00_base`, VID `0x2341`, PID
  `0x0043`). Use VID:PID `0x0483:0xa30e` as the SPARK MAX filter, then
  passively scan each Spark port for ~1 s to read the `device_id` from
  the lower 6 bits of any incoming CAN ID. Helper:
  `_find_spark_by_device_id()` in
  [warrior_serial/test_swerve_module.py](warrior_serial/warrior_serial/test_swerve_module.py).
- 2026-05-17 — **Mode-broadcast byte 0 is a device-id bitmask, not a
  control-mode enum.** All three SPARK MAXes were originally at CAN ID 1,
  so the old hard-coded `"T02052C80802" + "00"*7` (byte 0 = `0x02` = bit
  1) worked by coincidence. After renumbering to 2/3/4 it stopped working
  on all controllers; switching to `0x08` (bit 3) only fixed device 3.
  Caught by `sniff_usb.py` showing REV sending `0x04` / `0x08` / `0x10`
  for devices 2 / 3 / 4 respectively. Fix is to build the frame
  dynamically: `bitmask = (1 << device_id)` (or OR together for several).
  See `_make_mode_frame()` in `nudge_sparks.py`.
- 2026-05-17 — **Position-discovery silently times out when Status 2 is
  disabled.** During the same session, after re-IDing controllers in REV,
  devices 3 and 4 sent Status 0 at the usual rate but exactly zero
  Status 2 frames over 5 s — REV's Status 2 Period was either unset or
  not yet persisted. Resolved by Burn Flash + power-cycle. Added
  `status_0_count` / `status_2_count` / `other_frame_count` counters to
  `nudge_sparks.py` so a quiet controller can be distinguished from one
  that's just missing position broadcast.
- 2026-05-17 — **Collapsed `test_swerve_module` from three nodes to one
  coordinator.** Originally one node per SPARK MAX (spawned 3× by the
  launch file, with per-wheel `~/limit_status` Int8 pub/sub coordinating
  stuck-stops). Replaced with a single `swerve_coordinator` node that
  owns all 3 SPARK MAX `SparkSession`s in-process. Drives one shared
  `cmd` plus per-wheel `offset`, so A-button "all wheels" is just
  `cmd += rate * dt` while X/Y/B calibration modes adjust just the
  selected wheel's offset (offsets persist across mode switches). Lag is
  capped at ±10 motor rotations on `cmd` in ALL mode only. Removed: the
  3-way `limit_status` topics, the per-port `TimerAction` stagger in the
  launch file, the `peer_device_ids` parameter, and the old per-wheel
  1.5-rot stuck threshold.

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
