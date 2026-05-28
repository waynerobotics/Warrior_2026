# Warrior_2026

ROS 2 workspace for the Warrior swerve robot. Drive velocity is handled by
three swerve Arduinos addressed by name over USB-serial; steering position
is handled by three REV SPARK MAX motor controllers over USB-SLCAN. A
`/cmd_vel` (Twist) → `swerve_drive_controller` → `SwerveTopicBridge` →
`warrior_hardware_manager` pipeline fans commands to both transports. See
[warrior_hardware/README.md](warrior_hardware/README.md) for the topic-level
diagram and [warrior_hardware/TEST_PLAN.md](warrior_hardware/TEST_PLAN.md)
for the bring-up walkthrough.

## USB auto-connect — known issues and rules

When you find a new USB auto-connect issue, append it here with the date
(YYYY-MM-DD) and the fix. The history is more valuable than tidy prose.

### Rules (always apply)

1. **Never hardcode a port path.** `/dev/ttyACM*` numbering changes across
   reboots and replugs. Discover devices, don't pin them.
2. **Arduinos identify by handshake.** Send `<WHO>`, wait for `<NAME,…>`.
   Helper: `ArduinoSerialDevice::handshake()` in
   [warrior_hardware/warrior_hardware_manager/src/arduino_serial_device.cpp](warrior_hardware/warrior_hardware_manager/src/arduino_serial_device.cpp).
3. **SPARK MAXes have no software handshake.** They stream status frames
   unprompted. Identify by the lower 6 bits of any incoming CAN ID
   (`can_id & 0x3F == device_id`). Set the device_id once in REV Hardware
   Client and treat it as the only source of truth (logical name →
   device_id table lives in `hardware_manager.yaml`).
4. **SPARK MAX mode-broadcast byte 0 is a per-device-id bitmask.** Frame
   `T02052C80…`'s first data byte = `(1 << device_id)` enables that
   controller to follow setpoints; OR multiple bits together to drive
   multiple controllers in one frame. **Not** an enumerated control-mode
   value. The control mode itself (Position / Velocity / Duty / …) is
   persistent per-device config in REV Hardware Client. Hard-coding the
   byte (e.g. `0x02`) silently fails as soon as device_ids change.
   Helper: `_make_mode_frame()` in
   [warrior_serial/warrior_serial/nudge_sparks.py](warrior_serial/warrior_serial/nudge_sparks.py).
5. **If position-discovery times out on a SPARK MAX, suspect Status 2 is
   off.** Status 2 (encoder position) is a per-device periodic frame
   configured in REV Hardware Client (Periodic Frame Periods → Status 2
   Period). If it's set to 0 or huge, the controller still streams Status
   0 (faults / applied output) but you'll never see position. Diagnostic
   counters (`status_0_count`, `status_2_count`) in `nudge_sparks.py` let
   you tell that apart from a dead controller. Fix: in REV, set Status 2
   ≤ ~50 ms, Burn Flash, power-cycle that controller.
6. **Open serial ports exclusively.** `TIOCEXCL` on Linux. Without this,
   two nodes will fight over the same port and both will see corrupted
   bytes. See
   [arduino_serial_device.cpp:74](warrior_hardware/warrior_hardware_manager/src/arduino_serial_device.cpp#L74)
   (`ioctl(fd, TIOCEXCL)`).
7. **Wait after opening an Arduino.** DTR toggles on open and triggers a
   reset; the sketch is unresponsive for ~2 s. `ARDUINO_RESET_DELAY` in
   [device_registry.cpp:16](warrior_hardware/warrior_hardware_manager/src/device_registry.cpp#L16).
8. **Skip `/dev/ttyS*` during scans.** Those are hardware UARTs, not our
   devices, and opens are slow.
9. **Filter probe replies strictly.** Devices stream `<MOT,…>` or SLCAN
   status frames continuously. A discovery probe must match the specific
   expected reply (e.g. `<NAME,…>`) and discard everything else —
   otherwise you get false matches on background traffic.
10. **On any read/write error, drop the handle and let discovery reclaim
    it.** Do not try to recover the existing connection in-place. The
    discovery thread runs every `discovery_period_s` (default 2.0 s) and
    will pick the port back up after the kernel releases it.

### History

- **2026-05-15** — Initial set of rules above, captured during the
  position-driven swerve teleop rearchitecture (branch
  `position-driven-swerve-tele-op`). SPARK MAX support added via USB
  SLCAN; identification scheme switched from `<NAME>` handshake (used
  for Arduinos) to passive CAN device_id parsing (used for Sparks).
- **2026-05-15** — Never identify a SPARK MAX by "ACM" in `device.name`
  — the swerve Arduinos (Nano ESP32 with TinyUSB CDC) also enumerate as
  `/dev/ttyACM*` and you will pick one up. On this rig the 7 ACM ports
  break down as: 3 × SPARK MAX (VID `0x0483`, PID `0xa30e`, description
  `'SPARK MAX Motor Controller'`), 3 × Nano ESP32 (VID `0x2341`, PID
  `0x0070`), 1 × Arduino UNO (`00_base`, VID `0x2341`, PID `0x0043`).
  Use VID:PID `0x0483:0xa30e` as the SPARK MAX filter, then passively
  scan each Spark port for ~1 s to read the device_id from the lower 6
  bits of any incoming CAN ID. Helper: `_find_spark_by_device_id()` in
  `warrior_serial/test_swerve_module.py` (legacy path; replaced by C++
  port in [warrior_hardware_manager](warrior_hardware/warrior_hardware_manager/)
  TBD).
- **2026-05-17** — Mode-broadcast byte 0 is a device-id bitmask, not a
  control-mode enum. All three SPARK MAXes were originally at CAN ID 1,
  so the old hard-coded `"T02052C80802" + "00"*7` (byte 0 = `0x02` =
  bit 1) worked by coincidence. After renumbering to 2/3/4 it stopped
  working on all controllers; switching to `0x08` (bit 3) only fixed
  device 3. Caught by `sniff_usb.py` showing REV sending `0x04` /
  `0x08` / `0x10` for devices 2 / 3 / 4 respectively. Fix is to build
  the frame dynamically: `bitmask = (1 << device_id)` (or OR together
  for several). See `_make_mode_frame()` in `nudge_sparks.py`.
- **2026-05-17** — Position-discovery silently times out when Status 2
  is disabled. During the same session, after re-IDing controllers in
  REV, devices 3 and 4 sent Status 0 at the usual rate but exactly zero
  Status 2 frames over 5 s — REV's Status 2 Period was either unset or
  not yet persisted. Resolved by Burn Flash + power-cycle. Added
  `status_0_count` / `status_2_count` / `other_frame_count` counters to
  `nudge_sparks.py` so a quiet controller can be distinguished from one
  that's just missing position broadcast.
- **2026-05-17** — Collapsed `test_swerve_module` from three nodes to
  one coordinator. Originally one node per SPARK MAX (spawned 3× by the
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
- **2026-05-26** — Reset the hardware stack to a C++
  `warrior_hardware_manager` node + `warrior_system` ros2_control
  SystemInterface plugin (see [warrior_hardware/](warrior_hardware/)).
  The Python `warrior_serial` package is being phased out; its drive
  side (`<MOT,…>`) is replaced by the simpler `<DRV,name,pct>` wire
  protocol owned by the C++ manager. SPARK MAX SLCAN-over-USB support
  on the C++ side is **still ported from the Python reference** — see
  [warrior_serial/warrior_serial/nudge_sparks.py](warrior_serial/warrior_serial/nudge_sparks.py)
  for the authoritative byte layouts until the port lands.
- **2026-05-27** — `nudge_sparks.py` tx loop was gated on
  `device_id != None`, which was discovered passively from inbound
  frames — chicken-and-egg with SPARK MAXes that had stopped streaming
  after a heartbeat timeout. Fix: blast `_make_mode_frame(range(8)) +
  _ENABLE_FRAME` from session open before discovery completes, so
  controllers keep streaming Status 0/2 long enough to be discovered.
  After 12 V was confirmed on, all 3 controllers came up and nudged
  cleanly (+4.905 of commanded +5.000).
- **2026-05-27 — OPEN ISSUE** — on this session, none of the 3
  controllers streamed until REV Hardware Client was opened and
  connected once after 12 V power-on. After that, the bare `nudge_sparks`
  heartbeat alone is sufficient to keep them streaming across runs.
  Unknown whether REV did some persistent firmware-side state change
  during its connect, or whether the heartbeat fix above would have
  worked on a fully cold-booted controller given enough time. Capture
  REV's connect sequence with `sniff_usb.py` next time the bus is cold
  to find out — see [project-sparkmax-rev-bump-on-boot].

## Repository layout

- [warrior_hardware/](warrior_hardware/) — the real-robot stack:
  - [warrior_hardware_manager/](warrior_hardware/warrior_hardware_manager/) —
    C++ node that owns every USB connection (drive Arduinos + SPARK
    MAXes), subscribes `/warrior_swerve_command`, publishes
    `/warrior_swerve_state`.
  - [warrior_system/](warrior_hardware/warrior_system/) — ros2_control
    `SystemInterface` plugin (`SwerveTopicBridge`) translating between
    joint-level command/state interfaces and the
    `/warrior_swerve_command` / `/warrior_swerve_state` topics.
  - [warrior_serial/](warrior_hardware/warrior_serial/) — Python helpers
    kept around as the authoritative SLCAN reference (`nudge_sparks.py`,
    `sniff_usb.py`) until the C++ port lands.
- [warrior_msgs/](warrior_msgs/) — `SwerveCmd`, `SwerveState` (and
  legacy `MotorCommand`, `SparkCommand`, `SparkFeedback`).
- [warrior_control/](warrior_control/) — `swerve_drive_controller` and
  kinematics.
- [warrior_joy/](warrior_joy/) — joystick → `/cmd_vel`.
- Other packages (`warrior_localization`, `warrior_navigation`,
  `warrior_gps`, `warrior_perception`, `warrior_simulation`, …) are not
  part of the position-driven teleop pipeline and are documented in
  their own READMEs.
