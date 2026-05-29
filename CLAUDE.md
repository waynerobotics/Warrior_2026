# Warrior_2026

ROS 2 workspace for the Warrior swerve robot. Drive velocity is handled by
three swerve Arduinos addressed by name over USB-serial; steering position
is handled by three REV SPARK MAX motor controllers over USB-SLCAN. A
`/cmd_vel` (Twist) → `swerve_drive_controller` → `SwerveTopicBridge` →
`warrior_motor_manager` pipeline fans commands to both transports. See
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
  [warrior_hardware/warrior_motor_manager/src/arduino_serial_device.cpp](warrior_hardware/warrior_motor_manager/src/arduino_serial_device.cpp).
3. **SPARK MAXes have no software handshake.** They stream status frames
   unprompted. Identify by the lower 6 bits of any incoming CAN ID
   (`can_id & 0x3F == device_id`). Set the device_id once in REV Hardware
   Client and treat it as the only source of truth (logical name →
  device_id table lives in `motor_manager.yaml`).
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
   you tell that apart from a dead controller. **As of 2026-05-29 you no
   longer need Burn Flash for this** — send the telemetry-enable frame
   (rule 12) and Status 2-6 turn on in software. Burn-Flashing Status 2
   ≤ ~50 ms in REV still works as a fallback but is no longer required.
6. **Open serial ports exclusively.** `TIOCEXCL` on Linux. Without this,
   two nodes will fight over the same port and both will see corrupted
   bytes. See
  [arduino_serial_device.cpp:74](warrior_hardware/warrior_motor_manager/src/arduino_serial_device.cpp#L74)
   (`ioctl(fd, TIOCEXCL)`).
7. **Wait after opening an Arduino.** DTR toggles on open and triggers a
   reset; the sketch is unresponsive for ~2 s. `ARDUINO_RESET_DELAY` in
  [device_registry.cpp:16](warrior_hardware/warrior_motor_manager/src/device_registry.cpp#L16).
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
11. **Open the SLCAN CAN channel before writing any `T…` frame.** A
    cold-booted SPARK MAX brings its USB-SLCAN bridge up with the CAN
    channel *closed* — every `T…` frame you write is silently dropped and
    nothing ever streams back. Send `S8\r` (bitrate 1 Mbit/s) then `O\r`
    at session open. SLCAN keeps the channel open until `C\r` or a
    power-cycle, which is why a warm controller (one a prior REV/session
    already opened) appears to "just work" while a cold one is silent.
    Helpers: `_OPEN_FRAME` in
    [warrior_hardware/warrior_driver/scripts/nudge_sparks.py](warrior_hardware/warrior_driver/scripts/nudge_sparks.py);
    C++ `sparkmax::SLCAN_OPEN_SEQUENCE`, sent in `SparkMaxSession::open()`
    (the live class — note `SparkMaxSlcanDevice` also has a C/S/O open but
    is **not compiled** into any target, so don't be fooled by it).
12. **Status 2-6 (incl. position) need a one-time telemetry-enable
    frame.** Opening the channel (rule 11) gets Status 0 streaming, but a
    cold controller still won't broadcast Status 2-6 until it receives:
    CAN ID `0x02050400 | device_id`, dlc 4, payload `7C 00 FF FF`
    (api_class 0x01, api_index 0x00). The device_id is in the low 6 bits,
    so **build it per controller** (`…402` for dev 2, `…403` for dev 3 —
    do not hardcode). Resend each tx tick until Status 2 appears, then
    stop (it's a register write, not a heartbeat). Helpers:
    `_make_enable_telemetry_frame()` in `nudge_sparks.py`,
    `sparkmax::make_enable_telemetry_frame()` in
    [sparkmax_frame.hpp](warrior_hardware/warrior_driver/include/warrior_driver/sparkmax/sparkmax_frame.hpp).
    This replaces the old "open REV once after cold boot" workaround.

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
  port in [warrior_motor_manager](warrior_hardware/warrior_motor_manager/)
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
- **2026-05-27 — (was OPEN, RESOLVED 2026-05-29)** — on this session,
  none of the 3 controllers streamed until REV Hardware Client was opened
  and connected once after 12 V power-on. After that, the bare
  `nudge_sparks` heartbeat alone was sufficient to keep them streaming.
  Root cause turned out to be **two stacked problems**, both fixed below;
  REV's connect happened to do both, which is why "open REV once" worked.
- **2026-05-29** — Captured REV's cold-boot connect with the upgraded
  `sniff_usb.py` (now multi-device + timestamps; tails usbmon `0u`) into
  `/tmp/rev_cold_connect.log`. Two findings closed the 2026-05-27 issue:
  1. **CAN channel closed on cold boot.** REV's *first* two bytes are
     `S8\r` then `O\r`; Status 0 starts ~20 ms later. `nudge_sparks` never
     sent these and wrote `T…` frames into a closed channel that the
     adapter silently dropped — total silence. A *warm* controller worked
     because SLCAN keeps the channel open from a prior session. Fix:
     `_OPEN_FRAME = "S8\rO\r"` at `SparkSession` open (rule 11). After
     this, a cold power-cycle gave `S0=498` where before it was `S0=0`.
  2. **Status 2-6 need a telemetry-enable.** With the channel open we got
     Status 0 but `S2=0` (no position). In the log, Status 2-6 all turn on
     together ~20 ms after a single frame REV sent: CAN ID
     `0x02050400 | device_id`, dlc 4, payload `7C 00 FF FF`. Confirmed
     REV-free by replaying REV's own bytes with the new
     `scripts/probe_status2.py` (PASS on dev 2), then verified the
     device_id is embedded by re-sniffing dev 3 (`…403` vs dev 2's `…402`).
     Fix: `_make_enable_telemetry_frame(device_id)`, sent until Status 2
     appears (rule 12). Folded into the C++ side too
     (`sparkmax::make_enable_telemetry_frame()` + `SparkMaxSession::tx_loop`).
     The old REV-connect workaround ([project-sparkmax-rev-bump-on-boot])
     is no longer needed.

## Repository layout

- [warrior_hardware/](warrior_hardware/) — the real-robot stack:
   - [warrior_motor_manager/](warrior_hardware/warrior_motor_manager/) —
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
