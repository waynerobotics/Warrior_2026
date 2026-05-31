# Warrior_2026

ROS 2 workspace for the Warrior swerve robot.

- **Drive velocity** → three swerve Arduinos, addressed by name over USB-serial.
- **Steering position** → three REV SPARK MAX controllers over USB-SLCAN.
- **Pipeline:** `/cmd_vel` (TwistStamped) → `swerve_drive_controller` →
  `SwerveTopicBridge` (warrior_system) → `warrior_driver` → both transports.

One-shot real-robot teleop (joy → teleop_twist_joy → controller → bridge →
`warrior_driver`, all in one):

```bash
ros2 launch warrior_bringup warrior_swerve_teleop.launch.py
```

It owns `warrior_driver`, so don't also start the driver or
`steer_calibration_node` separately.

**Key locations**

- C++ driver package/node: **`warrior_driver`** (executable
  `warrior_driver_node`), under
  [warrior_hardware/warrior_driver/](warrior_hardware/warrior_driver/).
- SPARK MAX SLCAN reference scripts (`nudge_sparks.py`, `sniff_usb.py`,
  `probe_status2.py`) live in
  [warrior_hardware/warrior_driver/scripts/](warrior_hardware/warrior_driver/scripts/).
- Topic-level diagram & bring-up walkthrough:
  [warrior_hardware/README.md](warrior_hardware/README.md).

## USB auto-connect — known issues and rules

When you find a new USB auto-connect issue, append it to [CHANGES.md](CHANGES.md)
with the date (YYYY-MM-DD) and the fix. The history is more valuable than tidy
prose.

### Rules (always apply)

1. **Never hardcode a port path.** `/dev/ttyACM*` numbering changes across
   reboots and replugs. Discover devices, don't pin them.
2. **Arduinos identify by handshake.** Send `<WHO>`, wait for `<NAME,…>`.
   Helper: `ArduinoSerialDevice::handshake()` in
   [warrior_hardware/warrior_driver/src/arduino/arduino_serial_device.cpp](warrior_hardware/warrior_driver/src/arduino/arduino_serial_device.cpp).
3. **SPARK MAXes have no software handshake.** They stream status frames
   unprompted. Identify by the lower 6 bits of any incoming CAN ID
   (`can_id & 0x3F == device_id`). Set the device_id once in REV Hardware
   Client and treat it as the only source of truth. The logical name →
   `spark_can_id` table lives in
   [warrior_hardware/warrior_driver/config/warrior_driver.yaml](warrior_hardware/warrior_driver/config/warrior_driver.yaml)
   (`modules.<name>.steer_device_name` / `spark_can_id`).
4. **SPARK MAX mode-broadcast byte 0 is a per-device-id bitmask.** Frame
   `T02052C80…`'s first data byte = `(1 << device_id)` enables that
   controller to follow setpoints; OR multiple bits together to drive
   multiple controllers in one frame. **Not** an enumerated control-mode
   value. The control mode itself (Position / Velocity / Duty / …) is
   persistent per-device config in REV Hardware Client. Hard-coding the
   byte (e.g. `0x02`) silently fails as soon as device_ids change.
   Helper: `_make_mode_frame()` in
   [warrior_hardware/warrior_driver/scripts/nudge_sparks.py](warrior_hardware/warrior_driver/scripts/nudge_sparks.py).
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
   [arduino_serial_device.cpp:77](warrior_hardware/warrior_driver/src/arduino/arduino_serial_device.cpp#L77)
   (`ioctl(fd, TIOCEXCL)`).
7. **Wait after opening an Arduino.** DTR toggles on open and triggers a
   reset; the sketch is unresponsive for ~2 s. `ARDUINO_RESET_DELAY` in
   [device_registry.cpp:28](warrior_hardware/warrior_driver/src/swerve/device_registry.cpp#L28).
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
    [nudge_sparks.py](warrior_hardware/warrior_driver/scripts/nudge_sparks.py);
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
    `_make_enable_telemetry_frame()` in
    [nudge_sparks.py](warrior_hardware/warrior_driver/scripts/nudge_sparks.py),
    `sparkmax::make_enable_telemetry_frame()` in
    [sparkmax_frame.hpp](warrior_hardware/warrior_driver/include/warrior_driver/sparkmax/sparkmax_frame.hpp).
    This replaces the old "open REV once after cold boot" workaround.

### History

The dated record of every USB auto-connect issue and its fix lives in
[CHANGES.md](CHANGES.md). **Read it** before debugging hardware
discovery / SLCAN / SPARK MAX behaviour — the rules above are distilled
from it, and the original failure modes explain *why* each rule exists.
Append new findings there.

## Repository layout

See the package table in the [top-level README](README.md). **Read it**
for the workspace map and per-package READMEs.
</content>
