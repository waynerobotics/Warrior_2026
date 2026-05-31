# Changes — USB auto-connect / hardware-stack history

Dated record of USB auto-connect issues and fixes for the Warrior swerve
hardware stack. Append new entries here (newest at the bottom) with the date
(YYYY-MM-DD) and the fix. The actionable rules distilled from this history
live in [CLAUDE.md](CLAUDE.md) → *USB auto-connect — known issues and rules*.

> **Path note:** the Python `warrior_serial` package was later absorbed into
> `warrior_driver` — its SLCAN scripts now live in
> [warrior_hardware/warrior_driver/scripts/](warrior_hardware/warrior_driver/scripts/) —
> and `warrior_motor_manager` was renamed `warrior_driver`. History entries
> below keep the original path names they were written with.

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
  bits of any incoming CAN ID.
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
- **2026-05-26** — Reset the hardware stack to a C++ driver node +
  `warrior_system` ros2_control SystemInterface plugin (see
  [warrior_hardware/](warrior_hardware/)). The Python `warrior_serial`
  package is being phased out; its drive side (`<MOT,…>`) is replaced by
  the simpler `<DRV,name,pct>` wire protocol owned by the C++ driver.
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
     The old REV-connect workaround is no longer needed.
</content>
