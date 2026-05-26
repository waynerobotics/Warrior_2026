# warrior_hardware — bring-up test plan

This document walks the hardware-side stack up from "no hardware, no nodes" to
"full robot under controller". Run each phase in order; do not move on until
its **Pass** criteria are met.

## Architecture under test

```
/cmd_vel ─▶ swerve_drive_controller ─▶ ros2_control ─▶ SwerveTopicBridge
                                                          │
                                                          ▼
                                          /warrior_swerve_command   (SwerveCmd, 3 msgs / tick)
                                                          │
                                                          ▼
                                      warrior_hardware_manager
                                          │                  │
                                          │ <DRV,name,pct>   │ SPARK MAX position setpoints
                                          ▼                  ▼
                                    /dev/ttyACM*         SLCAN /dev/ttyACM*
                                    (drive Arduino)      (CAN adapter)
                                                          │
                                                          ▼
                                                       SPARK MAX
                                                          │
                                              periodic status frames
                                                          │
                                                          ▼
                                      warrior_hardware_manager
                                                          │
                                                          ▼
                                          /warrior_swerve_state   (SwerveState, 3 msgs / tick)
                                                          │
                                                          ▼
                                                  SwerveTopicBridge ─▶ controller (state ifs)
```

## Setup (once)

```bash
cd ~/ros2_ws
colcon build --packages-up-to warrior_hardware_manager warrior_system --symlink-install
source install/setup.bash
```

Useful sniffing tools (install once):

```bash
sudo apt install can-utils setserial socat
```

---

## Phase 0 — no hardware, hardware_manager alone

**Goal:** prove the node starts, advertises the right topics with the right
types, and the discovery loop runs without crashing.

```bash
ros2 launch warrior_hardware_manager hardware_manager.launch.py
```

In another terminal:

```bash
ros2 topic list | grep -E 'swerve|diag'
ros2 topic info /warrior_swerve_command   # should be warrior_msgs/msg/SwerveCmd
ros2 topic info /warrior_swerve_state     # should be warrior_msgs/msg/SwerveState
ros2 topic hz   /warrior_swerve_state     # should be ~150 Hz (3 modules × 50 Hz)
ros2 topic hz   /diagnostics              # should be ~1 Hz
```

**Pass:** all 3 topics exist with the right types, hz matches expected rates,
no segfaults, log shows `[discovery] scanning: 3 Arduino(s) missing, slcan=missing`
every 2 s.

---

## Phase 1 — wire contract sanity (still no hardware)

**Goal:** publish a fake command and confirm hardware_manager accepts it and
echoes it through to the diagnostics.

```bash
ros2 topic pub --once /warrior_swerve_command warrior_msgs/msg/SwerveCmd \
  "{swerve_id: 'front', steer_position_rad: 1.5708, drive_velocity_rad_s: 30.0}"

ros2 topic echo --once /warrior_swerve_state | head -12
ros2 topic echo --once /diagnostics | head -40
```

The state should show `swerve_id: front` with `steer_position_rad: 0.0` (no
feedback yet) and `steer_status: scanning`, `drive_status: scanning`.

The diagnostic for `front` should now have `cmd_recent: 'true'` and
`cmd_steer_rad: '1.5708'`. After 500 ms it flips back to `cmd_recent: false`
(command timeout).

**Pass:** SwerveCmd accepted (no `Ignoring command for unknown swerve_id`
warning), diagnostics reflect the command timestamp.

---

## Phase 2 — one drive Arduino

**Goal:** discovery finds an Arduino, drive commands are sent over serial.

1. Plug in **only** `02_swerve` (front).
2. `dmesg | tail` — confirm it shows up as `/dev/ttyACM0` (or similar).
3. Launch hardware_manager (or restart it).
4. Within ~10 s the log should print `[discovery] connected Arduino 02_swerve on /dev/ttyACMx`.
5. Publish a non-zero drive cmd for `front`:

   ```bash
   ros2 topic pub -r 20 /warrior_swerve_command warrior_msgs/msg/SwerveCmd \
     "{swerve_id: 'front', steer_position_rad: 0.0, drive_velocity_rad_s: 30.0}"
   ```

6. Sniff the serial in a third terminal (while hardware_manager has the port
   open you cannot also open it — instead stop hardware_manager and watch
   first, or use a USB pass-through):

   ```bash
   # Stop hardware_manager, then:
   stty -F /dev/ttyACM0 115200 raw -echo
   cat /dev/ttyACM0   # expect <DRV,02_swerve,50> at 50 Hz once node restarts
   ```

7. `ros2 topic echo /warrior_swerve_state` should show `drive_connected: true`
   and `drive_status: active` for `front`.
8. **Stop** the rate-limited publisher (Ctrl-C). Within 500 ms,
   `drive_status` flips to `timeout` and `<DRV,02_swerve,0>` goes out.

**Pass:** discovery log line, `drive_connected: true`, `<DRV,…>` visible on
the wire, timeout flips to 0 % when commands stop, motor stops within ~500 ms.

---

## Phase 3 — all three drive Arduinos

**Goal:** stress the discovery + per-module routing.

1. Plug in `02_swerve`, `03_swerve`, `04_swerve`.
2. Launch hardware_manager.
3. All three should appear in the log within ~10 s. `/diagnostics` for `front`,
   `left`, `right` all show `drive_connected: true`.
4. Drive each module independently:

   ```bash
   for m in front left right; do
     ros2 topic pub --once /warrior_swerve_command warrior_msgs/msg/SwerveCmd \
       "{swerve_id: '$m', steer_position_rad: 0.0, drive_velocity_rad_s: 10.0}"
     sleep 2
   done
   ```

5. **Unplug** `03_swerve` mid-run. Within ~2 s log shows
   `[03_swerve] write failed; dropping connection on /dev/ttyACMx`, and
   `[discovery] scanning: 1 Arduino(s) missing, …`.
6. **Plug back in.** Within one discovery cycle it reconnects automatically.

**Pass:** all three discovered, commands routed by `swerve_id`, hot-unplug
detected within 2 s, hot-replug reconnects without restarting hardware_manager.

---

## Phase 4 — SLCAN adapter (no SPARK MAX on the bus yet)

**Goal:** SLCAN discovery + `V`-probe doesn't collide with the Arduino probe.

1. Plug in the CAN adapter alongside the Arduinos.
2. Restart hardware_manager.
3. Log should show `[discovery] connected SLCAN on /dev/ttyACMx (auto)`.
4. `/diagnostics` for `slcan_adapter` flips from ERROR → OK with the port
   listed.
5. Per-module diagnostics still show `SPARK MAX feedback stale or absent`
   (WARN) because nothing is emitting on the bus.

If auto-detect mis-binds (e.g. probes an Arduino as SLCAN first), pin the
SLCAN port explicitly in [config/hardware_manager.yaml](warrior_hardware_manager/config/hardware_manager.yaml):

```yaml
sparkmax:
  slcan_interface: "/dev/ttyACM3"
```

**Pass:** slcan_adapter status OK, all 3 Arduinos still discovered, no port
collisions in the log.

---

## Phase 5 — SPARK MAX on the bus

**Goal:** the steer feedback loop populates state messages with real values.

**Pre-requisite:** the SPARK MAX controllers must be **pre-configured** in
the REV Hardware Client for closed-loop position mode with sensible PID
gains. The hardware_manager only sends setpoint frames — it does not
configure the controller.

1. Connect the 3 SPARK MAX controllers to the CAN bus (terminating resistors
   in place, IDs set to 2 / 3 / 4 matching the YAML).
2. Launch hardware_manager.
3. Within ~100 ms, `/warrior_swerve_state` should show non-zero
   `steer_position_rad` and `steer_velocity_rad_s` for whichever modules are
   currently receiving Status 1 / Status 2 frames.
4. `/diagnostics` per-module flips to OK.
5. Publish a position setpoint:

   ```bash
   ros2 topic pub -r 20 /warrior_swerve_command warrior_msgs/msg/SwerveCmd \
     "{swerve_id: 'front', steer_position_rad: 1.5708, drive_velocity_rad_s: 0.0}"
   ```

   The front module's steering motor should rotate to π/2 rad (90°), and the
   feedback `steer_position_rad` in /warrior_swerve_state should converge on
   1.5708.

If the motor does not move, the most likely cause is the
`API_INDEX_SET_POSITION` constant in
[sparkmax_frame.hpp](warrior_hardware_manager/include/warrior_hardware/sparkmax_frame.hpp)
not matching the deployed REV firmware. Capture a known-good
"set position 5.0 rot" frame from the REV Hardware Client and compare the
arbitration ID's bits 9–6 against the constant.

**Pass:** feedback values are non-zero and update at ≥10 Hz; commanded
position is reached within the SPARK MAX's PID convergence time.

---

## Phase 6 — full controller stack

**Goal:** drive the robot end-to-end via `/cmd_vel`.

1. With all hardware connected, launch the real-robot stack:

   ```bash
   ros2 launch warrior_control swerve_drive.real.launch.py
   ```

   This brings up `controller_manager`, `robot_state_publisher`,
   `joint_state_broadcaster`, `swerve_drive_controller`, and RViz. The
   `SwerveTopicBridge` is loaded as the ros2_control SystemInterface, so it
   starts publishing to `/warrior_swerve_command` automatically.

2. In another terminal, start hardware_manager:

   ```bash
   ros2 launch warrior_hardware_manager hardware_manager.launch.py
   ```

3. Verify the full pipe:

   ```bash
   ros2 topic hz /warrior_swerve_command   # ~150 Hz (3 × 50 Hz)
   ros2 topic hz /warrior_swerve_state     # ~150 Hz
   ros2 control list_controllers           # swerve_drive_controller: active
   ```

4. Drive with teleop:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
     --ros-args -p stamped:=true
   ```

   Press a forward key. Expected chain:
   `/cmd_vel → controller IK → joint cmd ifaces → bridge → /warrior_swerve_command → hardware_manager → drive Arduinos + SPARK MAXes → motors spin and steer`.

**Pass:** wheels actually turn in the direction commanded by teleop, with
the steering tracking the IK solution.

---

## Quick reference — failure modes

| Symptom | Likely cause |
|---|---|
| `Ignoring command for unknown swerve_id 'X'` | Controller is publishing a name that's not `front`/`left`/`right`. Check `SwerveTopicBridge::MODULE_NAMES`. |
| All Arduinos stay in `scanning` forever | Permission on `/dev/ttyACM*` — add user to `dialout` group, log out/in. |
| SLCAN found but feedback never arrives | SPARK MAX not on the bus, or bus bitrate mismatch — check `sparkmax.bitrate_code` (default `8` = 1 Mbps). |
| Motor spins on setpoint but feedback shows wrong angle | `steer_motor_rot_per_module_rot` gear ratio in YAML is wrong, or `steer_sign` is inverted. |
| `drive_status: timeout` even though publisher is running | `update_rate_hz × command_timeout_s < 1` — publisher rate is too low. |
| Whole node segfaults shortly after start | Look at the stack-overflow gotcha in [hardware_manager_node.cpp:42](warrior_hardware_manager/src/hardware_manager_node.cpp#L42) — `kv()` overload trap. |

## See also

- [warrior_hardware_manager/](warrior_hardware_manager/) — node source + config
- [warrior_system/](warrior_system/) — ros2_control SystemInterface plugin
- [warrior_msgs/msg/SwerveCmd.msg](../warrior_msgs/msg/SwerveCmd.msg) / [SwerveState.msg](../warrior_msgs/msg/SwerveState.msg) — wire contract
- [warrior_description/xacro/ros2_control/robot.ros2_control.xacro](../warrior_description/xacro/ros2_control/robot.ros2_control.xacro) — bridge plugin instantiation
