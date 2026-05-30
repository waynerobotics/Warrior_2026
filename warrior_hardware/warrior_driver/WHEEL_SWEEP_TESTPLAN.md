# Wheel sweep test plan

Bench test: at each steering increment from 0° to 360° (20° steps), point all
three swerve modules to that angle and drive the wheels at 10% for 5 seconds.

Script: [scripts/wheel_sweep_test.py](scripts/wheel_sweep_test.py)

> **Safety:** the chassis drives at each of 19 angles. Put the robot on
> blocks/stands so the wheels spin free, or give it clear floor space. Keep the
> e-stop / power switch in reach. Ctrl-C stops the script and commands zero
> drive.

---

## 0. Prerequisites

### Build (do not skip — stale binaries bite)

The `warrior_system` plugin and `warrior_driver` node are C++ and **must be
rebuilt on the distro you are running** (humble or jazzy). A stale
`libwarrior_system.so` shows up as a ros2_control "Discrepancy between robot
description file (urdf) and actually exported HW interfaces" error — it is not
an API problem, it is an out-of-date `.so`.

```bash
cd ~/ros2_ws
colcon build --packages-select warrior_msgs warrior_driver warrior_system
source install/setup.bash
```

### Confirm USB devices are present

7 ports expected on a full rig (this session saw 6–7; one Arduino occasionally
drops — replug it). Identify by VID:PID:

```bash
for d in /dev/ttyACM*; do
  b=$(basename "$d"); p=$(readlink -f /sys/class/tty/$b/device)
  while [ ${#p} -gt 1 ]; do
    [ -f "$p/idVendor" ] && { echo "$d -> $(cat $p/idVendor):$(cat $p/idProduct)"; break; }
    p=$(dirname "$p")
  done
done
```

- `0483:a30e` ×3 → SPARK MAX (steer), CAN IDs 2 / 3 / 4
- `2341:0070` ×3 → Nano ESP32 swerve Arduinos (drive): `02/03/04_swerve`
- `2341:0043` ×1 → Arduino UNO `00_base` (if fitted)

---

## 1. Bring up the driver

Two options. **Option A is simplest for this test** because it avoids the
controller fighting the script for `/warrior_swerve_command`.

### Option A — driver alone (recommended)

```bash
ros2 launch warrior_driver warrior_driver.launch.py
```

Wait for all three SPARK MAXes and three Arduinos to be discovered. Look for:

```
[discovery] connected SPARK dev=4 on /dev/ttyACMx
[discovery] connected SPARK dev=2 on /dev/ttyACMx
[discovery] connected SPARK dev=3 on /dev/ttyACMx
[discovery] connected Arduino 04_swerve / 02_swerve / 03_swerve
```

> Cold SPARK MAXes can take a couple of seconds to start streaming Status 2
> (position). Until then a module logs `no_feedback`; the driver's
> telemetry-enable frame brings it up automatically. Wait until all three
> modules report `idle` (not `no_feedback`) before starting the sweep.

### Option B — full stack, controller deactivated

If the full bringup is running (`warrior.real.launch.py`), the
`swerve_drive_controller` publishes `/warrior_swerve_command` at 50 Hz and will
override the script. Deactivate it first:

```bash
ros2 control set_controller_state swerve_drive_controller inactive
# verify:
ros2 control list_controllers     # swerve_drive_controller -> inactive
```

Re-activate when done: `ros2 control set_controller_state swerve_drive_controller active`

---

## 2. Run the sweep

```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/Warrior_2026/warrior_hardware/warrior_driver/scripts/wheel_sweep_test.py
```

Defaults = the requested test:

| Flag | Default | Meaning |
|------|---------|---------|
| `--speed-percent` | `10` | drive power; 10% → `drive_velocity_rad_s = 6.0` (`percent/100 × max_drive_rad_s`, max=60) |
| `--hold-s` | `5` | seconds driving at each angle |
| `--settle-s` | `2` | seconds to let steering reach the angle (drive 0) before driving; **set `0`** for strict "5 s per increment, no settle" |
| `--step-deg` | `20` | steering increment |
| `--start-deg` / `--end-deg` | `0` / `360` | inclusive range → 0,20,…,360 (19 angles) |
| `--max-drive-rad-s` | `60` | **must match** `modules.*.max_drive_rad_s` in [config/warrior_driver.yaml](config/warrior_driver.yaml) |

Total run ≈ 19 × (2 s settle + 5 s drive) ≈ **2 min 13 s**.

Examples:
```bash
# strict literal: 5 s drive at each angle, no settle phase
python3 .../wheel_sweep_test.py --settle-s 0

# gentler smoke test: just 0° and 90°, 5%, before committing to the full sweep
python3 .../wheel_sweep_test.py --start-deg 0 --end-deg 90 --step-deg 90 --speed-percent 5
```

---

## 3. What to watch

In the driver terminal, the throttled per-module line (1 Hz, rotates between
modules) should show, during each drive phase:

```
[front] steer cmd=X rad / fb≈X rad (active) | drive cmd=6.000 rad/s -> 10% (active)
```

Pass criteria per angle:

- **Steer reaches target**: `fb` tracks `cmd` within a few hundredths of a rad
  after the settle phase, for **all three** modules (front/left/right).
- **Drive engages**: `drive ... -> 10% (active)` on all three. (`right` drive
  needs the `03_swerve` Arduino connected; if it shows `scanning`, that port
  dropped — replug it.)
- **No `write_failed` / `no_feedback`** during the run.
- **Physical**: all three wheels rotate to roughly the commanded heading and
  spin at a low, steady speed; at 0° and 360° the wheels should be at the same
  heading.

Optionally record the run for later inspection:
```bash
ros2 bag record /warrior_swerve_command /warrior_swerve_state /diagnostics
```

---

## 4. Stop / cleanup

- **Stop the sweep**: Ctrl-C in the script terminal — it sends zero drive
  before exiting.
- **Emergency**: cut motor power. The driver also zeros drive on its own
  0.5 s command timeout once the script stops publishing.
- Shut down the driver: Ctrl-C in its terminal (it sends drive 0% to all
  Arduinos on shutdown).

---

## Known gotchas (from the 2026-05-29 bring-up session)

- **Rebuild per distro.** The repo targets humble but was debugged on jazzy.
  The source uses the deprecated-but-functional `on_init(HardwareInfo)` /
  by-value `export_*_interfaces()` API, which works on both — you just have to
  `colcon build` on the distro you run. Jazzy prints deprecation warnings only.
- **Stale `.so` → "Discrepancy ... missing state interfaces".** Always rebuild
  `warrior_system` after editing it. See §0.
- **`swerve_drive_controller` has no `cmd_vel` timeout** (separate from this
  test): with the full stack it keeps republishing the last `/cmd_vel` forever,
  so the robot keeps driving after teleop stops. Not used in Option A, but
  relevant if you test via `/cmd_vel`.
- **SPARK MAX cold-boot Status 2 delay**: wait for `idle` (not `no_feedback`)
  on all three modules before sweeping, so position feedback is live.
