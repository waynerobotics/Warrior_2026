# Warrior Serial Bridge

Python-side support for the Warrior microcontroller stack. Today this is a
single-process bridge that forwards control messages between Arduinos over USB
serial. Tomorrow it will be a set of ROS 2 nodes — see
[ROS 2 conversion prompt](#ros-2-conversion-prompt) at the bottom.

---

## Architecture

One base Arduino, three swerve Arduinos, one PC bridge. The bridge routes each
`<MOT,target,…>` message to the addressed swerve.

```
                                                    ┌──────────────┐
                                                ┌──>│ 02_swerve    │
                                                │   │ Nano ESP32   │
                                                │   │ D6/D7 PWM    │
                                                │   └──────────────┘
┌──────────────┐  USB   ┌──────────────┐  USB   │   ┌──────────────┐
│ 00_base      │ ─────> │ serial_      │ ─────> │──>│ 03_swerve    │
│ LCD keypad   │ <MOT,  │ bridge       │ <MOT,  │   │ Nano ESP32   │
│ + RadioLink  │ tgt,…> │ (this code)  │ tgt,…> │   │ D6/D7 PWM    │
│ @20 Hz       │        │ routes by    │        │   └──────────────┘
└──────────────┘        │ target field │        │   ┌──────────────┐
                        └──────────────┘        └──>│ 04_swerve    │
                                                    │ Nano ESP32   │
                                                    │ D6/D7 PWM    │
                                                    └──────────────┘
```

`00_base` is the input device — LCD Keypad Shield buttons and/or a RadioLink
SBUS receiver. Each `0X_swerve` is an output device that receives motor
commands addressed to it and drives an ESC pair via the ESP32Servo library.
The Python bridge is the glue: it auto-discovers all four boards by
**device name** (not COM port), so USB enumeration order doesn't matter.

Any subset of the three swerves may be missing — the bridge runs with whatever
it can find, and `00_base` lets you toggle which targets to address at
runtime (SELECT on the keypad, or buttonVRB on the radio).

---

## Wire protocol

Every message is ASCII, framed by `<` and `>`, terminated with `\n`:

```
<TYPE,field1,field2,...>
```

Defined in `Warrior/libraries/SerialProtocol/`. Both directions use the same
format. Known types:

| Direction         | Message                                   | Meaning                                                  |
| ----------------- | ----------------------------------------- | -------------------------------------------------------- |
| host → device     | `<WHO>`                                   | Ask device to identify itself                            |
| host → device     | `<PING>`                                  | Liveness probe                                           |
| 00_base → bridge  | `<MOT,target,spark,flipsky>`              | Motor command. `target` ∈ {02_swerve, 03_swerve, 04_swerve}. spark/flipsky -100..100 |
| bridge → 0X_swerve| `<MOT,target,spark,flipsky>`              | Same message, routed (or broadcast). The swerve filters by `target == DEVICE_NAME` |
| device → host     | `<NAME,name>`                             | Reply to WHO                                             |
| device → host     | `<ACK,type>`                              | Generic ack (e.g. `<ACK,PONG>` for PING)                 |
| device → host     | `<ERR,reason>`                            | Error                                                    |
| reserved          | `<CTRL,ail,ele,thr,rud,swA,btn,swB,knob>` | Controller state                                         |
| reserved          | `<FBK,driveVel,steerVel,steerPos>`        | Motor feedback                                           |

Devices identify themselves with stable string names:

- `00_base` — LCD keypad / RadioLink input device
- `02_swerve`, `03_swerve`, `04_swerve` — ESP32 PWM output devices

Names are constants at the top of each `.ino` (`const char* DEVICE_NAME = …;`).
The three swerve sketches are functionally identical — copy `04_set_pwm`,
change `DEVICE_NAME`, and upload.

Safety: each swerve returns motors to neutral (1500 µs) if no matching
`<MOT,DEVICE_NAME,…>` is received for 500 ms. Whatever produces motor
commands MUST send at ≥ 2 Hz per active target. When `00_base` toggles a
target off, that swerve stops receiving commands and its watchdog fires
within 500 ms — that's the intended behaviour.

---

## Files

| File                    | Purpose                                                        |
| ----------------------- | -------------------------------------------------------------- |
| `serial_protocol.py`    | `WarriorSerial` class + `query_device_name` helper             |
| `serial_bridge.py`      | The forwarding daemon. Auto-discovers, reconnects on drop      |
| `find_arduino_ports.py` | Standalone port-scanning utility                               |
| `test_serial_protocol.py` | Unit tests for the framing/parsing                           |
| `requirements.txt`      | `pyserial`                                                     |

---

## Running

### Windows

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
python .\serial_bridge.py
```

### Ubuntu 22.04 (x64 or ARM)

One-time setup — add yourself to `dialout` so you can open `/dev/ttyACM*`:

```bash
sudo usermod -aG dialout $USER
# log out, log back in (or reboot)
```

Then:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python serial_bridge.py
```

### CLI options

```
python serial_bridge.py                  # auto-discover everything
python serial_bridge.py --base COM5      # pin the base port; swerves auto-discovered
python serial_bridge.py --base /dev/ttyUSB0
python serial_bridge.py --scan           # list every port and its DEVICE_NAME
```

The bridge runs forever — closing/unplugging any Arduino triggers
re-discovery, not an exit. Stop with Ctrl+C.

---

## Cross-platform notes

- pyserial abstracts over Win/Linux/macOS port naming. The same Python code
  runs everywhere.
- Arduinos auto-reset when the host opens the serial port (DTR toggle).
  `WarriorSerial.open()` sleeps 2 s after open to wait for that reset to
  complete. Don't shorten this without testing.
- `00_base` streams `<MOT,…>` continuously. The `query_device_name` helper
  sends `<WHO>` and then **filters** for `<NAME,…>` — it ignores other
  messages until the deadline. Don't go back to "trust the first byte" or
  you'll mistake a stray `<MOT,…>` for the device name.
- On Linux the `dialout` group requirement is the #1 footgun.

---

## Known caveats

1. Discovery sends `<WHO>` to every COM port, including non-Arduino devices.
   Most just time out, but a misbehaving device that responds to arbitrary
   bytes could confuse things. Mitigation: only ports that send a properly
   framed `<NAME,…>` reply are accepted.
2. Arduinos auto-reset on every serial open. If you reopen mid-session (e.g.
   to update firmware) you'll briefly lose PWM output. The 500 ms safety
   watchdog on each swerve covers this — motors return to 1500 µs.
3. There is no flow control. If a swerve is overwhelmed, messages are
   dropped silently. The 20 Hz send rate per target is conservative enough
   that this hasn't been observed.
4. The bridge does not parse `<NAME>`/`<ACK>`/`<ERR>` replies from swerves —
   the swerves don't generate them anyway (see comment in `04_set_pwm.ino`).
   If you start consuming them on the bridge side, drain the swerve TX
   buffers too.
5. With `00_base` toggling targets, the bridge silently drops MOT messages
   addressed to a swerve that isn't currently connected. The status line
   shows a running `skipped:` counter so you can spot it.

---

## ROS 2 conversion prompt

> Paste this entire section into the new repo's Claude Code / Cursor session.
> It is self-contained — assume the reader has not seen the `Warrior/Python`
> code.

### Goal

Convert the existing single-process Python serial bridge described below into
a set of ROS 2 (Humble) Python nodes. Target Ubuntu 22.04 on x64 and ARM.
Maintain compatibility with the existing Arduino firmware — DO NOT change the
wire protocol.

### Hardware context

Four Arduino-family microcontrollers connected via USB to a Linux host:

1. **`00_base`** — Arduino with LCD Keypad Shield and/or RadioLink SBUS
   receiver. Generates motor commands at 20 Hz and streams them as
   `<MOT,target,spark,flipsky>` over USB serial, where `target` is one of
   `02_swerve`, `03_swerve`, `04_swerve`. Spark and flipsky are normalized
   integer commands in -100..100, where 0 = neutral. The user can toggle
   which targets receive commands at runtime (one MOT message is emitted per
   enabled target each cycle).
2. **`02_swerve`, `03_swerve`, `04_swerve`** — Arduino Nano ESP32 each. All
   run the same firmware with a different `DEVICE_NAME`. Each subscribes to
   `<MOT,…,…,…>` traffic, filters by `target == DEVICE_NAME`, maps -100..100
   to RC PWM 1000–2000 µs, and outputs on GPIO 9 (D6, "spark") and GPIO 10
   (D7, "flipsky") via the ESP32Servo library. Each has its own 500 ms
   safety watchdog — returns to 1500 µs neutral if no matching message
   arrives in time.

Any subset of swerves may be missing — `00_base` keeps emitting addressed
messages regardless. Devices use the same line-based ASCII framing. Every
message is `<TYPE,field1,field2,...>` followed by `\n`.

### Wire protocol (cannot change)

| Direction         | Message                          | Meaning                                  |
| ----------------- | -------------------------------- | ---------------------------------------- |
| host → device     | `<WHO>`                          | Ask device to identify itself            |
| host → device     | `<PING>`                         | Liveness probe                           |
| 00_base → host    | `<MOT,target,spark,flipsky>`     | Motor command. target ∈ {02/03/04}_swerve |
| host → 0X_swerve  | `<MOT,target,spark,flipsky>`     | Same message; swerve filters by DEVICE_NAME |
| device → host     | `<NAME,name>`                    | Reply to WHO                             |
| device → host     | `<ACK,type>`                     | Acknowledgement                          |
| device → host     | `<ERR,reason>`                   | Error                                    |

Discovery: send `<WHO>`, read messages until `<NAME,xxx>` appears (filtering
out other traffic), match against expected name. `00_base`, `02_swerve`,
`03_swerve`, `04_swerve` are all valid responses.

### Constraints

- **Auto-reset**: opening the serial port toggles DTR and resets the Arduino.
  Wait 2 s after open before sending. This is non-negotiable.
- **Linux permissions**: include setup docs for `sudo usermod -aG dialout
  $USER`.
- **Continuous streams**: `00_base` streams `<MOT,target,…>` at 20 Hz per
  enabled target. The discovery helper must skip these and look only for
  `<NAME,…>` replies.
- **Watchdog**: any node publishing motor commands to a given swerve must
  send to that target at ≥ 2 Hz, or the ESP32 will fail-safe to 1500 µs.
  This is a feature — preserve it. Toggling a target off naturally lets its
  watchdog fire, which is the intended way to "disable" a swerve.
- **Reconnect**: USB drops should not crash nodes. Re-discover and reconnect.
- **No backwards compat shims**: this is a fresh ROS 2 package. Don't carry
  over the old `serial_bridge.py` structure.

### Suggested architecture

One base driver + N swerve drivers (one per physical swerve). The Python
"bridge" concept goes away — each Arduino gets its own ROS 2 node:

```
                                             ┌──────────────────────┐
                                             │ warrior_swerve_driver│
                                          ┌─>│ name: 02_swerve      │
                                          │  │ subscribes /motor_cmd│
                                          │  │ filters target==self │
                                          │  └──────────────────────┘
┌──────────────────────┐  topic:          │  ┌──────────────────────┐
│ warrior_base_driver  │  /motor_cmd      │  │ warrior_swerve_driver│
│ opens 00_base by name│ ───────────────> │─>│ name: 03_swerve      │
│ parses <MOT,…> from  │  warrior_msgs/   │  │ ...                  │
│ device, publishes    │  MotorCommand    │  └──────────────────────┘
│ /motor_cmd           │  (with target)   │  ┌──────────────────────┐
└──────────────────────┘                  └─>│ warrior_swerve_driver│
                                             │ name: 04_swerve      │
                                             │ ...                  │
                                             └──────────────────────┘
```

Decoupling via the `/motor_cmd` topic means future input sources (Xbox
controller, autonomy stack) can publish to the same topic without touching
any swerve driver. Per-swerve filtering by `target` keeps deployment
flexible — bring up however many swerves are physically present.

Custom message recommendation:

```
# warrior_msgs/msg/MotorCommand.msg
string target   # "02_swerve" | "03_swerve" | "04_swerve"
int8 spark      # -100..100, 0 = neutral
int8 flipsky    # -100..100, 0 = neutral
```

The microsecond mapping (`-100..100 -> 1000..2000 µs`) lives in the firmware
on each swerve, not in the ROS 2 layer. Keep it that way — it lets you
re-calibrate ESC ranges in firmware without touching the control stack.

### ROS 2 parameters per node

- `device_name` (string, required) — e.g. `"00_base"` or `"02_swerve"`
- `baud_rate` (int, default 115200)
- `discovery_retry_period_s` (double, default 2.0)
- `read_timeout_s` (double, default 0.1)

### Implementation notes

- Each node's main work happens in a timer callback (not a blocking thread).
  Use a small read timeout (0.05–0.1 s) inside a periodic spin so callbacks
  stay responsive.
- Wrap port discovery in a state machine: `DISCONNECTED → DISCOVERING →
  CONNECTED → DISCONNECTED`. Re-enter discovery on any read/write exception.
- Use `rclpy`'s logging (`self.get_logger().info(...)`), not `print`.
- Provide a `launch/warrior_drivers.launch.py` that brings up both drivers.
- Include a `package.xml` with `rclpy`, `pyserial`, and `warrior_msgs` deps.

### Testing

- `pytest` for the framing/parser logic — port over the existing
  `test_serial_protocol.py` style.
- A "loopback" test that runs both nodes against `pyserial`'s loopback or a
  null-modem pipe, asserting that publishing on `/motor_cmd` produces the
  expected bytes on the addressed swerve's serial port and nothing on the
  other swerves.
- Manual integration test with real hardware: `ros2 topic pub /motor_cmd
  warrior_msgs/msg/MotorCommand "{target: '02_swerve', spark: 0, flipsky: 0}"`
  and observe that only the named swerve responds.

### Reference implementation in the legacy repo

The existing single-process Python at `Warrior/Python/` is a working
reference for the wire protocol and discovery logic. Read these in order:

1. `serial_protocol.py` — `WarriorSerial` class, `query_device_name` helper
2. `serial_bridge.py` — discovery + reconnect state machine, multi-swerve
   routing with target field
3. `Warrior/libraries/SerialProtocol/src/SerialProtocol.{h,cpp}` —
   firmware-side framing
4. `Warrior/sketches/00_base/00_base.ino` — `00_base` reference, with
   Shield (LCD keypad) and Controller (RadioLink SBUS) toggles and
   per-swerve enable flags
5. `Warrior/sketches/tests/04_set_pwm/04_set_pwm.ino` — `0X_swerve` reference;
   parses `<MOT,target,spark,flipsky>` and filters by `target == DEVICE_NAME`

Do not vendor the legacy Python — re-implement cleanly inside the ROS 2
package layout. Reuse only the framing/parsing helpers, ported to a
`warrior_serial` Python module under the new package.
