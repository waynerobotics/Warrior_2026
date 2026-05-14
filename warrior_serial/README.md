# Warrior Serial Bridge

Python-side support for the Warrior microcontroller stack. Today this is a
single-process bridge that forwards control messages between Arduinos over USB
serial. Tomorrow it will be a set of ROS 2 nodes — see
[ROS 2 conversion prompt](#ros-2-conversion-prompt) at the bottom.

---

## Architecture

One base Arduino, three swerve Arduinos, one PC bridge. The bridge routes each
`<MOT,target,…>` message to the addressed swerve.

## warrior_serial

Lightweight ROS 2 (rclpy) package that talks to the Warrior Arduino-family
devices over USB serial. It provides console entry points and a launch file
to run a base driver (reads `00_base`) and swerve drivers (one per swerve
controller).

Key points

- Devices speak a simple ASCII, line-framed protocol: <TYPE,field1,field2,...>\n
- Supported messages used by this package: `<WHO>`, `<NAME,xxx>`, `<MOT,target,spark,flipsky>`
- The package contains ROS 2 nodes (console scripts) and a launch file to
  start them; discovery is by device-reported name (e.g. `00_base`, `02_swerve`).

What is in this package

- Python nodes / entry points (configured in `setup.py`):
  - `warrior_base_driver` — opens `00_base`, parses `<MOT,…>` and publishes a `/motor_cmd` topic
  - `motor_manager`, `twist_to_motor`, `twist_to_spark` — helper/adapter entry points
- Module files (in `warrior_serial/`): `base_driver.py`, `motor_manager.py`, `serial_protocol.py`, `twist_to_motor.py`, `twist_to_spark.py`
- Launch file: `launch/warrior_drivers.launch.py` — starts the base driver plus swerve drivers (02/03/04)
- Package manifest: `package.xml` (declares `rclpy`, `warrior_msgs`, and serial dependency)

Quick run (ROS 2)

1. Build and source your workspace (example):

```bash
colcon build --packages-select warrior_serial
source install/setup.bash
```

2. Launch the drivers (adjust parameters if needed):

# warrior_serial

Lightweight ROS 2 (rclpy) package for the Warrior Arduino-family devices over
USB serial. The package exposes console entry points and a launch file to
start a base driver (reads `00_base`) and swerve drivers (e.g. `02_swerve`).

## At a glance

- Protocol: ASCII, line-framed messages of the form `<TYPE,field1,field2,...>` (terminated with `\n`).
- Messages used here: `<WHO>`, `<NAME,DEVICE_NAME>`, `<MOT,target,spark,flipsky>`.
- Discovery is by device-reported name (drivers send `<WHO>` and wait for `<NAME,…>`).

## Contents

- Console entry points (configured in `setup.py`):
  - `warrior_base_driver` — reads `00_base` and republishes motor commands on `/motor_cmd`.
  - `motor_manager`, `twist_to_motor`, `twist_to_spark` — helper adapters.
- Core modules: `warrior_serial/base_driver.py`, `motor_manager.py`, `serial_protocol.py`, `twist_to_motor.py`, `twist_to_spark.py`.
- Launch: `launch/warrior_drivers.launch.py` (starts base + swerve drivers).
- Packaging: `package.xml`, `setup.py`, `setup.cfg`.

# warrior_serial

Lightweight ROS 2 (rclpy) package for the Warrior Arduino-family devices over
USB serial. The package exposes console entry points and a launch file to
start a base driver (reads `00_base`) and swerve drivers (e.g. `02_swerve`).

## At a glance

- Protocol: ASCII, line-framed messages of the form `<TYPE,field1,field2,...>` (terminated with `\n`).
- Messages used here: `<WHO>`, `<NAME,DEVICE_NAME>`, `<MOT,target,spark,flipsky>`.
- Discovery is by device-reported name (drivers send `<WHO>` and wait for `<NAME,…>`).

## Contents

- Console entry points (configured in `setup.py`):
  - `warrior_base_driver` — reads `00_base` and republishes motor commands on `/motor_cmd`.
  - `motor_manager`, `twist_to_motor`, `twist_to_spark` — helper adapters.
- Core modules: `warrior_serial/base_driver.py`, `motor_manager.py`, `serial_protocol.py`, `twist_to_motor.py`, `twist_to_spark.py`.
- Launch: `launch/warrior_drivers.launch.py` (starts base + swerve drivers).
- Packaging: `package.xml`, `setup.py`, `setup.cfg`.

# warrior_serial

Lightweight ROS 2 (rclpy) package for the Warrior Arduino-family devices over
USB serial. The package exposes console entry points and a launch file to
start a base driver (reads `00_base`) and swerve drivers (e.g. `02_swerve`).

## At a glance

- Protocol: ASCII, line-framed messages of the form `<TYPE,field1,field2,...>` (terminated with `\n`).
- Messages used here: `<WHO>`, `<NAME,DEVICE_NAME>`, `<MOT,target,spark,flipsky>`.
- Discovery is by device-reported name (drivers send `<WHO>` and wait for `<NAME,…>`).

## Contents

- Console entry points (configured in `setup.py`):
  - `warrior_base_driver` — reads `00_base` and republishes motor commands on `/motor_cmd`.
  - `motor_manager`, `twist_to_motor`, `twist_to_spark` — helper adapters.
- Core modules: `warrior_serial/base_driver.py`, `motor_manager.py`, `serial_protocol.py`, `twist_to_motor.py`, `twist_to_spark.py`.
- Launch: `launch/warrior_drivers.launch.py` (starts base + swerve drivers).
- Packaging: `package.xml`, `setup.py`, `setup.cfg`.

# warrior_serial

Lightweight ROS 2 (rclpy) package for the Warrior Arduino-family devices over
USB serial. The package exposes console entry points and a launch file to
start a base driver (reads `00_base`) and swerve drivers (e.g. `02_swerve`).

## At a glance

- Protocol: ASCII, line-framed messages of the form `<TYPE,field1,field2,...>` (terminated with `\n`).
- Messages used here: `<WHO>`, `<NAME,DEVICE_NAME>`, `<MOT,target,spark,flipsky>`.
- Discovery is by device-reported name (drivers send `<WHO>` and wait for `<NAME,…>`).

## Contents

- Console entry points (configured in `setup.py`):
  - `warrior_base_driver` — reads `00_base` and republishes motor commands on `/motor_cmd`.
  - `motor_manager`, `twist_to_motor`, `twist_to_spark` — helper adapters.
- Core modules: `warrior_serial/base_driver.py`, `motor_manager.py`, `serial_protocol.py`, `twist_to_motor.py`, `twist_to_spark.py`.
- Launch: `launch/warrior_drivers.launch.py` (starts base + swerve drivers).
- Packaging: `package.xml`, `setup.py`, `setup.cfg`.

## Quick start (ROS 2)

1. Build and source the workspace:

```bash
colcon build --packages-select warrior_serial
source install/setup.bash
```

2. Launch the drivers:

```bash
ros2 launch warrior_serial warrior_drivers.launch.py
```

You can override parameters such as `baud_rate` and `discovery_retry_period_s`
from the launch command line.

## Notes

- Opening a serial port toggles DTR and resets the Arduino; drivers wait a
  short delay after open before communicating.
- Discovery filters other traffic (for example continuous `<MOT,…>` streams)
  and waits for a proper `<NAME,…>` response.
- Drivers retry discovery/reconnect on IO errors; disconnects should not crash
  the node.

## Permissions

On Linux add your user to the `dialout` group to access serial devices:

```bash
sudo usermod -aG dialout $USER
# log out and back in
```

If you'd like, I can expand the wire-protocol section or add example
`ros2 topic pub` commands for manual testing. The `test/` folder is present
but intentionally not described here per your request.
