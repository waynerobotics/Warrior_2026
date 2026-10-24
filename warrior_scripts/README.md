# warrior_scripts

Pixi activation and development helpers. See the
[workspace README](../README.md) for the quick start.

## Environment

From the repository root:

```bash
pixi install --locked
pixi run build
```

`pixi.toml` contains the dependencies and task definitions; `pixi.lock` pins
the x86_64 and ARM64 environments. The activation hook in this directory sets
`WARRIOR_WORKSPACE` and sources `install/pixi/setup.bash` after the first
successful build. Use `pixi shell` for an interactive ROS shell.

Do not source `/opt/ros/*/setup.bash` in the same shell. RoboStack supplies ROS
inside the Pixi prefix, and mixing apt and Pixi ROS/Python libraries can create
ABI and import conflicts.

## Scripts

| Script | What it does | Usage |
|---|---|---|
| `pixi_activate.sh` | Pixi activation hook; exports the repository root and loads the Pixi-built ROS overlay when present. | Sourced automatically by Pixi. |
| `pub_swerve_cmd.sh` | Publishes a `warrior_msgs/SwerveCmd` to `/warrior_swerve_command` at a fixed rate — manual swerve module bring-up/test. | `./pub_swerve_cmd.sh [id] [steer_rad] [drive_rad_s] [rate_hz]` |

`pub_swerve_cmd.sh` defaults: `id=left steer=0.0 drive=0.0 rate=10`. Example:

```bash
pixi run ./warrior_scripts/pub_swerve_cmd.sh front 0.785 32.0 10
```

## Launching the robot

Use `pixi run launch-sim` or `pixi run launch-real`, or enter `pixi shell` and
use any command documented in
[warrior_bringup](../warrior_bringup/README.md).

## Host-only setup

Pixi replaces the former Docker/apt provisioning, but it cannot configure
physical devices or proprietary GPU drivers:

- add the robot user to the appropriate serial/video/input groups or install
  equivalent udev rules;
- configure the Unitree LiDAR NIC as described in
  [PERCEPTION_SETUP.md](../warrior_perception/PERCEPTION_SETUP.md);
- on Jetson, install the supported JetPack release so CUDA, TensorRT, and
  `trtexec` are available to `pixi run build-perception`.
