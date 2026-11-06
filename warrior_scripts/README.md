# warrior_scripts

Pixi activation and development helpers. See the
[workspace README](../README.md) for the quick start.

## Environment

From the repository root:

```bash
pixi install
pixi run build
```

`pixi.toml` contains the dependencies and task definitions; `pixi.lock` pins
the x86_64 and ARM64 ROS environments. The activation hook in this directory sets
`WARRIOR_WORKSPACE` and sources `install/pixi/setup.bash` after the first
successful build. Use `pixi shell` for an interactive ROS shell.

Do not source `/opt/ros/*/setup.bash` in the same shell. RoboStack supplies ROS
inside the Pixi prefix, and mixing apt and Pixi ROS/Python libraries can create
ABI and import conflicts.

Isaac Sim has its own optional Pixi environment on Linux x86_64 GPU workstations:

```bash
pixi run install-isaac
pixi run configure-isaac
pixi run launch-igvc2027-isaac
```

The `isaac` environment installs Python 3.11 and
`isaacsim[all,extscache]==5.1.0` under `.pixi/envs/isaac/`, using the committed
lock. It excludes the default ROS dependencies and activation hook; ROS Humble
continues using Python 3.12 in `.pixi/envs/default/`. The Isaac launchers select
that separate runtime automatically. They do not use or modify a global Isaac
installation, and `ISAAC_SIM_PATH` no longer selects the simulator.
The shared [Isaac runner](../warrior_simulation/warrior_isaac/run_isaac.sh)
clears inherited ROS/Python/Kit paths before entering that environment and
stores Kit's portable data, configuration, logs, and caches under
`.pixi/isaac-runtime/`, including texture data and Warp's `cache/warp/` kernel
cache. It leaves the user's home directory unchanged. Kit may still inspect
normal host policy and configuration locations at startup.
The host still supplies the NVIDIA GPU and driver. Run `configure-isaac` in a
terminal once after installation: it presents NVIDIA's actual EULA prompt for
the user to review and accept. Complete this step before launching through ROS,
whose simulator subprocess cannot answer interactive prompts. No task sets
automatic acceptance.
See the [Isaac guide](../warrior_simulation/warrior_isaac/README.md) for the
full-map navigation commands and runtime details.

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
