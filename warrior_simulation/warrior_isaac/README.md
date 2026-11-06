# Warrior GPS navigation in Isaac Sim

Isaac runs in the repository's separate Pixi `isaac` environment at
`.pixi/envs/isaac/`, with Python 3.11 and `isaacsim[all,extscache]==5.1.0`.
The ROS Humble stack keeps its Python 3.12 `default` environment. Install Isaac
once on a Linux x86_64 (`linux-64`) workstation with a supported NVIDIA RTX GPU
and driver, then launch the robot in the complete IGVC 2027 course:

```bash
pixi run install-isaac
pixi run configure-isaac
pixi run launch-igvc2027-isaac
```

Run these commands from the repository root. The explicit install task uses
the committed Pixi lock; normal ROS/Gazebo setup does not install Isaac.
The Isaac environment does not include the ROS environment's dependencies or
activation hook. ROS remains available on both `linux-64` and `linux-aarch64`;
the optional Isaac environment is for the x86_64 GPU workstation.

Run `pixi run configure-isaac` interactively after installation and before the
first simulator launch. It presents NVIDIA's actual EULA prompt for the user
to review and accept. ROS starts Isaac through `ExecuteProcess`, which cannot
answer that interactive prompt, so complete this step in a terminal first.
An unconfigured ROS launch stops with instructions to run `configure-isaac`.
NVIDIA records acceptance inside the isolated Isaac package. Neither the
installer nor the launchers set automatic EULA acceptance. Launchers select
the repository environment and do not fall back to `$HOME/isaacsim` or
`ISAAC_SIM_PATH`. An existing shared installation is not modified.
Kit's portable data, configuration, logs, and cache live under
`.pixi/isaac-runtime/`, including its texture cache. Warp's kernel cache uses
`.pixi/isaac-runtime/cache/warp/`. The runner removes inherited ROS, Python,
and Kit paths before entering the Isaac environment, while retaining
display/GPU access and the user's normal home directory. Kit can still inspect
normal host policy and configuration locations during startup.

This launches Warrior at the shared no-man's-land spawn, starts physics and
sensors, waits for localization and Nav2, then follows the GPS waypoint YAML.
The default camera follows the robot. Closing Isaac stops the ROS processes;
Ctrl+C also stops the complete launch. The ROS launch commands below select
the isolated simulator automatically.

For launch arguments after building:

```bash
pixi run ros2 launch warrior_bringup igvc2027_isaac_nav.launch.py \
  headless:=true seconds:=150 report:=/tmp/isaac_mission.json \
  capture:=/tmp/isaac_robot.png capture_time:=35

# Open the simulator and sensors without starting waypoint navigation:
pixi run ros2 launch warrior_bringup igvc2027_isaac_nav.launch.py \
  autostart_waypoints:=false launch_rviz:=true

# Navigate using the MTi-680G GNSS/INS measurements without wheel odometry:
pixi run ros2 launch warrior_bringup igvc2027_isaac_nav.launch.py \
  use_wheel_odometry:=false
```

`waypoint_file`, `params_file`, `scene_file`, `stage_file`, `port`,
`use_wheel_odometry`, `gps_noise_stddev`, `launch_rviz`, `headless` and `seconds` are configurable.
Wheel odometry is enabled by default. With `use_wheel_odometry:=false`, the
bridge does not publish `/odom`, and neither localization filter subscribes to
wheel odometry. Both modes use the same GPS waypoints and Nav2 controller.
A zero duration runs until closed. Use a separate ROS domain and UDP port when
running another robot simulation concurrently. The default telemetry port is
17427; commands use 17428. Transport binds only to localhost.

## Robot and sensor implementation

`prepare_robot.py` expands the repository URDF and retains its visual meshes,
link transforms, mass and six actuated swerve joints. Gazebo plugins and
ros2_control tags are removed for the separate Isaac runtime. Tire collision
uses cylinders matching the visual tire's 0.10795 m radius; the axle in the STL
is excluded from the tire collider. `run_robot.py` imports the robot into
`build/isaac/` and references it into an in-memory course session. It never
saves robot changes into the generated course USD.

Actual PhysX joint drives implement swerve steering and wheel rotation at
120 Hz physics with 60 Hz command/sensor updates. Wheel odometry comes from
encoder angle increments, with continuous-joint wrapping handled. Lidar uses
PhysX raycasts at the robot's lidar mount, sampled every 1 degree horizontally
and 2 degrees vertically. The shared sensor-only ground filter estimates a
nearby road plane and follows gradual ground curvature through radial sectors.
Obstacle returns become a 360-degree `/navigation/scan` at 10 Hz. The IMU reads
the physics IMU sensor at the URDF's `imu_frame` mount. Simulated GNSS converts
the physical `gnss_antenna_frame` position through the shared WGS84 datum and
derives receiver velocity from antenna motion, including rotation about the
robot. GNSS starts after the initial two-second settling period and runs at
10 Hz independently of lidar returns.

Both simulators use the [MTi-680G profile](../../warrior_description/config/mti680g.yaml):
0.2° roll/pitch RMS, 0.5° heading RMS, 0.05 m/s velocity RMS, and horizontal
RTK accuracy of 1 cm + 1 ppm CEP. The default assumes a 1 km correction-source
baseline, giving 0.00934 m east/north Gaussian sigma after converting CEP;
0.02 m vertical sigma is a documented simulation assumption. Seeded noise is
added to IMU attitude, rates, acceleration, GNSS position and velocity; the
published covariances describe those errors. Gyroscope and accelerometer
noise densities are scaled to Isaac's actual 60 Hz sample rate. This is below
the device's maximum 400 Hz fused output rate. `gps_noise_stddev` defaults to
`-1`, which selects the profile; a nonnegative value overrides all three
position standard deviations in meters.

The model approximates nominal open-sky RTK-fixed measurements. It does not
reproduce Xsens' proprietary INS fusion, correction transport, RTK acquisition,
multipath, correlated biases, or outages. The lidar likewise does not reproduce
every detail of the Unitree L1 scan pattern.

The Pixi `isaac` environment uses Python 3.11 and the Pixi ROS environment uses
Python 3.12. A small localhost UDP adapter connects these separate processes. It
publishes `/clock`, `/joint_states`, `/imu/data`, `/gnss/fix`, `/gnss/velocity`
and `/navigation/scan`, plus `/odom` when enabled. GNSS velocity is expressed
in ENU (`gnss_enu`); localization rotates it using measured IMU attitude and
removes the antenna's rotational velocity using its static TF mount. The dual
EKFs, navsat `/fromLL` conversion, Nav2 parameters and GPS waypoint follower
are reused. The physical pose is
published separately as `/isaac/ground_truth` for validation; neither EKF
fuses that topic. Sensor mounting follows the same expanded URDF as ROS TF.
Commands require finite values, a current timestamp and an expected body
frame; a 0.75-second command watchdog stops the joint-drive command.

Before the MTi-680G update, native GUI validation reached all four no-man's-land GPS goals on
the DEM terrain in about 103 simulation seconds and then stopped, with no
planning failures. The robot-follow viewport and its motion were verified in
the actual application window. A separate straight-drive check measured
1.747 m physical travel versus 1.748 m wheel odometry, with physical lidar
returns and gravity in the IMU acceleration. Bounded reports and rendered
previews are recorded with the shared course validation artifacts. The final
stage passes OpenUSD checks for 942 meshes, 203 colliders, two cameras and
three local texture dependencies. A separate physical command-loss check
verified that stale and non-finite commands do not restart the robot; after
the watchdog stopped it, displacement was 6.5 mm over the remaining test.

Review the [native GUI capture](../igvc2027/preview/isaac_gui.png),
[moving robot render](../igvc2027/preview/isaac_robot.png),
[bounded mission report](../igvc2027/validation/isaac_robot_report.json) and
[command watchdog report](../igvc2027/validation/isaac_watchdog_report.json).
These artifacts predate the new sensor profile and isolated installation.
The MTi-680G bridge has separate tests for both navigation modes, sensor
covariance and sampled noise, and URDF
mount composition, runnable without an Isaac GPU:

```bash
pixi run python -m unittest discover -s warrior_simulation/warrior_isaac -v
```

The ROS tests need access to local DDS/UDP sockets. An eight-second native
headless check was repeated in the isolated Pixi environment on an RTX 5090
after the user accepted NVIDIA's EULA through `configure-isaac`. It completed
479 simulation frames, received 479 IMU packets and 80 GNSS samples, verified
the exact antenna transform, and measured 0.3012 m/s GNSS speed for a 0.3 m/s
physical drive command. This validates the isolated native sensor path.

A separate 30-second ROS launch without wheel odometry completed 1,799 frames,
initialized localization and Nav2, reached the first waypoint, and began the
second before a clean bounded shutdown. IMU, GNSS and local/global/GPS odometry
streams were observed, with zero `/odom` publishers or subscribers. Runtime
logs confirmed the project Kit and texture-cache paths without watcher
failures or error messages; a separate Warp initialization check verified its
project cache path. A full waypoint mission remains unvalidated in the isolated
environment.

## Environment asset

`worlds/igvc2027_autonav.usda` uses the same ENU meter coordinates, geometry,
datum and materials as the [shared course](../igvc2027/). Competition markers
are a simulation approximation, not surveyed 2027 waypoint positions.
The standalone USDA is an environment asset; use the navigation launch above
to spawn the robot and start its mission. To inspect only the environment:

```bash
bash warrior_simulation/warrior_isaac/launch_course.sh
bash warrior_simulation/warrior_isaac/launch_course.sh --headless --frames 30 \
  --capture /tmp/igvc2027_overview.png
```

The RTX realtime renderer (`RaytracedLighting`) uses local PBR textures,
normals and UVs, lighting, camera views, static collision meshes, contact
friction, gravity and `/World/RobotSpawn`. Keep `warrior_isaac/` and sibling
`igvc2027/` together when moving the environment. Regenerate and validate it:

```bash
python3 warrior_simulation/warrior_isaac/export_usd.py \
  warrior_simulation/igvc2027/scene.json \
  warrior_simulation/warrior_isaac/worlds/igvc2027_autonav.usda
bash warrior_simulation/warrior_isaac/validate_stage.sh \
  --scene warrior_simulation/igvc2027/scene.json
```

The exporter uses Python's standard library. The validation wrapper uses
the isolated Isaac environment's OpenUSD bindings without starting the simulator,
and checks composition, mesh topology, material bindings, textures, physics metadata and agreement with
the shared scene. Environment renders and robot physics were also exercised in
Isaac Sim 5.1 on an RTX 5090 with driver 580.178.04.

`launch_robot.sh`, `launch_course.sh`, and `validate_stage.sh` delegate to
`run_isaac.sh` and its `runtime.py` bootstrap. The stage validator loads the
bundled OpenUSD bindings directly; it does not start Isaac, require a GPU, or
accept the simulator EULA.

Installation of the isolated environment has been checked with Python 3.11.18
and Isaac Sim package version 5.1.0.0. Its OpenUSD validator passed for all 942
meshes, 203 colliders, two cameras and three local textures, with no warnings.
The native GPU sensor and bounded ROS integration checks are described above.

Implementation references:
[Isaac Sim 5.1 SimulationApp](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.simulation_app/docs/index.html),
[URDF import configuration](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/api/structisaacsim_1_1asset_1_1importer_1_1urdf_1_1_import_config.html),
[physics IMU sensor](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physics_imu.html),
[USD physics schema](https://openusd.org/release/api/usd_physics_page_front.html).
