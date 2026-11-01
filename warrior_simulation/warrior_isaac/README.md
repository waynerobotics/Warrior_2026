# Warrior GPS navigation in Isaac Sim

Run the robot and the complete IGVC 2027 simulation course from the repository root:

```bash
pixi run --locked launch-igvc2027-isaac
```

This launches Warrior at the shared no-man's-land spawn, starts physics and
sensors, waits for localization and Nav2, then follows the GPS waypoint YAML.
The default camera follows the robot. Closing Isaac stops the ROS processes;
Ctrl+C also stops the complete launch. Isaac Sim 5.1 and a supported NVIDIA RTX
GPU/driver are required. Set `ISAAC_SIM_PATH` if Isaac is not at `$HOME/isaacsim`.

For launch arguments after building:

```bash
pixi run --locked ros2 launch warrior_bringup igvc2027_isaac_nav.launch.py \
  headless:=true seconds:=150 report:=/tmp/isaac_mission.json \
  capture:=/tmp/isaac_robot.png capture_time:=35

# Open the simulator and sensors without starting waypoint navigation:
pixi run --locked ros2 launch warrior_bringup igvc2027_isaac_nav.launch.py \
  autostart_waypoints:=false launch_rviz:=true
```

`waypoint_file`, `params_file`, `scene_file`, `stage_file`, `port`,
`gps_noise_stddev`, `launch_rviz`, `headless` and `seconds` are configurable.
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
the physics IMU sensor. Simulated GPS converts the physical
antenna position through the shared WGS84 datum, with seeded 3 cm Gaussian
noise by default. GPS starts after the initial two-second settling period.
These are simulation sensor models, not calibrated models of every device
error or the Unitree L1 scan pattern.

Isaac 5.1 embeds Python 3.11 and this repository's Pixi ROS environment uses
Python 3.12. A small localhost UDP adapter keeps these runtimes separate. It
publishes `/clock`, `/joint_states`, `/odom`, `/mobile_sensor/imu`, `/gps1/fix`
and `/navigation/scan`. The existing dual EKFs, navsat `/fromLL` conversion,
Nav2 parameters and GPS waypoint follower are reused. The physical pose is
published separately as `/isaac/ground_truth` for validation; neither EKF
fuses that topic. The original `gps1_frame` antenna transform is preserved.
Commands require finite values, a current timestamp and an expected body
frame; a 0.75-second command watchdog stops the joint-drive command.

The final native GUI validation reached all four no-man's-land GPS goals on
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

The exporter uses Python's standard library. The validation wrapper locates
Isaac's OpenUSD bindings without starting the simulator, and checks composition,
mesh topology, material bindings, textures, physics metadata and agreement with
the shared scene. Environment renders and robot physics were also exercised in
Isaac Sim 5.1 on an RTX 5090 with driver 580.178.04.

Implementation references:
[Isaac Sim 5.1 SimulationApp](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.simulation_app/docs/index.html),
[URDF import configuration](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/api/structisaacsim_1_1asset_1_1importer_1_1urdf_1_1_import_config.html),
[physics IMU sensor](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physics_imu.html),
[USD physics schema](https://openusd.org/release/api/usd_physics_page_front.html).
