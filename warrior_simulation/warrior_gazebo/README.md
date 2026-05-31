# warrior_gazebo

Sim worlds + SDF models for the Warrior 2026 robot (`ament_cmake`, install
of `worlds/` + `models/` only — no nodes, no launch files here). See the
[warrior_simulation README](../README.md) for the package overview and how
worlds get launched.

## Target stack

**ROS 2 Humble** + **Gazebo Fortress** (a.k.a. Ignition Fortress) on
**Ubuntu 22.04**. Not compatible with:

- **Gazebo Classic** (9/10/11) — different ABI, EOL Jan 2025. Don't install
  `ros-humble-gazebo-ros-pkgs` / `gazebo-ros` / `gazebo-ros2-control`.
- **Gazebo Garden / Harmonic / Ionic** — newer, but `gz_ros2_control` ABI
  differs on Humble.

## Worlds

`worlds/` — loaded by the bringup sim launches via the `world_name:=` arg
(see [warrior_simulation README](../README.md)).

| File | What it is |
| --- | --- |
| `competition.world` | Full IGVC course (barrels, track) — default for swerve_sim |
| `competition_no_barrels.world` | Competition layout minus obstacle barrels |
| `fullTrack.world` | Long-form practice track |
| `map1.world` | Practice map |
| `map1ramp.world` | Practice map with a ramp |
| `empty.world` | Bare ground + sun — controller / gate smoke tests |
| `empty_test.world` | Empty variant for plugin debugging |
| `turtlebot3_world_gps.world` | TurtleBot3 world + GPS — default for `turtlebot_sim` |

## Models

`models/` — SDF assets auto-discovered by Fortress:

- obstacles: `construction_barrel`, `construction_cone`, `falling_rock3`,
  `first_2015_trash_can`, `stop_sign`
- track / scene: `igvc_track`, `sonoma_raceway`, `ramp`, `waypoint_flag`,
  `oak_tree`, `pine_tree`, `ground_plane`, `sun`
- robot: `turtlebot3_burger_gps`

Discovery: the sim launch
([warrior_control/launch/swerve_drive.gazebo.launch.py](../../warrior_control/launch/swerve_drive.gazebo.launch.py))
appends `<share>/warrior_gazebo/models` to `IGN_GAZEBO_RESOURCE_PATH` and
`GZ_SIM_RESOURCE_PATH` before starting `ros_gz_sim`.
