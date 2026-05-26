# warrior_gazebo

Sim worlds + SDF models for the Warrior 2026 robot. See the
[workspace README](../../README.md) for context, and
[warrior_bringup](../../warrior_bringup/README.md) for launching the sim.

## Target stack

**ROS 2 Humble** + **Gazebo Fortress** (a.k.a. Ignition Fortress) on
**Ubuntu 22.04**. Not compatible with:

- **Gazebo Classic** (9/10/11) — different ABI, EOL Jan 2025. Don't install
  `ros-humble-gazebo-ros-pkgs` / `gazebo-ros` / `gazebo-ros2-control`.
- **Gazebo Garden / Harmonic / Ionic** — newer, but `gz_ros2_control` ABI
  differs on Humble.

## Worlds

| File | Purpose |
|---|---|
| `competition.world` | Full IGVC course — default for `swerve_sim` |
| `competition_no_barrels.world` | Competition layout minus obstacle barrels |
| `empty.world` | Bare ground plane + sun. Gate G2 / controller smoke tests |
| `empty_test.world` | Variant of empty for plugin debugging |
| `fullTrack.world` | Long-form practice track |
| `map1.world`, `map1ramp.world` | Practice map variants (with ramp) |

`turtlebot3_world_gps.world` lives in
[warrior_description/worlds/](../../warrior_description/worlds/), not here.

## Models

`models/` holds SDF assets used by the worlds — barrels, cones, IGVC track
elements, trees, ramps, waypoint flags, and the `turtlebot3_burger_gps`
model. Fortress auto-discovers them when `IGN_GAZEBO_RESOURCE_PATH` includes
`<install>/share/warrior_gazebo/models` — set by this package's `hook/`
exports after build.
