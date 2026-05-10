# warrior_gazebo

Simulation assets (worlds and models) for the Warrior 2026 robot.

## Target stack — read this first

This package targets, **and only targets**:

- **ROS 2 Humble Hawksbill** (Ubuntu 22.04 / Jammy)
- **Gazebo Fortress**, also known as **Ignition Fortress** — the modern
  Gazebo line (formerly branded "Ignition", rebranded back to "Gazebo"
  in 2022). Fortress is the Tier 1 binary release that pairs with
  ROS 2 Humble.

It is **not** compatible with:

- **Gazebo Classic** (Gazebo 9/10/11) — different binary, different
  plugin ABI, different SDF dialects, EOL'd in January 2025. Do not
  install `ros-humble-gazebo-ros-pkgs`, `ros-humble-gazebo-ros`, or
  `ros-humble-gazebo-ros2-control` — those are the classic-Gazebo
  packages and they will conflict with this package's launch files.
- **Gazebo Garden / Harmonic / Ionic** — these are newer modern-Gazebo
  releases, but their `ros_gz` bindings on Humble are not Tier 1 and
  some plugins (notably `gz_ros2_control`) have ABI differences.

If you find documentation in this repo (including the top-level
`README.md`) that mentions Jazzy or Gazebo Harmonic, treat it as stale.
The authoritative install path is
[warrior_scripts/installWarriorDependencies.sh](../../warrior_scripts/installWarriorDependencies.sh)
and [warrior_scripts/README.md](../../warrior_scripts/README.md).

## Required apt packages (sim base)

```
ros-humble-ros-gz-sim
ros-humble-ros-gz-bridge
ros-humble-ros-gz-image
ros-humble-ros-gz-interfaces
```

These pull `ignition-fortress` as a transitive dependency, which
provides the actual Fortress simulator binary (`ign gazebo`). No
separate `packages.osrfoundation.org` apt repo is required — Fortress
ships through the ROS 2 apt repo for Humble.

For ros2_control integration, `ros-humble-gz-ros2-control` is the
Fortress-compatible package (note: `gz-`, not `gazebo-`). That comes
in via the `control` install section, not this one.

## Layout

```
warrior_gazebo/
├── CMakeLists.txt        # ament_cmake, installs worlds/ and models/
├── package.xml
├── worlds/               # SDF world files (Fortress dialect)
└── models/               # SDF model assets referenced by worlds
```

### World files

| File | Purpose |
|---|---|
| `empty.world` | Bare ground plane + sun. Used for gate **G2** and controller smoke tests. |
| `empty_test.world` | Variant of empty for plugin debugging. |
| `competition.world` | Full IGVC course. Default for `swerve_sim`. |
| `competition_no_barrels.world` | Competition layout minus obstacle barrels. |
| `fullTrack.world` | Long-form practice track. |
| `map1.world`, `map1ramp.world` | Practice map variants (with ramp). |

Other world files referenced by launch files (e.g. `turtlebot3_world_gps.world`)
live under [warrior_description/worlds/](../../warrior_description/worlds/).

### Models

`models/` contains the SDF assets used by the worlds above —
construction barrels and cones, IGVC track elements, trees, ramps,
waypoint flags, the `turtlebot3_burger_gps` model, etc. They are
auto-discovered by Fortress when `IGN_GAZEBO_RESOURCE_PATH` includes
`<install>/share/warrior_gazebo/models` (set by this package's
`hook/` exports after build).

## Quick launch (without controllers)

After running the `core` install section and a colcon build:

```bash
ros2 launch warrior_bringup main.launch.py \
    robot_type:=swerve_sim \
    world_name:=empty.world
```

At this stage controllers will fail to spawn — that's expected until the
`control` install section lands.

## WSL2 caveat

Fortress under WSL2 needs WSLg with working GPU passthrough. Verify
with `glxinfo | grep "OpenGL renderer"` — if you see `llvmpipe`,
software rendering will work but the simulator will be slow. Update
the Windows-side GPU driver to a WSL-aware build before assuming
Fortress itself is broken.
