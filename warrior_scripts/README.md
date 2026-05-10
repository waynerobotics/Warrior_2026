# Warrior 2026 — Layered Install & Bring-up Plan

Target stack: **ROS 2 Humble** + **Gazebo Fortress** (a.k.a. Ignition Fortress) on **Ubuntu 22.04** (Jammy), running under WSL2 with WSLg for GPU/display.

This directory holds the layered installer for the Warrior 2026 workspace.
Each section of [installWarriorDependencies.sh](installWarriorDependencies.sh)
maps 1:1 to a bring-up gate: install one layer, validate it works in
isolation, then move on. Running the script with no arguments runs every
implemented section in order, so as the project grows it becomes a
single-command full install.

---

## Why layered

A full ROS 2 + Gazebo + Nav2 + ros2_control stack has many moving parts.
If you install everything at once and something breaks, the search space
is huge. Installing layer by layer means each gate either passes or
points directly at the layer that broke. This is especially valuable
under WSL2, where GPU, USB, and networking edge cases can masquerade as
ROS bugs.

---

## How to use the installer

```bash
# Full install (== all currently-implemented sections, in order)
sudo bash installWarriorDependencies.sh

# Just the ROS + Gazebo Fortress base
sudo bash installWarriorDependencies.sh core

# Multiple specific sections, in the order you list them
sudo bash installWarriorDependencies.sh core control teleop

# Show help
bash installWarriorDependencies.sh --help
```

Re-running a section is idempotent — `apt install` skips already-installed
packages, locale generation is a no-op once set, and the `.bashrc` lines
are de-duplicated by `grep -qxF`.

---

## Section table

| # | Section | Status | What it installs (high level) | Bring-up gate it unlocks |
|---|---|---|---|---|
| 1 | `core` | implemented | ROS 2 Humble desktop, dev tools, xacro/URDF, robot_state_publisher, joint_state_publisher(+gui), RViz2, ros_gz_sim/bridge/image/interfaces (pulls Gazebo Fortress), `.bashrc` source lines + user-extras block | **G1** URDF/TF in RViz · **G2** Empty Gz world with robot spawned |
| 2 | `control` | pending | `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-gz-ros2-control`, `libeigen3-dev` | **G3** Controllers active in sim (joint_state_broadcaster + swerve/diff drive) |
| 3 | `teleop` | pending | `ros-humble-joy`, `ros-humble-teleop-twist-joy`, `ros-humble-teleop-twist-keyboard` | **G4** Joystick drives the robot in Gz |
| 4 | `localization` | pending | `ros-humble-robot-localization`, `ros-humble-tf2-ros`, `ros-humble-tf2-tools` | **G5** EKF publishes `/odometry/filtered`, full `map → odom → base_link` TF chain |
| 5 | `navigation` | pending | `ros-humble-navigation2`, `ros-humble-nav2-bringup` | **G6** Nav2 plans and executes a goal in sim |
| 6 | `gps` | pending | `ros-humble-nmea-msgs`, `ros-humble-mavros-msgs`, `python3-pupil-apriltags`, geographic msgs | **G7** AprilTag/GPS bridge produces world-frame waypoints |
| 7 | `hardware` | pending | `python3-serial`, `ros-humble-rosbridge-suite`, `ros-humble-rosapi`, anything else needed for the real robot | **G8** Real Warrior hardware brings up identically to sim |
| 8 | `workspace` | pending | `colcon build --symlink-install` of the full `ros2_ws` after all packages compile cleanly | Workspace fully built and sourced |

---

## Bring-up gates (run in order)

Each gate validates the layer the matching install section just enabled.
Don't move past a failing gate — fix it first. All commands assume
`source /opt/ros/humble/setup.bash` and (after at least one successful
build) `source ~/ros2_ws/install/setup.bash`.

### G1 — URDF parses and TF tree is complete (after `core`)

```bash
ros2 launch warrior_description display.launch.py     # if present
# or, equivalent manual check:
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro src/Warrior_2026/warrior_description/urdf/warrior.urdf.xacro)"
ros2 run joint_state_publisher_gui joint_state_publisher_gui
rviz2
```

Expected: complete TF tree in RViz, no xacro errors.

### G2 — Empty Gazebo world spawns the robot (after `core`)

```bash
ros2 launch warrior_bringup main.launch.py \
    robot_type:=swerve_sim \
    world_name:=empty.world
```

Expected: Gazebo Fortress GUI opens, robot loaded, `/clock` ticks. At
this stage **controllers will fail to load** — that's fine, gate G2 only
checks that the simulator and the robot description meet.

WSL2 sanity check:

```bash
glxinfo | grep -i "OpenGL renderer"   # should show your GPU, not llvmpipe
echo "$WAYLAND_DISPLAY $DISPLAY"      # WSLg sets these
```

### G3 — Controllers active (after `control`)

```bash
# while G2 is still running:
ros2 control list_controllers
```

Expected: `joint_state_broadcaster` and `swerve_drive_controller` (or
`diff_drive_controller`) both in state `active`.

### G4 — Teleop drives the robot (after `teleop`)

```bash
ros2 launch warrior_joy joy_swerve.launch.py
```

Expected: stick input drives the robot in Gazebo. WSL2: bind the
joystick with `usbipd` first (see "WSL2 hardware notes" below).

### G5 — EKF localization (after `localization`)

```bash
ros2 launch warrior_localization ekf.launch.py
ros2 topic hz /odometry/filtered
ros2 run tf2_tools view_frames
```

Expected: `/odometry/filtered` publishing, `map → odom → base_link`
chain present in the TF view.

### G6 — Nav2 (after `navigation`)

```bash
ros2 launch warrior_navigation nav2_complex_path.launch.py
```

Send a goal in RViz; expect a plan and motion. Costmaps should populate.

### G7 — GPS / AprilTag (after `gps`)

```bash
ros2 launch warrior_gps gps_localization.launch.py
ros2 launch warrior_navigation gps_waypoint_follower.launch.py
```

Expected: AprilTag detections, world-frame GPS waypoints, follower
executes the demo waypoint set.

### G8 — Real hardware (after `hardware`)

```bash
ros2 launch warrior_bringup main.launch.py robot_type:=warrior_real
```

Expected: same TF / control behavior as sim, real motors respond. Needs
the Arduino bound into WSL2 via `usbipd` if running from WSL.

---

## WSL2 notes

- **GPU / display**: WSLg on Windows 11 forwards the GPU automatically.
  Verify with `glxinfo` after Phase 1. If you see `llvmpipe`, install
  the latest NVIDIA/AMD WSL-aware driver on the Windows side and
  restart WSL.
- **USB passthrough** (joystick, Arduino, GPS dongle): install
  [`usbipd-win`](https://github.com/dorssel/usbipd-win) on Windows,
  then `usbipd bind --busid <id>` and `usbipd attach --wsl --busid <id>`.
  Defer this until you're working on `teleop` or `hardware`.
- **Networking**: ROS 2 multicast generally just works on WSLg's NAT;
  if you need to reach hosts outside WSL, set
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and `ROS_DOMAIN_ID` consistently.

---

## `.bashrc` extras

The `core` section appends a marked block (`# >>> warrior-bashrc-extras >>>`
… `# <<< warrior-bashrc-extras <<<`) to `~/.bashrc` containing aliases and
env vars captured from the pre-wipe shell:

- `sorce` — typo-tolerant alias for `source ~/.bashrc`
- `ign_gazebo` — short alias for `ign gazebo`
- `ros_rebuild` — `cd ~/ros2_ws && colcon build && source install/setup.bash`
- `mcp_activate` — sources Humble + the `~/ros2_mcp_env` venv
- `go2_sim` — Unitree Go2 sim launch shortcut
- `LD_LIBRARY_PATH` export for `faster-whisper` GPU (cuDNN under
  `~/.local/lib/python3.10/site-packages/nvidia/cudnn/lib`)

The block is **idempotent**: re-running `core` deletes the old block and
writes a fresh copy, so all customizations belong in
[installWarriorDependencies.sh](installWarriorDependencies.sh) — never
edit `~/.bashrc` by hand inside the markers.

The pre-wipe `~/.bashrc` also sourced `/opt/ros/jazzy/setup.bash`. That
line is intentionally dropped because the fresh install is Humble-only;
re-add it manually if you ever install Jazzy alongside.

---

## Adding a new section

When a layer is ready to be promoted from "pending" to "implemented":

1. Replace `section_pending "<name>"` with a real `section_<name>()` in
   [installWarriorDependencies.sh](installWarriorDependencies.sh).
2. Uncomment the matching `run_section <name>` line in `main()` so that
   `all` picks it up.
3. Update the **Status** column in the section table above.
4. Add or refine the matching gate description in this README if the
   commands change.

The script's `set -euo pipefail` means any failed `apt install` aborts
the install, so partial states don't leak between sections.
