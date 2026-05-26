# warrior_scripts

First-time install + dev helpers for the Warrior workspace. See the
[workspace README](../README.md) for context.

## First-time install

```bash
sudo bash installWarriorDependencies.sh
source ~/.bashrc
ros_rebuild
```

`installWarriorDependencies.sh` adds the ROS 2 apt repo and installs:

- ROS 2 **Humble** desktop
- **Gazebo Fortress** (`ros-humble-ros-gz`)
- `ros2_control`

It also appends two aliases to `~/.bashrc`:

| Alias | Expands to |
|---|---|
| `sorce` | `source ~/.bashrc` |
| `ros_rebuild` | `cd ~/ros2_ws && colcon build && source install/setup.bash` |

## Files

| File | Purpose |
|---|---|
| `installWarriorDependencies.sh` | One-shot installer (run with `sudo`) |
| `build_warrior_ws.sh` | Wrapper around `colcon build` |
| `deprecated_installWarriorDeps.sh` | Old installer — kept for reference, don't use |

## Launching

This package only sets up the environment. To launch the robot, see
[warrior_bringup](../warrior_bringup/README.md).
