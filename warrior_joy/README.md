# warrior_joy

Gamepad → `cmd_vel`. Three flavours depending on what's downstream. See the
[workspace README](../README.md) for context.

```mermaid
flowchart LR
    PAD["Xbox / RadioLink pad"] --> JOY["joy_node"]
    JOY -->|sensor_msgs/Joy| MUX{launch variant}
    MUX -->|joy_swerve| S["/swerve_drive_controller/cmd_vel"]
    MUX -->|joy_2stick| D["/diff_drive_controller/cmd_vel"]
    MUX -->|joy_turtle| T["/cmd_vel (TurtleBot3)"]
```

## Launchers

| Launcher | For | cmd_vel topic |
|---|---|---|
| `joy_swerve.launch.py` | swerve drive (sim or real) | `/swerve_drive_controller/cmd_vel` |
| `joy_2stick.launch.py` | diff drive (sim or real Warrior) | `/diff_drive_controller/cmd_vel` |
| `joy_turtle.launch.py` | TurtleBot3 | `/cmd_vel` |

ToDo: figure out the one Yihao is using and add it here / set it as the defualt

Run alongside any robot launcher from [warrior_bringup](../warrior_bringup/README.md):

```bash
ros2 launch warrior_joy joy_swerve.launch.py
```

Pad mapping is in `config/`. Axes/buttons follow the standard Xbox layout.

> **Real swerve robot:** don't use `joy_swerve` for it. The real
> `swerve_drive_controller` subscribes to **`/cmd_vel` as `TwistStamped`**,
> but `joy_swerve.py` publishes a plain (unstamped) `Twist`, so the controller
> ignores it. The working real-robot path is `teleop_twist_joy` driven by the
> `teleop_node` block in `config/joystick.yaml` (`publish_stamped_twist: true`),
> wired up end-to-end — including `warrior_driver` — by
> [`warrior_bringup/launch/warrior_swerve_teleop.launch.py`](../warrior_bringup/launch/warrior_swerve_teleop.launch.py).
