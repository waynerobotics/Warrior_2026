# warrior_localization

Sensor fusion + mapping. Wraps `robot_localization` (EKF + navsat) and
`slam_toolbox`. Outputs filtered odometry, the `map`→`odom`→`base_footprint` TF
chain, and (in SLAM) an occupancy map.

ament_python. Most work is config + launch over upstream nodes; one local node.

## Data flow

```mermaid
flowchart LR
    ODOM[/odom/] --> EKFL[ekf_node_odom<br/>world=odom]
    IMU[/imu/] --> EKFL
    EKFL -->|odometry/local<br/>odom→base_footprint TF| TF
    GPS[/gps/fix/] --> NAVSAT[navsat_transform]
    IMU --> NAVSAT
    NAVSAT -->|odometry/gps| EKFG[ekf_node_map<br/>world=map]
    ODOM --> EKFG
    EKFG -->|odometry/global<br/>map→odom TF| TF[TF tree]
    SCAN[/scan/] --> SLAM[slam_toolbox] -->|/map| TF
```

EKF runs in two modes: single (odom only) or **dual + navsat** (global GPS-fused).

## Nodes

- No local nodes. Everything is upstream: `robot_localization` (EKF + navsat),
  `slam_toolbox` (mapping).
- The robot pose in the `map` frame is published directly by the localization
  node (the old `map_robot_pose` helper was removed).

## Launch — [launch/](launch/)

| File | Brings up | Notes |
|---|---|---|
| [ekf.launch.py](launch/ekf.launch.py) | one `ekf_node` (`ekf_node_odom`) → `/odometry/filtered` | local odom fusion only. `use_sim_time` arg (default `true`) |
| [dual_ekf_navsat.launch.py](launch/dual_ekf_navsat.launch.py) | `ekf_node_odom` (→`odometry/local`) + `ekf_node_map` (→`odometry/global`) + `navsat_transform` | full GPS-fused stack |
| [gps_sim.launch.py](launch/gps_sim.launch.py) | two EKFs + one navsat transform using the Warrior Gazebo sensor topics | `use_sim_time` (default `true`), `params_file`; included by `warrior_bringup/swerve_gps_nav_sim.launch.py` |
| [online_async_launch.py](launch/online_async_launch.py) | `slam_toolbox/async_slam_toolbox_node` (lifecycle, autostarted) | SLAM mapping. Args: `autostart`, `use_lifecycle_manager`, `use_sim_time`, `slam_params_file` |

## Config — [config/](config/)

### [gps_sim_ekf.yaml](config/gps_sim_ekf.yaml)

The GPS simulation fuses `/odom` wheel velocities and `/mobile_sensor/imu`
yaw rate into continuous `/odometry/local`. Its global filter adds the IMU's
absolute ENU heading and `/odometry/gps` positions converted from `/gps1/fix`.
The local filter owns `odom` → `base_footprint`, so the swerve controller's
`enable_odom_tf` must be `false` in this mode. The GPS bringup sets this override.

The `map` origin is initialized from the first GPS fix and filtered base pose;
its axes use east/north/up, independent of the robot's spawn heading. The
Gazebo IMU explicitly uses ENU orientation. `navsat_transform` accounts for UTM
grid convergence and serves `/fromLL` to convert each latitude/longitude
waypoint into that same `map` frame. The GPS antenna offset is read from TF.
Startup waits for `/odometry/gps` before sending goals.

No saved occupancy map, SLAM pose, or `/odom_gt` pose feeds these filters.
LiDAR remains an obstacle sensor for Nav2. This configuration exercises the
simulated sensors; their ideal GPS/heading behavior does not characterize the
noise, bias, or dropouts of the physical robot's sensors.

### [ekf_config.yaml](config/ekf_config.yaml)
- `ekf_node_odom`: `world_frame=odom`, fuses `odom0=/odom` + `imu0=/imu`, `two_d_mode`, 30 Hz, `publish_tf`.
- `ekf_node_map`: `world_frame=map`, adds `odom1=/odometry/gps`.
- `navsat_transform`: `gps/fix`+`imu`+`odometry/global` → `odometry/gps`; `magnetic_declination_radians=0`, `zero_altitude`, `publish_filtered_gps`, `wait_for_datum=false`.

### [mapper_params_online_async.yaml](config/mapper_params_online_async.yaml)
- slam_toolbox: `mode: mapping` (toggle to `localization`), `scan_topic=/scan`, frames `map`/`odom`/`base_footprint`, `resolution 0.05`.

## Frames

`map` → `odom` (ekf_node_map / slam_toolbox) → `base_footprint` (ekf_node_odom).
Run only **one** map-frame TF publisher at a time (dual-EKF **or** SLAM, not both).

## Notes
- `package.xml` description/maintainer/license are TODO placeholders.
