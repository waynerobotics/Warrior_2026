# MTi-680G simulation

`mti680g_sim` adapts Gazebo Fortress's native IMU and NavSat measurements to
the ROS sensor interface used by Warrior's waypoint navigation. It retains
NavSat's antenna-point east/north/up velocity, which the standard NavSatFix
bridge discards. It never subscribes to wheel odometry or `/odom_gt`.

| Raw Gazebo topic | ROS output | Convention |
|---|---|---|
| `/mti680g/imu` | `/imu/data` (`sensor_msgs/Imu`) | ENU attitude, body-frame angular velocity and specific force; `imu_frame` |
| `/mti680g/navsat` | `/gnss/fix` (`sensor_msgs/NavSatFix`) | WGS84 latitude/longitude/altitude at `gnss_antenna_frame` |
| `/mti680g/navsat` | `/gnss/velocity` (`geometry_msgs/TwistWithCovarianceStamped`) | Antenna-point east/north/up m/s; `gnss_enu` denotes the vector basis |

Outputs preserve source simulation timestamps and use sensor-data QoS. The
velocity's angular components are unobserved (large covariance). The separate
localization adapter rotates the linear velocity into the body frame and
corrects its antenna lever arm using the IMU. `gnss_enu` is a convention label,
not a moving TF frame.

The shared [profile](../warrior_description/config/mti680g.yaml) supplies sensor
rates, mounting, noise, and covariance. Gazebo publishes 100 Hz IMU and 10 Hz
GNSS; these are selected simulation rates, not the device's maximum rates.
The description uses one antenna and the specified 56.5 × 40.9 × 36.75 mm,
98 g sensor body. Mount offsets are provisional until measured on the robot.

The [Xsens MTi-680G product specifications](https://www.xsens.com/sensor-modules/xsens-mti-680g-rtk-gnss-ins)
and [manufacturer leaflet](https://www.xsens.com/hubfs/Downloads/Leaflets/MTi-680G.pdf)
provide 0.2° roll/pitch RMS, 0.5° heading RMS, 1 cm + 1 ppm position CEP,
0.05 m/s velocity RMS, 0.007°/s/√Hz gyro noise, and 60 µg/√Hz acceleration
noise. The model makes the following explicit approximations:

- Attitude error and measurement noise are independent Gaussian samples. Raw
  sensor noise is added once by the adapter, with matching diagonal covariance.
- Gyro/accelerometer sample deviation equals noise density × √(sample rate/2),
  assuming ideal filtering at the sample rate's Nyquist frequency.
- Horizontal position CEP converts to per-axis standard deviation as
  `(0.01 + baseline_m × 1e-6) / sqrt(2 ln 2)`. The default 1 km RTK baseline
  gives 0.00934 m east/north deviation. Vertical deviation is a chosen 0.02 m,
  not a manufacturer specification. Fix status remains `STATUS_FIX`, because
  ROS NavSatStatus has no RTK-fixed status.
- The model assumes open sky, RTK convergence, and an initialized earth-referenced
  heading. It does not emulate the proprietary INS filter, heading acquisition,
  correlated bias, correction transport, multipath, outages, or reacquisition.
  Single-antenna hardware heading performance depends on motion and conditions.

The Gazebo world must use ENU axes with spherical `heading_deg=0`, as Warrior's
current competition and IGVC worlds do. Fortress's NavSat system copies world
linear velocity into the ENU fields without correcting a rotated world. Legacy
worlds with nonzero geographic heading need a corresponding velocity transform
before using this interface.

The node starts automatically with Warrior's Gazebo simulation. For an isolated
sensor stream:

```bash
pixi run ros2 run warrior_mti680g_sim mti680g_sim --ros-args \
  --params-file warrior_description/config/mti680g.yaml -p use_sim_time:=true
```

Build and validate model units, CEP conversion, geodetic offsets, orientation,
and covariance with:

```bash
pixi run colcon build --build-base build/pixi --install-base install/pixi \
  --packages-select warrior_mti680g_sim warrior_description
pixi run ctest --test-dir build/pixi/warrior_mti680g_sim --output-on-failure
```
