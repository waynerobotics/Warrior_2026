# Warrior_2026

This repo will contain all ROS2 Humble code needed for the 2026 IGVC competition for Wayne Robotics. 

#These packages need to be installed:
Make sure to also run
sudo apt update first.

```bash
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-joint-state-publisher \
    ros-humble-robot-localization \
    ros-humble-nav2-bringup \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-nmea-msgs \
    ros-humble-mavros-msgs \
    ros-humble-rosbridge-server \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-gazebo-ros2-control \
    ros-humble-gz-ros2-control

```

---

## Swerve Teleop — Xbox Controller

### Run manually

```bash
~/ros2_ws/src/Warrior_2026/warrior_scripts/launch_swerve_teleop.sh
```

Or directly:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch warrior_bringup warrior_swerve_teleop.launch.py
```

**RB** cycles the active swerve: `02_swerve → 03_swerve → 04_swerve → 02_swerve …`  
Left stick Y = spark motor, right stick X = flipsky motor.

---

### Launch on startup (systemd)

1. **Create the service file:**

```bash
sudo nano /etc/systemd/system/warrior-swerve-teleop.service
```

Paste:

```ini
[Unit]
Description=Warrior Swerve Teleop ROS 2 Launch
After=network.target bluetooth.service

[Service]
Type=simple
User=fire
Environment=ROS_DOMAIN_ID=30
ExecStart=/home/fire/ros2_ws/src/Warrior_2026/warrior_scripts/launch_swerve_teleop.sh
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

2. **Enable and start:**

```bash
sudo systemctl daemon-reload
sudo systemctl enable warrior-swerve-teleop.service
sudo systemctl start warrior-swerve-teleop.service
```

3. **Check status / logs:**

```bash
sudo systemctl status warrior-swerve-teleop.service
journalctl -u warrior-swerve-teleop.service -f
```

4. **Stop or disable:**

```bash
sudo systemctl stop warrior-swerve-teleop.service
sudo systemctl disable warrior-swerve-teleop.service
```

> **Note:** The Xbox controller must be paired before the service starts, or
> `joy_node` will wait until a joystick appears. The motor_manager will keep
> retrying USB discovery regardless, so plugging in swerve Arduinos after boot
> is fine.
