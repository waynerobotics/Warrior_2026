# Warrior Bringup

### Run manually

```bash
ros2 launch warrior_bringup warrior_swerve_teleop.launch.py
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
sudo nano /etc/systemd/system/warrior-2026.service
```

Paste:

```ini
[Unit]
Description=Warrior 2026 ROS 2 Launch
After=network.target bluetooth.service

[Service]
Type=simple
User=fire
Environment=ROS_DOMAIN_ID=30
ExecStart=/home/fire/ros2_ws/src/Warrior_2026/warrior_bringup/scripts/on_start.sh
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
sudo systemctl enable warrior-2026.service
sudo systemctl start warrior-2026.service
```

3. **Check status / logs:**

```bash
sudo systemctl status warrior-2026.service
journalctl -u warrior-2026.service -f
```

4. **Stop or disable:**

```bash
sudo systemctl stop warrior-2026.service
sudo systemctl disable warrior-2026.service
```

> **Note:** The Xbox controller must be paired before the service starts, or
> `joy_node` will wait until a joystick appears. The motor_manager will keep
> retrying USB discovery regardless, so plugging in swerve Arduinos after boot
> is fine.
