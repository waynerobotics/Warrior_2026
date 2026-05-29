# warrior_perception

Umbrella for Warrior 2026 perception drivers and processing nodes.

## Sub-packages

| Package | Build | Purpose |
| --- | --- | --- |
| [unitree_l2_lidar](unitree_l2_lidar/) | `ament_cmake` | Thin launch+config wrapper around the upstream `unitree_lidar_ros2_node`. |
| [unilidar_sdk2/](unilidar_sdk2/) | vendored | Unitree's SDK (C++ lib + ROS 2 driver `unitree_lidar_ros2`). Cloned from `github.com/unitreerobotics/unilidar_sdk2`. |
| [insta360_camera](insta360_camera/) | `ament_python` | Driver for the Insta360 X4 / X5 camera in USB / UVC mode. |
| [ai_perception](ai_perception/) | `ament_python` | YOLO wrapper: image → segmentation bitmask + 2D detections (label, score, bbox). Scaffold only — model not yet loaded. |
| [omnivision](omnivision/) | `ament_python` | 360° camera + LiDAR fusion: textured point clouds, depth maps, image overlays, mask-filtered obstacle clouds, plus yaw-calibration GUIs. |

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select \
  unitree_lidar_ros2 unitree_l2_lidar insta360_camera ai_perception omnivision
source install/setup.bash
```

The Unitree L2 expects a host NIC on `192.168.1.0/24`. One-time setup:

```bash
sudo nmcli con modify "<your-eth-conn>" \
    connection.id unitree-l2 \
    ipv4.method manual \
    ipv4.addresses 192.168.1.2/24 \
    ipv4.gateway "" ipv4.never-default yes \
    ipv6.method ignore connection.autoconnect yes
sudo nmcli con up unitree-l2
```

## Launch

Each sub-package ships a stand-alone launch file:

```bash
ros2 launch unitree_l2_lidar unitree_l2.launch.py
ros2 launch insta360_camera insta360.launch.py
ros2 launch ai_perception yolo.launch.py
```

Omnivision nodes are launched individually via `ros2 run`:

```bash
ros2 run omnivision fusion
ros2 run omnivision mask_to_pointcloud
ros2 run omnivision transformation_reconfigure
ros2 run omnivision transformation_calibrator    # OpenCV trackbar UI
ros2 run omnivision transformation_gui           # PyQt5 slider UI
```
