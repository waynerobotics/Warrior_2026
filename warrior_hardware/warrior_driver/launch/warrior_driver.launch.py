import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("warrior_driver")

    # Main configuration (loaded from the install share directory).
    warrior_driver_config = os.path.join(pkg_share, "config", "warrior_driver.yaml")

    # Auto-generated calibration file.
    #   - Loaded from the install share dir (overrides encoder_pos_forward).
    #   - Written back to the src tree so `colcon build` keeps it in sync and
    #     so we don't depend on the (possibly read-only) install dir.
    calib_load_path = os.path.join(pkg_share, "config", "steer_calibration.yaml")
    calib_write_path = os.path.expanduser(
        "~/warrior_ws/src/Warrior_2026/warrior_hardware/warrior_driver/config/steer_calibration.yaml"
    )  # adjust to your actual src path

    # Parameter layering: base config first, calibration overrides on top.
    params = [warrior_driver_config]
    if os.path.exists(calib_load_path):   # may not exist on the very first run
        params.append(calib_load_path)
    params.append({"calib.write_path": calib_write_path})

    return LaunchDescription([
        Node(
            package="warrior_driver",
            executable="warrior_driver_node",
            name="warrior_driver",
            output="screen",
            parameters=params,
        )
    ])