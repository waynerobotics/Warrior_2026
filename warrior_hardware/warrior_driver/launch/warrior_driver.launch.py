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

    # The src path differs per machine. Try known workspace roots in order and
    # use the first whose warrior_driver/config dir actually exists.
    #   - ~/warrior_ws/src is the PRIMARY (robot PC) and stays the default even
    #     if it's absent on this machine — it must keep working there.
    #   - ~/ros2_ws/src is the dev-PC fallback.
    _calib_rel = "Warrior_2026/warrior_hardware/warrior_driver/config/steer_calibration.yaml"
    _src_roots = [
        os.path.expanduser("~/warrior_ws/src"),   # robot PC (primary)
        os.path.expanduser("~/ros2_ws/src"),      # dev PC fallback
    ]
    calib_write_path = next(
        (os.path.join(r, _calib_rel) for r in _src_roots
         if os.path.isdir(os.path.dirname(os.path.join(r, _calib_rel)))),
        os.path.join(_src_roots[0], _calib_rel),   # default: primary robot path
    )

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