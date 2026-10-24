import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _is_writable_target(path):
    """Return whether an existing file or its parent directory is writable."""
    if path.exists():
        return os.access(path, os.W_OK)
    return path.parent.is_dir() and os.access(path.parent, os.W_OK)


def _calibration_write_path(packaged_calibration):
    """Choose durable writable storage, preferring this repository's config."""
    workspace = os.environ.get("WARRIOR_WORKSPACE")
    if workspace:
        source_calibration = (
            Path(workspace).expanduser()
            / "warrior_hardware"
            / "warrior_driver"
            / "config"
            / "steer_calibration.yaml"
        )
        if _is_writable_target(source_calibration):
            return source_calibration.resolve(strict=False)

    # With a colcon symlink install, the package-share file resolves back to
    # the source tree. Preserve that useful write-back behavior even when the
    # activation hook did not provide WARRIOR_WORKSPACE.
    packaged_calibration = packaged_calibration.absolute()
    resolved_calibration = packaged_calibration.resolve(strict=False)
    if (
        resolved_calibration != packaged_calibration
        and _is_writable_target(resolved_calibration)
    ):
        return resolved_calibration

    # A copied/read-only install has no source tree to update. Keep the robot's
    # calibration outside the install prefix so environment recreation cannot
    # discard it.
    config_home = Path(
        os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")
    ).expanduser()
    persistent_calibration = (
        config_home / "warrior-2026" / "warrior_driver" / "steer_calibration.yaml"
    )
    persistent_calibration.parent.mkdir(parents=True, exist_ok=True)
    return persistent_calibration


def generate_launch_description():
    pkg_share = get_package_share_directory("warrior_driver")

    # Main configuration (loaded from the install share directory).
    warrior_driver_config = os.path.join(pkg_share, "config", "warrior_driver.yaml")

    packaged_calibration = Path(pkg_share) / "config" / "steer_calibration.yaml"
    calib_write_path = _calibration_write_path(packaged_calibration)

    # Parameter layering: base config first, calibration overrides on top.
    params = [warrior_driver_config]
    # Prefer the durable file after the first calibration. Until then, load the
    # packaged defaults when they exist.
    calib_load_path = (
        calib_write_path if calib_write_path.exists() else packaged_calibration
    )
    if calib_load_path.exists():
        params.append(str(calib_load_path))
    params.append({"calib.write_path": str(calib_write_path)})

    return LaunchDescription([
        Node(
            package="warrior_driver",
            executable="warrior_driver_node",
            name="warrior_driver",
            output="screen",
            parameters=params,
        )
    ])
