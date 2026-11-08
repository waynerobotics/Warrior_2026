"""Preserve caller-supplied controller tuning in the GPS simulation wrapper."""

import copy
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
import pytest
import yaml


@pytest.mark.parametrize('odom_tf', [True, False])
def test_gps_sim_retains_custom_controller_parameters_and_assigns_tf_to_ekf(tmp_path, odom_tf):
    """Evaluate the actual include/rewrite without launching Gazebo or ROS nodes."""
    source = {
        'controller_manager': {'ros__parameters': {'update_rate': 777}},
        'swerve_drive_controller': {'ros__parameters': {
            'enable_odom_tf': odom_tf,
            'cmd_vel_timeout': 1.25,
            'smoother': {'max_accel_linear': 0.85},
            'custom_tuning': ['left', 'right', 'front'],
        }},
        'custom_controller': {'ros__parameters': {'gain': 4.2, 'label': 'field tuning'}},
    }
    custom = tmp_path / 'custom_controllers.yaml'
    custom.write_text(yaml.safe_dump(source))
    original = custom.read_bytes()
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location(
        'gps_sim_launch', root / 'warrior_bringup/launch_wrappers/swerve_gps_nav_sim.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations['controller_params_file'] = str(custom)
    entities = module.generate_launch_description().entities
    declarations = [action for action in entities if isinstance(action, DeclareLaunchArgument)]
    assert 'controller_params_file' in [action.name for action in declarations]
    for declaration in declarations:
        declaration.execute(context)
    simulation = next(action for action in entities if isinstance(action, IncludeLaunchDescription))
    # IncludeLaunchDescription installs its arguments before visiting child
    # actions. Execute just those assignments to exercise RewrittenYaml and
    # retain the launch context used by delayed simulator/controller callbacks.
    for action in simulation.execute(context):
        if isinstance(action, SetLaunchConfiguration):
            action.execute(context)
    rewritten = Path(context.launch_configurations['controller_params_file'])
    assert rewritten != custom
    try:
        actual = yaml.safe_load(rewritten.read_text())
        expected = copy.deepcopy(source)
        expected['swerve_drive_controller']['ros__parameters']['enable_odom_tf'] = False
        assert actual == expected
        assert custom.read_bytes() == original
    finally:
        rewritten.unlink()
