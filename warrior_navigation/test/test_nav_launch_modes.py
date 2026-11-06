"""Check the standalone Warrior launch routes sensors and modes consistently."""

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip('launch')
pytest.importorskip('launch_ros')

from launch import LaunchContext  # noqa: E402
from launch.actions import GroupAction, IncludeLaunchDescription  # noqa: E402
from launch_ros.actions import SetRemap  # noqa: E402


@pytest.mark.parametrize('simulation', ['true', 'false'])
@pytest.mark.parametrize('use_wheels', ['true', 'false'])
def test_warrior_navigation_selects_compatible_localization_and_odometry(
        simulation, use_wheels, monkeypatch):
    path = Path(__file__).resolve().parents[1] / 'launch/nav.launch.py'
    spec = importlib.util.spec_from_file_location('warrior_nav_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update({
        'use_sim': simulation, 'use_wheel_odometry': use_wheels,
        'waypoint_file': 'demo_nav',
    })
    entities = module.generate_launch_description().entities
    includes = [action for action in entities
                if isinstance(action, IncludeLaunchDescription) and
                (action.condition is None or action.condition.evaluate(context))]
    localization = [action for action in includes
                    if 'warrior_localization' in action.launch_description_source.location]
    assert len(localization) == 1
    selected = localization[0]
    if simulation == 'true':
        assert 'gps_sim.launch.py' in selected.launch_description_source.location
        assert dict(selected.launch_arguments)['use_wheel_odometry'].perform(context) == use_wheels
    else:
        assert 'dual_ekf_navsat.launch.py' in selected.launch_description_source.location

    navigation_group = next(action for action in entities if isinstance(action, GroupAction))
    remap = next(action for action in navigation_group.get_sub_entities()
                 if isinstance(action, SetRemap))
    if remap.condition.evaluate(context):
        remap.execute(context)
    assert context.launch_configurations.get('ros_remaps', []) == (
        [('/odom', '/odometry/local')] if simulation == 'true' else [])

    # Inspect the values handed to the actual ROS Node constructor without
    # spawning any process or using private launch_ros implementation details.
    node_type = module.Node
    captured = []

    def capture_node(**kwargs):
        captured.append(kwargs)
        return node_type(**kwargs)

    monkeypatch.setattr(module, 'Node', capture_node)
    module.launch_setup(context)
    follower = captured[0]['parameters'][0]
    assert follower['use_sim_time'] == (simulation == 'true')
    assert follower['conversion_mode'] == ('from_ll' if simulation == 'true' else 'projection')
