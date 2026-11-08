"""Prevent the downward lidar beam from becoming a Nav2 ground obstacle ring."""

import importlib.util
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]


def test_nav_sim_costmaps_consume_ground_filtered_obstacles():
    parameters = yaml.safe_load(
        (ROOT / 'warrior_navigation/config/nav2_sim_params.yaml').read_text())
    for name in ('local_costmap', 'global_costmap'):
        scan = parameters[name][name]['ros__parameters']['obstacle_layer']['scan']
        assert scan['topic'] == '/navigation/scan'
        assert scan['inf_is_valid'] and scan['clearing'] and scan['marking']


def test_nav_sim_starts_ground_filter_and_waits_for_its_scan(monkeypatch):
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    from launch import LaunchContext

    path = ROOT / 'warrior_bringup/launch_wrappers/swerve_nav_sim.launch.py'
    spec = importlib.util.spec_from_file_location('swerve_nav_sim_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = []
    node_type = module.Node

    def capture_node(**kwargs):
        node = node_type(**kwargs)
        captured.append((kwargs, node))
        return node

    monkeypatch.setattr(module, 'Node', capture_node)
    entities = module.generate_launch_description().entities
    filters = [(kwargs, node) for kwargs, node in captured
               if kwargs['executable'] == 'ground_obstacle_scan']
    assert len(filters) == 1
    kwargs, node = filters[0]
    assert node in entities
    assert kwargs['package'] == 'warrior_navigation'
    assert dict(kwargs['remappings']) == {
        'cloud_in': '/unilidar/cloud', 'scan': '/navigation/scan',
    }
    settings = kwargs['parameters'][0]
    assert settings['target_frame'] == 'base_footprint'
    assert settings['min_height'] == 0.15
    assert settings['ground_height'] == 0.0
    context = LaunchContext()
    context.launch_configurations['use_sim_time'] = 'true'
    assert settings['use_sim_time'].perform(context) == 'true'
    readiness = next(kwargs for kwargs, _ in captured
                     if kwargs['executable'] == 'wait_for_nav_ready.py')
    assert readiness['parameters'][0]['scan_topic'] == '/navigation/scan'
