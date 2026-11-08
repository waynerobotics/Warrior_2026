"""Exercise the Gazebo startup chain with process success, failure and shutdown."""

import importlib.util
from pathlib import Path
from unittest.mock import Mock
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import RegisterEventHandler, Shutdown
from launch.events.process import ProcessExited
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
import pytest


@pytest.fixture
def launch_module():
    """Load the simulator entrypoint without running launch actions."""
    path = Path(__file__).resolve().parents[1] / 'launch/swerve_drive.gazebo.launch.py'
    spec = importlib.util.spec_from_file_location('swerve_sim_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def startup(monkeypatch, launch_module):
    """Construct the real launch graph without spawning processes."""
    module = launch_module
    nodes = {}
    logger = Mock()
    monkeypatch.setattr(module, 'get_logger', lambda _name: logger)

    def capture_node(**kwargs):
        node = Node(**kwargs)
        key = kwargs['executable']
        if key == 'spawner':
            key = kwargs['arguments'][0]
        nodes[key] = node
        return node

    monkeypatch.setattr(module, 'Node', capture_node)
    entities = module.generate_launch_description().entities
    handlers = [action.event_handler for action in entities
                if isinstance(action, RegisterEventHandler)]
    return nodes, handlers, logger


def process_exit(startup, process, returncode, shutting_down=False):
    """Deliver an actual process-exit event to its registered launch handler."""
    nodes, handlers, _logger = startup
    event = ProcessExited(
        action=nodes[process], name=process, cmd=[], cwd=None, env=None,
        pid=1234, returncode=returncode)
    matches = [handler for handler in handlers if handler.matches(event)]
    assert len(matches) == 1
    context = LaunchContext()
    # LaunchService sets this before terminated children emit ProcessExited.
    context._set_is_shutdown(shutting_down)
    return matches[0].handle(event, context)


@pytest.mark.parametrize('process,next_process', [
    ('create', 'joint_state_broadcaster'),
    ('joint_state_broadcaster', 'swerve_drive_controller'),
    ('swerve_drive_controller', 'rviz2'),
])
def test_success_starts_only_the_next_stage(startup, process, next_process):
    """Preserve the successful create -> broadcaster -> controller -> RViz order."""
    assert process_exit(startup, process, 0) == [startup[0][next_process]]


@pytest.mark.parametrize('process,stage', [
    ('create', 'Gazebo robot creation'),
    ('joint_state_broadcaster', 'Joint state broadcaster startup'),
    ('swerve_drive_controller', 'Swerve drive controller startup'),
])
@pytest.mark.parametrize('returncode', [1, -11])
def test_failed_startup_reports_error_and_stops_launch(
        startup, process, stage, returncode):
    """Stop on both nonzero exit statuses and signals, identifying the failing stage."""
    actions = process_exit(startup, process, returncode)
    assert not any(isinstance(action, Node) for action in actions)
    shutdown = [action for action in actions if isinstance(action, Shutdown)]
    assert len(shutdown) == 1
    reason = f'{stage} failed with exit code {returncode}'
    assert shutdown[0].event.reason == reason
    startup[2].error.assert_called_once_with(reason)


@pytest.mark.parametrize('process', ['create', 'joint_state_broadcaster', 'swerve_drive_controller'])
@pytest.mark.parametrize('returncode', [0, 1, -2])
def test_shutdown_does_not_restart_children_or_report_startup_failure(startup, process, returncode):
    """Treat process exits during requested shutdown as expected cleanup."""
    assert process_exit(startup, process, returncode, shutting_down=True) == []
    startup[2].error.assert_not_called()


@pytest.mark.parametrize('filename', ['controller tuning.yaml', "controller's $literal; tuning.yaml"])
def test_robot_description_accepts_quoted_source_and_controller_paths(
        launch_module, tmp_path, monkeypatch, filename):
    """Run xacro with paths containing spaces, quotes and literal shell metacharacters."""
    root = Path(__file__).resolve().parents[2]
    description = tmp_path / "description package's"
    description.symlink_to(root / 'warrior_description', target_is_directory=True)
    controller = tmp_path / filename
    controller.write_bytes((root / 'warrior_control/config/warrior_controllers_sim.yaml').read_bytes())
    find_share = launch_module.FindPackageShare
    monkeypatch.setattr(launch_module, 'FindPackageShare', lambda name:
                        TextSubstitution(text=str(description)) if name == 'warrior_description'
                        else find_share(name))
    captured = []

    def capture_node(**kwargs):
        captured.append(kwargs)
        return Node(**kwargs)

    monkeypatch.setattr(launch_module, 'Node', capture_node)
    launch_module.generate_launch_description()
    state = next(node for node in captured if node['executable'] == 'robot_state_publisher')
    context = LaunchContext()
    context.launch_configurations['controller_params_file'] = str(controller)
    robot = ET.fromstring(state['parameters'][0]['robot_description'].evaluate(context))
    assert robot.findtext(
        "gazebo/plugin[@filename='libgz_ros2_control-system.so']/parameters") == str(controller)
