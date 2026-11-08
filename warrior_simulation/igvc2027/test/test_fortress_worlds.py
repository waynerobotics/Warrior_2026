"""Keep supported swerve worlds loadable by Ubuntu 22.04's native Fortress."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


WORLDS = Path(__file__).resolve().parents[2] / 'warrior_gazebo' / 'worlds'
SYSTEMS = {
    'Physics': 'physics',
    'UserCommands': 'user-commands',
    'SceneBroadcaster': 'scene-broadcaster',
    'Contact': 'contact',
    'Imu': 'imu',
    'NavSat': 'navsat',
}


@pytest.mark.parametrize('filename', [
    'competition.world', 'igvc2027_no_mans_land.world', 'empty.world',
    'igvc2027_autonav.world',
])
def test_supported_worlds_load_required_native_fortress_systems(filename):
    """The gz-sim aliases only exist in Fortress 6.16 and newer."""
    world = ET.parse(WORLDS / filename).getroot().find('world')
    plugins = {plugin.attrib['name']: plugin.attrib['filename']
               for plugin in world.findall('plugin')}
    for name, library in SYSTEMS.items():
        assert plugins[f'ignition::gazebo::systems::{name}'] == (
            f'ignition-gazebo-{library}-system')
    for plugin in world.findall('.//plugin'):
        assert not plugin.attrib['filename'].startswith('gz-sim-')
        assert not plugin.attrib['name'].startswith('gz::sim::')
    # The spawned Warrior owns the renderer. A second Sensors instance in the
    # world races to populate the same scene and can crash in glGenBuffers.
    assert not any(name.endswith('::Sensors') for name in plugins)
    robot_plugins = ET.parse(
        WORLDS.parents[2] / 'warrior_description/xacro/ros2_control/gzsim.ros2_control.xacro'
    ).getroot().findall('gazebo/plugin')
    assert sum(plugin.attrib['name'].endswith('::Sensors')
               for plugin in robot_plugins) == 1
