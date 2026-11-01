#!/usr/bin/env python3
"""Expand the repository Warrior URDF for Isaac, retaining its actual visuals and joints."""
import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def prepare(output):
    source = ROOT / 'warrior_description/urdf/gzsim.urdf.xacro'
    xml = subprocess.check_output(['xacro', str(source)], text=True)
    robot = ET.fromstring(xml)
    for child in list(robot):
        if child.tag in ('gazebo', 'ros2_control'):
            robot.remove(child)
    for mesh in robot.iter('mesh'):
        path = mesh.get('filename', '')
        if path.startswith('file://'):
            mesh.set('filename', path[7:])
    # PhysX uses a round tire collider rather than a convex hull containing the
    # STL's axle. Visual geometry and all URDF link/joint transforms are retained.
    for link in robot.findall('link'):
        if link.get('name', '').endswith('_drive_wheel_link'):
            for collision in link.findall('collision'):
                link.remove(collision)
            collision = ET.SubElement(link, 'collision')
            ET.SubElement(collision, 'origin', xyz='0 0 0', rpy='1.5707963267948966 0 0')
            geometry = ET.SubElement(collision, 'geometry')
            ET.SubElement(geometry, 'cylinder', radius='0.10795', length='0.0762')
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(robot)
    ET.ElementTree(robot).write(output, encoding='utf-8', xml_declaration=True)
    print(output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'build/isaac/warrior.urdf')
    prepare(parser.parse_args().output)
