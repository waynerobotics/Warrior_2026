#!/usr/bin/env python3
"""Regenerate the shared course, both simulator worlds and the GPS missions."""

from pathlib import Path
import subprocess
import sys

SIMULATION = Path(__file__).resolve().parents[1]


def main():
    commands = [
        [SIMULATION / 'igvc2027/build_course.py'],
        [SIMULATION / 'tools/export_igvc2027_gazebo.py'],
        [SIMULATION / 'warrior_isaac/export_usd.py',
         SIMULATION / 'igvc2027/scene.json',
         SIMULATION / 'warrior_isaac/worlds/igvc2027_autonav.usda'],
    ]
    for command in commands:
        subprocess.run([sys.executable, *map(str, command)], check=True)


if __name__ == '__main__':
    main()
