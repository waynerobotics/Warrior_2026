"""Resolve writable calibration paths without assuming a workspace layout."""

import os
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)


_SOURCE_CONFIG = Path(
    "warrior_perception/omnivision/config/transformation.yaml"
)


def _is_writable_target(path):
    """Return whether an existing file or its parent directory is writable."""
    if path.exists():
        return os.access(path, os.W_OK)
    return path.parent.is_dir() and os.access(path.parent, os.W_OK)


def default_transformation_config_path():
    """Prefer writable source config, then durable per-user storage."""
    workspace = os.environ.get("WARRIOR_WORKSPACE")
    if workspace:
        source_config = Path(workspace).expanduser() / _SOURCE_CONFIG
        if _is_writable_target(source_config):
            return str(source_config.resolve(strict=False))

    try:
        packaged_config = (
            Path(get_package_share_directory("omnivision"))
            / "config"
            / "transformation.yaml"
        ).absolute()
    except PackageNotFoundError:
        packaged_config = None

    # A symlink install exposes the source config through package share. Write
    # through that link rather than creating a machine-specific install file.
    if packaged_config is not None:
        resolved_config = packaged_config.resolve(strict=False)
        if resolved_config != packaged_config and _is_writable_target(resolved_config):
            return str(resolved_config)

    config_home = Path(
        os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")
    ).expanduser()
    return str(
        config_home
        / "warrior-2026"
        / "omnivision"
        / "transformation.yaml"
    )
