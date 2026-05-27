"""Logging helpers for the live inference demo."""

from __future__ import annotations

import logging


def setup_logging(level: str = "INFO") -> None:
    """Configure a simple console logger for demo use."""

    numeric_level = getattr(logging, level.upper(), logging.INFO)
    logging.basicConfig(
        level=numeric_level,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
