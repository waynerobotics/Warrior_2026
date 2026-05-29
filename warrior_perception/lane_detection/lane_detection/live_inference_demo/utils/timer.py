"""Timing helpers built on top of ``time.perf_counter``."""

from __future__ import annotations

import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Iterator


@dataclass
class StageTimer:
    """Container for a measured duration in milliseconds."""

    duration_ms: float = 0.0


@contextmanager
def measure_time() -> Iterator[StageTimer]:
    """Measure elapsed time in milliseconds for a block."""

    timer = StageTimer()
    start = time.perf_counter()
    try:
        yield timer
    finally:
        timer.duration_ms = (time.perf_counter() - start) * 1000.0
