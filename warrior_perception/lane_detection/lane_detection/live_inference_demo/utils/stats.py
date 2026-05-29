"""Rolling and summary statistics for live inference runs."""

from __future__ import annotations

import statistics
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class FrameMetrics:
    """Per-frame timing and state snapshot."""

    frame_idx: int
    timestamp: str
    capture_ms: float
    preprocess_ms: float
    inference_ms: float
    postprocess_ms: float
    total_ms: float
    instantaneous_fps: float
    rolling_fps: float
    saved_flag: int


@dataclass
class StatsTracker:
    """Track rolling FPS plus end-of-run summary statistics."""

    rolling_window: int
    start_time: float = field(default_factory=time.perf_counter)
    inference_history_ms: list[float] = field(default_factory=list)
    total_history_ms: list[float] = field(default_factory=list)
    fps_window: deque[float] = field(init=False)
    saved_frames: int = 0
    processed_frames: int = 0

    def __post_init__(self) -> None:
        self.fps_window = deque(maxlen=max(1, self.rolling_window))

    def update(self, metrics: FrameMetrics) -> None:
        """Update rolling and summary counters with a new frame record."""

        self.processed_frames += 1
        self.fps_window.append(metrics.instantaneous_fps)
        self.inference_history_ms.append(metrics.inference_ms)
        self.total_history_ms.append(metrics.total_ms)
        if metrics.saved_flag:
            self.saved_frames += 1

    def rolling_fps(self) -> float:
        """Return rolling mean FPS."""

        return float(sum(self.fps_window) / len(self.fps_window)) if self.fps_window else 0.0

    def average_fps(self) -> float:
        """Return run-wide average FPS based on total elapsed time."""

        elapsed = self.run_duration_sec()
        return float(self.processed_frames / elapsed) if elapsed > 0 else 0.0

    def run_duration_sec(self) -> float:
        """Return run duration in seconds."""

        return time.perf_counter() - self.start_time

    def build_summary(
        self,
        device: str,
        source_info: dict[str, Any],
        model_info: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Create JSON-serializable run summary statistics."""

        inference_values = self.inference_history_ms or [0.0]
        total_values = self.total_history_ms or [0.0]
        return {
            "total_frames_processed": self.processed_frames,
            "total_frames_saved": self.saved_frames,
            "mean_inference_time_ms": float(statistics.fmean(inference_values)),
            "median_inference_time_ms": float(statistics.median(inference_values)),
            "p90_inference_time_ms": float(np.percentile(inference_values, 90)),
            "p95_inference_time_ms": float(np.percentile(inference_values, 95)),
            "mean_total_pipeline_time_ms": float(statistics.fmean(total_values)),
            "average_fps": self.average_fps(),
            "run_duration_sec": self.run_duration_sec(),
            "device": device,
            "source_info": source_info,
            "model_info": model_info or {},
        }
