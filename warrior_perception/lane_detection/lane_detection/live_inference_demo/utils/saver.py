"""Frame and metrics persistence utilities for the live inference demo."""

from __future__ import annotations

import csv
import json
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2

from utils.stats import FrameMetrics


class RunSaver:
    """Save overlay frames, per-frame CSV metrics, and end-of-run summary."""

    def __init__(self, output_dir: str | Path, saving_enabled: bool = True) -> None:
        self.output_dir = Path(output_dir)
        self.saving_enabled = saving_enabled
        self.run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = self.output_dir / f"run_{self.run_id}"
        self.frames_dir = self.run_dir / "frames"
        self.metrics_csv_path = self.run_dir / "metrics.csv"
        self.summary_json_path = self.run_dir / "summary.json"
        self._csv_handle = None
        self._writer: csv.DictWriter[str] | None = None

        self.run_dir.mkdir(parents=True, exist_ok=True)
        if self.saving_enabled:
            self.frames_dir.mkdir(parents=True, exist_ok=True)

        self._open_csv()

    def _open_csv(self) -> None:
        """Create the metrics CSV and header."""

        self._csv_handle = self.metrics_csv_path.open("w", newline="", encoding="utf-8")
        fieldnames = [
            "frame_idx",
            "timestamp",
            "capture_ms",
            "preprocess_ms",
            "inference_ms",
            "postprocess_ms",
            "total_ms",
            "instantaneous_fps",
            "rolling_fps",
            "saved_flag",
        ]
        self._writer = csv.DictWriter(self._csv_handle, fieldnames=fieldnames)
        self._writer.writeheader()
        self._csv_handle.flush()

    def save_frame(self, frame_bgr: np.ndarray, frame_idx: int, timestamp: str) -> Path | None:
        """Persist a rendered frame to disk."""

        if not self.saving_enabled:
            return None

        safe_timestamp = timestamp.replace(":", "-").replace(".", "-")
        file_path = self.frames_dir / f"{safe_timestamp}_frame_{frame_idx:06d}.jpg"
        ok = cv2.imwrite(str(file_path), frame_bgr)
        return file_path if ok else None

    def append_metrics(self, metrics: FrameMetrics) -> None:
        """Append one metrics row to the CSV log."""

        if self._writer is None or self._csv_handle is None:
            return

        self._writer.writerow(metrics.__dict__)
        self._csv_handle.flush()

    def save_summary(self, summary: dict[str, Any]) -> None:
        """Write the run summary JSON to disk."""

        with self.summary_json_path.open("w", encoding="utf-8") as handle:
            json.dump(summary, handle, indent=2)

    def close(self) -> None:
        """Close open file handles."""

        if self._csv_handle is not None:
            self._csv_handle.close()
            self._csv_handle = None
            self._writer = None
