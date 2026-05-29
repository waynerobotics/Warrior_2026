"""Video source abstraction for camera, file, and stream inputs."""

from __future__ import annotations

import logging
import time
from datetime import datetime
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


LOGGER = logging.getLogger(__name__)


@dataclass
class VideoFrame:
    """Captured video frame payload."""

    ok: bool
    frame: np.ndarray | None
    timestamp: str


class VideoSource:
    """Thin OpenCV video capture wrapper with simple retry handling."""

    def __init__(self, source: Any, max_read_retries: int = 5, retry_delay_sec: float = 0.2) -> None:
        self.source = source
        self.max_read_retries = max_read_retries
        self.retry_delay_sec = retry_delay_sec
        self.capture = self._create_capture(source)

    @staticmethod
    def _create_capture(source: Any) -> cv2.VideoCapture:
        """Create an OpenCV capture object."""

        capture = cv2.VideoCapture(source)
        if not capture.isOpened():
            raise RuntimeError(f"Unable to open video source: {source}")
        return capture

    def read(self) -> VideoFrame:
        """Read a frame, retrying briefly for transient failures."""

        last_frame: np.ndarray | None = None
        timestamp = datetime.now().isoformat(timespec="milliseconds")
        for attempt in range(1, self.max_read_retries + 1):
            ok, frame = self.capture.read()
            if ok and frame is not None:
                timestamp = datetime.now().isoformat(timespec="milliseconds")
                return VideoFrame(ok=True, frame=frame, timestamp=timestamp)

            LOGGER.warning(
                "Frame read failed from source %s (%s/%s). Retrying in %.1fs.",
                self.source,
                attempt,
                self.max_read_retries,
                self.retry_delay_sec,
            )
            time.sleep(self.retry_delay_sec)
            last_frame = frame

        return VideoFrame(ok=False, frame=last_frame, timestamp=timestamp)

    def release(self) -> None:
        """Release the underlying capture."""

        self.capture.release()
