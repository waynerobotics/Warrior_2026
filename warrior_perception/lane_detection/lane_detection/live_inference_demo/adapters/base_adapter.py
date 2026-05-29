"""Base adapter interface for model-specific live inference logic."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

import numpy as np


class BaseAdapter(ABC):
    """Abstract interface used by the demo app to interact with a model."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config

    @abstractmethod
    def load_model(self) -> Any:
        """Load model weights and runtime state."""

    @abstractmethod
    def preprocess(self, frame_bgr: np.ndarray) -> Any:
        """Convert a BGR frame into model-ready input."""

    @abstractmethod
    def infer(self, model_input: Any) -> Any:
        """Run forward inference and return raw model output."""

    @abstractmethod
    def postprocess(self, raw_output: Any, original_frame: np.ndarray) -> dict[str, Any]:
        """Convert raw model output into a generic prediction dictionary."""

    @abstractmethod
    def draw_predictions(self, frame_bgr: np.ndarray, predictions: dict[str, Any]) -> np.ndarray:
        """Draw model predictions onto a frame."""
