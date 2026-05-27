"""Generic PyTorch multitask adapter example for the live inference demo."""

from __future__ import annotations

import logging
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch

from adapters.base_adapter import BaseAdapter

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    from src.models.multitask_model import MultiTaskPerceptionModel
except Exception:  # pragma: no cover - repo-dependent import
    MultiTaskPerceptionModel = None


LOGGER = logging.getLogger(__name__)


class MultiTaskAdapter(BaseAdapter):
    """Example adapter that accepts generic dict-like model outputs.

    This implementation is intentionally conservative: it tries to load a model
    checkpoint if available, but it does not assume a single architecture or
    checkpoint format. Update the TODO blocks in ``load_model`` and
    ``postprocess`` to connect a project-specific model.
    """

    def __init__(self, config: dict[str, Any], device: torch.device) -> None:
        super().__init__(config)
        self.device = device
        self.model = self.load_model()
        self.model_info = self._build_model_info()

    def _build_model_info(self) -> dict[str, Any]:
        """Return lightweight model metadata for the run summary."""

        checkpoint_path = str(self.config["model"].get("checkpoint_path", ""))
        return {
            "adapter": self.__class__.__name__,
            "checkpoint_path": checkpoint_path,
            "device": str(self.device),
            "model_class": type(self.model).__name__ if self.model is not None else "none",
        }

    def load_model(self) -> Any:
        """Load a PyTorch model checkpoint when provided.

        TODO:
        Replace this method if your checkpoint requires a different model
        constructor or state-dict handling.
        """

        checkpoint_path = str(self.config["model"].get("checkpoint_path", "")).strip()
        if not checkpoint_path:
            LOGGER.warning("No checkpoint_path configured. Running without model predictions.")
            return None

        checkpoint_file = Path(checkpoint_path)
        if not checkpoint_file.is_absolute():
            checkpoint_file = ROOT / checkpoint_file

        if not checkpoint_file.exists():
            LOGGER.warning("Checkpoint not found at %s. Running without model predictions.", checkpoint_file)
            return None

        checkpoint = torch.load(checkpoint_file, map_location="cpu")
        model: Any = None

        if isinstance(checkpoint, torch.nn.Module):
            model = checkpoint
        elif isinstance(checkpoint, dict):
            if "model" in checkpoint and isinstance(checkpoint["model"], torch.nn.Module):
                model = checkpoint["model"]
            elif "model_state_dict" in checkpoint and MultiTaskPerceptionModel is not None:
                model_config = checkpoint.get("config")
                if isinstance(model_config, dict):
                    try:
                        model = MultiTaskPerceptionModel(model_config)
                        model.load_state_dict(checkpoint["model_state_dict"], strict=False)
                    except Exception as exc:  # pragma: no cover - repo checkpoint dependent
                        LOGGER.warning("Failed to load MultiTaskPerceptionModel from checkpoint: %s", exc)

        if model is None:
            LOGGER.warning(
                "Checkpoint loaded but adapter could not build a model automatically. "
                "Edit live_inference_demo/adapters/multitask_adapter.py to connect your model."
            )
            return None

        model.to(self.device)
        model.eval()
        LOGGER.info("Loaded model from %s onto %s", checkpoint_file, self.device)
        return model

    def preprocess(self, frame_bgr: np.ndarray) -> dict[str, Any]:
        """Convert a BGR frame to normalized NCHW tensor input."""

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(frame_rgb).permute(2, 0, 1).float() / 255.0
        tensor = tensor.unsqueeze(0).to(self.device)
        return {"tensor": tensor}

    def infer(self, model_input: dict[str, Any]) -> Any:
        """Run model inference when a model is available."""

        if self.model is None:
            return {}

        with torch.no_grad():
            return self.model(model_input["tensor"])

    def postprocess(self, raw_output: Any, original_frame: np.ndarray) -> dict[str, Any]:
        """Map raw model output into a generic prediction structure.

        Supported generic keys:
        - ``boxes``: array-like ``[N, 4]`` in xyxy pixel coords
        - ``scores``: array-like ``[N]``
        - ``labels``: array-like ``[N]``
        - ``mask``: HxW segmentation map or HxWx3 visualization-ready image

        TODO:
        If your model returns a custom structure, translate it here.
        """

        confidence_threshold = float(self.config["model"].get("confidence_threshold", 0.3))
        predictions: dict[str, Any] = {"boxes": [], "scores": [], "labels": [], "mask": None}

        if raw_output is None:
            return predictions

        output = raw_output
        if isinstance(output, list) and output:
            output = output[0]

        if not isinstance(output, dict):
            return predictions

        mask = output.get("mask") or output.get("segmentation")
        if mask is not None:
            predictions["mask"] = self._to_mask_image(mask, original_frame.shape[:2])

        boxes = output.get("boxes")
        scores = output.get("scores")
        labels = output.get("labels")

        if boxes is None:
            detection = output.get("detection")
            if isinstance(detection, dict):
                boxes = detection.get("boxes")
                scores = detection.get("scores")
                labels = detection.get("labels")

        if boxes is not None:
            boxes_np = self._to_numpy(boxes)
            scores_np = self._to_numpy(scores) if scores is not None else np.ones((len(boxes_np),), dtype=np.float32)
            labels_np = self._to_numpy(labels) if labels is not None else np.array(["obj"] * len(boxes_np), dtype=object)

            keep_indices = [idx for idx, score in enumerate(scores_np.tolist()) if float(score) >= confidence_threshold]
            predictions["boxes"] = [boxes_np[idx].tolist() for idx in keep_indices]
            predictions["scores"] = [float(scores_np[idx]) for idx in keep_indices]
            predictions["labels"] = [str(labels_np[idx]) for idx in keep_indices]

        return predictions

    def draw_predictions(self, frame_bgr: np.ndarray, predictions: dict[str, Any]) -> np.ndarray:
        """Draw optional segmentation and detection results."""

        output = frame_bgr.copy()

        mask = predictions.get("mask")
        if mask is not None:
            mask_bgr = self._normalize_mask_for_overlay(mask, output.shape[:2])
            output = cv2.addWeighted(output, 0.75, mask_bgr, 0.25, 0.0)

        boxes = predictions.get("boxes", [])
        scores = predictions.get("scores", [])
        labels = predictions.get("labels", [])
        for idx, box in enumerate(boxes):
            x1, y1, x2, y2 = [int(v) for v in box]
            cv2.rectangle(output, (x1, y1), (x2, y2), (0, 220, 0), 2)
            label = labels[idx] if idx < len(labels) else "obj"
            score = scores[idx] if idx < len(scores) else None
            text = f"{label} {score:.2f}" if score is not None else label
            cv2.putText(output, text, (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1, cv2.LINE_AA)

        return output

    @staticmethod
    def _to_numpy(value: Any) -> np.ndarray:
        """Convert tensors or sequences to numpy arrays."""

        if isinstance(value, np.ndarray):
            return value
        if torch.is_tensor(value):
            return value.detach().cpu().numpy()
        return np.asarray(value)

    def _to_mask_image(self, mask: Any, target_shape: tuple[int, int]) -> np.ndarray:
        """Convert a segmentation mask into a simple overlay-ready image."""

        mask_np = self._to_numpy(mask)
        if mask_np.ndim == 4:
            mask_np = mask_np[0]
        if mask_np.ndim == 3 and mask_np.shape[0] in (1, 3):
            mask_np = np.transpose(mask_np, (1, 2, 0))
        if mask_np.ndim == 3 and mask_np.shape[2] > 1:
            mask_np = np.argmax(mask_np, axis=2).astype(np.uint8)
        elif mask_np.ndim == 3 and mask_np.shape[2] == 1:
            mask_np = mask_np[:, :, 0]

        if mask_np.ndim == 2:
            mask_np = mask_np.astype(np.uint8)
            if mask_np.max() <= 1:
                mask_np = mask_np * 255
            mask_np = cv2.applyColorMap(mask_np, cv2.COLORMAP_JET)

        return self._normalize_mask_for_overlay(mask_np, target_shape)

    @staticmethod
    def _normalize_mask_for_overlay(mask: np.ndarray, target_shape: tuple[int, int]) -> np.ndarray:
        """Resize and coerce a mask image to BGR uint8."""

        mask_image = mask
        if mask_image.ndim == 2:
            mask_image = cv2.cvtColor(mask_image.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        if mask_image.shape[:2] != target_shape:
            mask_image = cv2.resize(mask_image, (target_shape[1], target_shape[0]), interpolation=cv2.INTER_NEAREST)
        if mask_image.dtype != np.uint8:
            mask_image = np.clip(mask_image, 0, 255).astype(np.uint8)
        return mask_image
