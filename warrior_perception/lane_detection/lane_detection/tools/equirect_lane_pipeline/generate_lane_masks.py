from __future__ import annotations

import argparse
import importlib
import json
import logging
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

LOGGER = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")


class LaneBackend:
    name = "base"

    def __init__(self, checkpoint: str = "", config: str = "", device: str = "cpu", score_threshold: float = 0.6) -> None:
        self.checkpoint = checkpoint
        self.config = config
        self.device = device
        self.score_threshold = score_threshold

    def predict(self, image_bgr: np.ndarray) -> tuple[list[list[tuple[int, int]]], float]:
        raise NotImplementedError


class DummyBackend(LaneBackend):
    name = "dummy"

    def predict(self, image_bgr: np.ndarray) -> tuple[list[list[tuple[int, int]]], float]:
        h, w = image_bgr.shape[:2]
        y0, y1 = int(h * 0.58), int(h * 0.96)
        lanes = [
            [(int(w * 0.42), y0), (int(w * 0.37), int(h * 0.75)), (int(w * 0.31), y1)],
            [(int(w * 0.58), y0), (int(w * 0.63), int(h * 0.75)), (int(w * 0.69), y1)],
        ]
        return lanes, 0.75


def load_backend_config(path: str) -> dict[str, Any]:
    if not path:
        return {}
    config_path = Path(path)
    if not config_path.exists():
        raise FileNotFoundError(config_path)
    with config_path.open("r", encoding="utf-8") as handle:
        if config_path.suffix.lower() in {".yaml", ".yml"}:
            data = yaml.safe_load(handle)
        else:
            data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Expected a mapping in backend config: {config_path}")
    return data


def resolve_symbol(target: str) -> Any:
    if ":" not in target:
        raise ValueError("Backend adapter target must use 'module:object' syntax")
    module_name, object_name = target.split(":", 1)
    try:
        module = importlib.import_module(module_name)
    except ModuleNotFoundError as exc:
        raise ImportError(f"Could not import backend adapter module '{module_name}'. Is it on PYTHONPATH?") from exc
    try:
        return getattr(module, object_name)
    except AttributeError as exc:
        raise ImportError(f"Backend adapter module '{module_name}' does not define '{object_name}'") from exc


class ImportAdapterBackend(LaneBackend):
    """Bridge to a project-specific CLRNet/LaneATT adapter without vendoring the model repo.

    The config file must contain:

    adapter:
      target: "package.module:AdapterClass"
      kwargs: {}

    The imported object may be a class or factory. It is constructed with kwargs plus
    checkpoint/device/score_threshold when accepted, and must expose predict(image_bgr).
    predict may return either (lanes, confidence) or a dict with mask/lanes/confidence.
    """

    def __init__(self, checkpoint: str = "", config: str = "", device: str = "cpu", score_threshold: float = 0.6) -> None:
        super().__init__(checkpoint=checkpoint, config=config, device=device, score_threshold=score_threshold)
        cfg = load_backend_config(config)
        adapter_cfg = cfg.get("adapter", cfg)
        target = adapter_cfg.get("target")
        if not target:
            raise ValueError(
                f"{self.name} requires --config with adapter.target, for example "
                "'my_lane_repo.adapters:CLRNetEquirectAdapter'"
            )
        kwargs = dict(adapter_cfg.get("kwargs", {}))
        kwargs.setdefault("checkpoint", checkpoint)
        kwargs.setdefault("device", device)
        kwargs.setdefault("score_threshold", score_threshold)
        factory = resolve_symbol(str(target))
        try:
            self.adapter = factory(**kwargs)
        except TypeError:
            self.adapter = factory()

    def predict(self, image_bgr: np.ndarray) -> dict[str, Any] | tuple[list[list[tuple[int, int]]], float]:
        return self.adapter.predict(image_bgr)


class CLRNetBackend(ImportAdapterBackend):
    name = "clrnet"


class LaneATTBackend(ImportAdapterBackend):
    name = "laneatt"


BACKENDS = {"dummy": DummyBackend, "clrnet": CLRNetBackend, "laneatt": LaneATTBackend}


def lanes_to_mask(lanes: list[list[tuple[int, int]]], image_h: int, image_w: int, thickness: int) -> np.ndarray:
    """Rasterize lane polylines into a dense binary uint8 mask with values 0 or 255."""
    mask = Image.new("L", (image_w, image_h), 0)
    draw = ImageDraw.Draw(mask)
    for lane in lanes:
        if len(lane) >= 2:
            draw.line(lane, fill=255, width=int(thickness), joint="curve")
    return np.asarray(mask, dtype=np.uint8)


def normalize_backend_output(output: Any, image_h: int, image_w: int, thickness: int) -> tuple[np.ndarray, float]:
    if isinstance(output, tuple) and len(output) == 2:
        lanes, confidence = output
        mask = lanes_to_mask(lanes, image_h, image_w, thickness)
        return mask, float(confidence) if confidence is not None else fallback_confidence(mask)

    if not isinstance(output, dict):
        raise TypeError("Lane backend predict() must return (lanes, confidence) or a dict")

    confidence = output.get("confidence", output.get("seg_confidence"))
    if output.get("mask") is not None:
        mask = np.asarray(output["mask"])
        if mask.ndim == 3:
            mask = mask[..., 0]
        if mask.shape[:2] != (image_h, image_w):
            mask = cv2.resize(mask.astype(np.uint8), (image_w, image_h), interpolation=cv2.INTER_NEAREST)
        mask = (mask > 0).astype(np.uint8) * 255
        return mask, float(confidence) if confidence is not None else fallback_confidence(mask)

    lanes = output.get("lanes")
    if lanes is None:
        raise ValueError("Lane backend dict output must include 'mask' or 'lanes'")
    mask = lanes_to_mask(lanes, image_h, image_w, thickness)
    return mask, float(confidence) if confidence is not None else fallback_confidence(mask)


def fallback_confidence(mask: np.ndarray) -> float:
    """Fallback when a backend has no calibrated confidence: scale by plausible lane-pixel coverage."""
    lane_fraction = float((mask > 0).mean())
    if lane_fraction <= 0.0:
        return 0.0
    return max(0.05, min(0.75, lane_fraction * 50.0))


def read_roi(path: str | None, shape: tuple[int, int]) -> np.ndarray | None:
    if not path:
        return None
    roi = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if roi is None:
        raise FileNotFoundError(path)
    if roi.shape[:2] != shape:
        roi = cv2.resize(roi, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
    return roi > 0


def overlay_mask(image_bgr: np.ndarray, mask: np.ndarray) -> np.ndarray:
    overlay = image_bgr.copy()
    overlay[mask > 0] = (0, 255, 0)
    return cv2.addWeighted(image_bgr, 0.7, overlay, 0.3, 0.0)


def generate(args: argparse.Namespace) -> None:
    backend_cls = BACKENDS.get(args.backend)
    if backend_cls is None:
        raise ValueError(f"Unknown backend {args.backend}. Available: {sorted(BACKENDS)}")
    backend = backend_cls(args.checkpoint or "", args.config or "", args.device, args.score_threshold)

    frames_dir = Path(args.frames_dir)
    output_dir = Path(args.output_dir)
    masks_dir = output_dir / "masks"
    overlays_dir = output_dir / "overlays"
    masks_dir.mkdir(parents=True, exist_ok=True)
    if args.save_overlay:
        overlays_dir.mkdir(parents=True, exist_ok=True)

    image_paths = sorted([p for p in frames_dir.iterdir() if p.suffix.lower() in {".jpg", ".jpeg", ".png"}])
    records_path = output_dir / "records.jsonl"
    with records_path.open("w", encoding="utf-8") as handle:
        for image_path in image_paths:
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                continue
            h, w = image.shape[:2]
            mask, confidence = normalize_backend_output(backend.predict(image), h, w, args.mask_thickness)
            roi = read_roi(args.roi_mask, (h, w))
            if roi is not None:
                mask = np.where(roi, mask, 0).astype(np.uint8)
            if confidence < args.score_threshold:
                mask[:] = 0
            mask_path = masks_dir / f"{image_path.stem}.png"
            cv2.imwrite(str(mask_path), mask)
            if args.save_overlay:
                cv2.imwrite(str(overlays_dir / f"{image_path.stem}.jpg"), overlay_mask(image, mask))
            row = {
                "frame_id": image_path.stem,
                "image_path": str(image_path),
                "seg_mask_path": str(mask_path),
                "seg_confidence": float(confidence),
                "backend": args.backend,
                "num_lane_pixels": int((mask == 255).sum()),
            }
            handle.write(json.dumps(row) + "\n")
    LOGGER.info("records=%s images=%d backend=%s", records_path, len(image_paths), args.backend)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate lane pseudo-labels in equirectangular image space.")
    parser.add_argument("--backend", choices=sorted(BACKENDS), default="dummy")
    parser.add_argument("--frames_dir", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--checkpoint", default="")
    parser.add_argument("--config", default="")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--batch_size", type=int, default=4)
    parser.add_argument("--score_threshold", type=float, default=0.6)
    parser.add_argument("--mask_thickness", type=int, default=6)
    parser.add_argument("--save_overlay", action="store_true")
    parser.add_argument("--roi_mask", default=None)
    return parser.parse_args()


if __name__ == "__main__":
    try:
        generate(parse_args())
    except Exception as exc:
        LOGGER.error("%s", exc)
        raise SystemExit(1) from None
