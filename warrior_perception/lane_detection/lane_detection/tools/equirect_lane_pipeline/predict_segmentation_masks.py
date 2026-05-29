from __future__ import annotations

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
from PIL import Image
from torchvision.transforms import functional as TF

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.train import load_yaml, resolve_device
from models import MultiTaskPerceptionModel
from models.segmentation_roi import segmentation_argmax_in_roi

LOGGER = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")


def load_checkpoint(path: str | Path, config_path: str | Path, device: torch.device) -> tuple[MultiTaskPerceptionModel, dict[str, Any]]:
    checkpoint = torch.load(path, map_location=device)
    cfg = checkpoint.get("cfg") if isinstance(checkpoint, dict) else None
    if not isinstance(cfg, dict):
        cfg = load_yaml(config_path)
    model = MultiTaskPerceptionModel(cfg).to(device)
    state_dict = checkpoint.get("state_dict", checkpoint) if isinstance(checkpoint, dict) else checkpoint
    model.load_state_dict(state_dict)
    model.eval()
    return model, cfg


def image_paths_from_dir(images_dir: Path) -> list[Path]:
    suffixes = {".jpg", ".jpeg", ".png"}
    return sorted(path for path in images_dir.iterdir() if path.suffix.lower() in suffixes)


def load_image_tensor(path: Path, image_size: tuple[int, int], normalize: bool) -> tuple[torch.Tensor, tuple[int, int]]:
    image = Image.open(path).convert("RGB")
    orig_w, orig_h = image.size
    target_h, target_w = image_size
    image = image.resize((target_w, target_h), resample=Image.BILINEAR)
    tensor = TF.to_tensor(image)
    if normalize:
        tensor = TF.normalize(tensor, mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    return tensor, (orig_h, orig_w)


def load_roi(path: str | Path, image_size: tuple[int, int]) -> np.ndarray | None:
    if not path:
        return None
    target_h, target_w = image_size
    roi = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if roi is None:
        raise FileNotFoundError(path)
    if roi.shape[:2] != (target_h, target_w):
        roi = cv2.resize(roi, (target_w, target_h), interpolation=cv2.INTER_NEAREST)
    return roi > 0


def overlay_mask(image_path: Path, mask: np.ndarray, image_size: tuple[int, int]) -> np.ndarray:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(image_path)
    target_h, target_w = image_size
    image = cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
    overlay = image.copy()
    overlay[mask > 0] = (0, 255, 0)
    return cv2.addWeighted(image, 0.68, overlay, 0.32, 0.0)


@torch.no_grad()
def predict(args: argparse.Namespace) -> None:
    device = resolve_device(args.device)
    model, cfg = load_checkpoint(args.checkpoint, args.config, device)
    image_size = tuple(int(value) for value in cfg["dataset"]["image_size"])
    normalize = bool(cfg["dataset"].get("normalize", True))
    roi = load_roi(args.roi_mask, image_size)

    output_dir = Path(args.output_dir)
    masks_dir = output_dir / "masks"
    overlays_dir = output_dir / "overlays"
    masks_dir.mkdir(parents=True, exist_ok=True)
    if args.save_overlay:
        overlays_dir.mkdir(parents=True, exist_ok=True)

    image_paths = image_paths_from_dir(Path(args.images_dir))
    if args.limit:
        image_paths = image_paths[: args.limit]

    records_path = output_dir / "records.jsonl"
    with records_path.open("w", encoding="utf-8") as handle:
        for image_path in image_paths:
            tensor, _ = load_image_tensor(image_path, image_size=image_size, normalize=normalize)
            logits = model(tensor.unsqueeze(0).to(device), postprocess=False)["segmentation"]
            roi_tensor = None
            if roi is not None and not args.no_apply_roi:
                roi_tensor = torch.from_numpy(roi.astype(np.uint8)).unsqueeze(0).to(device)
            pred = segmentation_argmax_in_roi(logits, roi_mask=roi_tensor)[0].detach().cpu().numpy().astype(np.uint8)
            mask = (pred > 0).astype(np.uint8) * 255

            mask_path = masks_dir / f"{image_path.stem}.png"
            if not cv2.imwrite(str(mask_path), mask):
                raise RuntimeError(f"Failed to write mask: {mask_path}")
            if args.save_overlay:
                cv2.imwrite(str(overlays_dir / f"{image_path.stem}.jpg"), overlay_mask(image_path, mask, image_size))

            row = {
                "frame_id": image_path.stem,
                "image_path": str(image_path),
                "seg_mask_path": str(mask_path),
                "seg_confidence": 1.0,
                "backend": "multitask_resnet18_manual_roi" if not args.no_apply_roi else "multitask_resnet18_manual_raw",
                "num_lane_pixels": int((mask == 255).sum()),
            }
            handle.write(json.dumps(row) + "\n")

    LOGGER.info("records=%s images=%d roi_applied=%s", records_path, len(image_paths), not args.no_apply_roi)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export ROI-aware segmentation masks from a multitask checkpoint.")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--images_dir", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--config", default="configs/multitask/multitask_resnet18.yaml")
    parser.add_argument("--roi_mask", default="outputs/equirect_run/roi/roi_mask.png")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--save_overlay", action="store_true")
    parser.add_argument("--no_apply_roi", action="store_true")
    parser.add_argument("--limit", type=int, default=0)
    return parser.parse_args()


if __name__ == "__main__":
    try:
        predict(parse_args())
    except Exception as exc:
        LOGGER.error("%s", exc)
        raise SystemExit(1) from None
