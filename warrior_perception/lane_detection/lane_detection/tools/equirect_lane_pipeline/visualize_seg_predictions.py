from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from data.dataset import MultitaskDataset
from engine.train import load_yaml, resolve_device
from models import MultiTaskPerceptionModel
from models.segmentation_roi import segmentation_argmax_in_roi


IMAGENET_MEAN = np.asarray([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.asarray([0.229, 0.224, 0.225], dtype=np.float32)


def read_manifest(path: str | Path) -> list[dict[str, Any]]:
    with Path(path).open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    rows = data.get("samples", data) if isinstance(data, dict) else data
    if not isinstance(rows, list) or not rows:
        raise ValueError(f"Expected non-empty manifest list: {path}")
    return rows


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


def tensor_to_rgb(image: torch.Tensor, normalized: bool) -> np.ndarray:
    array = image.detach().cpu().permute(1, 2, 0).numpy().astype(np.float32)
    if normalized:
        array = array * IMAGENET_STD + IMAGENET_MEAN
    return (np.clip(array, 0.0, 1.0) * 255.0).round().astype(np.uint8)


def overlay_binary(image_rgb: np.ndarray, mask: np.ndarray, color_rgb: tuple[int, int, int], alpha: float = 0.35) -> np.ndarray:
    overlay = image_rgb.copy()
    overlay[mask > 0] = np.asarray(color_rgb, dtype=np.uint8)
    return cv2.addWeighted(image_rgb, 1.0 - alpha, overlay, alpha, 0.0)


def make_comparison(image_rgb: np.ndarray, gt: np.ndarray, pred: np.ndarray, roi: np.ndarray | None) -> np.ndarray:
    comparison = image_rgb.copy()
    tp = (pred > 0) & (gt > 0)
    fp = (pred > 0) & (gt == 0)
    fn = (pred == 0) & (gt > 0)
    color = comparison.copy()
    color[tp] = np.asarray([0, 255, 0], dtype=np.uint8)
    color[fp] = np.asarray([255, 0, 0], dtype=np.uint8)
    color[fn] = np.asarray([0, 128, 255], dtype=np.uint8)
    comparison = cv2.addWeighted(comparison, 0.62, color, 0.38, 0.0)
    if roi is not None:
        edges = cv2.Canny((roi > 0).astype(np.uint8) * 255, 50, 150)
        comparison[edges > 0] = np.asarray([255, 0, 255], dtype=np.uint8)
    return comparison


def save_preview(
    output_path: Path,
    image_rgb: np.ndarray,
    gt: np.ndarray,
    pred: np.ndarray,
    roi: np.ndarray | None,
) -> None:
    gt_overlay = overlay_binary(image_rgb, gt, (0, 255, 0))
    pred_overlay = overlay_binary(image_rgb, pred, (255, 0, 0))
    comparison = make_comparison(image_rgb, gt, pred, roi)
    panels = [image_rgb, gt_overlay, pred_overlay, comparison]
    preview_rgb = np.concatenate(panels, axis=1)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), cv2.cvtColor(preview_rgb, cv2.COLOR_RGB2BGR))


@torch.no_grad()
def visualize(args: argparse.Namespace) -> None:
    device = resolve_device(args.device)
    model, cfg = load_checkpoint(args.checkpoint, args.config, device)
    cfg["dataset"]["val_manifest"] = args.manifest
    if args.image_root:
        cfg["dataset"]["image_root"] = args.image_root
    if args.seg_root:
        cfg["dataset"]["seg_root"] = args.seg_root

    rows = read_manifest(args.manifest)
    indices = list(range(len(rows)))
    rng = random.Random(args.seed)
    if args.num_samples > 0 and len(indices) > args.num_samples:
        indices = sorted(rng.sample(indices, args.num_samples))

    dataset = MultitaskDataset(
        manifest_path=args.manifest,
        image_size=cfg["dataset"]["image_size"],
        image_root=cfg["dataset"].get("image_root", "."),
        seg_root=cfg["dataset"].get("seg_root", "."),
        normalize=bool(cfg["dataset"].get("normalize", True)),
        augment=None,
    )

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    summary = []
    for index in indices:
        item = dataset[index]
        image = item["image"].unsqueeze(0).to(device)
        preds = model(image, postprocess=False)
        gt_mask = item["seg_mask"].detach().cpu().numpy().astype(np.uint8)
        roi = item["seg_roi_mask"].detach().cpu().numpy().astype(np.uint8)
        full_pred_mask = preds["segmentation"].argmax(dim=1)[0].detach().cpu().numpy().astype(np.uint8)
        roi_tensor = None if args.show_full_prediction else item["seg_roi_mask"].unsqueeze(0).to(device)
        display_pred_mask = segmentation_argmax_in_roi(preds["segmentation"], roi_mask=roi_tensor)[0].detach().cpu().numpy().astype(np.uint8)
        image_rgb = tensor_to_rgb(item["image"], normalized=bool(cfg["dataset"].get("normalize", True)))

        frame_id = Path(str(rows[index].get("image", f"sample_{index:06d}"))).stem
        save_preview(output_dir / f"{frame_id}.jpg", image_rgb, gt_mask, display_pred_mask, roi)
        summary.append(
            {
                "frame_id": frame_id,
                "gt_pixels": int((gt_mask > 0).sum()),
                "pred_pixels": int((full_pred_mask > 0).sum()),
                "pred_pixels_in_roi": int((display_pred_mask > 0).sum()),
            }
        )

    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"output_dir={output_dir} previews={len(summary)}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize multitask segmentation predictions against manual masks.")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--config", default="configs/multitask/multitask_resnet18.yaml")
    parser.add_argument("--image_root", default=".")
    parser.add_argument("--seg_root", default=".")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--num_samples", type=int, default=50)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--show_full_prediction", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    visualize(parse_args())
