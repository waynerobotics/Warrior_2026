from __future__ import annotations

import argparse
import csv
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
from models import MultiTaskPerceptionModel, decode_detections, segmentation_argmax_in_roi


IMAGENET_MEAN = np.asarray([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.asarray([0.229, 0.224, 0.225], dtype=np.float32)


def parse_bool(value: str | bool | None) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().lower() in {"1", "true", "yes", "y"}


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


def draw_text(image_rgb: np.ndarray, text: str) -> np.ndarray:
    image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
    cv2.rectangle(image_bgr, (0, 0), (image_bgr.shape[1], 30), (20, 20, 20), thickness=-1)
    cv2.putText(image_bgr, text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (235, 235, 235), 1, cv2.LINE_AA)
    return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)


def overlay_segmentation(
    image_rgb: np.ndarray,
    gt_mask: np.ndarray | None,
    pred_mask: np.ndarray | None,
    roi_mask: np.ndarray | None,
) -> np.ndarray:
    canvas = image_rgb.copy()
    if gt_mask is not None:
        gt_overlay = canvas.copy()
        gt_overlay[gt_mask > 0] = np.asarray([0, 255, 0], dtype=np.uint8)
        canvas = cv2.addWeighted(canvas, 0.75, gt_overlay, 0.25, 0.0)
    if pred_mask is not None:
        pred_overlay = canvas.copy()
        pred_overlay[pred_mask > 0] = np.asarray([255, 0, 0], dtype=np.uint8)
        canvas = cv2.addWeighted(canvas, 0.72, pred_overlay, 0.28, 0.0)
    if roi_mask is not None:
        edges = cv2.Canny((roi_mask > 0).astype(np.uint8) * 255, 50, 150)
        canvas[edges > 0] = np.asarray([255, 0, 255], dtype=np.uint8)
    return canvas


def overlay_detections(image_rgb: np.ndarray, detections: dict[str, torch.Tensor]) -> np.ndarray:
    canvas_bgr = cv2.cvtColor(image_rgb.copy(), cv2.COLOR_RGB2BGR)
    boxes = detections.get("boxes", torch.zeros((0, 4)))
    scores = detections.get("scores", torch.zeros((0,)))
    labels = detections.get("labels", torch.zeros((0,), dtype=torch.int64))
    if torch.is_tensor(boxes):
        boxes = boxes.detach().cpu().numpy()
    if torch.is_tensor(scores):
        scores = scores.detach().cpu().numpy()
    if torch.is_tensor(labels):
        labels = labels.detach().cpu().numpy()
    for index, box in enumerate(boxes):
        x1, y1, x2, y2 = [int(round(value)) for value in box]
        cv2.rectangle(canvas_bgr, (x1, y1), (x2, y2), (20, 220, 20), 2)
        score = float(scores[index]) if index < len(scores) else 0.0
        label = int(labels[index]) if index < len(labels) else -1
        caption = f"cls {label} {score:.2f}"
        cv2.putText(canvas_bgr, caption, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 220, 20), 1, cv2.LINE_AA)
    return cv2.cvtColor(canvas_bgr, cv2.COLOR_BGR2RGB)


def load_runs_from_comparison(path: str | Path, include_exploratory: bool) -> list[dict[str, str]]:
    runs: list[dict[str, str]] = []
    with Path(path).open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            if not parse_bool(row.get("segmentation_enabled")):
                continue
            if not row.get("checkpoint_path"):
                continue
            if not include_exploratory and "exploratory" in row.get("run_name", ""):
                continue
            runs.append(
                {
                    "run_name": row["run_name"],
                    "backbone": row["backbone"],
                    "checkpoint_path": row["checkpoint_path"],
                }
            )
    if not runs:
        raise ValueError(f"No segmentation-enabled runs found in {path}")
    return runs


def select_indices(total: int, num_samples: int, seed: int) -> list[int]:
    indices = list(range(total))
    if num_samples <= 0 or total <= num_samples:
        return indices
    rng = random.Random(seed)
    return sorted(rng.sample(indices, num_samples))


@torch.no_grad()
def visualize(args: argparse.Namespace) -> None:
    device = resolve_device(args.device)
    runs = load_runs_from_comparison(args.comparison_csv, include_exploratory=args.include_exploratory)
    models_by_run: list[dict[str, Any]] = []
    base_cfg: dict[str, Any] | None = None
    for run in runs:
        model, cfg = load_checkpoint(run["checkpoint_path"], args.config, device)
        models_by_run.append(
            {
                "run_name": run["run_name"],
                "backbone": run["backbone"],
                "checkpoint_path": run["checkpoint_path"],
                "cfg": cfg,
                "model": model,
            }
        )
        if base_cfg is None:
            base_cfg = cfg

    if base_cfg is None:
        raise RuntimeError("No runs were loaded.")

    dataset = MultitaskDataset(
        manifest_path=args.manifest,
        image_size=base_cfg["dataset"]["image_size"],
        image_root=base_cfg["dataset"].get("image_root", "."),
        seg_root=base_cfg["dataset"].get("seg_root", "."),
        normalize=bool(base_cfg["dataset"].get("normalize", True)),
        augment=None,
    )
    manifest_rows = read_manifest(args.manifest)
    indices = select_indices(len(dataset), args.num_samples, args.seed)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    summary: list[dict[str, Any]] = []
    for index in indices:
        item = dataset[index]
        image_tensor = item["image"].unsqueeze(0).to(device)
        image_rgb = tensor_to_rgb(item["image"], normalized=bool(base_cfg["dataset"].get("normalize", True)))
        gt_mask = item["seg_mask"].detach().cpu().numpy().astype(np.uint8)
        roi_mask = item["seg_roi_mask"].detach().cpu().numpy().astype(np.uint8)
        frame_id = Path(str(manifest_rows[index].get("image", f"sample_{index:06d}"))).stem

        top_row = [
            draw_text(image_rgb.copy(), "Input"),
            draw_text(overlay_segmentation(image_rgb, gt_mask, None, roi_mask), "GT Segmentation"),
        ]
        backbone_rows: list[np.ndarray] = []
        frame_summary: dict[str, Any] = {"frame_id": frame_id, "runs": []}

        for run in models_by_run:
            cfg = run["cfg"]
            preds = run["model"](image_tensor, postprocess=False)
            detections = decode_detections(
                level_outputs=preds["detection"],
                image_size=(image_tensor.shape[-2], image_tensor.shape[-1]),
                score_threshold=float(cfg["detection"].get("score_threshold", 0.25)),
                nms_threshold=float(cfg["detection"].get("nms_threshold", 0.5)),
                max_detections=int(cfg["detection"].get("max_detections", 100)),
            )[0]
            pred_mask = segmentation_argmax_in_roi(
                preds["segmentation"],
                roi_mask=item["seg_roi_mask"].unsqueeze(0).to(device),
            )[0].detach().cpu().numpy().astype(np.uint8)

            det_panel = draw_text(overlay_detections(image_rgb, detections), f"{run['backbone']} Detection")
            seg_panel = draw_text(
                overlay_segmentation(image_rgb, gt_mask, pred_mask, roi_mask),
                f"{run['backbone']} Segmentation",
            )
            row = np.concatenate([det_panel, seg_panel], axis=1)
            backbone_rows.append(row)
            frame_summary["runs"].append(
                {
                    "run_name": run["run_name"],
                    "backbone": run["backbone"],
                    "num_detections": int(len(detections["scores"])),
                    "pred_pixels_in_roi": int((pred_mask > 0).sum()),
                }
            )

        header = np.concatenate(top_row, axis=1)
        grid = np.concatenate([header] + backbone_rows, axis=0)
        frame_path = output_dir / f"{frame_id}.jpg"
        cv2.imwrite(str(frame_path), cv2.cvtColor(grid, cv2.COLOR_RGB2BGR))
        frame_summary["output_path"] = str(frame_path)
        summary.append(frame_summary)

    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(f"output_dir={output_dir} frames={len(summary)} runs={len(models_by_run)}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize detection and segmentation heads across backbone checkpoints.")
    parser.add_argument("--comparison_csv", required=True, help="Comparison CSV produced by scripts/run_experiments.py")
    parser.add_argument("--manifest", required=True, help="Manifest used to sample visualization frames")
    parser.add_argument("--output_dir", required=True)
    parser.add_argument("--config", default="configs/multitask/multitask_resnet18.yaml")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--num_samples", type=int, default=4)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--include_exploratory", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    visualize(parse_args())
