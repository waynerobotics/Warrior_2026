from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import torch
from torchmetrics.detection import MeanAveragePrecision

from models.detection_postprocess import decode_detections
from models.segmentation_roi import roi_valid_mask, segmentation_argmax_in_roi


def compute_seg_iou(
    pred_logits: torch.Tensor,
    gt_masks: torch.Tensor,
    num_classes: int,
    roi_mask: torch.Tensor | None = None,
    has_seg: torch.Tensor | None = None,
) -> dict[str, Any]:
    pred = segmentation_argmax_in_roi(pred_logits, roi_mask=roi_mask)
    valid = roi_valid_mask(gt_masks, roi_mask=roi_mask, has_seg=has_seg)
    ious: list[float] = []
    dices: list[float] = []
    for class_idx in range(num_classes):
        pred_c = (pred == class_idx) & valid
        gt_c = (gt_masks == class_idx) & valid
        intersection = (pred_c & gt_c).sum().float()
        union = (pred_c | gt_c).sum().float()
        denom = pred_c.sum().float() + gt_c.sum().float()
        ious.append((intersection / union.clamp(min=1e-6)).item())
        dices.append(((2.0 * intersection) / denom.clamp(min=1e-6)).item())
    return {
        "per_class_iou": ious,
        "mean_iou": sum(ious) / len(ious),
        "per_class_dice": dices,
        "mean_dice": sum(dices) / len(dices),
    }


@torch.no_grad()
def evaluate(
    model: torch.nn.Module,
    dataloader: Any,
    cfg: dict[str, Any],
    device: torch.device,
    epoch: int | None = None,
    artifact_dir: str | Path | None = None,
) -> dict[str, Any]:
    model.eval()
    metric = MeanAveragePrecision(iou_type="bbox", class_metrics=True)
    seg_scores: list[dict[str, Any]] = []
    seg_enabled = bool(cfg.get("segmentation", {}).get("enabled", True))
    sample_count = 0
    start_time = time.perf_counter()

    for batch in dataloader:
        images = batch["image"].to(device)
        preds = model(images, postprocess=False)
        sample_count += int(images.shape[0])
        det_preds = decode_detections(
            level_outputs=preds["detection"],
            image_size=(images.shape[-2], images.shape[-1]),
            score_threshold=0.01,  # low threshold for mAP; inference threshold is applied separately
            nms_threshold=float(cfg["detection"].get("nms_threshold", 0.5)),
            max_detections=int(cfg["detection"].get("max_detections", 100)),
        )
        det_targets = [{"boxes": boxes.cpu(), "labels": labels.cpu()} for boxes, labels in zip(batch["boxes"], batch["labels"])]
        metric.update(preds=det_preds, target=det_targets)

        if seg_enabled and batch.get("has_seg") is not None and batch["has_seg"].any():
            seg_scores.append(
                compute_seg_iou(
                    pred_logits=preds["segmentation"].cpu(),
                    gt_masks=batch["seg_mask"].cpu(),
                    num_classes=int(cfg["segmentation"]["num_classes"]),
                    roi_mask=batch.get("seg_roi_mask"),
                    has_seg=batch.get("has_seg"),
                )
            )

    det_result = metric.compute()

    # Per-class detection AP — torchmetrics returns -1 for unseen classes
    raw_per_class_ap = det_result.get("map_per_class", torch.tensor([]))
    per_class_det_ap: list[float] = []
    if raw_per_class_ap.numel() > 0:
        per_class_det_ap = [
            float(v) if float(v) >= 0.0 else 0.0
            for v in raw_per_class_ap.reshape(-1).tolist()
        ]

    per_class_count = int(cfg["segmentation"]["num_classes"])
    per_class_iou = [0.0 for _ in range(per_class_count)]
    per_class_dice = [0.0 for _ in range(per_class_count)]
    mean_iou = 0.0
    mean_dice = 0.0
    if seg_scores:
        mean_iou = sum(item["mean_iou"] for item in seg_scores) / len(seg_scores)
        mean_dice = sum(item["mean_dice"] for item in seg_scores) / len(seg_scores)
        per_class_iou = [sum(item["per_class_iou"][idx] for item in seg_scores) / len(seg_scores) for idx in range(per_class_count)]
        per_class_dice = [sum(item["per_class_dice"][idx] for item in seg_scores) / len(seg_scores) for idx in range(per_class_count)]

    elapsed = time.perf_counter() - start_time
    latency_ms = (elapsed / sample_count * 1000.0) if sample_count else 0.0
    throughput_fps = sample_count / max(1e-6, elapsed)

    results = {
        "det_mAP": float(det_result.get("map", torch.tensor(0.0)).item()),
        "det_mAP_50": float(det_result.get("map_50", torch.tensor(0.0)).item()),
        "det_mAP_75": float(det_result.get("map_75", torch.tensor(0.0)).item()),
        "per_class_det_ap": per_class_det_ap,
        "seg_iou": float(mean_iou),
        "seg_dice": float(mean_dice),
        "per_class_iou": per_class_iou,
        "per_class_dice": per_class_dice,
        "latency_ms": float(latency_ms),
        "throughput_fps": float(throughput_fps),
    }
    if artifact_dir is not None and epoch is not None:
        artifact_path = Path(artifact_dir)
        artifact_path.mkdir(parents=True, exist_ok=True)
        json_path = artifact_path / f"epoch_{epoch:03d}_per_class_iou.json"
        json_path.write_text(json.dumps({"per_class_iou": per_class_iou, "per_class_dice": per_class_dice}, indent=2), encoding="utf-8")
        results["per_class_iou_path"] = str(json_path)
    return results
