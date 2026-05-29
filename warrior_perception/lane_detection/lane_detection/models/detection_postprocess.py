from __future__ import annotations

from typing import Any

import torch
from torchvision.ops import nms


def _decode_level(
    pred: torch.Tensor,
    image_size: tuple[int, int],
    score_threshold: float,
    topk: int,
) -> list[tuple[torch.Tensor, torch.Tensor, torch.Tensor]]:
    batch_size, _, feat_h, feat_w = pred.shape
    input_h, input_w = image_size
    stride_y = input_h / float(feat_h)
    stride_x = input_w / float(feat_w)

    box_raw = torch.relu(pred[:, :4])
    obj_logits = pred[:, 4]
    cls_logits = pred[:, 5:]

    y_coords = (torch.arange(feat_h, device=pred.device, dtype=pred.dtype) + 0.5) * stride_y
    x_coords = (torch.arange(feat_w, device=pred.device, dtype=pred.dtype) + 0.5) * stride_x
    yy, xx = torch.meshgrid(y_coords, x_coords, indexing="ij")
    xx = xx.unsqueeze(0).expand(batch_size, -1, -1)
    yy = yy.unsqueeze(0).expand(batch_size, -1, -1)

    left = box_raw[:, 0] * stride_x
    top = box_raw[:, 1] * stride_y
    right = box_raw[:, 2] * stride_x
    bottom = box_raw[:, 3] * stride_y

    x1 = (xx - left).clamp(min=0, max=input_w)
    y1 = (yy - top).clamp(min=0, max=input_h)
    x2 = (xx + right).clamp(min=0, max=input_w)
    y2 = (yy + bottom).clamp(min=0, max=input_h)

    cls_probs = torch.softmax(cls_logits, dim=1)
    cls_scores, cls_labels = cls_probs.max(dim=1)
    scores = torch.sigmoid(obj_logits) * cls_scores

    boxes = torch.stack([x1, y1, x2, y2], dim=-1).reshape(batch_size, -1, 4)
    scores = scores.reshape(batch_size, -1)
    labels = cls_labels.reshape(batch_size, -1)

    outputs: list[tuple[torch.Tensor, torch.Tensor, torch.Tensor]] = []
    for batch_idx in range(batch_size):
        keep = scores[batch_idx] >= score_threshold
        level_boxes = boxes[batch_idx][keep]
        level_scores = scores[batch_idx][keep]
        level_labels = labels[batch_idx][keep].to(torch.int64)
        if level_scores.numel() > topk:
            top_indices = torch.topk(level_scores, k=topk).indices
            level_boxes = level_boxes[top_indices]
            level_scores = level_scores[top_indices]
            level_labels = level_labels[top_indices]
        outputs.append((level_boxes, level_scores, level_labels))
    return outputs


def decode_detections(
    level_outputs: dict[str, torch.Tensor],
    image_size: tuple[int, int],
    score_threshold: float = 0.25,
    nms_threshold: float = 0.5,
    max_detections: int = 100,
) -> list[dict[str, Any]]:
    batch_size = next(iter(level_outputs.values())).shape[0]
    merged_boxes: list[list[torch.Tensor]] = [[] for _ in range(batch_size)]
    merged_scores: list[list[torch.Tensor]] = [[] for _ in range(batch_size)]
    merged_labels: list[list[torch.Tensor]] = [[] for _ in range(batch_size)]

    for pred in level_outputs.values():
        per_level = _decode_level(
            pred=pred,
            image_size=image_size,
            score_threshold=score_threshold,
            topk=max_detections,
        )
        for batch_idx, (boxes, scores, labels) in enumerate(per_level):
            if boxes.numel() == 0:
                continue
            merged_boxes[batch_idx].append(boxes)
            merged_scores[batch_idx].append(scores)
            merged_labels[batch_idx].append(labels)

    outputs: list[dict[str, Any]] = []
    for batch_idx in range(batch_size):
        if not merged_boxes[batch_idx]:
            outputs.append(
                {
                    "boxes": torch.zeros((0, 4), dtype=torch.float32),
                    "scores": torch.zeros((0,), dtype=torch.float32),
                    "labels": torch.zeros((0,), dtype=torch.int64),
                }
            )
            continue

        boxes = torch.cat(merged_boxes[batch_idx], dim=0)
        scores = torch.cat(merged_scores[batch_idx], dim=0)
        labels = torch.cat(merged_labels[batch_idx], dim=0)
        keep = nms(boxes, scores, iou_threshold=nms_threshold)[:max_detections]
        outputs.append(
            {
                "boxes": boxes[keep].detach().cpu(),
                "scores": scores[keep].detach().cpu(),
                "labels": labels[keep].detach().cpu(),
            }
        )
    return outputs
