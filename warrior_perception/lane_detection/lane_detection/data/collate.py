from __future__ import annotations

from typing import Any

import torch


def collate_fn(batch: list[dict[str, torch.Tensor]]) -> dict[str, Any]:
    images = torch.stack([b["image"] for b in batch])
    boxes = [b["boxes"] for b in batch]
    labels = [b["labels"] for b in batch]
    seg_masks = torch.stack([b["seg_mask"] for b in batch])
    seg_confidence = torch.stack([b["seg_confidence"] for b in batch])
    seg_roi_masks = torch.stack([b["seg_roi_mask"] for b in batch])
    has_seg = torch.stack([b["has_seg"] for b in batch])
    return {
        "image": images,
        "boxes": boxes,
        "labels": labels,
        "seg_mask": seg_masks,
        "seg_confidence": seg_confidence,
        "seg_roi_mask": seg_roi_masks,
        "has_seg": has_seg,
    }
