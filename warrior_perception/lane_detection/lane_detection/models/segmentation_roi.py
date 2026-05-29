from __future__ import annotations

import torch


def roi_valid_mask(
    reference: torch.Tensor,
    roi_mask: torch.Tensor | None = None,
    has_seg: torch.Tensor | None = None,
) -> torch.Tensor:
    valid = torch.ones_like(reference, dtype=torch.bool)
    if roi_mask is not None:
        valid &= roi_mask.to(reference.device) > 0
    if has_seg is not None:
        valid &= has_seg.to(reference.device).view(-1, 1, 1).bool()
    return valid


def segmentation_argmax_in_roi(
    logits: torch.Tensor,
    roi_mask: torch.Tensor | None = None,
    background_class: int = 0,
) -> torch.Tensor:
    pred = logits.argmax(dim=1)
    if roi_mask is None:
        return pred
    roi = roi_mask.to(pred.device) > 0
    return torch.where(roi, pred, torch.full_like(pred, int(background_class)))
