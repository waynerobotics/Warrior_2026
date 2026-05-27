from __future__ import annotations

from typing import Any

import torch
import torch.nn.functional as F
from torch import nn
from torchvision.ops import generalized_box_iou_loss

from .segmentation_roi import roi_valid_mask


def _focal_bce(logits: torch.Tensor, targets: torch.Tensor, gamma: float = 2.0, alpha: float = 0.25) -> torch.Tensor:
    """Sigmoid focal loss for dense objectness — down-weights easy negatives."""
    bce = F.binary_cross_entropy_with_logits(logits, targets, reduction="none")
    p = torch.sigmoid(logits)
    p_t = p * targets + (1 - p) * (1 - targets)
    alpha_t = alpha * targets + (1 - alpha) * (1 - targets)
    return (alpha_t * (1 - p_t) ** gamma * bce).mean()


class MultiTaskLoss(nn.Module):
    def __init__(self, cfg: dict[str, Any]) -> None:
        super().__init__()
        self.det_weight = float(cfg.get("loss_weights", {}).get("detection", 1.0))
        self.seg_weight = float(cfg.get("loss_weights", {}).get("segmentation", 1.0))
        self.num_det_classes = int(cfg["detection"]["num_classes"])
        self.detection_enabled = bool(cfg.get("detection", {}).get("enabled", True))
        self.segmentation_enabled = bool(cfg.get("segmentation", {}).get("enabled", True))
        self.seg_loss_type = str(cfg.get("segmentation", {}).get("loss", "ce")).lower()
        self.ignore_index = int(cfg.get("segmentation", {}).get("ignore_index", -100))
        class_weights = cfg.get("segmentation", {}).get("class_weights")
        if class_weights is not None:
            self.register_buffer("seg_class_weights", torch.tensor(class_weights, dtype=torch.float32))
        else:
            self.seg_class_weights = None

    def forward(self, preds: dict[str, Any], targets: dict[str, Any]) -> dict[str, torch.Tensor]:
        zero = next(iter(preds["detection"].values())).sum() * 0.0
        if self.detection_enabled:
            det_loss = self._detection_loss(
                preds["detection"],
                boxes_list=targets["detection"][0],
                labels_list=targets["detection"][1],
                image_size=tuple(targets["segmentation"].shape[-2:]),
            )
        else:
            det_loss = zero

        if self.segmentation_enabled and targets.get("has_seg", torch.tensor(False)).any():
            seg_loss = self._segmentation_loss(
                logits=preds["segmentation"],
                masks=targets["segmentation"],
                confidence=targets.get("seg_confidence"),
                roi_mask=targets.get("seg_roi_mask"),
                has_seg=targets.get("has_seg"),
            )
        else:
            seg_loss = zero

        total = self.det_weight * det_loss + self.seg_weight * seg_loss
        return {"det": det_loss, "seg": seg_loss, "total": total}

    def _segmentation_loss(
        self,
        logits: torch.Tensor,
        masks: torch.Tensor,
        confidence: torch.Tensor | None,
        roi_mask: torch.Tensor | None,
        has_seg: torch.Tensor | None,
    ) -> torch.Tensor:
        valid_bool = roi_valid_mask(masks.to(logits.device), roi_mask=roi_mask, has_seg=has_seg)
        valid_bool &= masks.to(logits.device) != self.ignore_index
        valid = valid_bool.to(logits.dtype)
        if confidence is None:
            confidence = torch.ones((logits.shape[0],), device=logits.device, dtype=logits.dtype)
        confidence = confidence.to(logits.device, dtype=logits.dtype).view(-1, 1, 1)
        safe_masks = masks.to(logits.device).clone()
        safe_masks = torch.where(valid_bool, safe_masks, torch.zeros_like(safe_masks))
        safe_masks = safe_masks.clamp(min=0)
        valid_weight = valid * confidence
        if self.seg_class_weights is not None:
            class_weights = self.seg_class_weights.to(logits.device, dtype=logits.dtype)
            if class_weights.numel() != logits.shape[1]:
                raise ValueError(
                    f"segmentation.class_weights has {class_weights.numel()} values, "
                    f"but the segmentation head has {logits.shape[1]} classes"
                )
            valid_weight = valid_weight * class_weights[safe_masks.long()]
        denom = valid_weight.sum().clamp(min=1.0)

        if self.seg_loss_type == "dice":
            return self._dice_loss(logits, safe_masks, valid_weight)
        ce = F.cross_entropy(logits, safe_masks.long(), reduction="none")
        return (ce * valid_weight).sum() / denom

    @staticmethod
    def _dice_loss(logits: torch.Tensor, masks: torch.Tensor, valid_weight: torch.Tensor) -> torch.Tensor:
        probs = torch.softmax(logits, dim=1)
        classes = logits.shape[1]
        one_hot = F.one_hot(masks.long().clamp(min=0, max=classes - 1), num_classes=classes).permute(0, 3, 1, 2).float()
        weight = valid_weight.unsqueeze(1)
        intersection = (probs * one_hot * weight).sum(dim=(0, 2, 3))
        union = ((probs + one_hot) * weight).sum(dim=(0, 2, 3))
        dice = (2.0 * intersection + 1e-6) / (union + 1e-6)
        return 1.0 - dice.mean()

    def _detection_loss(
        self,
        preds: dict[str, torch.Tensor],
        boxes_list: list[torch.Tensor],
        labels_list: list[torch.Tensor],
        image_size: tuple[int, int],
    ) -> torch.Tensor:
        total_obj = preds["p3"].new_tensor(0.0)
        total_box = preds["p3"].new_tensor(0.0)
        total_cls = preds["p3"].new_tensor(0.0)
        input_h, input_w = image_size
        for level_name, pred in preds.items():
            obj_target, box_target, cls_target, pos_mask = self._build_targets_for_level(
                pred, boxes_list, labels_list, image_size, level_name
            )
            batch_size, _, feat_h, feat_w = pred.shape
            stride_y = input_h / float(feat_h)
            stride_x = input_w / float(feat_w)

            obj_logits = pred[:, 4]
            box_logits = torch.relu(pred[:, :4].permute(0, 2, 3, 1))  # (B, H, W, 4) LTRB in stride units
            cls_logits = pred[:, 5:].permute(0, 2, 3, 1)

            total_obj = total_obj + _focal_bce(obj_logits, obj_target)

            if pos_mask.any():
                # Cell centres for every spatial position
                yi = torch.arange(feat_h, device=pred.device, dtype=pred.dtype)
                xi = torch.arange(feat_w, device=pred.device, dtype=pred.dtype)
                gy, gx = torch.meshgrid(yi, xi, indexing="ij")
                cell_cx = (gx + 0.5) * stride_x  # (H, W)
                cell_cy = (gy + 0.5) * stride_y
                cell_cx = cell_cx.unsqueeze(0).expand(batch_size, -1, -1)  # (B, H, W)
                cell_cy = cell_cy.unsqueeze(0).expand(batch_size, -1, -1)

                pos_cx = cell_cx[pos_mask]  # (N,)
                pos_cy = cell_cy[pos_mask]

                # Decode predicted LTRB → absolute xyxy, clamped to image bounds
                pl = box_logits[pos_mask]  # (N, 4)
                pred_boxes = torch.stack([
                    (pos_cx - pl[:, 0] * stride_x).clamp(0, input_w),
                    (pos_cy - pl[:, 1] * stride_y).clamp(0, input_h),
                    (pos_cx + pl[:, 2] * stride_x).clamp(0, input_w),
                    (pos_cy + pl[:, 3] * stride_y).clamp(0, input_h),
                ], dim=1)

                # Decode target LTRB → absolute xyxy
                tl = box_target[pos_mask]  # (N, 4)
                tgt_boxes = torch.stack([
                    pos_cx - tl[:, 0] * stride_x,
                    pos_cy - tl[:, 1] * stride_y,
                    pos_cx + tl[:, 2] * stride_x,
                    pos_cy + tl[:, 3] * stride_y,
                ], dim=1)

                total_box = total_box + generalized_box_iou_loss(pred_boxes, tgt_boxes, reduction="mean")
                cls_t = cls_target[pos_mask].clamp(0, self.num_det_classes - 1)
                total_cls = total_cls + F.cross_entropy(cls_logits[pos_mask], cls_t, reduction="mean")
        return total_obj + total_box + total_cls

    def _build_targets_for_level(
        self,
        pred: torch.Tensor,
        boxes_list: list[torch.Tensor],
        labels_list: list[torch.Tensor],
        image_size: tuple[int, int],
        level_name: str,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        batch_size, _, feat_h, feat_w = pred.shape
        input_h, input_w = image_size
        stride_y = input_h / float(feat_h)
        stride_x = input_w / float(feat_w)
        obj_target = pred.new_zeros((batch_size, feat_h, feat_w))
        box_target = pred.new_zeros((batch_size, feat_h, feat_w, 4))
        cls_target = torch.zeros((batch_size, feat_h, feat_w), device=pred.device, dtype=torch.long)
        pos_mask = torch.zeros((batch_size, feat_h, feat_w), device=pred.device, dtype=torch.bool)
        for batch_idx, (boxes, labels) in enumerate(zip(boxes_list, labels_list)):
            if boxes.numel() == 0:
                continue
            boxes = boxes.to(pred.device)
            labels = labels.to(pred.device).long()
            for box, label in zip(boxes, labels):
                if label < 0 or label >= self.num_det_classes:
                    continue
                x1, y1, x2, y2 = float(box[0]), float(box[1]), float(box[2]), float(box[3])
                if self._select_level(max(x2 - x1, y2 - y1)) != level_name:
                    continue
                # FCOS-style: assign every cell whose centre falls strictly inside the GT box
                gx_lo = max(0, int(x1 / stride_x))
                gx_hi = min(feat_w - 1, int((x2 - 1e-6) / stride_x))
                gy_lo = max(0, int(y1 / stride_y))
                gy_hi = min(feat_h - 1, int((y2 - 1e-6) / stride_y))
                assigned = False
                for gy in range(gy_lo, gy_hi + 1):
                    for gx in range(gx_lo, gx_hi + 1):
                        ccx = (gx + 0.5) * stride_x
                        ccy = (gy + 0.5) * stride_y
                        if not (x1 < ccx < x2 and y1 < ccy < y2):
                            continue
                        obj_target[batch_idx, gy, gx] = 1.0
                        box_target[batch_idx, gy, gx] = torch.tensor([
                            (ccx - x1) / stride_x,
                            (ccy - y1) / stride_y,
                            (x2 - ccx) / stride_x,
                            (y2 - ccy) / stride_y,
                        ], device=pred.device, dtype=pred.dtype)
                        cls_target[batch_idx, gy, gx] = label
                        pos_mask[batch_idx, gy, gx] = True
                        assigned = True
                if not assigned:
                    # Fallback for very small objects: use the centre cell
                    gx = min(feat_w - 1, max(0, int((x1 + x2) * 0.5 / stride_x)))
                    gy = min(feat_h - 1, max(0, int((y1 + y2) * 0.5 / stride_y)))
                    ccx = (gx + 0.5) * stride_x
                    ccy = (gy + 0.5) * stride_y
                    obj_target[batch_idx, gy, gx] = 1.0
                    box_target[batch_idx, gy, gx] = torch.tensor([
                        max((ccx - x1) / stride_x, 0.0),
                        max((ccy - y1) / stride_y, 0.0),
                        max((x2 - ccx) / stride_x, 0.0),
                        max((y2 - ccy) / stride_y, 0.0),
                    ], device=pred.device, dtype=pred.dtype)
                    cls_target[batch_idx, gy, gx] = label
                    pos_mask[batch_idx, gy, gx] = True
        return obj_target, box_target, cls_target, pos_mask

    @staticmethod
    def _select_level(max_side: float) -> str:
        if max_side <= 64:
            return "p3"
        if max_side <= 128:
            return "p4"
        return "p5"
