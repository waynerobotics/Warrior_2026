from __future__ import annotations

from typing import Any

import torch
from torch import nn

from .backbone import build_backbone
from .detection_head import DetectionHead
from .detection_postprocess import decode_detections
from .fpn import FPN
from .segmentation_head import SegmentationHead


class MultiTaskPerceptionModel(nn.Module):
    def __init__(self, cfg: dict[str, Any]) -> None:
        super().__init__()
        self.cfg = cfg
        self.backbone = build_backbone(cfg)
        self.fpn = FPN(
            in_channels=self.backbone.out_channels,
            out_channels=int(cfg["fpn"]["out_channels"]),
        )
        self.detection_head = DetectionHead(
            in_channels=int(cfg["fpn"]["out_channels"]),
            num_classes=int(cfg["detection"]["num_classes"]),
        )
        self.segmentation_head = SegmentationHead(
            in_channels=int(cfg["fpn"]["out_channels"]),
            num_classes=int(cfg["segmentation"]["num_classes"]),
        )

    def forward(self, x: torch.Tensor, postprocess: bool = True) -> dict[str, Any]:
        backbone_feats = self.backbone(x)
        pyramid_feats = self.fpn(backbone_feats)
        detection_logits = self.detection_head(pyramid_feats)
        segmentation_logits = self.segmentation_head(
            pyramid_feats,
            output_size=(x.shape[-2], x.shape[-1]),
        )
        outputs: dict[str, Any] = {
            "detection": detection_logits,
            "segmentation": segmentation_logits,
        }
        if not postprocess:
            return outputs
        outputs["detections"] = decode_detections(
            level_outputs=detection_logits,
            image_size=(x.shape[-2], x.shape[-1]),
            score_threshold=float(self.cfg["detection"].get("score_threshold", 0.25)),
            nms_threshold=float(self.cfg["detection"].get("nms_threshold", 0.5)),
            max_detections=int(self.cfg["detection"].get("max_detections", 100)),
        )
        return outputs
