from __future__ import annotations

import torch
import torch.nn.functional as F
from torch import nn


class SegmentationHead(nn.Module):
    def __init__(self, in_channels: int, num_classes: int) -> None:
        super().__init__()
        fused_channels = in_channels * 3
        self.fuse = nn.Sequential(
            nn.Conv2d(fused_channels, in_channels, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(in_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(in_channels, in_channels // 2, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(in_channels // 2),
            nn.ReLU(inplace=True),
            nn.Conv2d(in_channels // 2, num_classes, kernel_size=1),
        )

    def forward(self, feats: dict[str, torch.Tensor], output_size: tuple[int, int]) -> torch.Tensor:
        p3, p4, p5 = feats["p3"], feats["p4"], feats["p5"]
        p4_up = F.interpolate(p4, size=p3.shape[-2:], mode="bilinear", align_corners=False)
        p5_up = F.interpolate(p5, size=p3.shape[-2:], mode="bilinear", align_corners=False)
        fused = torch.cat([p3, p4_up, p5_up], dim=1)
        logits = self.fuse(fused)
        return F.interpolate(logits, size=output_size, mode="bilinear", align_corners=False)
