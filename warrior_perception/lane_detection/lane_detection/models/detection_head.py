from __future__ import annotations

import torch
from torch import nn


class DetectionHead(nn.Module):
    def __init__(self, in_channels: int, num_classes: int) -> None:
        super().__init__()
        self.num_classes = num_classes
        self.pred_dim = 4 + 1 + num_classes
        self.stem = nn.Sequential(
            nn.Conv2d(in_channels, in_channels, kernel_size=3, padding=1, bias=False),
            nn.BatchNorm2d(in_channels),
            nn.ReLU(inplace=True),
        )
        self.head_p3 = nn.Conv2d(in_channels, self.pred_dim, kernel_size=1)
        self.head_p4 = nn.Conv2d(in_channels, self.pred_dim, kernel_size=1)
        self.head_p5 = nn.Conv2d(in_channels, self.pred_dim, kernel_size=1)

    def forward(self, feats: dict[str, torch.Tensor]) -> dict[str, torch.Tensor]:
        p3 = self.stem(feats["p3"])
        p4 = self.stem(feats["p4"])
        p5 = self.stem(feats["p5"])
        return {"p3": self.head_p3(p3), "p4": self.head_p4(p4), "p5": self.head_p5(p5)}
