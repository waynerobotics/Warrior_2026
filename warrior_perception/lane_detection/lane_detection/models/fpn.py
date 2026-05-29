from __future__ import annotations

import torch
import torch.nn.functional as F
from torch import nn


class FPN(nn.Module):
    def __init__(self, in_channels: dict[str, int], out_channels: int = 256) -> None:
        super().__init__()
        self.lateral_c3 = nn.Conv2d(in_channels["c3"], out_channels, kernel_size=1)
        self.lateral_c4 = nn.Conv2d(in_channels["c4"], out_channels, kernel_size=1)
        self.lateral_c5 = nn.Conv2d(in_channels["c5"], out_channels, kernel_size=1)
        self.out_p3 = nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1)
        self.out_p4 = nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1)
        self.out_p5 = nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1)
        self.out_channels = {"p3": out_channels, "p4": out_channels, "p5": out_channels}

    def forward(self, feats: dict[str, torch.Tensor]) -> dict[str, torch.Tensor]:
        c3, c4, c5 = feats["c3"], feats["c4"], feats["c5"]
        p5 = self.lateral_c5(c5)
        p4 = self.lateral_c4(c4) + F.interpolate(p5, size=c4.shape[-2:], mode="nearest")
        p3 = self.lateral_c3(c3) + F.interpolate(p4, size=c3.shape[-2:], mode="nearest")
        p5 = self.out_p5(p5)
        p4 = self.out_p4(p4)
        p3 = self.out_p3(p3)
        return {"p3": p3, "p4": p4, "p5": p5}
