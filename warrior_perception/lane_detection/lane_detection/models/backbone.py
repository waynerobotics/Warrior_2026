from __future__ import annotations

from collections.abc import Callable
from typing import Any

import torch
from torch import nn
from torchvision import models


BACKBONE_REGISTRY: dict[str, Callable[[dict[str, Any]], nn.Module]] = {}


def register_backbone(name: str) -> Callable[[Callable[[dict[str, Any]], nn.Module]], Callable[[dict[str, Any]], nn.Module]]:
    def decorator(builder: Callable[[dict[str, Any]], nn.Module]) -> Callable[[dict[str, Any]], nn.Module]:
        BACKBONE_REGISTRY[name] = builder
        return builder

    return decorator


def build_backbone(cfg: dict[str, Any]) -> nn.Module:
    backbone_cfg = cfg["backbone"]
    backbone_name = str(backbone_cfg["name"])
    if backbone_name not in BACKBONE_REGISTRY:
        available = ", ".join(sorted(BACKBONE_REGISTRY))
        raise ValueError(f"Unsupported backbone '{backbone_name}'. Available: {available}")
    return BACKBONE_REGISTRY[backbone_name](cfg)


class ResNetBackbone(nn.Module):
    def __init__(self, base_model: nn.Module, in_channels: int, out_channels: dict[str, int]) -> None:
        super().__init__()
        if in_channels != 3:
            stem_width = 64
            base_model.conv1 = nn.Conv2d(
                in_channels,
                stem_width,
                kernel_size=7,
                stride=2,
                padding=3,
                bias=False,
            )

        self.stem = nn.Sequential(
            base_model.conv1,
            base_model.bn1,
            base_model.relu,
        )
        self.maxpool = base_model.maxpool
        self.layer1 = base_model.layer1
        self.layer2 = base_model.layer2
        self.layer3 = base_model.layer3
        self.layer4 = base_model.layer4
        self.out_channels = out_channels

    def forward(self, x: torch.Tensor) -> dict[str, torch.Tensor]:
        x = self.stem(x)
        x = self.maxpool(x)
        c2 = self.layer1(x)
        c3 = self.layer2(c2)
        c4 = self.layer3(c3)
        c5 = self.layer4(c4)
        return {"c2": c2, "c3": c3, "c4": c4, "c5": c5}


class ConvNeXtBaseBackbone(nn.Module):
    def __init__(self, pretrained: bool) -> None:
        super().__init__()
        weights = models.ConvNeXt_Base_Weights.DEFAULT if pretrained else None
        self.model = models.convnext_base(weights=weights)
        self.out_channels = {"c2": 128, "c3": 256, "c4": 512, "c5": 1024}

    def forward(self, x: torch.Tensor) -> dict[str, torch.Tensor]:
        features = self.model.features
        c2 = features[1](features[0](x))
        c3 = features[3](features[2](c2))
        c4 = features[5](features[4](c3))
        c5 = features[7](features[6](c4))
        return {"c2": c2, "c3": c3, "c4": c4, "c5": c5}


class SwinBBackbone(nn.Module):
    def __init__(self, pretrained: bool) -> None:
        super().__init__()
        weights = models.Swin_B_Weights.DEFAULT if pretrained else None
        self.model = models.swin_b(weights=weights)
        self.out_channels = {"c2": 128, "c3": 256, "c4": 512, "c5": 1024}

    @staticmethod
    def _to_nchw(x: torch.Tensor) -> torch.Tensor:
        return x.permute(0, 3, 1, 2).contiguous()

    def forward(self, x: torch.Tensor) -> dict[str, torch.Tensor]:
        features = self.model.features
        x0 = features[0](x)
        x1 = features[1](x0)
        x2 = features[2](x1)
        x3 = features[3](x2)
        x4 = features[4](x3)
        x5 = features[5](x4)
        x6 = features[6](x5)
        x7 = features[7](x6)
        return {
            "c2": self._to_nchw(x1),
            "c3": self._to_nchw(x3),
            "c4": self._to_nchw(x5),
            "c5": self._to_nchw(x7),
        }


class HRNetW32Backbone(nn.Module):
    """Lightweight HRNet-style fallback with the requested stage widths."""

    def __init__(self, in_channels: int) -> None:
        super().__init__()
        self.stem = nn.Sequential(
            nn.Conv2d(in_channels, 32, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
        )
        self.stage2 = nn.Sequential(
            nn.Conv2d(32, 32, kernel_size=3, stride=1, padding=1, bias=False),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
        )
        self.stage3 = nn.Sequential(
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
        )
        self.stage4 = nn.Sequential(
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
        )
        self.stage5 = nn.Sequential(
            nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1, bias=False),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
        )
        self.out_channels = {"c2": 32, "c3": 64, "c4": 128, "c5": 256}

    def forward(self, x: torch.Tensor) -> dict[str, torch.Tensor]:
        c2 = self.stage2(self.stem(x))
        c3 = self.stage3(c2)
        c4 = self.stage4(c3)
        c5 = self.stage5(c4)
        return {"c2": c2, "c3": c3, "c4": c4, "c5": c5}


@register_backbone("resnet18")
def build_resnet18(cfg: dict[str, Any]) -> nn.Module:
    backbone_cfg = cfg["backbone"]
    weights = models.ResNet18_Weights.DEFAULT if backbone_cfg.get("pretrained", False) else None
    base = models.resnet18(weights=weights)
    return ResNetBackbone(
        base_model=base,
        in_channels=int(backbone_cfg.get("in_channels", 3)),
        out_channels={"c2": 64, "c3": 128, "c4": 256, "c5": 512},
    )


@register_backbone("resnet50")
def build_resnet50(cfg: dict[str, Any]) -> nn.Module:
    backbone_cfg = cfg["backbone"]
    weights = models.ResNet50_Weights.DEFAULT if backbone_cfg.get("pretrained", False) else None
    base = models.resnet50(weights=weights)
    return ResNetBackbone(
        base_model=base,
        in_channels=int(backbone_cfg.get("in_channels", 3)),
        out_channels={"c2": 256, "c3": 512, "c4": 1024, "c5": 2048},
    )


@register_backbone("convnext_base")
def build_convnext_base(cfg: dict[str, Any]) -> nn.Module:
    return ConvNeXtBaseBackbone(pretrained=bool(cfg["backbone"].get("pretrained", False)))


@register_backbone("swin_b")
def build_swin_b(cfg: dict[str, Any]) -> nn.Module:
    return SwinBBackbone(pretrained=bool(cfg["backbone"].get("pretrained", False)))


@register_backbone("hrnet_w32")
def build_hrnet_w32(cfg: dict[str, Any]) -> nn.Module:
    return HRNetW32Backbone(in_channels=int(cfg["backbone"].get("in_channels", 3)))
