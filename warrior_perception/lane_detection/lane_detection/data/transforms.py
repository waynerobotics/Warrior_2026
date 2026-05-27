from __future__ import annotations

import os
import sys
from typing import Any

from torchvision.transforms import functional as TF


def build_albumentations_transform(cfg: dict[str, Any], image_size: tuple[int, int]):
    """Build paired image/mask/box augmentations for multitask training."""
    enabled = bool(cfg.get("enabled", False))
    if not enabled:
        return None

    try:
        os.environ.setdefault("NO_ALBUMENTATIONS_UPDATE", "1")
        if sys.platform == "win32":
            os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
        import albumentations as A
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError(
            "Albumentations augmentation is enabled, but albumentations is not installed. "
            "Install it with `pip install albumentations` or disable augment.albumentations.enabled."
        ) from exc

    transforms = []
    horizontal_flip_p = float(cfg.get("horizontal_flip_p", 0.0))
    if horizontal_flip_p > 0:
        transforms.append(A.HorizontalFlip(p=horizontal_flip_p))

    affine_cfg = cfg.get("affine", {})
    if affine_cfg.get("enabled", False):
        scale_limit = float(affine_cfg.get("scale_limit", 0.05))
        translate_percent = float(affine_cfg.get("translate_percent", 0.03))
        rotate_limit = float(affine_cfg.get("rotate_limit", 2.0))
        transforms.append(
            A.Affine(
                scale=(1.0 - scale_limit, 1.0 + scale_limit),
                translate_percent=(-translate_percent, translate_percent),
                rotate=(-rotate_limit, rotate_limit),
                fit_output=False,
                keep_ratio=True,
                p=float(affine_cfg.get("p", 0.35)),
            )
        )

    brightness_contrast = cfg.get("brightness_contrast", {})
    if brightness_contrast.get("enabled", False):
        transforms.append(
            A.RandomBrightnessContrast(
                brightness_limit=float(brightness_contrast.get("brightness_limit", 0.15)),
                contrast_limit=float(brightness_contrast.get("contrast_limit", 0.15)),
                p=float(brightness_contrast.get("p", 0.4)),
            )
        )

    hue_saturation = cfg.get("hue_saturation", {})
    if hue_saturation.get("enabled", False):
        transforms.append(
            A.HueSaturationValue(
                hue_shift_limit=int(hue_saturation.get("hue_shift_limit", 5)),
                sat_shift_limit=int(hue_saturation.get("sat_shift_limit", 10)),
                val_shift_limit=int(hue_saturation.get("val_shift_limit", 8)),
                p=float(hue_saturation.get("p", 0.2)),
            )
        )

    blur = cfg.get("blur", {})
    if blur.get("enabled", False):
        transforms.append(
            A.OneOf(
                [
                    A.MotionBlur(blur_limit=int(blur.get("blur_limit", 3)), p=1.0),
                    A.GaussianBlur(blur_limit=int(blur.get("blur_limit", 3)), p=1.0),
                ],
                p=float(blur.get("p", 0.15)),
            )
        )

    if not transforms:
        return None

    return A.Compose(
        transforms,
        bbox_params=A.BboxParams(format="pascal_voc", label_fields=["class_labels"], min_visibility=0.05),
        additional_targets={"roi_mask": "mask"},
    )


__all__ = ["TF", "build_albumentations_transform"]
