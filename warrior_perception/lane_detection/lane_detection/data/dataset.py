from __future__ import annotations

import json
import random
from pathlib import Path
from typing import Any

import numpy as np
import torch
from PIL import Image
from torch.utils.data import Dataset

from .collate import collate_fn
from .transforms import TF, build_albumentations_transform


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def _resolve_path(ref: str | Path, root: str | Path | None) -> Path:
    path = Path(ref)
    if path.is_absolute():
        return path
    return (Path(root) if root is not None else Path(".")) / path


def read_manifest_samples(manifest_path: str | Path) -> list[dict[str, Any]]:
    with Path(manifest_path).open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if isinstance(data, dict):
        data = data.get("samples", [])
    if not isinstance(data, list) or not data:
        raise ValueError(f"Expected non-empty sample list in {manifest_path}")
    return data


def validate_manifest(
    manifest_path: str | Path,
    image_root: str | Path | None = None,
    seg_root: str | Path | None = None,
    num_classes: int | None = None,
) -> dict[str, Any]:
    samples = read_manifest_samples(manifest_path)
    errors: list[str] = []
    warnings: list[str] = []
    seg_count = 0
    box_count = 0

    for index, sample in enumerate(samples):
        image_ref = sample.get("image")
        if not image_ref:
            errors.append(f"sample {index}: missing image")
            continue
        image_path = _resolve_path(str(image_ref), image_root)
        if image_path.suffix.lower() not in IMAGE_EXTENSIONS:
            warnings.append(f"sample {index}: unusual image extension {image_path.suffix}")
        try:
            with Image.open(image_path) as image:
                image.verify()
            with Image.open(image_path) as image:
                width, height = image.size
        except Exception as exc:
            errors.append(f"sample {index}: unreadable image {image_path}: {exc}")
            continue

        boxes = sample.get("boxes", [])
        labels = sample.get("labels", [])
        if boxes is None:
            boxes = []
        if labels is None:
            labels = []
        if not isinstance(boxes, list) or not isinstance(labels, list):
            errors.append(f"sample {index}: boxes and labels must be lists")
            continue
        if len(boxes) != len(labels):
            errors.append(f"sample {index}: boxes length {len(boxes)} != labels length {len(labels)}")
        for box_index, box in enumerate(boxes):
            if not isinstance(box, list) or len(box) != 4:
                errors.append(f"sample {index}: box {box_index} must contain four coordinates")
                continue
            x1, y1, x2, y2 = [float(value) for value in box]
            if not (0.0 <= x1 < x2 <= width and 0.0 <= y1 < y2 <= height):
                errors.append(f"sample {index}: box {box_index} outside image bounds {width}x{height}")
        for label_index, label in enumerate(labels):
            label_int = int(label)
            if label_int < 0 or (num_classes is not None and label_int >= num_classes):
                errors.append(f"sample {index}: label {label_index}={label_int} outside class range")
        box_count += len(boxes)

        seg_ref = sample.get("seg_mask")
        if seg_ref:
            seg_count += 1
            mask_path = _resolve_path(str(seg_ref), seg_root)
            try:
                with Image.open(mask_path) as mask:
                    mask.verify()
                with Image.open(mask_path) as mask:
                    if mask.size != (width, height):
                        warnings.append(f"sample {index}: seg mask size {mask.size} differs from image {(width, height)}")
            except Exception as exc:
                errors.append(f"sample {index}: unreadable seg_mask {mask_path}: {exc}")
        roi_ref = sample.get("seg_roi_mask")
        if roi_ref:
            roi_path = _resolve_path(str(roi_ref), seg_root)
            try:
                with Image.open(roi_path) as roi:
                    roi.verify()
            except Exception as exc:
                errors.append(f"sample {index}: unreadable seg_roi_mask {roi_path}: {exc}")

    report = {
        "manifest": str(manifest_path),
        "samples": len(samples),
        "segmentation_samples": seg_count,
        "boxes": box_count,
        "errors": errors,
        "warnings": warnings,
    }
    if errors:
        preview = "; ".join(errors[:5])
        raise ValueError(f"Manifest validation failed for {manifest_path}: {preview}")
    return report


class MultitaskDataset(Dataset[dict[str, torch.Tensor]]):
    def __init__(
        self,
        manifest_path: str | Path,
        image_size: tuple[int, int] | list[int],
        image_root: str | Path | None = None,
        seg_root: str | Path | None = None,
        normalize: bool = True,
        augment: dict[str, Any] | None = None,
    ) -> None:
        self.manifest_path = Path(manifest_path)
        self.image_size = (int(image_size[0]), int(image_size[1]))
        self.image_root = Path(image_root) if image_root is not None else self.manifest_path.parent
        self.seg_root = Path(seg_root) if seg_root is not None else self.manifest_path.parent
        self.normalize = normalize
        self.augment = augment or {}
        self.albumentations_transform = build_albumentations_transform(
            self.augment.get("albumentations", {}),
            self.image_size,
        )
        self.samples = self._load_manifest()

    def _load_manifest(self) -> list[dict[str, Any]]:
        return read_manifest_samples(self.manifest_path)

    def __len__(self) -> int:
        return len(self.samples)

    def __getitem__(self, index: int) -> dict[str, torch.Tensor]:
        sample = self.samples[index]
        image = self._load_image(sample["image"])
        boxes = torch.tensor(sample.get("boxes", []), dtype=torch.float32)
        labels = torch.tensor(sample.get("labels", []), dtype=torch.int64)
        has_seg = bool(sample.get("seg_mask"))

        orig_w, orig_h = image.size
        target_h, target_w = self.image_size
        image = image.resize((target_w, target_h), resample=Image.BILINEAR)
        boxes = self._resize_boxes(boxes, orig_w, orig_h, target_w, target_h)

        if has_seg:
            seg_mask = self._load_mask(sample["seg_mask"], self.seg_root).resize((target_w, target_h), resample=Image.NEAREST)
            seg_array = np.array(seg_mask, dtype=np.int64)
            if seg_array.max(initial=0) > 1:
                seg_array = (seg_array > 0).astype(np.int64)
            roi_array = self._load_roi_array(sample.get("seg_roi_mask"), target_h, target_w)
            seg_confidence = float(sample.get("seg_confidence", 1.0))
        else:
            seg_array = np.zeros((target_h, target_w), dtype=np.int64)
            roi_array = np.zeros((target_h, target_w), dtype=np.float32)
            seg_confidence = 0.0

        image_array = np.array(image, dtype=np.uint8)
        image_array, seg_array, roi_array, boxes, labels = self._maybe_albumentations(
            image_array=image_array,
            seg_mask=seg_array,
            roi_mask=roi_array,
            boxes=boxes,
            labels=labels,
        )

        image_tensor = TF.to_tensor(image_array)
        seg_mask_tensor = torch.from_numpy(seg_array.astype(np.int64))
        roi_tensor = torch.from_numpy(roi_array.astype(np.float32))

        image_tensor, seg_mask_tensor, roi_tensor, boxes = self._maybe_circular_shift(
            image_tensor, seg_mask_tensor, roi_tensor, boxes
        )

        if self.normalize:
            image_tensor = TF.normalize(
                image_tensor,
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225],
            )

        if boxes.numel() == 0:
            boxes = boxes.reshape(0, 4)
        return {
            "image": image_tensor,
            "boxes": boxes,
            "labels": labels,
            "seg_mask": seg_mask_tensor,
            "seg_confidence": torch.tensor(seg_confidence, dtype=torch.float32),
            "seg_roi_mask": roi_tensor,
            "has_seg": torch.tensor(has_seg, dtype=torch.bool),
        }

    def _load_image(self, image_ref: str) -> Image.Image:
        image_path = Path(image_ref)
        if not image_path.is_absolute():
            image_path = self.image_root / image_path
        return Image.open(image_path).convert("RGB")

    def _load_mask(self, mask_ref: str, root: Path) -> Image.Image:
        mask_path = Path(mask_ref)
        if not mask_path.is_absolute():
            mask_path = root / mask_path
        return Image.open(mask_path)

    def _load_roi_array(self, roi_ref: str | None, h: int, w: int) -> np.ndarray:
        if not roi_ref:
            return np.ones((h, w), dtype=np.float32)
        roi = self._load_mask(roi_ref, self.seg_root).resize((w, h), resample=Image.NEAREST)
        return (np.array(roi, dtype=np.uint8) > 0).astype(np.float32)

    def _maybe_albumentations(
        self,
        image_array: np.ndarray,
        seg_mask: np.ndarray,
        roi_mask: np.ndarray,
        boxes: torch.Tensor,
        labels: torch.Tensor,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, torch.Tensor, torch.Tensor]:
        if self.albumentations_transform is None:
            return image_array, seg_mask, roi_mask, boxes, labels

        box_list = boxes.detach().cpu().tolist() if boxes.numel() else []
        label_list = labels.detach().cpu().tolist() if labels.numel() else []
        transformed = self.albumentations_transform(
            image=image_array,
            mask=seg_mask.astype(np.uint8),
            roi_mask=(roi_mask > 0).astype(np.uint8),
            bboxes=box_list,
            class_labels=label_list,
        )
        aug_boxes = torch.tensor(transformed.get("bboxes", []), dtype=torch.float32)
        if aug_boxes.numel() == 0:
            aug_boxes = aug_boxes.reshape(0, 4)
        aug_labels = torch.tensor(transformed.get("class_labels", []), dtype=torch.int64)
        return (
            np.asarray(transformed["image"], dtype=np.uint8),
            (np.asarray(transformed["mask"], dtype=np.uint8) > 0).astype(np.int64),
            (np.asarray(transformed["roi_mask"], dtype=np.uint8) > 0).astype(np.float32),
            aug_boxes,
            aug_labels,
        )

    def _maybe_circular_shift(
        self,
        image: torch.Tensor,
        seg_mask: torch.Tensor,
        roi_mask: torch.Tensor,
        boxes: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        cfg = self.augment.get("circular_shift", {})
        if not cfg.get("enabled", False):
            return image, seg_mask, roi_mask, boxes
        width = image.shape[-1]
        max_fraction = float(cfg.get("max_fraction", 0.15))
        max_shift = max(1, int(width * max_fraction))
        shift = random.randint(-max_shift, max_shift)
        if shift == 0:
            return image, seg_mask, roi_mask, boxes
        image = torch.roll(image, shifts=shift, dims=-1)
        seg_mask = torch.roll(seg_mask, shifts=shift, dims=-1)
        roi_mask = torch.roll(roi_mask, shifts=shift, dims=-1)
        if boxes.numel() > 0 and bool(cfg.get("shift_boxes", False)):
            boxes = boxes.clone()
            boxes[:, [0, 2]] = (boxes[:, [0, 2]] + shift) % width
            keep = boxes[:, 0] < boxes[:, 2]
            boxes = boxes[keep]
        return image, seg_mask, roi_mask, boxes

    @staticmethod
    def _resize_boxes(boxes: torch.Tensor, orig_w: int, orig_h: int, target_w: int, target_h: int) -> torch.Tensor:
        if boxes.numel() == 0:
            return boxes.reshape(0, 4)
        scaled = boxes.clone()
        scaled[:, [0, 2]] *= target_w / float(orig_w)
        scaled[:, [1, 3]] *= target_h / float(orig_h)
        return scaled
