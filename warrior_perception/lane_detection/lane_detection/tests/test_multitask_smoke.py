from __future__ import annotations

import json
from pathlib import Path

import torch
from PIL import Image

from data import MultitaskDataset, collate_fn
from engine.eval import compute_seg_iou
from engine.train import load_yaml
from models import MultiTaskLoss, MultiTaskPerceptionModel
from models.segmentation_roi import segmentation_argmax_in_roi


def _small_cfg() -> dict:
    return {
        "backbone": {"name": "resnet18", "in_channels": 3, "pretrained": False},
        "fpn": {"out_channels": 32},
        "detection": {"enabled": True, "num_classes": 3, "score_threshold": 0.25, "nms_threshold": 0.5},
        "segmentation": {"enabled": True, "num_classes": 2, "loss": "ce", "ignore_index": -100},
        "optimizer": {"lr": 1e-4},
        "dataset": {"image_size": [64, 64], "normalize": True, "num_workers": 0},
        "loss_weights": {"detection": 1.0, "segmentation": 1.0},
        "epochs": 1,
        "batch_size": 2,
        "device": "cpu",
    }


def _write_sample_manifest(tmp_path: Path) -> Path:
    image_path = tmp_path / "image.jpg"
    mask_path = tmp_path / "mask.png"
    roi_path = tmp_path / "roi.png"
    Image.new("RGB", (80, 80), color=(32, 64, 96)).save(image_path)
    Image.new("L", (80, 80), color=1).save(mask_path)
    Image.new("L", (80, 80), color=255).save(roi_path)
    manifest = [
        {
            "image": str(image_path),
            "boxes": [[8, 8, 32, 32]],
            "labels": [1],
            "seg_mask": str(mask_path),
            "seg_confidence": 0.93,
            "seg_roi_mask": str(roi_path),
        }
    ]
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
    return manifest_path


def test_config_loads() -> None:
    cfg = load_yaml("configs/multitask/multitask_resnet18.yaml")
    assert cfg["backbone"]["name"] == "resnet18"
    assert "train_manifest" in cfg["dataset"]


def test_dataset_manifest_loading(tmp_path: Path) -> None:
    dataset = MultitaskDataset(_write_sample_manifest(tmp_path), image_size=[64, 64])
    sample = dataset[0]
    assert sample["image"].shape == (3, 64, 64)
    assert sample["boxes"].shape == (1, 4)
    assert sample["seg_mask"].shape == (64, 64)
    assert sample["has_seg"].item() is True


def test_dataset_albumentations_keeps_segmentation_aligned(tmp_path: Path) -> None:
    augment = {
        "albumentations": {
            "enabled": True,
            "horizontal_flip_p": 1.0,
        }
    }
    dataset = MultitaskDataset(_write_sample_manifest(tmp_path), image_size=[64, 64], augment=augment)
    sample = dataset[0]
    assert sample["image"].shape == (3, 64, 64)
    assert sample["seg_mask"].shape == (64, 64)
    assert sample["seg_roi_mask"].shape == (64, 64)
    assert sample["boxes"].shape == (1, 4)
    assert sample["labels"].tolist() == [1]
    assert sample["seg_mask"].max().item() == 1
    assert sample["seg_roi_mask"].max().item() == 1


def test_model_forward_and_loss() -> None:
    cfg = _small_cfg()
    model = MultiTaskPerceptionModel(cfg)
    model.eval()
    images = torch.randn(2, 3, 64, 64)
    preds = model(images, postprocess=False)
    assert set(preds["detection"]) == {"p3", "p4", "p5"}
    assert preds["segmentation"].shape == (2, 2, 64, 64)

    criterion = MultiTaskLoss(cfg)
    targets = {
        "detection": (
            [torch.tensor([[8.0, 8.0, 32.0, 32.0]]), torch.empty((0, 4))],
            [torch.tensor([1]), torch.empty((0,), dtype=torch.long)],
        ),
        "segmentation": torch.zeros((2, 64, 64), dtype=torch.long),
        "seg_confidence": torch.ones((2,), dtype=torch.float32),
        "seg_roi_mask": torch.ones((2, 64, 64), dtype=torch.float32),
        "has_seg": torch.ones((2,), dtype=torch.bool),
    }
    losses = criterion(preds, targets)
    assert torch.isfinite(losses["total"])


def test_segmentation_predictions_are_background_outside_roi() -> None:
    logits = torch.zeros((1, 2, 4, 4), dtype=torch.float32)
    logits[:, 1] = 10.0
    roi = torch.zeros((1, 4, 4), dtype=torch.float32)
    roi[:, 2:, :] = 1.0
    pred = segmentation_argmax_in_roi(logits, roi)
    assert pred[:, :2, :].sum().item() == 0
    assert pred[:, 2:, :].sum().item() == 8


def test_segmentation_iou_ignores_predictions_outside_roi() -> None:
    logits = torch.zeros((1, 2, 4, 4), dtype=torch.float32)
    logits[:, 1] = 10.0
    gt = torch.zeros((1, 4, 4), dtype=torch.long)
    gt[:, 2:, :] = 1
    roi = torch.zeros((1, 4, 4), dtype=torch.float32)
    roi[:, 2:, :] = 1.0
    score = compute_seg_iou(logits, gt, num_classes=2, roi_mask=roi, has_seg=torch.ones(1, dtype=torch.bool))
    assert score["per_class_iou"][1] == 1.0


def test_segmentation_class_weights_change_loss() -> None:
    cfg = _small_cfg()
    weighted_cfg = _small_cfg()
    weighted_cfg["segmentation"]["class_weights"] = [1.0, 3.0]
    logits = torch.zeros((1, 2, 4, 4), dtype=torch.float32)
    logits[:, 0] = 1.0
    targets = {
        "detection": ([torch.empty((0, 4))], [torch.empty((0,), dtype=torch.long)]),
        "segmentation": torch.tensor(
            [[[0, 0, 0, 0], [0, 0, 0, 0], [1, 1, 1, 1], [1, 1, 1, 1]]],
            dtype=torch.long,
        ),
        "seg_confidence": torch.ones((1,), dtype=torch.float32),
        "seg_roi_mask": torch.ones((1, 4, 4), dtype=torch.float32),
        "has_seg": torch.ones((1,), dtype=torch.bool),
    }
    preds = {
        "detection": {
            "p3": torch.zeros((1, 8, 4, 4), dtype=torch.float32),
            "p4": torch.zeros((1, 8, 2, 2), dtype=torch.float32),
            "p5": torch.zeros((1, 8, 1, 1), dtype=torch.float32),
        },
        "segmentation": logits,
    }
    plain = MultiTaskLoss(cfg)(preds, targets)["seg"]
    weighted = MultiTaskLoss(weighted_cfg)(preds, targets)["seg"]
    assert torch.isfinite(weighted)
    assert weighted > plain


def test_one_training_step(tmp_path: Path) -> None:
    cfg = _small_cfg()
    augment = {
        "albumentations": {
            "enabled": True,
            "horizontal_flip_p": 0.5,
        }
    }
    dataset = MultitaskDataset(_write_sample_manifest(tmp_path), image_size=[64, 64], augment=augment)
    batch = collate_fn([dataset[0], dataset[0]])
    model = MultiTaskPerceptionModel(cfg)
    criterion = MultiTaskLoss(cfg)
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4)

    preds = model(batch["image"], postprocess=False)
    targets = {
        "detection": (batch["boxes"], batch["labels"]),
        "segmentation": batch["seg_mask"],
        "seg_confidence": batch["seg_confidence"],
        "seg_roi_mask": batch["seg_roi_mask"],
        "has_seg": batch["has_seg"],
    }
    loss = criterion(preds, targets)["total"]
    loss.backward()
    optimizer.step()
    assert torch.isfinite(loss.detach())
