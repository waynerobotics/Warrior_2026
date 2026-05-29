from __future__ import annotations

import json
import logging
import random
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch
import yaml
from torch.optim import AdamW
from torch.optim.lr_scheduler import CosineAnnealingLR
from torch.utils.data import DataLoader

from data import MultitaskDataset, collate_fn, validate_manifest
from engine.eval import evaluate
from models import MultiTaskLoss, MultiTaskPerceptionModel

try:
    import mlflow
    import mlflow.pytorch
except ImportError:
    mlflow = None


LOGGER = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")


def resolve_device(device_name: str) -> torch.device:
    if device_name == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device_name == "cuda" and not torch.cuda.is_available():
        return torch.device("cpu")
    return torch.device(device_name)


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Expected a mapping in YAML file: {path}")
    return data


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True


def build_dataloader(manifest_path: str | Path, cfg: dict[str, Any], shuffle: bool) -> DataLoader:
    device = resolve_device(str(cfg.get("device", "auto")))
    dataset = MultitaskDataset(
        manifest_path=manifest_path,
        image_size=cfg["dataset"]["image_size"],
        image_root=cfg["dataset"].get("image_root"),
        seg_root=cfg["dataset"].get("seg_root"),
        normalize=bool(cfg["dataset"].get("normalize", True)),
        augment=cfg.get("augment") if shuffle else None,
    )
    return DataLoader(
        dataset,
        batch_size=int(cfg["batch_size"]),
        shuffle=shuffle,
        collate_fn=collate_fn,
        num_workers=int(cfg["dataset"].get("num_workers", 4)),
        pin_memory=device.type == "cuda",
    )


def save_checkpoint(
    model: torch.nn.Module,
    optimizer: torch.optim.Optimizer,
    scheduler: torch.optim.lr_scheduler.LRScheduler,
    epoch: int,
    metrics: dict[str, Any],
    cfg: dict[str, Any],
    run_id: str,
    best_combined_so_far: float,
) -> tuple[float, str]:
    checkpoint = {
        "epoch": epoch,
        "run_id": run_id,
        "backbone": cfg["backbone"]["name"],
        "state_dict": model.state_dict(),
        "optimizer": optimizer.state_dict(),
        "scheduler": scheduler.state_dict(),
        "metrics": metrics,
        "cfg": cfg,
    }
    path = Path(cfg.get("checkpoint_root", "checkpoints")) / str(cfg["backbone"]["name"]) / f"epoch_{epoch:03d}.pt"
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, path)
    combined = float(metrics["det_mAP"]) + float(metrics["seg_iou"])
    if combined > best_combined_so_far:
        best_path = path.parent / "best.pt"
        torch.save(checkpoint, best_path)
        return combined, str(best_path)
    return best_combined_so_far, str(path.parent / "best.pt")


def save_named_checkpoint(
    model: torch.nn.Module,
    optimizer: torch.optim.Optimizer,
    scheduler: torch.optim.lr_scheduler.LRScheduler,
    epoch: int,
    metrics: dict[str, Any],
    cfg: dict[str, Any],
    run_id: str,
    name: str,
) -> str:
    checkpoint = {
        "epoch": epoch,
        "run_id": run_id,
        "backbone": cfg["backbone"]["name"],
        "state_dict": model.state_dict(),
        "optimizer": optimizer.state_dict(),
        "scheduler": scheduler.state_dict(),
        "metrics": metrics,
        "cfg": cfg,
    }
    path = Path(cfg.get("checkpoint_root", "checkpoints")) / str(cfg["backbone"]["name"]) / name
    path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(checkpoint, path)
    return str(path)


@torch.no_grad()
def evaluate_loss(
    model: torch.nn.Module,
    criterion: torch.nn.Module,
    dataloader: DataLoader,
    device: torch.device,
    amp_enabled: bool,
) -> dict[str, float]:
    model.eval()
    det_running = seg_running = total_running = 0.0
    num_batches = 0
    for batch in dataloader:
        images = batch["image"].to(device)
        boxes = [tensor.to(device) for tensor in batch["boxes"]]
        labels = [tensor.to(device) for tensor in batch["labels"]]
        seg_mask = batch["seg_mask"].to(device)
        seg_confidence = batch["seg_confidence"].to(device)
        seg_roi_mask = batch["seg_roi_mask"].to(device)
        has_seg = batch["has_seg"].to(device)
        with torch.autocast(device_type=device.type, enabled=amp_enabled):
            preds = model(images, postprocess=False)
            losses = criterion(
                preds,
                {
                    "detection": (boxes, labels),
                    "segmentation": seg_mask,
                    "seg_confidence": seg_confidence,
                    "seg_roi_mask": seg_roi_mask,
                    "has_seg": has_seg,
                },
            )
        det_running += float(losses["det"].detach().item())
        seg_running += float(losses["seg"].detach().item())
        total_running += float(losses["total"].detach().item())
        num_batches += 1
    return {
        "val_loss_det": det_running / max(1, num_batches),
        "val_loss_seg": seg_running / max(1, num_batches),
        "val_loss_total": total_running / max(1, num_batches),
    }


def train(cfg: dict[str, Any], config_path: str | Path = "configs/multitask/multitask_resnet18.yaml") -> dict[str, Any]:
    seed = int(cfg.get("seed", 42))
    set_seed(seed)
    device = resolve_device(str(cfg.get("device", "auto")))
    if device.type == "cuda":
        torch.cuda.reset_peak_memory_stats(device)
    validation_reports = []
    if bool(cfg.get("validate_data", True)):
        for manifest_key in ("train_manifest", "val_manifest"):
            manifest = cfg["dataset"].get(manifest_key)
            if manifest:
                validation_reports.append(
                    validate_manifest(
                        manifest,
                        image_root=cfg["dataset"].get("image_root"),
                        seg_root=cfg["dataset"].get("seg_root"),
                        num_classes=int(cfg["detection"]["num_classes"]),
                    )
                )
    train_loader = build_dataloader(cfg["dataset"]["train_manifest"], cfg, shuffle=True)
    val_manifest = cfg["dataset"].get("val_manifest")
    val_loader = build_dataloader(val_manifest, cfg, shuffle=False) if val_manifest else None

    model = MultiTaskPerceptionModel(cfg).to(device)
    criterion = MultiTaskLoss(cfg)
    base_lr = float(cfg["optimizer"]["lr"])
    freeze_backbone_epochs = int(cfg.get("freeze_backbone_epochs", 0))
    backbone_lr_scale = float(cfg.get("backbone_lr_scale", 0.1))
    backbone_frozen = False

    if freeze_backbone_epochs > 0:
        model.backbone.requires_grad_(False)
        backbone_frozen = True
        LOGGER.info("backbone frozen for first %d epochs", freeze_backbone_epochs)

    optimizer = AdamW(
        [p for p in model.parameters() if p.requires_grad],
        lr=base_lr,
        weight_decay=1e-4,
    )
    total_steps = max(1, int(cfg["epochs"]) * len(train_loader))
    scheduler = CosineAnnealingLR(optimizer, T_max=total_steps)
    amp_cfg = cfg.get("mixed_precision", {})
    amp_requested = bool(amp_cfg.get("enabled", True))
    amp_enabled = amp_requested and device.type == "cuda"
    scaler = torch.cuda.amp.GradScaler(enabled=amp_enabled)
    grad_clip_norm = float(cfg.get("gradient_clip_norm", 10.0))
    early_cfg = cfg.get("early_stopping", {})
    early_enabled = bool(early_cfg.get("enabled", True)) and val_loader is not None
    early_patience = int(early_cfg.get("patience", 10))
    early_min_delta = float(early_cfg.get("min_delta", 0.0))
    log_every = int(cfg.get("log_every", 10))
    best_combined = float("-inf")
    best_val_loss = float("inf")
    best_epoch = 0
    best_loss_checkpoint = ""
    epochs_without_improvement = 0
    stopped_early = False
    completed_epoch = 0
    best_metrics = {"det_mAP": 0.0, "seg_iou": 0.0, "seg_dice": 0.0, "latency_ms": 0.0, "throughput_fps": 0.0}
    best_checkpoint = ""
    run_name = str(cfg.get("run_name") or cfg["backbone"]["name"])
    training_start = time.perf_counter()
    notes: list[str] = []
    if amp_requested and not amp_enabled:
        notes.append(f"mixed precision unavailable on device={device.type}")
    LOGGER.info(
        "seed python=%d numpy=%d torch=%d cuda=%s amp=%s grad_clip_norm=%.3f early_stopping=%s patience=%d",
        seed,
        seed,
        seed,
        seed if torch.cuda.is_available() else "unavailable",
        amp_enabled,
        grad_clip_norm,
        early_enabled,
        early_patience,
    )

    class _NullRun:
        class _Info:
            run_id = ""

        info = _Info()

        def __enter__(self) -> "_NullRun":
            return self

        def __exit__(self, exc_type: object, exc: object, tb: object) -> None:
            return None

    if mlflow is not None:
        mlflow.set_experiment(str(cfg.get("experiment_name", "multitask_perception")))
        run_context = mlflow.start_run(run_name=run_name)
    else:
        notes.append("mlflow unavailable")
        run_context = _NullRun()

    with run_context as run:
        if mlflow is not None:
            mlflow.log_params(
                {
                    "backbone": cfg["backbone"]["name"],
                    "fpn_channels": cfg["fpn"]["out_channels"],
                    "det_classes": cfg["detection"]["num_classes"],
                    "seg_classes": cfg["segmentation"]["num_classes"],
                    "segmentation_enabled": bool(cfg.get("segmentation", {}).get("enabled", True)),
                    "label_source": cfg.get("label_source", "unknown"),
                    "train_manifest": cfg["dataset"].get("train_manifest", ""),
                    "val_manifest": cfg["dataset"].get("val_manifest", ""),
                    "roi_mode": cfg.get("roi_mode", "none"),
                    "roi_mask": cfg.get("roi_mask", cfg["dataset"].get("seg_roi_mask", "")),
                    "loss_weighting": cfg.get("loss_weighting", "manual"),
                    "epochs": cfg["epochs"],
                    "batch_size": cfg["batch_size"],
                    "lr": cfg["optimizer"]["lr"],
                    "seed": cfg.get("seed", 42),
                    "mixed_precision": amp_enabled,
                    "gradient_clip_norm": grad_clip_norm,
                    "early_stopping_patience": early_patience,
                }
            )

        global_step = 0
        for epoch in range(int(cfg["epochs"])):
            completed_epoch = epoch + 1

            if backbone_frozen and epoch >= freeze_backbone_epochs:
                model.backbone.requires_grad_(True)
                backbone_frozen = False
                head_params = [p for name, p in model.named_parameters() if not name.startswith("backbone.")]
                optimizer = AdamW(
                    [
                        {"params": model.backbone.parameters(), "lr": base_lr * backbone_lr_scale},
                        {"params": head_params, "lr": base_lr},
                    ],
                    weight_decay=1e-4,
                )
                remaining_steps = max(1, (int(cfg["epochs"]) - epoch) * len(train_loader))
                scheduler = CosineAnnealingLR(optimizer, T_max=remaining_steps)
                LOGGER.info(
                    "backbone unfrozen at epoch %d — backbone_lr=%.2e head_lr=%.2e",
                    epoch + 1, base_lr * backbone_lr_scale, base_lr,
                )

            start_time = time.perf_counter()
            model.train()
            if backbone_frozen:
                model.backbone.eval()  # keep BN stats fixed during warmup
            det_running = seg_running = total_running = 0.0
            seg_batch_fraction = mean_seg_conf = mean_roi_fraction = 0.0
            num_batches = 0
            samples_seen = 0
            max_steps = int(cfg.get("max_steps_per_epoch", 0))

            for batch in train_loader:
                images = batch["image"].to(device)
                boxes = [tensor.to(device) for tensor in batch["boxes"]]
                labels = [tensor.to(device) for tensor in batch["labels"]]
                seg_mask = batch["seg_mask"].to(device)
                seg_confidence = batch["seg_confidence"].to(device)
                seg_roi_mask = batch["seg_roi_mask"].to(device)
                has_seg = batch["has_seg"].to(device)

                optimizer.zero_grad(set_to_none=True)
                with torch.autocast(device_type=device.type, enabled=amp_enabled):
                    preds = model(images, postprocess=False)
                    targets = {
                        "detection": (boxes, labels),
                        "segmentation": seg_mask,
                        "seg_confidence": seg_confidence,
                        "seg_roi_mask": seg_roi_mask,
                        "has_seg": has_seg,
                    }
                    losses = criterion(preds, targets)
                scaler.scale(losses["total"]).backward()
                scaler.unscale_(optimizer)
                torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=grad_clip_norm)
                scaler.step(optimizer)
                scaler.update()
                scheduler.step()

                det_value = float(losses["det"].detach().item())
                seg_value = float(losses["seg"].detach().item())
                total_value = float(losses["total"].detach().item())
                det_running += det_value
                seg_running += seg_value
                total_running += total_value
                seg_batch_fraction += float(has_seg.float().mean().item())
                mean_seg_conf += float(seg_confidence[has_seg].mean().item()) if has_seg.any() else 0.0
                mean_roi_fraction += float(seg_roi_mask[has_seg].mean().item()) if has_seg.any() else 0.0
                num_batches += 1
                samples_seen += int(images.shape[0])

                if global_step % log_every == 0:
                    LOGGER.info(
                        "epoch=%03d step=%05d loss_det=%.4f loss_seg=%.4f loss_total=%.4f seg_batch_frac=%.3f seg_conf=%.3f roi_frac=%.3f",
                        epoch + 1, global_step, det_value, seg_value, total_value,
                        float(has_seg.float().mean().item()),
                        float(seg_confidence[has_seg].mean().item()) if has_seg.any() else 0.0,
                        float(seg_roi_mask[has_seg].mean().item()) if has_seg.any() else 0.0,
                    )
                global_step += 1
                if max_steps and num_batches >= max_steps:
                    break

            avg_det_loss = det_running / max(1, num_batches)
            avg_seg_loss = seg_running / max(1, num_batches)
            avg_total_loss = total_running / max(1, num_batches)
            avg_seg_batch_fraction = seg_batch_fraction / max(1, num_batches)
            avg_seg_conf = mean_seg_conf / max(1, num_batches)
            avg_roi_fraction = mean_roi_fraction / max(1, num_batches)
            throughput_fps = samples_seen / max(1e-6, time.perf_counter() - start_time)

            if val_loader is not None:
                loss_metrics = evaluate_loss(model, criterion, val_loader, device, amp_enabled=amp_enabled)
                metrics = evaluate(model, val_loader, cfg, device, epoch + 1, Path("artifacts") / run_name)
                metrics.update(loss_metrics)
            else:
                metrics = {"det_mAP": 0.0, "det_mAP_50": 0.0, "seg_iou": 0.0, "per_class_iou": []}
                metrics["seg_dice"] = 0.0
                metrics["latency_ms"] = 0.0
                metrics["throughput_fps"] = 0.0
                metrics["val_loss_det"] = 0.0
                metrics["val_loss_seg"] = 0.0
                metrics["val_loss_total"] = avg_total_loss

            if mlflow is not None:
                mlflow_metrics: dict[str, float] = {
                    "train/loss_det": avg_det_loss,
                    "train/loss_seg": avg_seg_loss,
                    "train/loss_total": avg_total_loss,
                    "train/seg_batch_fraction": avg_seg_batch_fraction,
                    "train/mean_seg_confidence": avg_seg_conf,
                    "train/valid_roi_fraction": avg_roi_fraction,
                    "train/throughput_fps": throughput_fps,
                    "val/det_mAP": metrics["det_mAP"],
                    "val/det_mAP_50": metrics.get("det_mAP_50", 0.0),
                    "val/det_mAP_75": metrics.get("det_mAP_75", 0.0),
                    "val/seg_iou": metrics["seg_iou"],
                    "val/seg_dice": metrics["seg_dice"],
                    "val/loss_det": metrics["val_loss_det"],
                    "val/loss_seg": metrics["val_loss_seg"],
                    "val/loss_total": metrics["val_loss_total"],
                    "val/latency_ms": metrics["latency_ms"],
                    "val/throughput_fps": metrics["throughput_fps"],
                    "lr": optimizer.param_groups[0]["lr"],
                }
                class_names = cfg.get("detection", {}).get("class_names", [])
                for idx, ap in enumerate(metrics.get("per_class_det_ap", [])):
                    name = class_names[idx] if idx < len(class_names) else str(idx)
                    mlflow_metrics[f"val/det_ap_{name}"] = float(ap)
                mlflow.log_metrics(mlflow_metrics, step=epoch)
            epoch_record = {
                "epoch": epoch + 1,
                "train_loss_det": avg_det_loss,
                "train_loss_seg": avg_seg_loss,
                "train_loss_total": avg_total_loss,
                "train_throughput_fps": throughput_fps,
                "val_loss_det": metrics["val_loss_det"],
                "val_loss_seg": metrics["val_loss_seg"],
                "val_loss_total": metrics["val_loss_total"],
                "val_det_mAP": metrics["det_mAP"],
                "val_det_mAP_50": metrics.get("det_mAP_50", 0.0),
                "val_det_mAP_75": metrics.get("det_mAP_75", 0.0),
                "val_seg_iou": metrics["seg_iou"],
                "val_seg_dice": metrics["seg_dice"],
                "val_per_class_det_ap": metrics.get("per_class_det_ap", []),
                "lr": optimizer.param_groups[0]["lr"],
            }
            metrics_dir = Path(cfg.get("checkpoint_root", "checkpoints")) / str(cfg["backbone"]["name"])
            metrics_dir.mkdir(parents=True, exist_ok=True)
            metrics_jsonl = metrics_dir / "metrics.jsonl"
            with metrics_jsonl.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(epoch_record) + "\n")
            per_class_path = Path("artifacts") / run_name / f"epoch_{epoch + 1:03d}_per_class_iou.json"
            per_class_path.parent.mkdir(parents=True, exist_ok=True)
            per_class_path.write_text(
                json.dumps(
                    {
                        "epoch": epoch + 1,
                        "per_class_iou": metrics.get("per_class_iou", []),
                        "per_class_dice": metrics.get("per_class_dice", []),
                    },
                    indent=2,
                ),
                encoding="utf-8",
            )
            if mlflow is not None:
                mlflow.log_artifact(str(per_class_path))

            previous_best = best_combined
            best_combined, best_checkpoint = save_checkpoint(model, optimizer, scheduler, epoch + 1, metrics, cfg, run.info.run_id, best_combined)
            if best_combined > previous_best:
                best_epoch = epoch + 1
                best_metrics = {
                    "det_mAP": metrics["det_mAP"],
                    "seg_iou": metrics["seg_iou"],
                    "seg_dice": metrics["seg_dice"],
                    "latency_ms": metrics["latency_ms"],
                    "throughput_fps": metrics["throughput_fps"],
                }
            current_val_loss = float(metrics["val_loss_total"])
            if current_val_loss < best_val_loss - early_min_delta:
                best_val_loss = current_val_loss
                epochs_without_improvement = 0
                best_loss_checkpoint = save_named_checkpoint(
                    model,
                    optimizer,
                    scheduler,
                    epoch + 1,
                    metrics,
                    cfg,
                    run.info.run_id,
                    "best_by_val_loss.pt",
                )
            else:
                epochs_without_improvement += 1
                if early_enabled and epochs_without_improvement >= early_patience:
                    stopped_early = True
                    LOGGER.info(
                        "early stopping run=%s epoch=%d best_val_loss=%.6f patience=%d",
                        run_name,
                        epoch + 1,
                        best_val_loss,
                        early_patience,
                    )
                    break

        if mlflow is not None:
            try:
                mlflow.pytorch.log_model(model, artifact_path="model")
            except Exception as exc:
                notes.append(f"mlflow model artifact skipped: {exc}")
                LOGGER.warning("mlflow model artifact skipped for %s: %s", run_name, exc)
            if Path(config_path).exists():
                try:
                    mlflow.log_artifact(str(config_path))
                except Exception as exc:
                    notes.append(f"mlflow config artifact skipped: {exc}")
                    LOGGER.warning("mlflow config artifact skipped for %s: %s", run_name, exc)

    restored_checkpoint = ""
    if best_loss_checkpoint:
        checkpoint = torch.load(best_loss_checkpoint, map_location=device)
        model.load_state_dict(checkpoint["state_dict"])
        restored_checkpoint = best_loss_checkpoint
    final_checkpoint = save_named_checkpoint(
        model,
        optimizer,
        scheduler,
        completed_epoch,
        {"best_val_loss": best_val_loss, "restored_from": restored_checkpoint},
        cfg,
        "",
        "final.pt",
    )
    total_training_time_sec = time.perf_counter() - training_start
    peak_memory_mb = ""
    if device.type == "cuda":
        peak_memory_mb = torch.cuda.max_memory_allocated(device) / (1024.0 * 1024.0)

    summary = {
        "run_name": run_name,
        "backbone": cfg["backbone"]["name"],
        "segmentation_enabled": bool(cfg.get("segmentation", {}).get("enabled", True)),
        "best_val_map": "" if float(best_metrics["det_mAP"]) < 0.0 else best_metrics["det_mAP"],
        "best_val_seg_iou": best_metrics["seg_iou"],
        "best_val_seg_dice": best_metrics["seg_dice"],
        "best_val_loss": "" if best_val_loss == float("inf") else best_val_loss,
        "best_epoch": best_epoch,
        "checkpoint_path": best_checkpoint,
        "best_loss_checkpoint_path": best_loss_checkpoint,
        "final_checkpoint_path": final_checkpoint,
        "batch_size": int(cfg["batch_size"]),
        "latency_ms": best_metrics["latency_ms"],
        "throughput_fps": best_metrics["throughput_fps"],
        "total_training_time_sec": total_training_time_sec,
        "peak_memory_mb": peak_memory_mb,
        "stopped_early": stopped_early,
        "completed_epoch": completed_epoch,
        "seed": seed,
        "mixed_precision": amp_enabled,
        "data_validation": validation_reports,
        "notes": "; ".join(["det mAP unavailable"] + notes) if float(best_metrics["det_mAP"]) < 0.0 else "; ".join(notes),
    }
    return summary
