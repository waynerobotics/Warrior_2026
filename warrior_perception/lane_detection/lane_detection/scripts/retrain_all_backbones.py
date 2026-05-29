from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.train import load_yaml, train
from models.backbone import BACKBONE_REGISTRY


def timestamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def run(cmd: list[str]) -> None:
    print(" ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)


def write_empty_detection_annotations(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("[]\n", encoding="utf-8")


def build_updated_manifests(args: argparse.Namespace, run_dir: Path) -> tuple[Path, Path, Path]:
    labels_dir = run_dir / "manual_tape_labels"
    run(
        [
            sys.executable,
            "tools/equirect_lane_pipeline/labelme_tape_to_masks.py",
            "--annotations_dir",
            args.annotations_dir,
            "--image_root",
            args.image_root,
            "--output_dir",
            str(labels_dir),
            "--label",
            args.segmentation_label,
            "--recursive",
        ]
    )
    detection_path = Path(args.detection_annotations) if args.detection_annotations else run_dir / "empty_detection_annotations.json"
    if not args.detection_annotations:
        write_empty_detection_annotations(detection_path)

    manifests_dir = run_dir / "manifests"
    train_manifest = manifests_dir / "train_manifest.json"
    val_manifest = manifests_dir / "val_manifest.json"
    build_cmd = [
        sys.executable,
        "tools/equirect_lane_pipeline/build_multitask_manifest.py",
        "--images_dir",
        args.image_root,
        "--detection_annotations",
        str(detection_path),
        "--lane_records",
        str(labels_dir / "records.jsonl"),
        "--train_manifest",
        str(train_manifest),
        "--val_manifest",
        str(val_manifest),
        "--val_ratio",
        str(args.val_ratio),
        "--seed",
        str(args.seed),
        "--require_segmentation",
    ]
    if args.shared_roi_mask:
        build_cmd.extend(["--shared_roi_mask", args.shared_roi_mask])
    run(build_cmd)
    return train_manifest, val_manifest, labels_dir / "records.jsonl"


def configure_backbone(
    base_cfg: dict[str, Any],
    args: argparse.Namespace,
    run_dir: Path,
    backbone: str,
    train_manifest: Path,
    val_manifest: Path,
) -> dict[str, Any]:
    cfg = deepcopy(base_cfg)
    cfg["backbone"]["name"] = backbone
    cfg["backbone"]["pretrained"] = False
    cfg.setdefault("segmentation", {})["enabled"] = True
    cfg["dataset"]["train_manifest"] = str(train_manifest)
    cfg["dataset"]["val_manifest"] = str(val_manifest)
    cfg["dataset"]["image_root"] = "."
    cfg["dataset"]["seg_root"] = "."
    cfg["dataset"]["num_workers"] = args.num_workers
    cfg["epochs"] = args.epochs
    cfg["batch_size"] = args.batch_size
    cfg["device"] = args.device
    cfg["seed"] = args.seed
    cfg["validate_data"] = True
    cfg["gradient_clip_norm"] = args.gradient_clip_norm
    cfg["mixed_precision"] = {"enabled": True}
    cfg["early_stopping"] = {
        "enabled": True,
        "patience": args.early_stopping_patience,
        "min_delta": args.early_stopping_min_delta,
    }
    if args.max_steps_per_epoch:
        cfg["max_steps_per_epoch"] = args.max_steps_per_epoch
    cfg["run_name"] = f"{backbone}_scratch"
    cfg["experiment_name"] = "full_backbone_scratch_retrain"
    cfg["checkpoint_root"] = str(run_dir / "checkpoints" / cfg["run_name"])
    return cfg


def score_row(row: dict[str, Any]) -> float:
    return float(row.get("best_val_seg_iou") or 0.0) + float(row.get("best_val_map") or 0.0)


def write_comparison(rows: list[dict[str, Any]], output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    ranked = sorted(rows, key=score_row, reverse=True)
    for rank, row in enumerate(ranked, start=1):
        row["rank"] = rank
        row["best_validation_score"] = score_row(row)

    json_path = output_dir / "comparison.json"
    csv_path = output_dir / "comparison.csv"
    json_path.write_text(json.dumps(ranked, indent=2, default=str), encoding="utf-8")
    fields = [
        "rank",
        "run_name",
        "backbone",
        "segmentation_enabled",
        "best_validation_score",
        "best_val_map",
        "best_val_seg_iou",
        "best_val_seg_dice",
        "best_val_loss",
        "best_epoch",
        "completed_epoch",
        "checkpoint_path",
        "best_loss_checkpoint_path",
        "final_checkpoint_path",
        "batch_size",
        "latency_ms",
        "throughput_fps",
        "total_training_time_sec",
        "peak_memory_mb",
        "stopped_early",
        "seed",
        "mixed_precision",
        "notes",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in ranked:
            writer.writerow({field: row.get(field, "") for field in fields})
    print(f"comparison_csv={csv_path}", flush=True)
    return csv_path


def visualize_successes(args: argparse.Namespace, comparison_csv: Path, val_manifest: Path, run_dir: Path) -> None:
    visual_dir = run_dir / "visual_sanity"
    run(
        [
            sys.executable,
            "tools/equirect_lane_pipeline/visualize_backbone_heads.py",
            "--comparison_csv",
            str(comparison_csv),
            "--manifest",
            str(val_manifest),
            "--output_dir",
            str(visual_dir),
            "--config",
            args.config,
            "--device",
            args.device,
            "--num_samples",
            str(args.num_visual_samples),
            "--seed",
            str(args.seed),
            "--include_exploratory",
        ]
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Retrain every registered backbone from scratch on the updated dataset.")
    parser.add_argument("--config", default="configs/multitask/multitask_resnet18.yaml")
    parser.add_argument("--output_root", default="outputs/full_backbone_retrain")
    parser.add_argument("--annotations_dir", default="data")
    parser.add_argument("--image_root", default="data")
    parser.add_argument("--segmentation_label", default="lane")
    parser.add_argument("--detection_annotations", default="")
    parser.add_argument("--shared_roi_mask", default="")
    parser.add_argument("--val_ratio", type=float, default=0.2)
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch_size", type=int, default=4)
    parser.add_argument("--num_workers", type=int, default=4)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--early_stopping_patience", type=int, default=10)
    parser.add_argument("--early_stopping_min_delta", type=float, default=0.0)
    parser.add_argument("--gradient_clip_norm", type=float, default=10.0)
    parser.add_argument("--max_steps_per_epoch", type=int, default=0)
    parser.add_argument("--num_visual_samples", type=int, default=4)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_dir = Path(args.output_root) / timestamp()
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "args.json").write_text(json.dumps(vars(args), indent=2), encoding="utf-8")

    train_manifest, val_manifest, lane_records = build_updated_manifests(args, run_dir)
    base_cfg = load_yaml(args.config)
    rows: list[dict[str, Any]] = []
    backbones = sorted(BACKBONE_REGISTRY)
    print(f"backbones={backbones}", flush=True)
    print(f"train_manifest={train_manifest} val_manifest={val_manifest} lane_records={lane_records}", flush=True)

    for backbone in backbones:
        cfg = configure_backbone(base_cfg, args, run_dir, backbone, train_manifest, val_manifest)
        print(f"starting backbone={backbone}", flush=True)
        started = time.perf_counter()
        try:
            summary = train(cfg, config_path=args.config)
            summary.setdefault("total_training_time_sec", time.perf_counter() - started)
        except Exception as exc:
            summary = {
                "run_name": cfg["run_name"],
                "backbone": backbone,
                "segmentation_enabled": True,
                "best_val_map": "",
                "best_val_seg_iou": "",
                "best_val_seg_dice": "",
                "best_val_loss": "",
                "best_epoch": "",
                "completed_epoch": "",
                "checkpoint_path": "",
                "best_loss_checkpoint_path": "",
                "final_checkpoint_path": "",
                "batch_size": args.batch_size,
                "latency_ms": "",
                "throughput_fps": "",
                "total_training_time_sec": time.perf_counter() - started,
                "peak_memory_mb": "",
                "stopped_early": "",
                "seed": args.seed,
                "mixed_precision": "",
                "notes": f"failed: {exc}",
            }
            print(f"failed backbone={backbone} error={exc}", flush=True)
        rows.append(summary)
        (run_dir / "runs").mkdir(exist_ok=True)
        (run_dir / "runs" / f"{backbone}.json").write_text(json.dumps(summary, indent=2, default=str), encoding="utf-8")
        write_comparison(rows, run_dir)

    comparison_csv = write_comparison(rows, run_dir)
    if any(row.get("checkpoint_path") for row in rows):
        try:
            visualize_successes(args, comparison_csv, val_manifest, run_dir)
        except Exception as exc:
            (run_dir / "visual_sanity_failed.txt").write_text(str(exc), encoding="utf-8")
            print(f"visual_sanity_failed={exc}", flush=True)
    print(f"run_dir={run_dir}", flush=True)


if __name__ == "__main__":
    main()
