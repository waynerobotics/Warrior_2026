"""
Compare trained ResNet-18 and ConvNeXt-Base checkpoints.

Reads per-epoch metrics from metrics.jsonl, loads each best.pt for a
latency benchmark, then writes comparison.json / comparison.csv /
recommendation.json and generates the Markdown report.

Usage:
  python scripts/compare_checkpoints.py
  python scripts/compare_checkpoints.py --sweep_dir outputs/backbone_sweep
  python scripts/compare_checkpoints.py --no_plots
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path
from typing import Any

import torch
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from models.multitask_model import MultiTaskPerceptionModel

BACKBONES = [
    {
        "name": "resnet18",
        "config": "configs/multitask/multitask_resnet18.yaml",
        "note": "Fast CNN baseline, ImageNet pretrained",
    },
    {
        "name": "convnext_base",
        "config": "configs/multitask/multitask_convnext.yaml",
        "note": "High-accuracy CNN, ImageNet pretrained",
    },
]

COMPARISON_COLS = [
    "backbone", "best_val_map", "best_val_map_50", "best_val_seg_iou",
    "best_val_seg_dice", "best_epoch", "completed_epoch",
    "latency_ms", "throughput_fps", "peak_memory_mb",
    "total_training_time_sec", "stopped_early", "note", "status",
]

N_WARMUP = 5
N_MEASURE = 20


def load_jsonl(path: Path) -> list[dict]:
    if not path.exists():
        return []
    records = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return records


def load_cfg(config_path: Path) -> dict:
    with open(config_path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def benchmark_latency(
    model: torch.nn.Module,
    cfg: dict,
    device: torch.device,
) -> tuple[float, float]:
    h, w = cfg["dataset"]["image_size"]
    dummy = torch.zeros(1, 3, h, w, device=device)
    model.eval()
    with torch.no_grad():
        for _ in range(N_WARMUP):
            model(dummy, postprocess=False)
        if device.type == "cuda":
            torch.cuda.synchronize()
        t0 = time.perf_counter()
        for _ in range(N_MEASURE):
            model(dummy, postprocess=False)
        if device.type == "cuda":
            torch.cuda.synchronize()
        elapsed = time.perf_counter() - t0
    return (elapsed / N_MEASURE * 1000.0), (N_MEASURE / elapsed)


def build_entry(
    backbone_name: str,
    sweep_dir: Path,
    cfg: dict,
    device: torch.device,
    note: str,
) -> dict[str, Any]:
    ckpt_dir = sweep_dir / "checkpoints" / backbone_name
    epochs = sorted(
        load_jsonl(ckpt_dir / "metrics.jsonl"),
        key=lambda r: int(r.get("epoch", 0)),
    )

    if not epochs:
        print(f"  WARNING: no metrics found for {backbone_name}")
        return {"backbone": backbone_name, "status": "no_metrics", "note": note}

    best = max(epochs, key=lambda r: float(r.get("val_det_mAP", 0.0)))
    last = epochs[-1]

    latency_ms, throughput_fps, peak_memory_mb = 0.0, 0.0, 0.0
    best_ckpt = ckpt_dir / "best.pt"
    if best_ckpt.exists():
        print(f"  Loading checkpoint for latency benchmark: {best_ckpt.name}")
        model = MultiTaskPerceptionModel(cfg).to(device)
        raw = torch.load(str(best_ckpt), map_location=device, weights_only=True)
        state = raw.get("state_dict", raw)
        model.load_state_dict(state)

        if device.type == "cuda":
            torch.cuda.reset_peak_memory_stats(device)

        latency_ms, throughput_fps = benchmark_latency(model, cfg, device)

        if device.type == "cuda":
            peak_memory_mb = torch.cuda.max_memory_allocated(device) / 1e6

        del model
        if device.type == "cuda":
            torch.cuda.empty_cache()

        print(f"  latency={latency_ms:.1f} ms  fps={throughput_fps:.2f}")
    else:
        print(f"  WARNING: checkpoint not found at {best_ckpt}")

    planned_epochs = int(cfg.get("epochs", 50))
    status = "ok" if last["epoch"] >= planned_epochs - 1 else "partial"

    return {
        "backbone": backbone_name,
        "best_val_map": best.get("val_det_mAP", 0.0),
        "best_val_map_50": best.get("val_det_mAP_50", 0.0),
        "best_val_map_75": best.get("val_det_mAP_75", 0.0),
        "best_val_seg_iou": best.get("val_seg_iou", 0.0),
        "best_val_seg_dice": best.get("val_seg_dice", 0.0),
        "best_epoch": best["epoch"],
        "completed_epoch": last["epoch"],
        "latency_ms": latency_ms,
        "throughput_fps": throughput_fps,
        "peak_memory_mb": peak_memory_mb,
        "total_training_time_sec": None,
        "stopped_early": status == "partial",
        "note": note,
        "status": status,
    }


def make_recommendation(entries: list[dict]) -> dict:
    valid = [e for e in entries if e.get("status") in ("ok", "partial") and e.get("latency_ms", 0) > 0]
    if not valid:
        return {}

    def score(e: dict) -> float:
        return float(e.get("best_val_map", 0.0)) + float(e.get("best_val_seg_iou", 0.0))

    best_overall = max(valid, key=score)["backbone"]
    fastest = min(valid, key=lambda e: float(e.get("latency_ms", 1e9)))["backbone"]

    notes = [
        f"{e['backbone']} only trained {e['completed_epoch']} / {50} epochs "
        f"— metrics may improve with full training"
        for e in valid
        if int(e.get("completed_epoch", 50)) < 45
    ]
    return {"best_overall": best_overall, "fastest": fastest, "notes": notes}


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare ResNet-18 and ConvNeXt-Base checkpoints.")
    parser.add_argument("--sweep_dir", default="outputs/backbone_sweep")
    parser.add_argument("--device", default="auto")
    parser.add_argument("--no_plots", action="store_true")
    args = parser.parse_args()

    sweep_dir = Path(args.sweep_dir)
    if not sweep_dir.exists():
        print(f"[ERROR] sweep_dir not found: {sweep_dir}")
        sys.exit(1)

    if args.device == "auto":
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    else:
        device = torch.device(args.device)
    print(f"Device: {device}")

    if device.type == "cuda":
        print("Warming up CUDA...")
        _w = torch.zeros(1, 3, 64, 64, device=device)
        for _ in range(10):
            torch.nn.functional.relu(_w)
        torch.cuda.synchronize()
        del _w
    print()

    entries: list[dict] = []
    for bb in BACKBONES:
        print(f"=== {bb['name']} ===")
        cfg = load_cfg(ROOT / bb["config"])
        entry = build_entry(bb["name"], sweep_dir, cfg, device, bb["note"])
        entries.append(entry)
        print(
            f"  best_mAP={entry.get('best_val_map', 0):.4f}  "
            f"mAP@50={entry.get('best_val_map_50', 0):.4f}  "
            f"seg_iou={entry.get('best_val_seg_iou', 0):.4f}  "
            f"best_epoch={entry.get('best_epoch', '—')}\n"
        )

    (sweep_dir / "comparison.json").write_text(json.dumps(entries, indent=2), encoding="utf-8")

    with open(sweep_dir / "comparison.csv", "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=COMPARISON_COLS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(entries)

    rec = make_recommendation(entries)
    (sweep_dir / "recommendation.json").write_text(json.dumps(rec, indent=2), encoding="utf-8")

    print(f"Written: comparison.json, comparison.csv, recommendation.json -> {sweep_dir}")
    print(f"Best overall: {rec.get('best_overall', '-')}  |  Fastest: {rec.get('fastest', '-')}")

    print("\nGenerating report...")
    from scripts.generate_report import generate_report
    report_md = generate_report(sweep_dir, no_plots=args.no_plots)
    report_path = sweep_dir / "report" / "report.md"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(report_md, encoding="utf-8")
    print(f"Report written -> {report_path}")


if __name__ == "__main__":
    main()
