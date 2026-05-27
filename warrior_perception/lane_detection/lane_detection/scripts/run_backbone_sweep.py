"""
Professional backbone sweep — trains all four backbones with both detection and
segmentation heads on the new 6-class dataset and produces a ranked comparison.

Backbones:
  resnet18      fast baseline, ImageNet pretrained
  convnext_base strong CNN, ImageNet pretrained
  swin_b        Vision Transformer, ImageNet pretrained
  hrnet_w32     lightweight custom backbone (no pretrained — more epochs)

Outputs (under --output_dir):
  checkpoints/<backbone>/best.pt         best model by det_mAP + seg_iou
  checkpoints/<backbone>/final.pt        weights after last / early-stop epoch
  checkpoints/<backbone>/metrics.jsonl   per-epoch metric stream
  runs/<backbone>.json                   per-run summary
  comparison.json / comparison.csv       ranked backbone comparison
  recommendation.json                    textual recommendation

Usage (from project root):
  python scripts/run_backbone_sweep.py
  python scripts/run_backbone_sweep.py --epochs 30 --output_dir outputs/sweep_v1
  python scripts/run_backbone_sweep.py --dry_run       # validate configs only
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine.train import load_yaml, train

try:
    import mlflow
except ImportError:
    mlflow = None


# ---------------------------------------------------------------------------
# Backbone sweep definition
# ---------------------------------------------------------------------------

BACKBONE_SPECS: list[dict[str, Any]] = [
    {
        "backbone": "resnet18",
        "config": "configs/multitask/multitask_resnet18.yaml",
        "note": "Fast CNN baseline, ImageNet pretrained",
    },
    {
        "backbone": "convnext_base",
        "config": "configs/multitask/multitask_convnext.yaml",
        "note": "Strong CNN, ImageNet pretrained",
    },
    {
        "backbone": "swin_b",
        "config": "configs/multitask/multitask_swinb.yaml",
        "note": "Vision Transformer, ImageNet pretrained",
    },
    {
        "backbone": "hrnet_w32",
        "config": "configs/multitask/multitask_hrnet.yaml",
        "note": "Lightweight custom backbone, trained from scratch",
    },
]

COMPARISON_FIELDS = [
    "backbone",
    "best_val_map",
    "best_val_map_50",
    "best_val_seg_iou",
    "best_val_seg_dice",
    "best_epoch",
    "completed_epoch",
    "latency_ms",
    "throughput_fps",
    "peak_memory_mb",
    "total_training_time_sec",
    "stopped_early",
    "note",
    "status",
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def apply_overrides(cfg: dict[str, Any], args: argparse.Namespace, spec: dict[str, Any], output_dir: Path) -> None:
    run_name = spec["backbone"]
    cfg["run_name"] = run_name
    cfg["experiment_name"] = "backbone_sweep_v1"
    cfg["checkpoint_root"] = str(output_dir / "checkpoints")
    if args.train_manifest:
        cfg["dataset"]["train_manifest"] = args.train_manifest
    if args.val_manifest:
        cfg["dataset"]["val_manifest"] = args.val_manifest
    if args.epochs is not None:
        cfg["epochs"] = args.epochs
    if args.batch_size is not None:
        cfg["batch_size"] = args.batch_size
    if args.num_workers is not None:
        cfg["dataset"]["num_workers"] = args.num_workers
    if args.device:
        cfg["device"] = args.device
    if args.seed is not None:
        cfg["seed"] = args.seed


def write_comparison(rows: list[dict[str, Any]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "comparison.json"
    csv_path = output_dir / "comparison.csv"
    json_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=COMPARISON_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({f: row.get(f, "") for f in COMPARISON_FIELDS})
    print(f"\n[sweep] Comparison written -> {csv_path}")


def write_recommendation(rows: list[dict[str, Any]], output_dir: Path) -> None:
    completed = [r for r in rows if r.get("status") == "ok"]
    if not completed:
        return

    def _score(r: dict[str, Any]) -> tuple[float, float, float]:
        return (
            float(r.get("best_val_map") or 0.0),
            float(r.get("best_val_seg_iou") or 0.0),
            -float(r.get("latency_ms") or 1e9),
        )

    best = max(completed, key=_score)
    fastest = min(completed, key=lambda r: float(r.get("latency_ms") or 1e9))

    rec = {
        "best_overall": best["backbone"],
        "best_overall_score": f"mAP={best.get('best_val_map', 'N/A'):.3f}  "
                              f"seg_IoU={best.get('best_val_seg_iou', 'N/A'):.3f}",
        "fastest": fastest["backbone"],
        "fastest_latency_ms": fastest.get("latency_ms", "N/A"),
        "notes": [
            "hrnet_w32 is a lightweight custom backbone — not the full HRNet paper architecture.",
            "Treat small score differences (<0.01 mAP) as measurement noise.",
            "Run pseudo_label.py with the best checkpoint to expand the training set.",
        ],
    }
    (output_dir / "recommendation.json").write_text(json.dumps(rec, indent=2), encoding="utf-8")
    print(f"[sweep] Best backbone: {best['backbone']}  "
          f"mAP={best.get('best_val_map', 'N/A')}  "
          f"seg_IoU={best.get('best_val_seg_iou', 'N/A')}")
    print(f"[sweep] Fastest:       {fastest['backbone']}  "
          f"latency={fastest.get('latency_ms', 'N/A')} ms/sample")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Backbone sweep: train all 4 backbones with det+seg.")
    parser.add_argument("--output_dir", default="outputs/backbone_sweep")
    parser.add_argument("--train_manifest", default="")
    parser.add_argument("--val_manifest", default="")
    parser.add_argument("--epochs", type=int, default=None)
    parser.add_argument("--batch_size", type=int, default=None)
    parser.add_argument("--num_workers", type=int, default=None)
    parser.add_argument("--device", default="")
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--skip_backbones", nargs="*", default=[],
                        help="Backbone names to skip, e.g. --skip_backbones swin_b hrnet_w32")
    parser.add_argument("--only_backbones", nargs="*", default=[],
                        help="Run only these backbones (overrides --skip_backbones)")
    parser.add_argument("--dry_run", action="store_true",
                        help="Load configs and validate without training")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    specs = BACKBONE_SPECS
    if args.only_backbones:
        specs = [s for s in specs if s["backbone"] in args.only_backbones]
    elif args.skip_backbones:
        specs = [s for s in specs if s["backbone"] not in args.skip_backbones]

    print(f"\n{'='*60}")
    print(f"  Backbone sweep - {len(specs)} model(s)")
    print(f"  Output: {output_dir}")
    print(f"{'='*60}\n")

    summaries: list[dict[str, Any]] = []
    sweep_start = time.perf_counter()

    for spec in specs:
        backbone = spec["backbone"]
        config_path = ROOT / spec["config"]
        print(f"\n{'-'*60}")
        print(f"  [{backbone}]  {spec['note']}")
        print(f"{'-'*60}")

        try:
            cfg = load_yaml(config_path)
        except Exception as exc:
            print(f"  [ERROR] Could not load {config_path}: {exc}")
            summaries.append({"backbone": backbone, "status": f"config_error: {exc}"})
            continue

        apply_overrides(cfg, args, spec, output_dir)

        if args.dry_run:
            print(f"  [dry-run] Config OK - backbone={cfg['backbone']['name']}  "
                  f"num_classes={cfg['detection']['num_classes']}  "
                  f"epochs={cfg['epochs']}  batch={cfg['batch_size']}")
            summaries.append({"backbone": backbone, "status": "dry_run_ok", "note": spec["note"]})
            continue

        run_start = time.perf_counter()
        try:
            summary = train(cfg, config_path=str(config_path))
        except Exception as exc:
            elapsed = time.perf_counter() - run_start
            print(f"  [FAILED] {backbone}: {exc}")
            summaries.append({
                "backbone": backbone,
                "status": f"failed: {exc}",
                "total_training_time_sec": round(elapsed, 1),
                "note": spec["note"],
            })
            write_comparison(summaries, output_dir)
            continue

        summary["note"] = spec["note"]
        summary["status"] = "ok"
        summary["best_val_map_50"] = summary.pop("best_val_map", None)  # rename for clarity
        summary["best_val_map"] = summary.get("best_val_map_50")  # keep both

        # Re-read best_val_map from the summary (train() returns best_val_map)
        # The train() return dict already has best_val_map as the combined-metric best
        summaries.append(summary)

        run_path = output_dir / "runs" / f"{backbone}.json"
        run_path.parent.mkdir(parents=True, exist_ok=True)
        run_path.write_text(json.dumps(summary, indent=2, default=str), encoding="utf-8")

        print(f"\n  [{backbone}] Done - "
              f"mAP={summary.get('best_val_map', 'N/A')}  "
              f"seg_IoU={summary.get('best_val_seg_iou', 'N/A')}  "
              f"latency={summary.get('latency_ms', 'N/A')} ms  "
              f"time={summary.get('total_training_time_sec', 'N/A'):.0f}s")

        write_comparison(summaries, output_dir)

    total = time.perf_counter() - sweep_start
    print(f"\n{'='*60}")
    print(f"  Sweep complete in {total/3600:.1f} h ({total:.0f} s)")
    print(f"{'='*60}\n")

    write_comparison(summaries, output_dir)
    write_recommendation(summaries, output_dir)

    print("\nNext steps:")
    print("  1. python scripts/pseudo_label.py --checkpoint outputs/backbone_sweep/checkpoints/<best_backbone>/best.pt")
    print("  2. python scripts/generate_report.py --sweep_dir outputs/backbone_sweep")


if __name__ == "__main__":
    main()
