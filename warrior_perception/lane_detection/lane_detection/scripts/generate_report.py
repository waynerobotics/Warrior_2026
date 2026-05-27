"""
Generate a professional Markdown comparison report from a completed backbone sweep.

Reads:
  <sweep_dir>/comparison.json
  <sweep_dir>/recommendation.json
  <sweep_dir>/checkpoints/<backbone>/metrics.jsonl   (one JSONL per backbone)

Writes:
  <sweep_dir>/report/report.md
  <sweep_dir>/report/curves_<backbone>.png           (training curves)
  <sweep_dir>/report/per_class_ap.png                (grouped bar chart)

Usage (from project root):
  python scripts/generate_report.py --sweep_dir outputs/backbone_sweep
  python scripts/generate_report.py --sweep_dir outputs/backbone_sweep --no_plots
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

CLASS_NAMES = ["barrel", "pedestrian", "stop_sign", "unknown", "tire", "pothole"]
BACKBONE_COLORS = {
    "resnet18": "#4C72B0",
    "convnext_base": "#DD8452",
    "swin_b": "#55A868",
    "hrnet_w32": "#C44E52",
}


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

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


def load_comparison(sweep_dir: Path) -> list[dict]:
    p = sweep_dir / "comparison.json"
    if not p.exists():
        return []
    return json.loads(p.read_text(encoding="utf-8"))


def load_recommendation(sweep_dir: Path) -> dict:
    p = sweep_dir / "recommendation.json"
    if not p.exists():
        return {}
    return json.loads(p.read_text(encoding="utf-8"))


def load_all_metrics(sweep_dir: Path, backbones: list[str]) -> dict[str, list[dict]]:
    result = {}
    for bb in backbones:
        metrics_path = sweep_dir / "checkpoints" / bb / "metrics.jsonl"
        result[bb] = load_jsonl(metrics_path)
    return result


# ---------------------------------------------------------------------------
# Plot helpers
# ---------------------------------------------------------------------------

def plot_training_curves(backbone: str, epochs: list[dict], out_path: Path) -> None:
    if not HAS_MPL or not epochs:
        return

    ep = [r["epoch"] for r in epochs]
    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    fig.suptitle(f"Training curves — {backbone}", fontsize=14, fontweight="bold")
    color = BACKBONE_COLORS.get(backbone, "#666666")

    def _plot(ax: Any, y_key: str, y2_key: str | None, title: str, ylabel: str) -> None:
        y = [r.get(y_key, 0.0) for r in epochs]
        ax.plot(ep, y, color=color, label="train" if y2_key else title, linewidth=1.5)
        if y2_key:
            y2 = [r.get(y2_key, 0.0) for r in epochs]
            ax.plot(ep, y2, color=color, linestyle="--", label="val", linewidth=1.5)
            ax.legend(fontsize=8)
        ax.set_title(title, fontsize=10)
        ax.set_xlabel("Epoch")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)

    _plot(axes[0, 0], "train_loss_total", "val_loss_total", "Total Loss", "Loss")
    _plot(axes[0, 1], "train_loss_det",   "val_loss_det",   "Detection Loss", "Loss")
    _plot(axes[0, 2], "train_loss_seg",   "val_loss_seg",   "Segmentation Loss", "Loss")
    _plot(axes[1, 0], "val_det_mAP",      None,             "val mAP (0.5:0.95)", "mAP")
    _plot(axes[1, 1], "val_seg_iou",      None,             "val Seg IoU", "IoU")
    _plot(axes[1, 2], "lr",               None,             "Learning Rate", "LR")

    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(out_path), dpi=150)
    plt.close()


def plot_per_class_ap(
    backbone_data: dict[str, list[dict]],
    class_names: list[str],
    out_path: Path,
) -> None:
    if not HAS_MPL:
        return

    backbones = list(backbone_data.keys())
    # Use last epoch's per-class AP for each backbone
    ap_matrix: dict[str, list[float]] = {}
    for bb, epochs in backbone_data.items():
        if not epochs:
            ap_matrix[bb] = [0.0] * len(class_names)
            continue
        last = epochs[-1]
        per_class = last.get("val_per_class_det_ap", [])
        while len(per_class) < len(class_names):
            per_class.append(0.0)
        ap_matrix[bb] = per_class[:len(class_names)]

    x = np.arange(len(class_names))
    width = 0.8 / max(len(backbones), 1)

    fig, ax = plt.subplots(figsize=(12, 5))
    for i, bb in enumerate(backbones):
        offset = (i - len(backbones) / 2 + 0.5) * width
        bars = ax.bar(x + offset, ap_matrix[bb], width * 0.9,
                      label=bb, color=BACKBONE_COLORS.get(bb, "#888888"), alpha=0.85)
        for bar in bars:
            h = bar.get_height()
            if h > 0.01:
                ax.text(bar.get_x() + bar.get_width() / 2, h + 0.005,
                        f"{h:.2f}", ha="center", va="bottom", fontsize=6)

    ax.set_xticks(x)
    ax.set_xticklabels(class_names, rotation=15, ha="right")
    ax.set_ylabel("AP (0.5:0.95)")
    ax.set_title("Per-class Detection AP by Backbone (final epoch)", fontweight="bold")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, axis="y", alpha=0.3)
    ax.set_ylim(0, 1.05)

    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(out_path), dpi=150)
    plt.close()


# ---------------------------------------------------------------------------
# Markdown generation
# ---------------------------------------------------------------------------

def fmt(v: Any, decimals: int = 3) -> str:
    if v is None or v == "":
        return "—"
    try:
        return f"{float(v):.{decimals}f}"
    except (TypeError, ValueError):
        return str(v)


def generate_report(sweep_dir: Path, no_plots: bool = False) -> str:
    comparison = load_comparison(sweep_dir)
    recommendation = load_recommendation(sweep_dir)
    backbones = [r["backbone"] for r in comparison if "backbone" in r]
    all_metrics = load_all_metrics(sweep_dir, backbones)

    report_dir = sweep_dir / "report"
    report_dir.mkdir(parents=True, exist_ok=True)

    if not no_plots and HAS_MPL:
        for bb in backbones:
            plot_training_curves(bb, all_metrics[bb], report_dir / f"curves_{bb}.png")
        plot_per_class_ap(all_metrics, CLASS_NAMES, report_dir / "per_class_ap.png")

    lines: list[str] = []
    ts = datetime.now().strftime("%Y-%m-%d %H:%M")

    lines += [
        f"# Multi-Task Perception — Backbone Comparison Report",
        f"",
        f"**Generated:** {ts}  ",
        f"**Sweep directory:** `{sweep_dir}`  ",
        f"**Detection classes:** {', '.join(CLASS_NAMES)}  ",
        f"**Task:** Joint object detection + lane segmentation  ",
        f"",
        "---",
        "",
    ]

    # --- Recommendation ---
    if recommendation:
        best = recommendation.get("best_overall", "—")
        fastest = recommendation.get("fastest", "—")
        lines += [
            "## Recommendation",
            "",
            f"| Criterion | Backbone |",
            f"|-----------|----------|",
            f"| **Best overall** (mAP + seg-IoU) | `{best}` |",
            f"| **Fastest** (lowest latency) | `{fastest}` |",
            "",
        ]
        for note in recommendation.get("notes", []):
            lines.append(f"> ⚠️  {note}")
        lines.append("")

    # --- Summary table ---
    lines += [
        "## Performance Summary",
        "",
        "| Backbone | mAP | mAP@50 | mAP@75 | Seg IoU | Seg Dice | "
        "Latency (ms) | FPS | Epochs | Status |",
        "|----------|-----|--------|--------|---------|----------|"
        "-------------|-----|--------|--------|",
    ]
    for row in comparison:
        bb = row.get("backbone", "?")
        # Pull final-epoch metrics if available
        epochs = all_metrics.get(bb, [])
        last = epochs[-1] if epochs else {}
        lines.append(
            f"| `{bb}` "
            f"| {fmt(row.get('best_val_map'))} "
            f"| {fmt(last.get('val_det_mAP_50'))} "
            f"| {fmt(last.get('val_det_mAP_75'))} "
            f"| {fmt(row.get('best_val_seg_iou'))} "
            f"| {fmt(row.get('best_val_seg_dice'))} "
            f"| {fmt(row.get('latency_ms'), 1)} "
            f"| {fmt(row.get('throughput_fps'), 1)} "
            f"| {row.get('completed_epoch', '—')} "
            f"| {row.get('status', '—')} |"
        )
    lines += ["", ""]

    # --- Per-class AP table ---
    lines += [
        "## Per-Class Detection AP (final epoch)",
        "",
        "| Backbone | " + " | ".join(CLASS_NAMES) + " |",
        "|----------|" + "|".join(["------"] * len(CLASS_NAMES)) + "|",
    ]
    for bb in backbones:
        epochs = all_metrics.get(bb, [])
        last = epochs[-1] if epochs else {}
        per_class = last.get("val_per_class_det_ap", [0.0] * len(CLASS_NAMES))
        while len(per_class) < len(CLASS_NAMES):
            per_class.append(0.0)
        cells = " | ".join(fmt(v) for v in per_class[:len(CLASS_NAMES)])
        lines.append(f"| `{bb}` | {cells} |")
    lines += ["", ""]

    # --- Per-backbone training details ---
    lines += ["## Training Details by Backbone", ""]
    for row in comparison:
        bb = row.get("backbone", "?")
        epochs = all_metrics.get(bb, [])
        lines += [
            f"### {bb}",
            "",
            f"- **Status:** {row.get('status', '—')}",
            f"- **Best epoch:** {row.get('best_epoch', '—')} / {row.get('completed_epoch', '—')}",
            f"- **Early stopped:** {row.get('stopped_early', False)}",
            f"- **Peak GPU memory:** {fmt(row.get('peak_memory_mb'), 0)} MB",
            f"- **Total training time:** {fmt(row.get('total_training_time_sec'), 0)} s",
            f"- **Note:** {row.get('note', '—')}",
            "",
        ]
        if not no_plots and HAS_MPL:
            lines.append(f"![Training curves](curves_{bb}.png)")
            lines.append("")
        if epochs:
            best_ep = max(epochs, key=lambda r: float(r.get("val_det_mAP", 0.0)))
            lines += [
                f"**Best epoch ({best_ep['epoch']}) metrics:**",
                "",
                f"| Metric | Value |",
                f"|--------|-------|",
                f"| val mAP | {fmt(best_ep.get('val_det_mAP'))} |",
                f"| val mAP@50 | {fmt(best_ep.get('val_det_mAP_50'))} |",
                f"| val Seg IoU | {fmt(best_ep.get('val_seg_iou'))} |",
                f"| val Seg Dice | {fmt(best_ep.get('val_seg_dice'))} |",
                f"| val Total Loss | {fmt(best_ep.get('val_loss_total'))} |",
                "",
            ]
        lines.append("")

    # --- Visualisation ---
    if not no_plots and HAS_MPL:
        lines += [
            "## Per-Class AP Comparison",
            "",
            "![Per-class AP](per_class_ap.png)",
            "",
        ]

    # --- Methodology ---
    lines += [
        "## Methodology",
        "",
        "- **Architecture:** Custom anchor-free detector (FPN P3/P4/P5) + binary segmentation head",
        "- **Detection head:** Objectness + L1 box regression + cross-entropy classification",
        "- **Segmentation head:** Bilinear upsampling to input resolution, binary CE loss",
        "- **Backbone freeze:** ResNet18/ConvNeXt/Swin pre-trained on ImageNet (ILSVRC)",
        "  HRNet-W32 trained from scratch (custom lightweight fallback, not full HRNet)",
        "- **Augmentation:** Affine (scale ±8%, translate ±5%, rotate ±4°), brightness/contrast ±25%, "
        "HSV jitter, motion/gaussian blur",
        "- **Optimiser:** AdamW with cosine annealing LR schedule",
        "- **Evaluation:** torchmetrics MeanAveragePrecision (COCO IoU 0.5:0.95 and 0.5)",
        "- **Data:** ~1,386 manually annotated frames split 80/10/10 train/val/test",
        "",
        "## Next Steps",
        "",
        "1. Run `scripts/pseudo_label.py` with the best checkpoint to expand the training set "
        "by ~3,600 unlabeled images",
        "2. Retrain the best backbone on the expanded dataset",
        "3. Export best model to ONNX: `torch.onnx.export(...)`",
        "4. Quantise with TensorRT for deployment on the robot",
        "",
    ]

    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="Generate Markdown comparison report from sweep results.")
    parser.add_argument("--sweep_dir", default="outputs/backbone_sweep")
    parser.add_argument("--no_plots", action="store_true",
                        help="Skip matplotlib plots (useful on headless servers without matplotlib)")
    args = parser.parse_args()

    sweep_dir = Path(args.sweep_dir)
    if not sweep_dir.exists():
        print(f"[ERROR] sweep_dir not found: {sweep_dir}")
        sys.exit(1)

    if not HAS_MPL and not args.no_plots:
        print("[WARNING] matplotlib not available — plots will be skipped")

    print(f"Generating report from {sweep_dir} ...")
    report_md = generate_report(sweep_dir, no_plots=args.no_plots)

    report_dir = sweep_dir / "report"
    report_dir.mkdir(parents=True, exist_ok=True)
    report_path = report_dir / "report.md"
    report_path.write_text(report_md, encoding="utf-8")
    print(f"Report written → {report_path}")


if __name__ == "__main__":
    main()
