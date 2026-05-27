"""
Generate portfolio documentation assets from existing training results.

Writes to docs/assets/ (committed to git):

  curves_<backbone>.png   — train/val loss + metric curves
  inference_<frame>.jpg   — detection boxes + lane mask overlay on val images

Usage (from repo root):

  # Everything (recommended first run)
  python scripts/generate_portfolio_assets.py

  # Curves only (no GPU / checkpoint needed)
  python scripts/generate_portfolio_assets.py --curves_only

  # Inference only
  python scripts/generate_portfolio_assets.py --inference_only

  # Custom paths
  python scripts/generate_portfolio_assets.py \\
      --sweep_dir outputs/backbone_sweep \\
      --checkpoint outputs/backbone_sweep/checkpoints/resnet18/best.pt \\
      --manifest data/val_manifest.json \\
      --num_samples 6
"""
from __future__ import annotations

import argparse
import json
import random
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv2
import numpy as np
import torch

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("[WARN] matplotlib not found — skipping curve plots. Run: pip install matplotlib")

# ── Constants ──────────────────────────────────────────────────────────────────

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)

CLASS_NAMES = ["barrel", "pedestrian", "stop_sign", "unknown", "tire", "pothole"]

CLASS_COLORS_BGR: dict[int, tuple[int, int, int]] = {
    0: (50,  50,  220),   # barrel      — red
    1: (50,  220,  50),   # pedestrian  — green
    2: (0,   200, 240),   # stop_sign   — yellow
    3: (180, 180, 180),   # unknown     — silver
    4: (0,   130, 240),   # tire        — orange
    5: (220,  0,  150),   # pothole     — purple
}

BACKBONE_COLORS = {
    "resnet18":      "#4C72B0",
    "convnext_base": "#DD8452",
    "swin_b":        "#55A868",
    "hrnet_w32":     "#C44E52",
}

SEG_COLOR_BGR = (220, 180, 0)
SEG_ALPHA     = 0.40
SKY_ZONE_FRAC   = 0.35
ROBOT_BODY_FRAC = 0.85


# ── Training curves ────────────────────────────────────────────────────────────

def load_jsonl(path: Path) -> list[dict]:
    if not path.exists():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return rows


def plot_curves(backbone: str, rows: list[dict], out_path: Path) -> None:
    if not HAS_MPL or not rows:
        return

    ep     = [r["epoch"] for r in rows]
    color  = BACKBONE_COLORS.get(backbone, "#666666")

    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    fig.suptitle(f"Training curves — {backbone}", fontsize=14, fontweight="bold")

    def _plot(ax, train_key: str, val_key: str | None, title: str, ylabel: str) -> None:
        ax.plot(ep, [r.get(train_key, 0.0) for r in rows],
                color=color, linewidth=1.8, label="train")
        if val_key:
            ax.plot(ep, [r.get(val_key, 0.0) for r in rows],
                    color=color, linestyle="--", linewidth=1.8, label="val")
            ax.legend(fontsize=8)
        ax.set_title(title, fontsize=10)
        ax.set_xlabel("Epoch")
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)

    _plot(axes[0, 0], "train_loss_total",  "val_loss_total",  "Total Loss",           "Loss")
    _plot(axes[0, 1], "train_loss_det",    "val_loss_det",    "Detection Loss",        "Loss")
    _plot(axes[0, 2], "train_loss_seg",    "val_loss_seg",    "Segmentation Loss",     "Loss")
    _plot(axes[1, 0], "val_det_mAP",       None,              "val mAP (0.5:0.95)",    "mAP")
    _plot(axes[1, 1], "val_seg_iou",       None,              "val Lane IoU",          "IoU")
    _plot(axes[1, 2], "lr",                None,              "Learning Rate",         "LR")

    plt.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(str(out_path), dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  curves → {out_path.relative_to(ROOT)}")


def generate_curves(sweep_dir: Path, out_dir: Path) -> None:
    print("\n[1/2] Generating training curves...")
    if not HAS_MPL:
        print("  Skipped — matplotlib not installed.")
        return

    found = False
    for bb_dir in sorted((sweep_dir / "checkpoints").iterdir()):
        if not bb_dir.is_dir():
            continue
        rows = load_jsonl(bb_dir / "metrics.jsonl")
        if not rows:
            print(f"  [SKIP] {bb_dir.name} — metrics.jsonl not found or empty")
            continue
        found = True
        plot_curves(bb_dir.name, rows, out_dir / f"curves_{bb_dir.name}.png")

    if not found:
        print(f"  [WARN] No metrics.jsonl files found under {sweep_dir / 'checkpoints'}")


# ── Inference visualisation ────────────────────────────────────────────────────

def load_model(checkpoint_path: Path, device: torch.device):
    from models import MultiTaskPerceptionModel

    ckpt = torch.load(str(checkpoint_path), map_location=device)
    cfg  = ckpt["cfg"]
    model = MultiTaskPerceptionModel(cfg)
    model.load_state_dict(ckpt["state_dict"])
    model.to(device).eval()
    backbone = cfg["backbone"]["name"]
    epoch    = ckpt.get("epoch", "?")
    print(f"  Loaded {backbone} checkpoint (epoch {epoch}) from {checkpoint_path.relative_to(ROOT)}")
    return model, cfg


def preprocess(image_path: Path, hw: tuple[int, int]) -> tuple[torch.Tensor, np.ndarray]:
    bgr = cv2.imread(str(image_path))
    if bgr is None:
        raise FileNotFoundError(f"Cannot read: {image_path}")
    h, w = hw
    bgr_r = cv2.resize(bgr, (w, h), interpolation=cv2.INTER_LINEAR)
    rgb   = bgr_r[..., ::-1].astype(np.float32) / 255.0
    rgb   = (rgb - IMAGENET_MEAN) / IMAGENET_STD
    tensor = torch.from_numpy(rgb.transpose(2, 0, 1)).unsqueeze(0)  # (1, 3, H, W)
    return tensor, bgr_r


@torch.no_grad()
def run_inference(model, tensor: torch.Tensor, hw: tuple[int, int],
                  score_threshold: float, nms_threshold: float) -> dict[str, Any]:
    from models.detection_postprocess import decode_detections

    outputs = model(tensor)
    level_outputs = {k.replace("det_", ""): v for k, v in outputs.items() if k.startswith("det_")}
    dets = decode_detections(
        level_outputs=level_outputs,
        image_size=hw,
        score_threshold=score_threshold,
        nms_threshold=nms_threshold,
        max_detections=100,
    )[0]
    seg_mask = outputs["seg_logits"][0].argmax(0).cpu().numpy().astype(np.uint8)
    return {
        "boxes":    dets["boxes"].cpu().numpy(),
        "scores":   dets["scores"].cpu().numpy(),
        "labels":   dets["labels"].cpu().numpy(),
        "seg_mask": seg_mask,
    }


def _text_with_bg(canvas: np.ndarray, text: str, origin: tuple[int, int],
                  font_scale: float, color_bgr: tuple[int, int, int], thickness: int = 1) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = origin
    cv2.rectangle(canvas, (x, y - th - baseline), (x + tw + 2, y + baseline), (20, 20, 20), -1)
    cv2.putText(canvas, text, (x + 1, y), font, font_scale, color_bgr, thickness, cv2.LINE_AA)


def annotate(bgr: np.ndarray, result: dict[str, Any],
             header_text: str, score_threshold: float) -> np.ndarray:
    canvas = bgr.copy()
    H, W   = canvas.shape[:2]
    font_scale    = max(0.35, W / 3200.0)
    box_thickness = max(1, int(W / 640))
    label_line_h  = int(font_scale * 20) + 6

    # Lane mask overlay
    sky_cutoff = int(H * SKY_ZONE_FRAC)
    lane_mask  = result["seg_mask"] > 0
    lane_mask[:sky_cutoff, :] = False
    if lane_mask.any():
        overlay = canvas.copy()
        overlay[lane_mask] = SEG_COLOR_BGR
        canvas = cv2.addWeighted(canvas, 1.0 - SEG_ALPHA, overlay, SEG_ALPHA, 0)
        contours, _ = cv2.findContours(lane_mask.astype(np.uint8),
                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(canvas, contours, -1, SEG_COLOR_BGR, max(1, box_thickness - 1))

    # Detection boxes
    robot_cutoff = int(H * ROBOT_BODY_FRAC)
    strip_w      = max(100, W // 16)
    label_slots: dict[int, int] = {}
    num_det = 0

    for box, score, label in zip(result["boxes"], result["scores"], result["labels"]):
        if score < score_threshold:
            continue
        x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        if (y1 + y2) // 2 >= robot_cutoff:
            continue
        num_det += 1
        color    = CLASS_COLORS_BGR.get(int(label), (200, 200, 200))
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, box_thickness)

        cls_name   = CLASS_NAMES[int(label)] if int(label) < len(CLASS_NAMES) else str(label)
        label_text = f"{cls_name} {score:.2f}"
        strip_idx  = x1 // strip_w
        label_y    = max(label_line_h, label_slots.get(strip_idx, y1))
        if y1 > label_slots.get(strip_idx, 0):
            label_y = max(label_line_h, y1)
        label_slots[strip_idx] = label_y + label_line_h
        _text_with_bg(canvas, label_text, (x1, label_y), font_scale, color)

    # Header bar
    bar_h  = max(36, int(H * 0.055))
    header = np.zeros((bar_h, W, 3), dtype=np.uint8)
    seg_pct = 100.0 * lane_mask.sum() / max(1, H * W)
    htext   = f"{header_text}   dets={num_det}   lane={seg_pct:.1f}%"
    hfont   = max(0.45, font_scale * 1.1)
    cv2.putText(header, htext, (10, bar_h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, hfont, (200, 220, 255), 1, cv2.LINE_AA)

    return np.concatenate([header, canvas], axis=0)


def collect_images(manifest_path: Path, num_samples: int, seed: int) -> list[Path]:
    if not manifest_path.exists():
        raise FileNotFoundError(f"Manifest not found: {manifest_path}")
    with manifest_path.open(encoding="utf-8") as f:
        rows = json.load(f)
    if isinstance(rows, dict):
        rows = rows.get("samples", [])

    candidates: list[Path] = []
    for row in rows:
        p = Path(row.get("image", ""))
        if not p.is_absolute():
            p = ROOT / p
        if p.exists():
            candidates.append(p)

    if not candidates:
        raise RuntimeError(f"No accessible images found in {manifest_path}")

    rng = random.Random(seed)
    if len(candidates) > num_samples:
        candidates = sorted(rng.sample(candidates, num_samples))
    return candidates


def generate_inference(checkpoint_path: Path, manifest_path: Path,
                       num_samples: int, out_dir: Path,
                       score_threshold: float, seed: int) -> None:
    print("\n[2/2] Generating inference samples...")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"  Device: {device}")

    model, cfg = load_model(checkpoint_path, device)
    hw = tuple(cfg["dataset"]["image_size"])  # (H, W)
    backbone = cfg["backbone"]["name"]
    epoch    = 0
    ckpt_ep  = torch.load(str(checkpoint_path), map_location="cpu").get("epoch", "best")
    header   = f"{backbone}  ep={ckpt_ep}"

    images = collect_images(manifest_path, num_samples, seed)
    print(f"  Running on {len(images)} image(s)  score_thresh={score_threshold}")

    out_dir.mkdir(parents=True, exist_ok=True)
    for img_path in images:
        try:
            tensor, bgr_display = preprocess(img_path, hw)
            tensor = tensor.to(device)
            result = run_inference(model, tensor, hw, score_threshold, 0.5)
        except Exception as exc:
            print(f"  [SKIP] {img_path.name}: {exc}")
            continue

        panel    = annotate(bgr_display, result, header, score_threshold)
        out_path = out_dir / f"inference_{img_path.stem}.jpg"
        cv2.imwrite(str(out_path), panel, [cv2.IMWRITE_JPEG_QUALITY, 92])
        n_det = int((result["scores"] >= score_threshold).sum())
        seg_pct = 100.0 * (result["seg_mask"] > 0).mean()
        print(f"  {img_path.name}  dets={n_det}  lane={seg_pct:.1f}%  → {out_path.relative_to(ROOT)}")


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate training curves and inference samples for portfolio docs."
    )
    parser.add_argument("--sweep_dir",   default="outputs/backbone_sweep",
                        help="Backbone sweep output directory")
    parser.add_argument("--checkpoint",  default="",
                        help="Path to a .pt checkpoint for inference (default: sweep best.pt)")
    parser.add_argument("--manifest",    default="",
                        help="Validation manifest JSON for sampling inference images")
    parser.add_argument("--out_dir",     default="docs/assets",
                        help="Output directory for generated assets")
    parser.add_argument("--num_samples", type=int, default=6,
                        help="Number of validation images to visualise")
    parser.add_argument("--score_threshold", type=float, default=0.20)
    parser.add_argument("--seed",        type=int, default=42)
    parser.add_argument("--curves_only",    action="store_true")
    parser.add_argument("--inference_only", action="store_true")
    args = parser.parse_args()

    sweep_dir = ROOT / args.sweep_dir
    out_dir   = ROOT / args.out_dir

    do_curves    = not args.inference_only
    do_inference = not args.curves_only

    # ── Curves ────────────────────────────────────────────────────────────────
    if do_curves:
        if not (sweep_dir / "checkpoints").exists():
            print(f"[WARN] No checkpoints dir found at {sweep_dir} — skipping curves.")
        else:
            generate_curves(sweep_dir, out_dir)

    # ── Inference ─────────────────────────────────────────────────────────────
    if do_inference:
        # Resolve checkpoint
        ckpt_path = Path(args.checkpoint) if args.checkpoint else Path("")
        if not ckpt_path.is_absolute():
            ckpt_path = ROOT / ckpt_path if args.checkpoint else Path("")
        if not ckpt_path.exists():
            # Try to auto-discover best.pt from sweep
            candidates = sorted((sweep_dir / "checkpoints").glob("*/best.pt"))
            if not candidates:
                print("[ERROR] No checkpoint found. Pass --checkpoint path/to/best.pt")
                if do_curves:
                    return
                sys.exit(1)
            ckpt_path = candidates[0]
            print(f"  Auto-selected checkpoint: {ckpt_path.relative_to(ROOT)}")

        # Resolve manifest
        manifest_path = Path(args.manifest) if args.manifest else Path("")
        if not manifest_path.is_absolute():
            manifest_path = ROOT / manifest_path if args.manifest else Path("")
        if not manifest_path.exists():
            # Try to find any val_manifest.json
            guesses = [
                ROOT / "data" / "val_manifest.json",
                sweep_dir / "val_manifest.json",
            ]
            manifest_path = next((p for p in guesses if p.exists()), Path(""))
            if not manifest_path.exists():
                print("[ERROR] No manifest found. Pass --manifest path/to/val_manifest.json")
                sys.exit(1)
            print(f"  Auto-selected manifest: {manifest_path.relative_to(ROOT)}")

        generate_inference(ckpt_path, manifest_path, args.num_samples,
                           out_dir, args.score_threshold, args.seed)

    print(f"\nDone. Assets saved to {out_dir.relative_to(ROOT)}/")
    print("Commit docs/assets/ to make results visible on GitHub.")


if __name__ == "__main__":
    main()
